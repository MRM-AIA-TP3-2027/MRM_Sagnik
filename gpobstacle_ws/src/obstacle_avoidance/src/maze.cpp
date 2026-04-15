/**
 * maze.cpp — TARS AUTONOMOUS MAZE Navigator (LiDAR + Odometry)
 * ──────────────────────────────────────────────────────────────
 * Sensors  : /tars/odom  (nav_msgs/Odometry)   — position & yaw
 *            /scan       (sensor_msgs/LaserScan) — 360° obstacle data
 *
 * Goal topics (pick ONE):
 *   /goal_xy  (geometry_msgs/Point)  ← PREFERRED for Gazebo
 *             x = world-X (east),  y = world-Y (north)
 *             Example:  ros2 topic pub /goal_xy geometry_msgs/msg/Point
 *                         "{x: 8.5, y: -7.0, z: 0.0}"
 *
 *   /goal_gps (geometry_msgs/Point)  ← GPS path (kept for compatibility)
 *             x = latitude,  y = longitude  (converted via gpsToENU)
 *
 * ── MAZE COORDINATES ──────────────────────────────────────────────────
 *   Arena  : 18 m × 18 m  (x: -9 → +9, y: -9 → +9)
 *   ENTRY  : x = -9,  y = 4.5 → 9   — spawn near (-8.5,  7.0)
 *   EXIT   : x = +9,  y = -9 → -4.5 — goal  near ( 8.5, -7.0)  ◄──────
 *
 * OA algo  : LiDAR-sector VFH-lite
 * States   : IDLE → ALIGNING → DRIVING → ESCAPING → DONE
 *
 * Local minima escape: left-hand wall-following (Bug2-style)
 *   When stuck for ~2 s, robot hugs the left wall until it has made
 *   measurable progress toward the goal, then resumes goal-seeking.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <cmath>
#include <deque>
#include <memory>
#include <random>
#include <vector>
#include <algorithm>

// ── Constants ─────────────────────────────────────────────────────────────────
static constexpr double EARTH_R = 6371000.0;
static constexpr double RAD     = M_PI / 180.0;
static constexpr double DEG     = 180.0 / M_PI;

// ── GPS reference origin (used ONLY by /goal_gps path) ───────────────────────
static constexpr double REF_LAT = 12.9716;
static constexpr double REF_LON = 77.5946;

static constexpr double STOP_D  = 0.45;   // [m] emergency stop / backup
static constexpr double SLOW_D  = 0.60;   // [m] start slowing & VFH steering

// ── Wall-following escape parameters ─────────────────────────────────────────
static constexpr double WALL_FOLLOW_DIST = 0.55;   // [m] target left-wall gap
static constexpr double WALL_FOLLOW_SECS = 12.0;   // [s] max escape time before retry
static constexpr double ESC_PROGRESS_THR = 0.50;   // [m] progress to resume goal-seek

static constexpr int NSEC = 36;  // LiDAR sectors (10° each)

enum class State { IDLE, ALIGNING, DRIVING, ESCAPING, DONE };

// ── Helpers ───────────────────────────────────────────────────────────────────
struct ENU { double e=0, n=0; };

ENU gpsToENU(double lat, double lon) {
    double dlat = (lat - REF_LAT) * RAD;
    double dlon = (lon - REF_LON) * RAD;
    double mlat = 0.5 * (lat + REF_LAT) * RAD;
    return { EARTH_R * std::cos(mlat) * dlon, EARTH_R * dlat };
}

inline double wrap(double a) {
    while (a >  M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
}

inline double q2yaw(double x, double y, double z, double w) {
    double nm = std::sqrt(x*x + y*y + z*z + w*w);
    if (nm < 1e-9) return 0.0;
    x /= nm; y /= nm; z /= nm; w /= nm;
    return std::atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
}

// ══════════════════════════════════════════════════════════════════════════════
// OdomPose — reads /tars/odom for position (x=east, y=north) and yaw
// ══════════════════════════════════════════════════════════════════════════════
class OdomPose {
    double e_=0, n_=0, yaw_=0;
    bool ok_=false;
public:
    void update(const nav_msgs::msg::Odometry& msg) {
        e_   = msg.pose.pose.position.x;
        n_   = msg.pose.pose.position.y;
        auto& q = msg.pose.pose.orientation;
        yaw_ = q2yaw(q.x, q.y, q.z, q.w);
        ok_  = true;
    }
    bool   ok()  const { return ok_;  }
    double e()   const { return e_;   }
    double n()   const { return n_;   }
    double yaw() const { return yaw_; }
};

// ══════════════════════════════════════════════════════════════════════════════
// LidarVFH — 360° Vector Field Histogram built from /scan
// ══════════════════════════════════════════════════════════════════════════════
class LidarVFH {
    std::vector<float> sec_;
    bool ok_ = false;

    int angleToSector(float angle) const {
        float norm = angle + static_cast<float>(M_PI);   // [0, 2π)
        int s = static_cast<int>(norm / (2*M_PI) * NSEC);
        return std::clamp(s, 0, NSEC-1);
    }

public:
    LidarVFH() : sec_(NSEC, 99.f) {}

    void update(const sensor_msgs::msg::LaserScan& scan) {
        std::fill(sec_.begin(), sec_.end(), 99.f);
        const int n = static_cast<int>(scan.ranges.size());
        for (int i = 0; i < n; ++i) {
            float r = scan.ranges[i];
            if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max)
                continue;
            float angle = scan.angle_min + i * scan.angle_increment;
            int s = angleToSector(angle);
            sec_[s] = std::min(sec_[s], r);
        }
        ok_ = true;
    }

    static double sectorAngle(int s) {
        return -M_PI + (s + 0.5) * (2*M_PI / NSEC);
    }

    // ── Returns the minimum range in sector s ─────────────────────────────
    float sectorDist(int sector) const {
        int s = std::clamp(sector, 0, NSEC-1);
        return sec_[s];
    }

    // ── Average of sectors covering a direction ───────────────────────────
    // dir=0 → front, dir=NSEC*3/4 → left, dir=NSEC/4 → right
    float avgSectorDist(int center, int half_width = 1) const {
        float sum = 0.f; int cnt = 0;
        for (int d = -half_width; d <= half_width; ++d) {
            int s = (center + d + NSEC) % NSEC;
            sum += sec_[s]; ++cnt;
        }
        return sum / cnt;
    }

    bool frontBlocked(double thr) const {
        if (!ok_) return false;
        for (int s = 0; s < NSEC; ++s)
            if (std::abs(wrap(sectorAngle(s))) < M_PI/7.2 && sec_[s] < thr)
                return true;
        return false;
    }

    float frontDist() const {
        float md = 99.f;
        for (int s = 0; s < NSEC; ++s)
            if (std::abs(wrap(sectorAngle(s))) < M_PI/7.2)
                md = std::min(md, sec_[s]);
        return md;
    }

    double bestAngle(double goalAngle) const {
        if (!ok_) return goalAngle;
        int best = -1; double bs = -1e9;
        for (int s = 0; s < NSEC; ++s) {
            double sa = sectorAngle(s);
            if (std::abs(wrap(sa)) > M_PI/2) continue;
            if (sec_[s] < STOP_D)            continue;
            if (std::abs(wrap(sa - goalAngle)) > M_PI/3) continue;
            double score = sec_[s] - 1.8 * std::abs(wrap(sa - goalAngle));
            if (score > bs) { bs = score; best = s; }
        }
        return (best >= 0) ? sectorAngle(best) : goalAngle;
    }

    double widestAngle() const {
        int b = 0; float bd = -1e9f;
        for (int s = 0; s < NSEC; ++s) {
            float bonus = (std::abs(wrap(sectorAngle(s))) < M_PI/2) ? 0.5f : 0.f;
            float score = sec_[s] + bonus;
            if (score > bd) { bd = score; b = s; }
        }
        return sectorAngle(b);
    }

    bool isOk() const { return ok_; }
};

// ══════════════════════════════════════════════════════════════════════════════
// LocalMinimaDetector — 2 s sliding window
// ══════════════════════════════════════════════════════════════════════════════
class LocalMinimaDetector {
    std::deque<std::pair<double,double>> buf_;
    const int WIN = 40; const double THR = 0.15;
    double dist_ = 1e9;
public:
    void update(double e, double n, double d) {
        buf_.push_back({e,n});
        if ((int)buf_.size() > WIN) buf_.pop_front();
        dist_ = d;
    }
    bool stuck() const {
        if ((int)buf_.size() < WIN || dist_ < 2.5) return false;
        auto& ref = buf_.front(); double mx = 0;
        for (auto& p : buf_)
            mx = std::max(mx, std::hypot(p.first-ref.first, p.second-ref.second));
        return mx < THR;
    }
    void reset() { buf_.clear(); }
};

// ══════════════════════════════════════════════════════════════════════════════
// TARSNavigator — main ROS 2 node
// ══════════════════════════════════════════════════════════════════════════════

// ── Escape sub-state ──────────────────────────────────────────────────────────
enum class EscapeMode {
    ROTATE_TO_WALL,   // Phase 0: turn right until we have a wall on the left
    WALL_FOLLOWING    // Phase 1: left-hand wall follow until progress made
};

class TARSNavigator : public rclcpp::Node {
    State               state_ = State::IDLE;
    OdomPose            pose_;
    LidarVFH            lidar_;
    LocalMinimaDetector minima_;
    std::mt19937        rng_{42};

    double lin_=0.4, ang_=0.6, gtol_=0.8, htol_=0.10;
    double ge_=0, gn_=0;
    double se_=0, sn_=0;            // start position for CTE
    rclcpp::Time drive_start_time_;

    // ── Escape state ──────────────────────────────────────────────────────
    EscapeMode  esc_mode_       = EscapeMode::ROTATE_TO_WALL;
    double      esc_start_dist_ = 0.0;  // dist-to-goal when escape started
    rclcpp::Time esc_deadline_;

    bool   marker_spawned_=false;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmdp_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr         distp_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr         ctep_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odoms_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidars_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr   goals_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr   goalxy_s_;
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr     spawn_;
    rclcpp::TimerBase::SharedPtr tim_;

public:
    TARSNavigator() : Node("tars_maze") {
        declare_parameter("linear_speed",      0.4);
        declare_parameter("angular_speed",     0.6);
        declare_parameter("goal_tolerance",    0.8);
        declare_parameter("heading_tolerance", 0.10);
        lin_  = get_parameter("linear_speed").as_double();
        ang_  = get_parameter("angular_speed").as_double();
        gtol_ = get_parameter("goal_tolerance").as_double();
        htol_ = get_parameter("heading_tolerance").as_double();

        cmdp_  = create_publisher<geometry_msgs::msg::Twist>("/tars/cmd_vel", 10);
        distp_ = create_publisher<std_msgs::msg::Float64>("/tars/distance_remaining", 10);
        ctep_  = create_publisher<std_msgs::msg::Float64>("/tars/cross_track_error", 10);

        odoms_ = create_subscription<nav_msgs::msg::Odometry>(
            "/tars/odom", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr m){ pose_.update(*m); });

        lidars_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr m){ lidar_.update(*m); });

        goals_ = create_subscription<geometry_msgs::msg::Point>("/goal_gps", 10,
            [this](geometry_msgs::msg::Point::SharedPtr m){ goalCb(m); });

        goalxy_s_ = create_subscription<geometry_msgs::msg::Point>("/goal_xy", 10,
            [this](geometry_msgs::msg::Point::SharedPtr m){ goalXyCb(m); });

        spawn_ = create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
        tim_   = create_wall_timer(std::chrono::milliseconds(50), [this]{ loop(); });

        RCLCPP_INFO(get_logger(),
            "TARS Navigator ready | Topics: /goal_xy (world XY) or /goal_gps (GPS)");
        RCLCPP_INFO(get_logger(),
            "  Maze exit goal → ros2 topic pub /goal_xy geometry_msgs/msg/Point "
            "'{x: 8.5, y: -7.0, z: 0.0}'");
    }

private:
    double dist()   const { return std::hypot(ge_ - pose_.e(), gn_ - pose_.n()); }
    double brobot() const {
        return wrap(std::atan2(gn_ - pose_.n(), ge_ - pose_.e()) - pose_.yaw()); }

    double crossTrackError() const {
        double dx = ge_ - se_, dy = gn_ - sn_;
        double pl = std::hypot(dx, dy);
        if (pl < 1e-6) return 0.0;
        double rx = pose_.e() - se_, ry = pose_.n() - sn_;
        return (dx * ry - dy * rx) / pl;
    }

    void stop()           { cmdp_->publish(geometry_msgs::msg::Twist()); }
    void pubD(double v)   { std_msgs::msg::Float64 m; m.data=v; distp_->publish(m); }
    void pubCTE(double v) { std_msgs::msg::Float64 m; m.data=v; ctep_->publish(m); }

    // ── /goal_gps callback ────────────────────────────────────────────────────
    void goalCb(const geometry_msgs::msg::Point::SharedPtr m) {
        if (state_==State::ALIGNING || state_==State::DRIVING ||
            state_==State::ESCAPING) {
            RCLCPP_WARN(get_logger(), "Navigating — goal ignored"); return; }
        auto g = gpsToENU(m->x, m->y); ge_ = g.e; gn_ = g.n;
        se_ = pose_.e(); sn_ = pose_.n();
        marker_spawned_ = false;
        minima_.reset(); state_ = State::ALIGNING;
        RCLCPP_INFO(get_logger(), "GPS Goal: lat=%.6f lon=%.6f  ENU=(%.2f, %.2f)  dist=%.2fm",
            m->x, m->y, ge_, gn_, dist());
        if (!marker_spawned_) { spawnMarker(ge_, gn_); marker_spawned_ = true; }
    }

    // ── /goal_xy callback ─────────────────────────────────────────────────────
    void goalXyCb(const geometry_msgs::msg::Point::SharedPtr m) {
        if (state_==State::ALIGNING || state_==State::DRIVING ||
            state_==State::ESCAPING) {
            RCLCPP_WARN(get_logger(), "Navigating — goal ignored"); return; }
        ge_ = m->x;  gn_ = m->y;
        se_ = pose_.e(); sn_ = pose_.n();
        marker_spawned_ = false;
        minima_.reset(); state_ = State::ALIGNING;
        RCLCPP_INFO(get_logger(),
            "XY Goal: world=(%.2f, %.2f)  dist=%.2fm", ge_, gn_, dist());
        if (!marker_spawned_) { spawnMarker(ge_, gn_); marker_spawned_ = true; }
    }

    void loop() {
        if (!pose_.ok()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "Waiting for /tars/odom..."); return; }
        if (!lidar_.isOk()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "Waiting for /scan..."); return; }
        double d_now = dist();
        pubD(d_now);
        pubCTE(crossTrackError());
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[CTE] cross_track=%.3fm  dist=%.2fm", crossTrackError(), d_now);
        if(state_ == State::DRIVING){
            minima_.update(pose_.e(), pose_.n(), d_now);
        }
        switch (state_) {
            case State::IDLE:
                RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,
                    "[IDLE] waiting for goal on /goal_xy or /goal_gps..."); break;
            case State::ALIGNING: doAlign();  break;
            case State::DRIVING:  doDrive();  break;
            case State::ESCAPING: doEscape(); break;
            case State::DONE:     stop();     break;
        }
    }

    // ── ALIGNING ──────────────────────────────────────────────────────────────
    void doAlign() {
        double err = brobot();
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),500,
            "[ALIGN] err=%.1f°", err*DEG);
        if (std::abs(err) < htol_) {
            stop(); 
            state_ = State::DRIVING;
            drive_start_time_ = now();
            RCLCPP_INFO(get_logger(), "-> DRIVING"); return; }
        geometry_msgs::msg::Twist c;
        c.angular.z = std::copysign(ang_ * std::clamp(std::abs(err)/0.5, 0.2, 1.0), err);
        cmdp_->publish(c);
    }

    // ── DRIVING ───────────────────────────────────────────────────────────────
    void doDrive() {
        double d    = dist();
        double herr = brobot();

        if (d < gtol_) {
            stop(); state_ = State::DONE;
            RCLCPP_INFO(get_logger(), "GOAL REACHED  residual=%.3fm", d); return; }

        if ((now() - drive_start_time_).seconds() > 2.0 && minima_.stuck()) {
            RCLCPP_WARN(get_logger(), "LOCAL MINIMA detected — starting wall-follow escape");
            startEscape(); 
            return; 
        }

        double front = lidar_.frontDist();

        if (lidar_.frontBlocked(STOP_D)) {
            geometry_msgs::msg::Twist c;
            c.linear.x  = -0.08;
            c.angular.z = std::clamp(1.0 * lidar_.bestAngle(herr), -ang_, ang_);
            cmdp_->publish(c);
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),400,
                "EMG backup — front=%.2fm", front);
            return; }

	static double last_steer = 0.0;

	double raw_steer;

	if (front > SLOW_D && std::abs(herr) < 0.15) {
    		raw_steer = 1.5 * herr;
	} else {
    		raw_steer = lidar_.bestAngle(herr);
	}

	// smooth it
	double steer = 0.7 * last_steer + 0.3 * raw_steer;
	steer = std::clamp(steer, -ang_, ang_);
	last_steer = steer;

        double spd = lin_ * std::clamp(
            std::min({ (front-STOP_D)/(SLOW_D-STOP_D), d/1.5, 1.0 }),
            0.22, 0.8);

        geometry_msgs::msg::Twist c;
        c.linear.x  = spd;
        c.angular.z = steer;
        if(front > STOP_D + 0.1){
            c.linear.x = std::max(c.linear.x, 0.15);
        }

        if(std::abs(steer) > 0.5 && front > STOP_D){
            c.angular.z *= 0.6;
        }
        cmdp_->publish(c);

        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
            "[DRIVE] d=%.2fm  hdg=%.1f°  steer=%.1f°  front=%.2fm  spd=%.2f",
            d, herr*DEG, steer*DEG, front, spd);
    }

    // ══════════════════════════════════════════════════════════════════════════
    // LOCAL MINIMA ESCAPE — Left-hand wall following (Bug2-style)
    //
    // Phase 0 (ROTATE_TO_WALL):
    //   Rotate right (clockwise) until the robot has a wall within
    //   WALL_FOLLOW_DIST * 1.5 on its left side, or the front is blocked.
    //   This acquires a wall to follow.
    //
    // Phase 1 (WALL_FOLLOWING):
    //   Proportional controller keeps left-wall distance at WALL_FOLLOW_DIST.
    //   At corners (front blocked), turn right to follow the wall around.
    //   Exit when dist-to-goal < esc_start_dist_ - ESC_PROGRESS_THR,
    //   meaning real progress toward the goal has been made.
    //
    // Timeout: if WALL_FOLLOW_SECS elapses without progress, restart ALIGNING.
    // ══════════════════════════════════════════════════════════════════════════

    void startEscape() {
        esc_mode_       = EscapeMode::ROTATE_TO_WALL;
        esc_start_dist_ = dist();
        esc_deadline_   = now() + rclcpp::Duration::from_seconds(WALL_FOLLOW_SECS);
        minima_.reset();
        state_ = State::ESCAPING;
        RCLCPP_WARN(get_logger(),
            "[ESC] Starting left-hand wall-follow | dist=%.2fm | timeout=%.0fs",
            esc_start_dist_, WALL_FOLLOW_SECS);
    }

    void doEscape() {
        bool timeout = (now() >= esc_deadline_);

        // ── Exit condition: meaningful progress toward goal ────────────────
        double current_dist = dist();
        if (current_dist < esc_start_dist_ - ESC_PROGRESS_THR) {
            RCLCPP_INFO(get_logger(),
                "[ESC] Progress made (%.2fm → %.2fm) → resuming goal-seek",
                esc_start_dist_, current_dist);
            minima_.reset();
            state_ = State::ALIGNING;
            return;
        }

        if (timeout) {
            RCLCPP_WARN(get_logger(),
                "[ESC] Timeout after %.0fs without progress → retrying ALIGNING",
                WALL_FOLLOW_SECS);
            minima_.reset();
            // Update escape start so the next stuck detection has a fresh baseline
            esc_start_dist_ = current_dist;
            state_ = State::ALIGNING;
            return;
        }

        geometry_msgs::msg::Twist c;

        // Sector indices:
        //   Front  : sector 0  (angle ≈ 0)
        //   Left   : sector NSEC*3/4 = 27  (angle ≈ +90°, robot-frame left)
        //   Right  : sector NSEC/4   =  9  (angle ≈ -90°, robot-frame right)
        // Note: with NSEC=36, each sector covers 10°.
        //   Left sectors: 23-31 (230°-310° from -π origin) ≈ +70° to +130°
        //   We sample the three sectors around ~90° (left) for robustness.
        const int LEFT_SECTOR  = NSEC * 3 / 4;   // ≈ sector 27 (+90°)
        const int FRONT_SECTOR = 0;               // ≈ sector 0  (0°)
        float leftDist  = lidar_.avgSectorDist(LEFT_SECTOR,  1);
        float frontDist = lidar_.frontDist();

        if (esc_mode_ == EscapeMode::ROTATE_TO_WALL) {
            // ── Phase 0: rotate right until a wall appears on the left ──────
            bool wallOnLeft  = leftDist  < WALL_FOLLOW_DIST * 1.8f;
            bool frontClose  = frontDist < STOP_D * 2.5f;

            if (wallOnLeft || frontClose) {
                esc_mode_ = EscapeMode::WALL_FOLLOWING;
                RCLCPP_INFO(get_logger(),
                    "[ESC] Wall acquired (left=%.2fm, front=%.2fm) → wall-following",
                    leftDist, frontDist);
            } else {
                // Rotate right (negative angular) to sweep left side toward walls
                c.angular.z = -ang_ * 0.55;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 800,
                    "[ESC/ROT] left=%.2fm front=%.2fm — rotating to find wall",
                    leftDist, frontDist);
            }

        } else {
            // ── Phase 1: left-hand wall following ────────────────────────────

            if (frontDist < STOP_D * 1.8f) {
                // Corner: front is blocked → turn right to follow wall around
                c.linear.x  = 0.0;
                c.angular.z = -ang_ * 0.75;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 400,
                    "[ESC/WF] Corner — turning right (front=%.2fm)", frontDist);

            } else if (leftDist > WALL_FOLLOW_DIST * 3.0f) {
                // Lost the wall entirely → turn left to re-acquire
                c.linear.x  = lin_ * 0.3;
                c.angular.z =  ang_ * 0.5;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 800,
                    "[ESC/WF] Lost wall (left=%.2fm) — steering left", leftDist);

            } else {
                // Proportional wall-distance controller
                //   Error > 0 → too far from wall → steer left  (positive z)
                //   Error < 0 → too close to wall → steer right (negative z)
                double wallErr = leftDist - WALL_FOLLOW_DIST;
                double steer   = std::clamp(1.4 * wallErr, -ang_, ang_);
                double spd     = lin_ * std::clamp(
                    (frontDist - STOP_D) / (SLOW_D - STOP_D), 0.25, 0.7);

                c.linear.x  = spd;
                c.angular.z = steer;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 800,
                    "[ESC/WF] left=%.2fm err=%.2f steer=%.1f° spd=%.2f  dist_goal=%.2fm",
                    leftDist, wallErr, steer*DEG, spd, current_dist);
            }
        }

        cmdp_->publish(c);
    }

    // ── Red sphere goal marker ────────────────────────────────────────────────
    void spawnMarker(double e, double n) {
        if (!spawn_->wait_for_service(std::chrono::seconds(2))) return;
        auto r = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        r->name = "goal_marker";
        r->xml  = "<sdf version='1.6'><model name='goal_marker'><static>true</static>"
                  "<link name='link'><visual name='v'><geometry>"
                  "<sphere><radius>0.4</radius></sphere></geometry>"
                  "<material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse>"
                  "</material></visual></link></model></sdf>";
        r->initial_pose.position.x = e;
        r->initial_pose.position.y = n;
        r->initial_pose.position.z = 0.4;
        using F = rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture;
        spawn_->async_send_request(r, [this](F future) {
            RCLCPP_INFO(get_logger(),
                future.get()->success ? "Goal marker spawned" : "Marker spawn failed"); });
    }
};

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TARSNavigator>());
    rclcpp::shutdown();
    return 0;
}
