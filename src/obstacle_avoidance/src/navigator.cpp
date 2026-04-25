/**
 * navigator.cpp — TARS AUTONOMOUS OBSTACLE Navigator (LiDAR + GPS + IMU + Odometry)
 * ──────────────────────────────────────────────────────────────────────────────
 * Sensors  : /tars/odom   (nav_msgs/Odometry)      — position & yaw (fallback)
 *            /scan        (sensor_msgs/LaserScan)   — 360° obstacle data
 *            /gps_plugin/out (sensor_msgs/NavSatFix)   — GPS position
 *            /imu/data    (sensor_msgs/Imu)         — IMU yaw
 *
 * Pose priority:
 *   Position  → GPS (gpsToENU) when fix available, else odom
 *   Yaw       → IMU orientation when available, else odom
 *
 * OA algo  : LiDAR-sector VFH-lite (unchanged)
 * States   : IDLE → ALIGNING → DRIVING → ESCAPING → WALL_FOLLOW → DONE  ← CHANGED
 *
 * Local-minima handling (NEW):
 *   - 1st stuck detection  → regular ESCAPING (widest-angle dash)
 *   - 2nd+ stuck detection → WALL_FOLLOW (Bug2-style boundary trace)
 *     The rover keeps the obstacle at a fixed lateral distance on the
 *     "goal side" and exits only when dist-to-goal improves by >0.5 m.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
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
static constexpr double REF_LAT = 12.9716;
static constexpr double REF_LON = 77.5946;

static constexpr double STOP_D  = 0.55;
static constexpr double SLOW_D  = 1.50;   // FIX: was 0.65 — widen slowdown window
static constexpr int    NSEC    = 36;

// ← CHANGED: added WALL_FOLLOW state
enum class State { IDLE, ALIGNING, DRIVING, ESCAPING, WALL_FOLLOW, DONE };

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
// OdomPose
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
// GpsPose
// ══════════════════════════════════════════════════════════════════════════════
class GpsPose {
    double e_=0, n_=0;
    bool ok_=false;
public:
    void update(const sensor_msgs::msg::NavSatFix& msg) {
        if (msg.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
            return;
        if (msg.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN
            && msg.position_covariance[0] > 25.0)
            return;
        auto enu = gpsToENU(msg.latitude, msg.longitude);
        e_ = enu.e; n_ = enu.n; ok_ = true;
    }
    bool   ok() const { return ok_; }
    double e()  const { return e_;  }
    double n()  const { return n_;  }
};

// ══════════════════════════════════════════════════════════════════════════════
// ImuPose
// ══════════════════════════════════════════════════════════════════════════════
class ImuPose {
    double yaw_=0;
    bool ok_=false;
public:
    void update(const sensor_msgs::msg::Imu& msg) {
        auto& q = msg.orientation;
        // covariance[0]==-1 means "unknown" in Gazebo — treat as valid
        if (msg.orientation_covariance[0] > 0.5) return;  // only reject if covariance is genuinely large
        yaw_ = q2yaw(q.x, q.y, q.z, q.w);
        ok_  = true;
    }
    bool   ok()  const { return ok_;  }
    double yaw() const { return yaw_; }
};

// ══════════════════════════════════════════════════════════════════════════════
// LidarVFH — 360° Vector Field Histogram built from /scan
// ══════════════════════════════════════════════════════════════════════════════
class LidarVFH {
    std::vector<float> sec_;
    bool ok_ = false;

    int angleToSector(float angle) const {
        float norm = angle + static_cast<float>(M_PI);
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

    // bestAngle: VFH steering — returns best relative angle (radians, robot frame)
    // blending goal direction (goal_rel) with obstacle clearance from sec_[].
    // Each sector is scored by its range PLUS a goal-alignment bonus so the rover
    // steers toward the goal whenever a clear path exists, and detours only when
    // forced by an obstacle.
    double bestAngle(double goal_rel) const
    {
        if (!ok_) return goal_rel;

        const double max_range  = 6.0;
        const double goal_bonus = 1.5;   // weight of goal alignment vs. clearance

        double best_score = -1e9;
        int    best_s     = 0;

        for (int s = 0; s < NSEC; ++s) {
            double sa          = sectorAngle(s);
            double range_score = std::min(static_cast<double>(sec_[s]), max_range);
            double goal_align  = std::cos(wrap(sa - goal_rel));   // +1 = toward goal
            double score       = range_score + goal_bonus * goal_align;
            if (score > best_score) { best_score = score; best_s = s; }
        }
        return sectorAngle(best_s);
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

    // ── NEW ───────────────────────────────────────────────────────────────────
    // sideDistance: minimum range in a ±30° cone at ±90° (left or right)
    //   side > 0 → left (+π/2),  side < 0 → right (−π/2)
    double sideDistance(int side) const {
        double target = (side > 0) ? M_PI/2 : -M_PI/2;
        float  minD   = 99.f;
        for (int s = 0; s < NSEC; ++s) {
            double sa = sectorAngle(s);
            if (std::abs(wrap(sa - target)) < M_PI/6)   // 30° half-width
                minD = std::min(minD, sec_[s]);
        }
        return static_cast<double>(minD);
    }

    bool isOk() const { return ok_; }
};

// ══════════════════════════════════════════════════════════════════════════════
// LocalMinimaDetector — 2 s sliding window (unchanged)
// ══════════════════════════════════════════════════════════════════════════════
class LocalMinimaDetector {
    std::deque<std::pair<double,double>> buf_;
    const int WIN = 40; const double THR = 0.25;
    double dist_ = 1e9;
public:
    void update(double e, double n, double d) {
        buf_.push_back({e,n});
        if ((int)buf_.size() > WIN) buf_.pop_front();
        dist_ = d;
    }
    bool stuck() const {
        if ((int)buf_.size() < WIN || dist_ < 1.5) return false;
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
class TARSNavigator : public rclcpp::Node {
    State               state_ = State::IDLE;
    OdomPose            pose_;
    GpsPose             gps_;
    ImuPose             imu_;
    LidarVFH            lidar_;
    LocalMinimaDetector minima_;
    std::mt19937        rng_{42};

    double lin_=0.4, ang_=0.6, gtol_=0.25, htol_=0.10;
    double ge_=0, gn_=0;
    double se_=0, sn_=0;
    int    esc_phase_=0;
    double esc_dir_=0;
    rclcpp::Time esc_deadline_;
    bool   marker_spawned_=false;

    // ── NEW: escape-attempt counter & wall-follow state ───────────────────────
    int    esc_count_     = 0;    // resets only when genuine progress is made
    double esc_entry_dist_= 1e9;  // dist-to-goal when the current escape started
    int    wf_side_       = 1;    // +1 keep wall on LEFT, -1 keep wall on RIGHT
    double wf_entry_dist_ = 1e9;  // dist-to-goal when wall-following started
    double wf_flip_dist_  = 1e9;  // dist at last side-flip (prevents flip thrashing)
    rclcpp::Time wf_deadline_;    // hard timeout per wall-follow leg
    // ── Wall-follow tuning ────────────────────────────────────────────────────
    static constexpr double WF_WALL_TARGET  = 0.60;  // desired lateral wall gap (m)
    static constexpr double WF_WALL_KP      = 1.20;  // P-gain for lateral error
    static constexpr double WF_PROGRESS_THR = 0.50;  // dist improvement to exit (m)
    static constexpr double WF_LEG_TIMEOUT  = 25.0;  // seconds per leg before flip

    // ── FIX: GPS↔odom frame reconciliation ───────────────────────────────────
    // Gazebo GPS-ENU and odom share the same axis orientation but may differ in
    // origin.  We record the offset once (at first GPS fix) so that goals given
    // in GPS-ENU can be expressed in the odom frame, keeping position, yaw and
    // goal all in one consistent coordinate system.
    double gps_odom_offset_e_ = 0.0;   // gps_e - odom_e  at first fix
    double gps_odom_offset_n_ = 0.0;   // gps_n - odom_n  at first fix
    bool   gps_origin_set_    = false;

    // ── FIX: steering deadband — prevents perpetual-circle orbiting ───────────
    // With lin=0.4 m/s, a steer of 0.05 rad/s gives radius ≈8 m.  Below this
    // threshold the correction is smaller than sensor noise; go straight.
    static constexpr double STEER_DEAD = 0.12;  // ≈4.6° — zero steer below this

    // ── FIX: ALIGNING hysteresis — require N consecutive stable ticks ─────────
    int align_stable_ticks_ = 0;
    static constexpr int ALIGN_STABLE_REQ = 10;  // 5 × 50 ms = 250 ms stable

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          cmdp_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr             distp_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr             ctep_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         odoms_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr     lidars_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr       goals_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr     gpss_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr           imus_;
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr         spawn_;
    rclcpp::TimerBase::SharedPtr                                     tim_;

public:
    TARSNavigator() : Node("tars_navigator") {
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

        gpss_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps_plugin/out", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::NavSatFix::SharedPtr m){
                bool was_ok = gps_.ok();
                gps_.update(*m);
                if (!was_ok && gps_.ok()) {
                    RCLCPP_INFO(get_logger(), "GPS fix acquired  E=%.2f N=%.2f",
                        gps_.e(), gps_.n());
                    // FIX: Record offset so GPS goals can be converted to odom frame.
                    // Only do this once (at first fix); odom is (0,0) at startup so
                    // offset = gps_enu - odom = gps_enu - (0,0) initially, but we
                    // use the live odom value in case there was motion before first fix.
                    if (!gps_origin_set_ && pose_.ok()) {
                        gps_odom_offset_e_ = gps_.e() - pose_.e();
                        gps_odom_offset_n_ = gps_.n() - pose_.n();
                        gps_origin_set_    = true;
                        RCLCPP_INFO(get_logger(),
                            "GPS-odom offset calibrated: dE=%.3f dN=%.3f",
                            gps_odom_offset_e_, gps_odom_offset_n_);
                    }
                }
            });

        imus_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Imu::SharedPtr m){
                bool was_ok = imu_.ok();
                imu_.update(*m);
                if (!was_ok && imu_.ok())
                    RCLCPP_INFO(get_logger(), "IMU online  yaw=%.2f°",
                        imu_.yaw() * DEG);
            });

        goals_ = create_subscription<geometry_msgs::msg::Point>("/goal_gps", 10,
            [this](geometry_msgs::msg::Point::SharedPtr m){ goalCb(m); });

        spawn_ = create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
        tim_   = create_wall_timer(std::chrono::milliseconds(50), [this]{ loop(); });

        RCLCPP_INFO(get_logger(),
            "TARS Navigator ready | odom+GPS position | odom+IMU yaw | LiDAR VFH | Wall-Follow escape");
    }

private:
    // ── Pose accessors ────────────────────────────────────────────────────────
    // FIX: Always use odom for position and yaw so that position, yaw, and the
    // converted goal all live in the same consistent frame.  GPS-ENU shares axis
    // orientation with the Gazebo world/odom frame but may differ in origin, and
    // the Gazebo IMU world frame can have a small rotation offset vs GPS-ENU.
    // Mixing them produced a persistent ~2-4° heading error that caused the robot
    // to orbit in circles and drift away from the goal.
    double posE()   const { return pose_.e();   }
    double posN()   const { return pose_.n();   }
    double posYaw() const { return pose_.yaw(); }

    double dist()    const { return std::hypot(ge_ - posE(), gn_ - posN()); }
    double brobot()  const {
        return wrap(std::atan2(gn_ - posN(), ge_ - posE()) - posYaw()); }

    double crossTrackError() const {
        double dx = ge_ - se_, dy = gn_ - sn_;
        double pathLen = std::hypot(dx, dy);
        if (pathLen < 1e-6) return 0.0;
        double rx = posE() - se_, ry = posN() - sn_;
        return (dx * ry - dy * rx) / pathLen;
    }

    void stop()          { cmdp_->publish(geometry_msgs::msg::Twist()); }
    void pubD(double v)  { std_msgs::msg::Float64 m; m.data=v; distp_->publish(m); }
    void pubCTE(double v){ std_msgs::msg::Float64 m; m.data=v; ctep_->publish(m); }

    void goalCb(const geometry_msgs::msg::Point::SharedPtr m) {
        if (state_==State::ALIGNING || state_==State::DRIVING ||
            state_==State::ESCAPING || state_==State::WALL_FOLLOW) {
            RCLCPP_WARN(get_logger(), "Navigating — goal ignored"); return; }

        // FIX: Convert GPS goal → odom frame.
        // The goal topic publishes GPS lat/lon.  gpsToENU gives absolute ENU.
        // We subtract the GPS-odom offset so ge_/gn_ are in the odom frame and
        // consistent with posE()/posN() (which now always return odom).
        auto g = gpsToENU(m->x, m->y);
        if (gps_origin_set_) {
            ge_ = g.e - gps_odom_offset_e_;
            gn_ = g.n - gps_odom_offset_n_;
        } else {
            // GPS origin not yet calibrated; use raw ENU and warn.
            // This will be wrong if GPS and odom origins differ.
            ge_ = g.e; gn_ = g.n;
            RCLCPP_WARN(get_logger(),
                "GPS origin not yet calibrated — goal may be in wrong frame!");
        }
        se_ = posE(); sn_ = posN();
        marker_spawned_ = false;
        align_stable_ticks_ = 0;   // FIX: reset alignment hysteresis counter
        esc_count_ = 0;
        minima_.reset(); state_ = State::ALIGNING;
        RCLCPP_INFO(get_logger(),
            "Goal: lat=%.6f lon=%.6f  odom_goal=[%.2f, %.2f]  dist=%.2fm",
            m->x, m->y, ge_, gn_, dist());
        if (!marker_spawned_) {
            // Spawn marker at the odom-frame goal position
            spawnMarker(ge_, gn_); marker_spawned_ = true; }
    }

    void loop() {
        if (!pose_.ok()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "Waiting for /tars/odom..."); return; }
        if (!lidar_.isOk()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "Waiting for /scan..."); return; }

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
            "[TARS] pos_src=%s  yaw_src=%s  E=%.2f N=%.2f  yaw=%.1f°",
            gps_.ok() ? "GPS" : "odom",
            imu_.ok() ? "IMU" : "odom",
            posE(), posN(), posYaw()*DEG);

        double d_now = dist();
        pubD(d_now);
        pubCTE(crossTrackError());
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[CTE] cross_track=%.3fm  dist=%.2fm", crossTrackError(), d_now);
        minima_.update(posE(), posN(), d_now);

        switch (state_) {
            case State::IDLE:
                RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,
                    "[IDLE] waiting for goal..."); break;
            case State::ALIGNING:   doAlign();      break;
            case State::DRIVING:    doDrive();      break;
            case State::ESCAPING:   doEscape();     break;
            case State::WALL_FOLLOW:doWallFollow(); break;   // ← NEW
            case State::DONE:       stop();         break;
        }
    }

    // ── ALIGNING ──────────────────────────────────────────────────────────────
    void doAlign() {
        double err = brobot();
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),500,
            "[ALIGN] err=%.1f°  stable=%d/%d", err*DEG, align_stable_ticks_, ALIGN_STABLE_REQ);

        if (std::abs(err) < htol_) {
            align_stable_ticks_++;
            // FIX: require ALIGN_STABLE_REQ consecutive stable ticks before driving
            // to prevent entering DRIVING while still oscillating around the target.
            if (align_stable_ticks_ >= ALIGN_STABLE_REQ) {
                stop();
                minima_.reset();   // FIX: buffer was full of stationary alignment samples
                state_ = State::DRIVING;
                align_stable_ticks_ = 0;
                RCLCPP_INFO(get_logger(), "-> DRIVING"); }
            else { stop(); }   // hold still while accumulating stable count
            return;
        }
        // Not aligned yet — reset the counter and steer
        align_stable_ticks_ = 0;
        geometry_msgs::msg::Twist c;
        c.angular.z = std::copysign(ang_ * std::clamp(std::abs(err)/0.5, 0.2, 1.0), err);
        cmdp_->publish(c);
    }

    // ── DRIVING ───────────────────────────────────────────────────────────────
    void doDrive() {
        double d    = dist();
        double herr = brobot();

        if (d < gtol_) {
            stop();
            state_ = State::DONE;
            return;
        }

        // ── Stuck / local-minima check ────────────────────────────────────────
        if (minima_.stuck()) {
            stop();
            RCLCPP_WARN(get_logger(), "[DRIVE] stuck detected -> ESCAPING");
            startEscape();
            return;
        }

        // ── Front obstacle check ──────────────────────────────────────────────
        // Trigger before an actual collision, not at the same instant.
        if (lidar_.frontBlocked(STOP_D * 1.5)) {
            stop();
            RCLCPP_WARN(get_logger(), "[DRIVE] front blocked (%.2fm) -> ESCAPING",
                static_cast<double>(lidar_.frontDist()));
            startEscape();
            return;
        }

        // ── Re-align if heading drifted too far ───────────────────────────────
        if (std::abs(herr) > 0.9) {
            stop();
            align_stable_ticks_ = 0;
            state_ = State::ALIGNING;
            RCLCPP_INFO(get_logger(),
                "[DRIVE] heading drift %.1f deg -> ALIGNING", herr * DEG);
            return;
        }

        geometry_msgs::msg::Twist c;

        // ── VFH steering ──────────────────────────────────────────────────────
        // bestAngle and herr are both in robot frame, so directly comparable.
	double steer = (std::abs(herr) < STEER_DEAD) ? 0.0
             		: std::clamp(herr * 1.5, -ang_, ang_);

        // ── Speed scaling ─────────────────────────────────────────────────────
        double front      = lidar_.frontDist();
        double speed_obs  = lin_ * std::clamp(
            (front - STOP_D) / (SLOW_D - STOP_D), 0.15, 1.0);
        double speed_head = lin_ * std::clamp(
            1.0 - std::abs(herr) / (M_PI / 2.0), 0.2, 1.0);
        c.linear.x  = std::min(speed_obs, speed_head);
        c.angular.z = steer;

        cmdp_->publish(c);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 800,
		"[DRIVE] d=%.2fm herr=%.1f steer=%.2f spd=%.2f front=%.2fm",
		d, herr * DEG, steer, c.linear.x, front);
    }

    // ── ESCAPING ──────────────────────────────────────────────────────────────
    // 3-phase escape: back up → rotate to widest open sector → drive forward.
    // Backing up first is critical for sphere clusters — it physically clears
    // the immediate blockage so that the subsequent rotation and drive find
    // genuinely open space, rather than the same blocked direction.
    void startEscape() {
        esc_entry_dist_ = dist();   // ← FIX: snapshot dist so phase-2 can judge progress
        esc_count_++;

        if (esc_count_ >= 3) {
            RCLCPP_WARN(get_logger(),
                "Escape #%d failed — escalating to WALL_FOLLOW", esc_count_);
            startWallFollow();
            return;
        }

        esc_phase_    = 0;   // phase 0 = backup
        esc_deadline_ = now() + rclcpp::Duration::from_seconds(1.5);
        minima_.reset();
        state_ = State::ESCAPING;
        RCLCPP_INFO(get_logger(), "[ESC #%d] phase0=BACKUP", esc_count_);
    }

    void doEscape() {
        bool deadline_passed = (now() >= esc_deadline_);
        geometry_msgs::msg::Twist c;

        if (esc_phase_ == 0) {
            // ── Early-exit: if the widest gap is clearly to the side (>20°) and
            //    the front is still close, there is no need to reverse first —
            //    just pivot directly toward the open gap.  This prevents the
            //    backup→rotate→blocked loop seen when a corridor exists beside us.
            if (lidar_.isOk()) {
                double gap     = lidar_.widestAngle();   // relative, radians
                double front_d = static_cast<double>(lidar_.frontDist());
                if (std::abs(gap) > 20.0 * RAD && front_d < 0.7) {
                    esc_dir_      = wrap(posYaw() + gap);
                    esc_phase_    = 1;
                    esc_deadline_ = now() + rclcpp::Duration::from_seconds(2.5);
                    RCLCPP_INFO(get_logger(),
                        "[ESC] side gap %.1f° — skipping backup -> ROTATE to %.1f°",
                        gap * DEG, esc_dir_ * DEG);
                    return;
                }
            }

            // ── Phase 0: back up (ignore front — we're going backward) ────────
            c.linear.x = -lin_ * 0.5;
            cmdp_->publish(c);
            if (deadline_passed) {
                esc_dir_      = lidar_.isOk()
                                ? wrap(posYaw() + lidar_.widestAngle())
                                : posYaw();
                esc_phase_    = 1;
                esc_deadline_ = now() + rclcpp::Duration::from_seconds(2.5);
                RCLCPP_INFO(get_logger(),
                    "[ESC] backed up -> phase1=ROTATE to %.1f°", esc_dir_ * DEG);
            }

        } else if (esc_phase_ == 1) {
            // ── Phase 1: rotate toward widest open sector ─────────────────────
            // Re-query every cycle so we always aim at the currently-open gap,
            // not a direction that was computed before rotation started.
            if (lidar_.isOk())
                esc_dir_ = wrap(posYaw() + lidar_.widestAngle());

            double yerr = wrap(esc_dir_ - posYaw());
            c.angular.z = std::copysign(ang_ * 0.7, yerr);
            cmdp_->publish(c);
            if (std::abs(yerr) < 0.15 || deadline_passed) {
                esc_phase_    = 2;
                esc_deadline_ = now() + rclcpp::Duration::from_seconds(4.0);
                RCLCPP_INFO(get_logger(), "[ESC] aligned -> phase2=DRIVE");
            }

        } else {
            // ── Phase 2: drive forward ─────────────────────────────────────────
            if (!lidar_.frontBlocked(STOP_D))
                c.linear.x = lin_ * 0.6;
            else
                c.angular.z = ang_ * 0.5;   // nudge if still blocked
            cmdp_->publish(c);
            if (deadline_passed) {
                // ── FIX: only reset esc_count_ when we made genuine progress.
                // Previously it reset unconditionally, so esc_count_ never
                // reached 3 and WALL_FOLLOW was never triggered even when the
                // robot kept hitting the exact same wall on every attempt.
                if (dist() < esc_entry_dist_ - 1.0) {
                    esc_count_ = 0;   // genuinely past this obstacle — fresh budget
                    RCLCPP_INFO(get_logger(),
                        "[ESC] done + progress (%.2fm) -> ALIGNING  esc_count reset",
                        esc_entry_dist_ - dist());
                } else {
                    RCLCPP_WARN(get_logger(),
                        "[ESC] done (no progress, esc_count=%d) -> ALIGNING",
                        esc_count_);
                }
                minima_.reset();
                state_ = State::ALIGNING;
            }
        }
    }

    // ══════════════════════════════════════════════════════════════════════════
    // WALL_FOLLOW — Bug2-inspired boundary tracing             ← NEW SECTION
    // ══════════════════════════════════════════════════════════════════════════
    /**
     * Side selection rationale
     * ────────────────────────
     * brobot() is the bearing to goal in the robot's own frame:
     *   > 0  → goal is to the LEFT   → put wall on LEFT  (wf_side_=+1)
     *          so the rover circles counter-clockwise around the obstacle
     *          and naturally sweeps toward the goal's side.
     *   < 0  → goal is to the RIGHT  → put wall on RIGHT (wf_side_=-1)
     *
     * Why this helps in a U-shape
     * ────────────────────────────
     * The rover enters the U mouth-first because the goal is inside.
     * After being trapped, the goal bearing (brobot) will be roughly
     * straight ahead (±small). We break the tie by defaulting to +1.
     * The rover then traces one arm of the U outward, clears the rim,
     * and dist() finally starts to shrink → WF_PROGRESS_THR exit fires.
     *
     * If the chosen arm leads deeper (rare), the WF_LEG_TIMEOUT fires,
     * the side flips, and the rover tries the other arm.
     */
    void startWallFollow() {
        double bear  = brobot();
        wf_side_     = (bear >= 0.0) ? +1 : -1;
        wf_entry_dist_ = dist();
        wf_flip_dist_  = wf_entry_dist_;
        wf_deadline_   = now() + rclcpp::Duration::from_seconds(WF_LEG_TIMEOUT);
        minima_.reset();
        state_ = State::WALL_FOLLOW;
        RCLCPP_WARN(get_logger(),
            "[WF] start  entry_dist=%.2fm  wall_side=%s  bear=%.1f°",
            wf_entry_dist_,
            wf_side_ > 0 ? "LEFT" : "RIGHT",
            bear * DEG);
    }

    void doWallFollow() {
        double d_now = dist();

        // ── Exit: genuine progress toward goal ───────────────────────────────
        if (d_now < wf_entry_dist_ - WF_PROGRESS_THR) {
            RCLCPP_INFO(get_logger(),
                "[WF] progress %.2fm → resuming navigation  esc_count_=%d",
                wf_entry_dist_ - d_now, esc_count_);
            esc_count_ = 0;       // successfully escaped — reset counter
            minima_.reset();
            state_ = State::ALIGNING;
            return;
        }

        // ── Exit: goal reached while wall-following (shouldn't happen often) ─
        if (d_now < gtol_) {
            stop(); state_ = State::DONE;
            RCLCPP_INFO(get_logger(), "[WF] GOAL REACHED mid-wall-follow"); return;
        }

        // ── Timeout: flip side and try the other arm ──────────────────────────
        if (now() >= wf_deadline_) {
            wf_side_       = -wf_side_;
            wf_flip_dist_  = d_now;
            wf_deadline_   = now() + rclcpp::Duration::from_seconds(WF_LEG_TIMEOUT);
            minima_.reset();
            RCLCPP_WARN(get_logger(),
                "[WF] timeout — flipping to %s wall", wf_side_>0 ? "LEFT":"RIGHT");
            return;
        }

        // ── Control law ───────────────────────────────────────────────────────
        // The rover tries to keep an obstacle at WF_WALL_TARGET distance on
        // wf_side_ while moving forward. If the front is blocked it turns
        // away from the wall (into the open corridor).
        //
        //   lateral_error = side_distance − WF_WALL_TARGET
        //     +ve → wall too far → steer toward wall  (+wf_side_ direction)
        //     −ve → wall too close → steer away        (−wf_side_ direction)
        //
        double side_dist    = lidar_.sideDistance(wf_side_);
        double lateral_err  = side_dist - WF_WALL_TARGET;
        double steer_wall   = std::clamp(wf_side_ * WF_WALL_KP * lateral_err,
                                         -ang_, ang_);

        geometry_msgs::msg::Twist c;

        if (lidar_.frontBlocked(STOP_D)) {
            // Front blocked → pivot away from wall (into the open)
            c.angular.z = std::clamp(-wf_side_ * ang_ * 0.8, -ang_, ang_);
            RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),500,
                "[WF] front blocked — pivoting away from %s wall",
                wf_side_>0?"LEFT":"RIGHT");
        } else {
            // Normal wall-following: creep forward + lateral correction
            double front = lidar_.frontDist();
            double spd = lin_ * std::clamp(
                (front - STOP_D) / (SLOW_D - STOP_D), 0.25, 0.65);
            c.linear.x  = spd;
            c.angular.z = steer_wall;
        }

        cmdp_->publish(c);

        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),800,
            "[WF] d=%.2fm entry=%.2fm side=%s side_dist=%.2fm lat_err=%.2f",
            d_now, wf_entry_dist_,
            wf_side_>0?"L":"R",
            side_dist, lateral_err);
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
                future.get()->success ? "Marker spawned" : "Marker failed"); });
    }
};

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TARSNavigator>());
    rclcpp::shutdown();
    return 0;
}
