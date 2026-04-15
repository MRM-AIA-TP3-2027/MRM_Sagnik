/**
 * navigator.cpp — TARS AUTONOMOUS OBSTACLE Navigator (LiDAR + GPS + IMU + Odometry)
 * ──────────────────────────────────────────────────────────────────────────────
 * Sensors  : /tars/odom   (nav_msgs/Odometry)      — position & yaw (fallback)
 *            /scan        (sensor_msgs/LaserScan)   — 360° obstacle data
 *            /gps/fix     (sensor_msgs/NavSatFix)   — GPS position 
 *            /imu/data    (sensor_msgs/Imu)         — IMU yaw       
 *
 * Pose priority:
 *   Position  → GPS (gpsToENU) when fix available, else odom
 *   Yaw       → IMU orientation when available, else odom
 *
 * OA algo  : LiDAR-sector VFH-lite (unchanged)
 * States   : IDLE → ALIGNING → DRIVING → ESCAPING → DONE
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>   // ← GPS
#include <sensor_msgs/msg/imu.hpp>           // ← IMU
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
static constexpr double SLOW_D  = 0.65;
static constexpr int    NSEC    = 36;

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
    // Normalise first so un-normalised quaternions don't skew the result
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
// GpsPose — reads /gps/fix, converts lat/lon → ENU  ← NEW
// ══════════════════════════════════════════════════════════════════════════════
class GpsPose {
    double e_=0, n_=0;
    bool ok_=false;
public:
    void update(const sensor_msgs::msg::NavSatFix& msg) {
        // Only trust fix if status >= STATUS_FIX (0)
        if (msg.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
            return;
        // Reject if horizontal accuracy (cov[0]) is unknown or worse than 25 m²
        if (msg.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN
            && msg.position_covariance[0] > 25.0)
            return;
        auto enu = gpsToENU(msg.latitude, msg.longitude);
        e_ = enu.e;
        n_ = enu.n;
        ok_ = true;
    }
    bool   ok() const { return ok_; }
    double e()  const { return e_;  }
    double n()  const { return n_;  }
};

// ══════════════════════════════════════════════════════════════════════════════
// ImuPose — reads /imu/data for orientation (yaw)  ← NEW
// ══════════════════════════════════════════════════════════════════════════════
class ImuPose {
    double yaw_=0;
    bool ok_=false;
public:
    void update(const sensor_msgs::msg::Imu& msg) {
        auto& q = msg.orientation;
        // Only use if covariance[0] != -1 (unknown) 
        if (msg.orientation_covariance[0] < -0.9) return;
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

    double bestAngle(double goalAngle) const {
        if (!ok_) return goalAngle;
        int best = -1; double bs = -1e9;
        for (int s = 0; s < NSEC; ++s) {
            double sa = sectorAngle(s);
            if (std::abs(wrap(sa)) > M_PI/2) continue;
            if (sec_[s] < STOP_D)            continue;
            if (std::abs(wrap(sa - goalAngle)) > M_PI/4) continue;
            double score = sec_[s] - 2.5 * std::abs(wrap(sa - goalAngle));
            if (score > bs) { bs = score; best = s; }
        }
        if (best < 0) {
            // No clear sector found — falling back to raw goal bearing
            RCLCPP_WARN_ONCE(rclcpp::get_logger("LidarVFH"),
                "bestAngle: no clear sector; falling back to goal bearing");
            return goalAngle;
        }
        return sectorAngle(best);
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
    OdomPose            pose_;           // odometry (always subscribed, fallback)
    GpsPose             gps_;            // GPS position  ← NEW
    ImuPose             imu_;            // IMU yaw       ← NEW
    LidarVFH            lidar_;
    LocalMinimaDetector minima_;
    std::mt19937        rng_{42};

    double lin_=0.4, ang_=0.6, gtol_=0.25, htol_=0.10;
    double ge_=0, gn_=0;
    double se_=0, sn_=0;          // start position for CTE computation
    int    esc_phase_=0;
    double esc_dir_=0;
    rclcpp::Time esc_deadline_;   // wall-clock deadline instead of tick counter
    bool   marker_spawned_=false; // guard against duplicate Gazebo model name

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          cmdp_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr             distp_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr             ctep_;  // cross-track error
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         odoms_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr     lidars_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr       goals_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr     gpss_;   // ← NEW
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr           imus_;   // ← NEW
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

        // ── Odometry (always subscribed as fallback) ──────────────────────────
        odoms_ = create_subscription<nav_msgs::msg::Odometry>(
            "/tars/odom", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr m){ pose_.update(*m); });

        // ── LiDAR ─────────────────────────────────────────────────────────────
        lidars_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr m){ lidar_.update(*m); });

        // ── GPS /gps/fix → GpsPose ← NEW ─────────────────────────────────────
        gpss_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::NavSatFix::SharedPtr m){
                bool was_ok = gps_.ok();
                gps_.update(*m);
                if (!was_ok && gps_.ok())
                    RCLCPP_INFO(get_logger(), "GPS fix acquired  E=%.2f N=%.2f",
                        gps_.e(), gps_.n());
            });

        // ── IMU /imu/data → ImuPose ← NEW ────────────────────────────────────
        imus_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Imu::SharedPtr m){
                bool was_ok = imu_.ok();
                imu_.update(*m);
                if (!was_ok && imu_.ok())
                    RCLCPP_INFO(get_logger(), "IMU online  yaw=%.2f°",
                        imu_.yaw() * DEG);
            });

        // ── Goal ──────────────────────────────────────────────────────────────
        goals_ = create_subscription<geometry_msgs::msg::Point>("/goal_gps", 10,
            [this](geometry_msgs::msg::Point::SharedPtr m){ goalCb(m); });

        spawn_ = create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
        tim_   = create_wall_timer(std::chrono::milliseconds(50), [this]{ loop(); });

        RCLCPP_INFO(get_logger(),
            "TARS Navigator ready | odom+GPS position | odom+IMU yaw | LiDAR VFH");
    }

private:
    // ── Fused pose accessors ──────────────────────────────────────────────────
    // Position: prefer GPS (real-world coordinates); fall back to odom
    double posE()  const { return gps_.ok()  ? gps_.e()  : pose_.e(); }
    double posN()  const { return gps_.ok()  ? gps_.n()  : pose_.n(); }
    // Yaw: prefer IMU; fall back to odom
    double posYaw() const { return imu_.ok() ? imu_.yaw() : pose_.yaw(); }

    double dist()   const { return std::hypot(ge_ - posE(), gn_ - posN()); }
    double brobot() const {
        return wrap(std::atan2(gn_ - posN(), ge_ - posE()) - posYaw()); }

    // Cross-Track Error: signed perpendicular deviation from the start→goal line
    double crossTrackError() const {
        double dx = ge_ - se_, dy = gn_ - sn_;
        double pathLen = std::hypot(dx, dy);
        if (pathLen < 1e-6) return 0.0;          // start ≈ goal, no line defined
        // 2-D cross product (path × robot_offset) / |path|
        double rx = posE() - se_, ry = posN() - sn_;
        return (dx * ry - dy * rx) / pathLen;     // +left / -right of path
    }

    void stop()          { cmdp_->publish(geometry_msgs::msg::Twist()); }
    void pubD(double v)  { std_msgs::msg::Float64 m; m.data=v; distp_->publish(m); }
    void pubCTE(double v){ std_msgs::msg::Float64 m; m.data=v; ctep_->publish(m); }

    void goalCb(const geometry_msgs::msg::Point::SharedPtr m) {
        if (state_==State::ALIGNING || state_==State::DRIVING ||
            state_==State::ESCAPING) {
            RCLCPP_WARN(get_logger(), "Navigating — goal ignored"); return; }
        auto g = gpsToENU(m->x, m->y); ge_ = g.e; gn_ = g.n;
        se_ = posE(); sn_ = posN();  // record start for CTE
        marker_spawned_ = false;
        minima_.reset(); state_ = State::ALIGNING;
        RCLCPP_INFO(get_logger(),
            "Goal: lat=%.6f lon=%.6f  dist=%.2fm  src=[pos:%s yaw:%s]",
            m->x, m->y, dist(),
            gps_.ok() ? "GPS" : "odom",
            imu_.ok() ? "IMU" : "odom");
        if (!marker_spawned_) { spawnMarker(ge_, gn_); marker_spawned_ = true; }
    }

    void loop() {
        if (!pose_.ok()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "Waiting for /tars/odom..."); return; }
        if (!lidar_.isOk()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "Waiting for /scan..."); return; }

        // Log sensor source once per second so you can verify GPS/IMU are live
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
            stop(); state_ = State::DRIVING;
            RCLCPP_INFO(get_logger(), "-> DRIVING"); return; }
        geometry_msgs::msg::Twist c;
        c.angular.z = std::copysign(ang_ * std::clamp(std::abs(err)/0.5, 0.2, 1.0), err);
        cmdp_->publish(c);
    }

    // ── DRIVING ───────────────────────────────────────────────────────────────
    void doDrive() {
        double d    = dist();  // already published above
        double herr = brobot();

        if (d < gtol_) {
            stop(); state_ = State::DONE;
            RCLCPP_INFO(get_logger(), "GOAL REACHED  residual=%.3fm", d); return; }

        if (minima_.stuck()) {
            RCLCPP_WARN(get_logger(), "LOCAL MINIMA — escaping");
            startEscape(); return; }

        double front = lidar_.frontDist();

        // Emergency backup
        if (lidar_.frontBlocked(STOP_D)) {
            geometry_msgs::msg::Twist c;
            c.linear.x  = -0.08;
            c.angular.z = std::clamp(1.0 * lidar_.bestAngle(herr), -ang_, ang_);
            cmdp_->publish(c);
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),400,
                "EMG backup — front=%.2fm", front);
            return; }

        // Steering
        double steer;
        if (front > SLOW_D && std::abs(herr) < 0.15) {
            steer = std::clamp(1.5 * herr, -ang_, ang_);
        } else {
            steer = std::clamp(0.8 * lidar_.bestAngle(herr), -ang_, ang_);
        }

        double goal_scale = std::clamp(d / 1.0, 0.1, 1.0);

        double spd = lin_ * std::clamp(
            std::min({ (front-STOP_D)/(SLOW_D-STOP_D), goal_scale}),
            0.1, 1.0);

        geometry_msgs::msg::Twist c;
        c.linear.x  = spd;
        c.angular.z = steer;
        cmdp_->publish(c);

        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
            "[DRIVE] d=%.2fm  hdg=%.1f°  steer=%.1f°  front=%.2fm  spd=%.2f",
            d, herr*DEG, steer*DEG, front, spd);
    }

    // ── ESCAPING ──────────────────────────────────────────────────────────────
    void startEscape() {
        double rel = lidar_.isOk() ? lidar_.widestAngle()
                   : std::uniform_real_distribution<double>(-M_PI, M_PI)(rng_);
        esc_dir_   = wrap(posYaw() + rel);
        esc_phase_ = 0;
        esc_deadline_ = now() + rclcpp::Duration::from_seconds(2.5);
        minima_.reset(); state_ = State::ESCAPING;
        RCLCPP_INFO(get_logger(), "[ESC] target=%.1f°", esc_dir_*DEG);
    }

    void doEscape() {
        bool deadline_passed = (now() >= esc_deadline_);
        double yerr = wrap(esc_dir_ - posYaw());
        geometry_msgs::msg::Twist c;
        if (esc_phase_ == 0) {
            c.angular.z = std::copysign(ang_ * 0.7, yerr);
            if (std::abs(yerr) < 0.15 || deadline_passed) {
                esc_phase_ = 1;
                esc_deadline_ = now() + rclcpp::Duration::from_seconds(4.0);
                RCLCPP_INFO(get_logger(), "[ESC] -> drive phase"); }
        } else {
            if (!lidar_.frontBlocked(STOP_D)) c.linear.x = lin_ * 0.6;
            else                              c.angular.z = ang_ * 0.7;
            if (deadline_passed) {
                minima_.reset(); state_ = State::ALIGNING;
                RCLCPP_INFO(get_logger(), "[ESC] done -> ALIGNING"); }
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