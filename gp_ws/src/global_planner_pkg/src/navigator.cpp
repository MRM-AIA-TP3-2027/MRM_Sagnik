/**
 * navigator.cpp — TARS Autonomous GPS Navigator (ROS2)
 *
 * POSITION SOURCE: /tars/odom  (nav_msgs/Odometry)
 *   The diff_drive plugin publishes ground-truth world XY here.
 *   We no longer rely on libgazebo_ros_gps_sensor (broken in Gazebo Classic 11).
 *
 * State Machine:  IDLE → ROTATING → DRIVING → DONE
 *
 * Topics in  : /tars/odom     (nav_msgs/Odometry)   position + heading
 *              /goal_gps      (geometry_msgs/Point)  x=lat, y=lon
 *
 * Topics out : /tars/cmd_vel            (geometry_msgs/Twist)
 *              /tars/cross_track_error  (std_msgs/Float64)
 *              /tars/heading_error      (std_msgs/Float64)
 *              /tars/distance_remaining (std_msgs/Float64)
 *
 * Gazebo     : Spawns a RED sphere at goal position in the world
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>

#include <cmath>
#include <string>
#include <sstream>

// ─── Constants ────────────────────────────────────────────────────────────────
static constexpr double EARTH_R = 6371000.0;
static constexpr double RAD     = M_PI / 180.0;
static constexpr double DEG     = 180.0 / M_PI;

// GPS reference — must match TARS.xacro referenceLatitude/referenceLongitude.
// Gazebo world origin (0,0) corresponds to this GPS coordinate.
static constexpr double REF_LAT = 12.9716;
static constexpr double REF_LON = 77.5946;

// ─── State machine ────────────────────────────────────────────────────────────
enum class State { IDLE, ROTATING, DRIVING, DONE };

// ─── GPS → ENU metres from world origin ──────────────────────────────────────
struct ENU { double east = 0, north = 0; };

ENU gpsToENU(double lat_deg, double lon_deg)
{
    double dlat    = (lat_deg - REF_LAT) * RAD;
    double dlon    = (lon_deg - REF_LON) * RAD;
    double mid_lat = ((lat_deg + REF_LAT) / 2.0) * RAD;
    return { EARTH_R * std::cos(mid_lat) * dlon,   // east
             EARTH_R * dlat };                       // north
}

inline double wrapAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

double quatToYaw(double qx, double qy, double qz, double qw) {
    return std::atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz));
}

// ─── Red sphere SDF for goal marker ──────────────────────────────────────────
std::string makeMarkerSDF()
{
    return
        "<sdf version='1.6'>"
          "<model name='goal_marker'>"
            "<static>true</static>"
            "<link name='link'>"
              "<visual name='vis'>"
                "<geometry><sphere><radius>0.4</radius></sphere></geometry>"
                "<material>"
                  "<ambient>1 0 0 1</ambient>"
                  "<diffuse>1 0 0 1</diffuse>"
                "</material>"
              "</visual>"
            "</link>"
          "</model>"
        "</sdf>";
}

// ══════════════════════════════════════════════════════════════════════════════
class TARSNavigator : public rclcpp::Node
{
public:
    TARSNavigator() : Node("tars_navigator"), state_(State::IDLE)
    {
        declare_parameter("linear_speed",      0.4);
        declare_parameter("angular_speed",     0.6);
        declare_parameter("goal_tolerance",    0.8);
        declare_parameter("heading_tolerance", 0.10);

        lin_spd_  = get_parameter("linear_speed").as_double();
        ang_spd_  = get_parameter("angular_speed").as_double();
        goal_tol_ = get_parameter("goal_tolerance").as_double();
        hdg_tol_  = get_parameter("heading_tolerance").as_double();

        // Publishers
        cmd_pub_  = create_publisher<geometry_msgs::msg::Twist>("/tars/cmd_vel", 10);
        dist_pub_ = create_publisher<std_msgs::msg::Float64>("/tars/distance_remaining", 10);
        hdg_pub_  = create_publisher<std_msgs::msg::Float64>("/tars/heading_error", 10);
        cte_pub_  = create_publisher<std_msgs::msg::Float64>("/tars/cross_track_error", 10);

        // Odometry subscriber — diff_drive publishes world XY + orientation here
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/tars/odom", 10,
            std::bind(&TARSNavigator::odomCb, this, std::placeholders::_1));

        // Goal subscriber
        goal_sub_ = create_subscription<geometry_msgs::msg::Point>(
            "/goal_gps", 10,
            std::bind(&TARSNavigator::goalCb, this, std::placeholders::_1));

        // Spawn service client
        spawn_client_ = create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

        // 10 Hz control loop
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TARSNavigator::controlLoop, this));

        RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        RCLCPP_INFO(get_logger(), " TARS Navigator ready");
        RCLCPP_INFO(get_logger(), " Position : /tars/odom (diff_drive odometry)");
        RCLCPP_INFO(get_logger(), " GPS orig : lat=%.4f  lon=%.4f", REF_LAT, REF_LON);
        RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    }

private:
    State  state_;

    // Robot pose in ENU world frame (metres from Gazebo origin)
    double cur_east_  = 0.0;
    double cur_north_ = 0.0;
    double cur_yaw_   = 0.0;
    double smooth_yaw_ = 0.0;
    bool yaw_init_ = false;
    bool   odom_ok_   = false;

    // Goal in ENU world frame
    double goal_east_  = 0.0;
    double goal_north_ = 0.0;
    double goal_lat_   = 0.0;
    double goal_lon_   = 0.0;

    double lin_spd_, ang_spd_, goal_tol_, hdg_tol_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr      dist_pub_, hdg_pub_, cte_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr   spawn_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ── Odometry callback ─────────────────────────────────────────────────────
    void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
    	cur_east_  = msg->pose.pose.position.x;
    	cur_north_ = msg->pose.pose.position.y;

    	double raw_yaw = quatToYaw(
        	msg->pose.pose.orientation.x,
        	msg->pose.pose.orientation.y,
        	msg->pose.pose.orientation.z,
        	msg->pose.pose.orientation.w);

    // Low-pass filter — smooths out odom noise
    // alpha=0.15 means new reading only contributes 15% each update
    	if (!yaw_init_) {
        	smooth_yaw_ = raw_yaw;
        	yaw_init_   = true;
    	} else {
        	double diff  = wrapAngle(raw_yaw - smooth_yaw_);
        	smooth_yaw_  = wrapAngle(smooth_yaw_ + 0.05 * diff);
    	}

    	cur_yaw_  = smooth_yaw_;   // controller uses filtered yaw
    	odom_ok_  = true;
     }
    // ── Goal callback ─────────────────────────────────────────────────────────
    void goalCb(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        if (state_ == State::ROTATING || state_ == State::DRIVING) {
            RCLCPP_WARN(get_logger(), "Already navigating — goal ignored.");
            return;
        }

        goal_lat_ = msg->x;
        goal_lon_ = msg->y;

        ENU g     = gpsToENU(goal_lat_, goal_lon_);
        goal_east_  = g.east;
        goal_north_ = g.north;

        double dist = std::hypot(goal_east_ - cur_east_, goal_north_ - cur_north_);

        state_ = State::ROTATING;

        RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        RCLCPP_INFO(get_logger(), "New goal : lat=%.6f  lon=%.6f", goal_lat_, goal_lon_);
        RCLCPP_INFO(get_logger(), "Goal ENU : east=%.2f m  north=%.2f m", goal_east_, goal_north_);
        RCLCPP_INFO(get_logger(), "Robot at : east=%.2f m  north=%.2f m", cur_east_, cur_north_);
        RCLCPP_INFO(get_logger(), "Distance : %.2f m", dist);
        RCLCPP_INFO(get_logger(), "State → ROTATING");
        RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

        spawnGoalMarker(goal_east_, goal_north_);
    }

    // ── Spawn red sphere ──────────────────────────────────────────────────────
    void spawnGoalMarker(double east, double north)
    {
        if (!spawn_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(get_logger(), "Spawn service unavailable — skipping marker");
            return;
        }
        auto req = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        req->name = "goal_marker";
        req->xml  = makeMarkerSDF();
        req->initial_pose.position.x = east;
        req->initial_pose.position.y = north;
        req->initial_pose.position.z = 0.4;

        spawn_client_->async_send_request(req,
            [this](rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture f) {
                auto r = f.get();
                if (r->success)
                    RCLCPP_INFO(get_logger(), "Red goal marker spawned");
                else
                    RCLCPP_WARN(get_logger(), "Marker spawn failed: %s",
                                r->status_message.c_str());
            });
    }

    // ── Control loop (10 Hz) ──────────────────────────────────────────────────
    void controlLoop()
    {
        if (!odom_ok_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "Waiting for odometry on /tars/odom ...");
            return;
        }
        switch (state_) {
            case State::IDLE:
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                    "[IDLE] Waiting for goal ...");
                break;
            case State::ROTATING: doRotate(); break;
            case State::DRIVING:  doDrive();  break;
            case State::DONE:     stopRobot(); break;
        }
    }

    double distToGoal()    { return std::hypot(goal_east_ - cur_east_, goal_north_ - cur_north_); }
    double bearingToGoal() { return std::atan2(goal_north_ - cur_north_, goal_east_ - cur_east_); }

    // ── Phase 1: Rotate to face goal ──────────────────────────────────────────
    void doRotate()
    {
        double dist    = distToGoal();
        double hdg_err = wrapAngle(bearingToGoal() - cur_yaw_);

        pubFloat(dist_pub_, dist);
        pubFloat(hdg_pub_,  hdg_err);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "[ROTATING] hdg_err=%.2f°  dist=%.2fm", hdg_err * DEG, dist);

        if (std::abs(hdg_err) <= hdg_tol_ || std::abs(hdg_err) < 0.06) {
            stopRobot();
            state_ = State::DRIVING;
            RCLCPP_INFO(get_logger(), "State → DRIVING");
            return;
        }

        geometry_msgs::msg::Twist cmd;
        // Slow down proportionally when close to target — no hard snap
        double scale  = std::min(1.0, std::abs(hdg_err) / 0.3);
        cmd.angular.z = std::copysign(ang_spd_ * scale, hdg_err);
        // Clamp to minimum so it doesn't stall
        if (std::abs(cmd.angular.z) < 0.08)
            cmd.angular.z = std::copysign(0.08, hdg_err);
        cmd_pub_->publish(cmd);
    }

    // ── Phase 2: Drive toward goal ────────────────────────────────────────────
void doDrive()
{
    double dist    = distToGoal();
    double hdg_err = wrapAngle(bearingToGoal() - cur_yaw_);

    // Cross-track error
    double ideal_bear = std::atan2(goal_north_, goal_east_);
    double robot_bear = std::atan2(cur_north_,  cur_east_);
    double robot_dist = std::hypot(cur_east_, cur_north_);
    double cte        = robot_dist * std::sin(wrapAngle(robot_bear - ideal_bear));

    pubFloat(dist_pub_, dist);
    pubFloat(hdg_pub_,  hdg_err);
    pubFloat(cte_pub_,  cte);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[DRIVING] dist=%.2fm | hdg_err=%.2f° | CTE=%.3fm",
        dist, hdg_err * DEG, cte);

    // Goal reached
    if (dist < goal_tol_) {
        stopRobot();
        state_ = State::DONE;
        RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        RCLCPP_INFO(get_logger(), "✅  GOAL REACHED!");
        RCLCPP_INFO(get_logger(), "    Residual: %.3f m  |  CTE: %.3f m", dist, cte);
        RCLCPP_INFO(get_logger(), "    State → DONE  (send new goal to continue)");
        RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        return;
    }

    // Re-rotate only on large drift
    if (std::abs(hdg_err) > 0.5) {
        state_ = State::ROTATING;
        RCLCPP_WARN(get_logger(),
            "Heading drifted %.2f° — State → ROTATING", hdg_err * DEG);
        return;
    }

    geometry_msgs::msg::Twist cmd;

    // Scale speed down in last 2 metres — gentle stop, no overshoot
    double speed_scale = std::min(1.0, dist / 2.0);
    cmd.linear.x = lin_spd_ * std::max(speed_scale, 0.3);  // min 30% speed

    // Dead-band: ignore tiny heading errors — stops micro-jitter
    double corrected_err = (std::abs(hdg_err) < 0.12) ? 0.0 : hdg_err;
    cmd.angular.z = 0.4 * corrected_err;

    cmd_pub_->publish(cmd);
}

    void stopRobot() { cmd_pub_->publish(geometry_msgs::msg::Twist()); }

    void pubFloat(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& p, double v) {
        std_msgs::msg::Float64 m;  m.data = v;  p->publish(m);
    }
};

// ══════════════════════════════════════════════════════════════════════════════
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TARSNavigator>());
    rclcpp::shutdown();
    return 0;
}
