/**
 * maze.cpp — TARS Bug2 Maze Solver  v5.0
 * ────────────────────────────────────────────────────────────────────────────
 *
 * ── ARCHITECTURE ─────────────────────────────────────────────────────────────
 * States: IDLE → ALIGNING → DRIVING → WALL_FOLLOWING → DONE
 *
 * ALIGNING       : Rotate in place toward goal.
 * DRIVING        : Straight to goal + mild VFH to avoid grazing walls.
 *                  Transition to WALL_FOLLOWING when front < STOP_D.
 * WALL_FOLLOWING : Left-hand rule. Sub-states:
 *                    done, bug2_exit, timeout, corner, clearing, lost, normal
 *
 * ── MAZE COORDINATES ─────────────────────────────────────────────────────────
 *   Arena : 18×18 m  (x: −9→+9, y: −9→+9)
 *   ENTRY : spawn (−8.5, +7.0)      EXIT : goal (+8.5, −7.0)
 *   M-line: SE direction, ≈ −39.5° from +X axis
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <memory>

// ── Constants ─────────────────────────────────────────────────────────────────
static constexpr double EARTH_R = 6371000.0;
static constexpr double RAD     = M_PI / 180.0;
static constexpr double DEG     = 180.0 / M_PI;
static constexpr double REF_LAT = 12.9716;
static constexpr double REF_LON = 77.5946;

static constexpr double STOP_D  = 0.45;  // [m] wall-follow trigger
static constexpr double SLOW_D  = 0.65;  // [m] start slowing in DRIVING

static constexpr double WF_DIST    = 0.55;  // [m] target left-wall gap
static constexpr double WF_TIMEOUT = 30.0;  // [s] relax hit_dist_ after this

// Bug2 exit conditions
static constexpr double ON_MLINE_THR = 0.45;  // [m] |CTE| for "on M-line"
static constexpr double PROGRESS_THR = 0.30;  // [m] must be closer than hit_dist_
static constexpr double MIN_WF_SECS  = 3.0;   // [s] minimum wall-follow time

static constexpr int NSEC     = 36;      // LiDAR sectors (10° each)
static constexpr int LEFT_S   = NSEC*3/4; // sector ≈ +95° (left)
static constexpr int RIGHT_S  = NSEC/4;   // sector ≈ −85° (right)

enum class State { IDLE, ALIGNING, DRIVING, WALL_FOLLOWING, DONE };

// ── Helpers ───────────────────────────────────────────────────────────────────
struct ENU { double e=0, n=0; };

ENU gpsToENU(double lat, double lon) {
    double dlat=(lat-REF_LAT)*RAD, dlon=(lon-REF_LON)*RAD;
    double mlat=0.5*(lat+REF_LAT)*RAD;
    return { EARTH_R*std::cos(mlat)*dlon, EARTH_R*dlat };
}

inline double wrap(double a) {
    while (a> M_PI) a-=2*M_PI;
    while (a<-M_PI) a+=2*M_PI;
    return a;
}

inline double q2yaw(double x,double y,double z,double w) {
    double nm=std::sqrt(x*x+y*y+z*z+w*w);
    if (nm<1e-9) return 0.0;
    x/=nm;y/=nm;z/=nm;w/=nm;
    return std::atan2(2*(w*z+x*y),1-2*(y*y+z*z));
}

// ═════════════════════════════════════════════════════════════════════════════
// OdomPose
// ═════════════════════════════════════════════════════════════════════════════
class OdomPose {
    double e_=0,n_=0,yaw_=0; bool ok_=false;
public:
    void update(const nav_msgs::msg::Odometry& m) {
        e_=m.pose.pose.position.x; n_=m.pose.pose.position.y;
        auto& q=m.pose.pose.orientation; yaw_=q2yaw(q.x,q.y,q.z,q.w); ok_=true;
    }
    bool   ok()  const{return ok_;}
    double e()   const{return e_;}
    double n()   const{return n_;}
    double yaw() const{return yaw_;}
};

// ═════════════════════════════════════════════════════════════════════════════
// LidarVFH
// ═════════════════════════════════════════════════════════════════════════════
class LidarVFH {
    std::vector<float> sec_;
    bool ok_=false;

    int toSector(float angle) const {
        int s=static_cast<int>((angle+M_PI)/(2*M_PI)*NSEC);
        return std::clamp(s,0,NSEC-1);
    }
public:
    LidarVFH():sec_(NSEC,99.f){}

    void update(const sensor_msgs::msg::LaserScan& scan) {
        std::fill(sec_.begin(),sec_.end(),99.f);
        for (int i=0;i<(int)scan.ranges.size();++i) {
            float r=scan.ranges[i];
            if (!std::isfinite(r)||r<scan.range_min||r>scan.range_max) continue;
            int s=toSector(scan.angle_min+i*scan.angle_increment);
            sec_[s]=std::min(sec_[s],r);
        }
        ok_=true;
    }

    static double sAngle(int s){ return -M_PI+(s+0.5)*(2*M_PI/NSEC); }

    float avgDist(int center, int hw=1) const {
        float sum=0; int cnt=0;
        for (int d=-hw;d<=hw;++d){sum+=sec_[(center+d+NSEC)%NSEC];++cnt;}
        return sum/cnt;
    }

    float frontDist() const {
        float md=99.f;
        for (int s=0;s<NSEC;++s)
            if (std::abs(wrap(sAngle(s)))<M_PI/7.2) md=std::min(md,sec_[s]);
        return md;
    }

    bool frontBlocked(double thr) const { return ok_&&frontDist()<(float)thr; }

    // Mild VFH: used only in DRIVING to avoid grazing walls.
    // Two-pass: prefer sectors near goal, fall back to any clear forward sector.
    double bestForward(double goalAngle) const {
        if (!ok_) return goalAngle;
        int best=-1; double bs=-1e9;
        for (int pass=0;pass<2;++pass) {
            for (int s=0;s<NSEC;++s) {
                double sa=sAngle(s);
                if (std::abs(wrap(sa))>M_PI/2) continue;
                if (sec_[s]<STOP_D)            continue;
                if (pass==0&&std::abs(wrap(sa-goalAngle))>M_PI/2) continue;
                double score=sec_[s]-1.8*std::abs(wrap(sa-goalAngle));
                if (score>bs){bs=score;best=s;}
            }
            if (best>=0) return sAngle(best);
        }
        return goalAngle;
    }

    bool isOk() const{return ok_;}
};

// ═════════════════════════════════════════════════════════════════════════════
// TARSNavigator — Bug2 maze solver
// ═════════════════════════════════════════════════════════════════════════════
class TARSNavigator : public rclcpp::Node {
    State    state_=State::IDLE;
    OdomPose pose_;
    LidarVFH lidar_;

    double lin_=0.4,ang_=0.6,gtol_=0.8,htol_=0.10;

    // Goal + M-line
    double ge_=0,gn_=0;   // goal
    double se_=0,sn_=0;   // M-line start (= robot position when goal was set)

    // DRIVING
    double last_steer_=0.0;

    // WALL_FOLLOWING (Bug2)
    double       hit_dist_=1e9;   // dist-to-goal when WF started
    rclcpp::Time wf_start_;
    rclcpp::Time wf_relax_;
    bool         corner_clearing_=false;

    bool marker_spawned_=false;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmdp_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr         distp_,ctep_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odoms_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidars_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr   goals_,goalxy_;
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr     spawn_;
    rclcpp::TimerBase::SharedPtr tim_;

public:
    TARSNavigator():Node("tars_maze"){
        declare_parameter("linear_speed",   0.4);
        declare_parameter("angular_speed",  0.6);
        declare_parameter("goal_tolerance", 0.8);
        declare_parameter("heading_tolerance",0.10);
        lin_ =get_parameter("linear_speed").as_double();
        ang_ =get_parameter("angular_speed").as_double();
        gtol_=get_parameter("goal_tolerance").as_double();
        htol_=get_parameter("heading_tolerance").as_double();

        cmdp_ =create_publisher<geometry_msgs::msg::Twist>("/tars/cmd_vel",10);
        distp_=create_publisher<std_msgs::msg::Float64>("/tars/distance_remaining",10);
        ctep_ =create_publisher<std_msgs::msg::Float64>("/tars/cross_track_error",10);

        odoms_=create_subscription<nav_msgs::msg::Odometry>(
            "/tars/odom",rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr m){pose_.update(*m);});

        lidars_=create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr m){lidar_.update(*m);});

        goals_=create_subscription<geometry_msgs::msg::Point>("/goal_gps",10,
            [this](geometry_msgs::msg::Point::SharedPtr m){
                if (navigating()){RCLCPP_WARN(get_logger(),"Navigating — ignored");return;}
                auto g=gpsToENU(m->x,m->y); setGoal(g.e,g.n);});

        goalxy_=create_subscription<geometry_msgs::msg::Point>("/goal_xy",10,
            [this](geometry_msgs::msg::Point::SharedPtr m){
                if (navigating()){RCLCPP_WARN(get_logger(),"Navigating — ignored");return;}
                setGoal(m->x,m->y);});

        spawn_=create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
        tim_  =create_wall_timer(std::chrono::milliseconds(50),[this]{loop();});

        RCLCPP_INFO(get_logger(),"TARS Maze v5.0 — Bug2 algorithm");
        RCLCPP_INFO(get_logger(),
            "  Send goal: ros2 topic pub /goal_xy geometry_msgs/msg/Point"
            " \"{x: 8.5, y: -7.0, z: 0.0}\"");
    }

private:
    bool navigating() const {
        return state_==State::ALIGNING||state_==State::DRIVING||
               state_==State::WALL_FOLLOWING;
    }

    double dist()   const{return std::hypot(ge_-pose_.e(),gn_-pose_.n());}
    double brobot() const{return wrap(std::atan2(gn_-pose_.n(),ge_-pose_.e())-pose_.yaw());}

    // Signed cross-track distance from M-line (start→goal).
    // +ve = robot is to the LEFT of the line.
    double mCTE() const {
        double dx=ge_-se_,dy=gn_-sn_,L=std::hypot(dx,dy);
        if (L<1e-6) return 0.0;
        return (dx*(pose_.n()-sn_)-dy*(pose_.e()-se_))/L;
    }

    // Normalised projection onto M-line: 0=start, 1=goal.
    double mProj() const {
        double dx=ge_-se_,dy=gn_-sn_,L2=dx*dx+dy*dy;
        if (L2<1e-6) return 0.0;
        return ((pose_.e()-se_)*dx+(pose_.n()-sn_)*dy)/L2;
    }

    // Bug2 exit: on M-line, closer than hit_dist, minimum time elapsed.
    bool bug2Exit(double elapsed) const {
        if (elapsed<MIN_WF_SECS)               return false;
        if (std::abs(mCTE())>ON_MLINE_THR)     return false;
        double p=mProj();
        if (p<0.05||p>0.98)                    return false;
        if (dist()>hit_dist_-PROGRESS_THR)     return false;
        return true;
    }

    void cmd(double vx,double wz){
        geometry_msgs::msg::Twist c; c.linear.x=vx; c.angular.z=wz;
        cmdp_->publish(c);
    }
    void stop(){cmdp_->publish(geometry_msgs::msg::Twist());}
    void pubD(double v){std_msgs::msg::Float64 m;m.data=v;distp_->publish(m);}
    void pubCTE(double v){std_msgs::msg::Float64 m;m.data=v;ctep_->publish(m);}

    void setGoal(double e,double n){
        ge_=e; gn_=n;
        se_=pose_.e(); sn_=pose_.n();  // M-line starts at current position
        last_steer_=0.0; corner_clearing_=false; marker_spawned_=false;
        state_=State::ALIGNING;
        RCLCPP_INFO(get_logger(),
            "Goal: (%.2f,%.2f)  dist=%.2fm  M-line start: (%.2f,%.2f)",
            ge_,gn_,dist(),se_,sn_);
        if (!marker_spawned_){spawnMarker(ge_,gn_);marker_spawned_=true;}
    }

    void loop(){
        if (!pose_.ok()){
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,
                "Waiting for /tars/odom..."); return;}
        if (!lidar_.isOk()){
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,
                "Waiting for /scan..."); return;}
        pubD(dist()); pubCTE(mCTE());
        switch (state_){
            case State::IDLE:
                RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,
                    "[IDLE] waiting for goal..."); break;
            case State::ALIGNING:       doAlign();     break;
            case State::DRIVING:        doDrive();     break;
            case State::WALL_FOLLOWING: doWallFollow();break;
            case State::DONE:           stop();        break;
        }
    }

    // ── ALIGNING ──────────────────────────────────────────────────────────────
    void doAlign(){
        double err=brobot();
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),500,
            "[ALIGN] err=%.1f°  dist=%.2fm",err*DEG,dist());
        if (std::abs(err)<htol_){
            stop(); last_steer_=0.0; state_=State::DRIVING;
            RCLCPP_INFO(get_logger(),"[ALIGN] → DRIVING"); return;
        }
        cmd(0.0, std::copysign(ang_*std::clamp(std::abs(err)/0.5,0.3,1.0),err));
    }

    // ── DRIVING ───────────────────────────────────────────────────────────────
    // Straight to goal. Mild VFH only to avoid grazing walls.
    // Hard wall hit (front < STOP_D) → WALL_FOLLOWING (Bug2 transition).
    void doDrive(){
        double d=dist(), herr=brobot(), front=lidar_.frontDist();

        if (d<gtol_){
            stop(); state_=State::DONE;
            RCLCPP_INFO(get_logger(),"GOAL REACHED  residual=%.3fm",d); return;
        }

        // Bug2: wall hit → record hit_dist_, start wall-following
        if (front<STOP_D){
            hit_dist_=d; wf_start_=now(); wf_relax_=now();
            corner_clearing_=false; last_steer_=0.0;
            state_=State::WALL_FOLLOWING;
            RCLCPP_WARN(get_logger(),
                "[DRIVE] Wall hit → Bug2 WF  hit_dist=%.2fm  CTE=%.2fm",
                hit_dist_,mCTE());
            return;
        }

        // Normal drive: head to goal, mild VFH near obstacles
        double raw = (front>SLOW_D&&std::abs(herr)<0.25)
                     ? 2.0*herr
                     : lidar_.bestForward(herr);
        double steer=std::clamp(0.65*last_steer_+0.35*raw,-ang_,ang_);
        last_steer_=steer;
        double spd=lin_*std::clamp(
            std::min({(front-STOP_D)/(SLOW_D-STOP_D),d/2.0,1.0}),0.20,1.0);
        cmd(spd,steer);

        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
            "[DRIVE] d=%.2fm hdg=%.1f° steer=%.1f° F=%.2fm spd=%.2f",
            d,herr*DEG,steer*DEG,front,spd);
    }

    // ── WALL_FOLLOWING ────────────────────────────────────────────────────────
    // Left-hand rule.  Exit ONLY via Bug2 condition.
    // Never exits because the goal direction "looks clear" — that is the v1–v4 bug.
    void doWallFollow(){
        double d=dist(), elapsed=(now()-wf_start_).seconds();
        float  left=lidar_.avgDist(LEFT_S,1), front=lidar_.frontDist();

        // 1. Goal reached (can happen from inside wall-following)
        if (d<gtol_){
            stop(); state_=State::DONE;
            RCLCPP_INFO(get_logger(),"GOAL REACHED (in WF)  residual=%.3fm",d); return;
        }

        // 2. Bug2 exit: on M-line AND closer than hit_dist_ − PROGRESS_THR
        if (bug2Exit(elapsed)){
            RCLCPP_INFO(get_logger(),
                "[WF] Bug2 exit — dist=%.2fm hit=%.2fm CTE=%.2fm t=%.1fs",
                d,hit_dist_,mCTE(),elapsed);
            corner_clearing_=false; last_steer_=0.0;
            state_=State::ALIGNING; return;
        }

        // 3. Timeout: 30 s without crossing M-line → relax hit_dist_ by 1 m
        if ((now()-wf_relax_).seconds()>WF_TIMEOUT){
            hit_dist_+=1.0; wf_relax_=now();
            RCLCPP_WARN(get_logger(),"[WF] Timeout — relaxed hit_dist to %.2fm",hit_dist_);
        }

        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
            "[WF] L=%.2f F=%.2f d=%.2f CTE=%.2f hit=%.2f t=%.1f",
            left,front,d,mCTE(),hit_dist_,elapsed);

        geometry_msgs::msg::Twist c;

        // 4. Corner: front blocked → stop + spin right
        if (front<STOP_D*1.8f){
            corner_clearing_=true;
            c.linear.x=0.0; c.angular.z=-ang_*0.75;
            RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),400,
                "[WF] Corner spin right (F=%.2fm)",front);

        // 5. Corner-clearing: creep right until wall re-acquired and front open
        } else if (corner_clearing_){
            if ((left<WF_DIST*2.5f)&&(front>SLOW_D)){
                corner_clearing_=false;
                RCLCPP_INFO(get_logger(),"[WF] Corner cleared (L=%.2fm)",left);
            } else {
                c.linear.x=lin_*0.15; c.angular.z=-ang_*0.55;
                RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),600,
                    "[WF] Clearing (L=%.2f F=%.2f)",left,front);
            }

        // 6. Lost wall: open junction → gentle left turn to re-acquire
        } else if (left>WF_DIST*3.0f){
            c.linear.x=lin_*0.30; c.angular.z=ang_*0.55;
            RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),800,
                "[WF] Lost wall (L=%.2fm) — left",left);

        // 7. Normal proportional wall-distance control
        } else {
            double wallErr=left-WF_DIST;
            double steer=(std::abs(wallErr)>0.05)
                          ? std::clamp(1.4*wallErr,-ang_,ang_) : 0.0;
            double spd=lin_*std::clamp((front-STOP_D)/(SLOW_D-STOP_D),0.25,0.70);
            c.linear.x=spd; c.angular.z=steer;
        }

        cmdp_->publish(c);
    }

    void spawnMarker(double e,double n){
        if (!spawn_->wait_for_service(std::chrono::seconds(2))) return;
        auto r=std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        r->name="goal_marker";
        r->xml="<sdf version='1.6'><model name='goal_marker'><static>true</static>"
               "<link name='link'><visual name='v'><geometry>"
               "<sphere><radius>0.4</radius></sphere></geometry>"
               "<material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse>"
               "</material></visual></link></model></sdf>";
        r->initial_pose.position.x=e; r->initial_pose.position.y=n;
        r->initial_pose.position.z=0.4;
        using F=rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture;
        spawn_->async_send_request(r,[this](F f){
            RCLCPP_INFO(get_logger(),f.get()->success?"Marker ok":"Marker fail");});
    }
};

int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TARSNavigator>());
    rclcpp::shutdown(); return 0;
}
