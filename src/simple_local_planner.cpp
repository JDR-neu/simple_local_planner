#include <nav_core/base_local_planner.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace simple_local_planner {

const double MIN_CHANGE_THRESH = 0.0001;  //Threshold for determining when a localization update has occured.

class SimpleLocalPlanner : public nav_core::BaseLocalPlanner {
    public:
        
    void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros){
        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("max_trans_vel", max_trans_vel_, 1.0);
        private_nh.param("max_rot_vel", max_rot_vel_, 1.0);
        private_nh.param("position_window", p_window_, 0.5);
        private_nh.param("orientation_window", o_window_, 0.1);
        private_nh.param("position_deadband", p_deadband_, 0.1);
        private_nh.param("orientation_deadband", o_deadband_, 0.05);                
        private_nh.param("position_precision", p_precision_, 0.2);
        private_nh.param("orientation_precision", o_precision_, 0.05);
        private_nh.param("forward_angle", forward_angle_, 0.0);
        private_nh.param("base_frame", base_frame_, std::string("/base_footprint"));
        private_nh.param("t_p_gain", t_p_gain_, 1.0);
        private_nh.param("t_d_gain", t_d_gain_, 1.0);
        private_nh.param("r_p_gain", r_p_gain_, 1.0);
        private_nh.param("r_d_gain", r_d_gain_, 1.0);
                
        pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("target_pose", 5);
        
        vx = vy = vth = 0.0;
        prev_ex_ = prev_ey_ = prev_eth_ = 0.0; //initialize previous error values to 0
        
        tf_ = tf;
        plan_index_ = 1;
    }
    
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
        global_plan_ = orig_global_plan;
        plan_index_ = 0;
        prev_ex_ = prev_ey_ = prev_eth_ = 0.0;
        return true;
    }
    
    bool isGoalReached(){
        return plan_index_ >= global_plan_.size();
    }
    
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
        double ex,ey,eth;
        int n = global_plan_.size();

        // advance to next goal pose
        while( plan_index_ < n ){
            getTransformedPosition(global_plan_[plan_index_], ex, ey, eth);
            double dist = hypot(ex, ey);
            
            if(plan_index_== n-1 && dist < p_precision_ && fabs(eth) < o_precision_)
            {
                plan_index_++;
                ROS_INFO_STREAM("Reached goal precision(dist, angle): (" << dist << "," << eth << ")");
            }
            else if(plan_index_ < n-1 && dist < p_window_ && fabs(eth) < o_window_)
            {
                plan_index_++;
                ROS_INFO_STREAM("Reached waypoint window(dist, angle): (" << dist << "," << eth << ")");
            }else{
                break;
            }            
        }
        
        if( plan_index_ >= n ){
            return true;
        }
        pub_.publish(global_plan_[plan_index_]);

        getTransformedPosition(global_plan_[plan_index_], ex, ey, eth);
        
        if( fabs(ex - prev_ex_) > MIN_CHANGE_THRESH ||
            fabs(ey - prev_ey_) > MIN_CHANGE_THRESH ||
            fabs(eth - prev_eth_) > MIN_CHANGE_THRESH )
        { 
          //PD control law assumes fixed rate (d-term is not scaled by loop time).
          if( fabs(ex) > p_deadband_ || plan_index_== n-1)
          {
            vx = t_p_gain_ * ex + t_d_gain_ * (ex - prev_ex_);
          }
          else
          {
            vx = 0;
            ROS_INFO_STREAM("x-error: " << ex << " within deadband: " << p_deadband_);
          }
          
          
          if( fabs(ey) > p_deadband_ || plan_index_== n-1)
          {
            vy = t_p_gain_ * ey + t_d_gain_ * (ey - prev_ey_);
          }
          else
          {
            vy = 0;
            ROS_INFO_STREAM("y-error: " << ey << " within deadband: " << p_deadband_);
          }
          
          if( fabs(eth) > o_deadband_ || plan_index_== n-1)
          {
            vth = r_p_gain_ * eth + r_d_gain_ * (eth - prev_eth_);
          }
          else
          {
            vth = 0;
            ROS_INFO_STREAM("th-error: " << eth << " within deadband: " << o_deadband_);
          }
          
          ROS_INFO_STREAM("Trans-x: vx, ex, prev_ex: " << vx << "," << ex << "," << prev_ex_);
          ROS_INFO_STREAM("Trans-y: vy, ey, prev_ey: " << vy << "," << ey << "," << prev_ey_);
          ROS_INFO_STREAM("Theta: vth, eth, prev_eth:" << vth << "," << eth << "," << prev_eth_);
          
          
          prev_ex_ = ex;
          prev_ey_ = ey;
          prev_eth_ = eth;
                 
          if(fabs(vx) > max_trans_vel_){
              vx = copysign(max_trans_vel_, vx);
          }
          if(fabs(vy) > max_trans_vel_){
              vy = copysign(max_trans_vel_, vy);
          }
          if(fabs(vth) > max_rot_vel_){
              vth = copysign(max_rot_vel_, vth);
          }
        }
        else
        {
          ROS_WARN_STREAM("Change in error below min threshold( " << MIN_CHANGE_THRESH << "), not updating velocity");
        }
       
        cmd_vel.linear.x = vx;
        cmd_vel.linear.y = vy;
        cmd_vel.angular.z = vth;
        
        ROS_INFO_STREAM("Veclocity(vx, vy, vth), index: " << vx << "," << vy << "," << vth << "," << plan_index_);
                
        return true;
    }
    
    protected:
    void getTransformedPosition(geometry_msgs::PoseStamped& pose, double& x, double& y, double& theta)
    {
        geometry_msgs::PoseStamped ps;
        pose.header.stamp = ros::Time(0);
        tf_->transformPose(base_frame_, pose, ps);
        x = ps.pose.position.x;
        y = ps.pose.position.y,
        theta = tf::getYaw(ps.pose.orientation)-forward_angle_;
    }
    
        std::vector<geometry_msgs::PoseStamped> global_plan_;
        ros::Publisher pub_;
        tf::TransformListener* tf_;
        std::string base_frame_;
        int plan_index_;
        double max_trans_vel_, max_rot_vel_;
        double p_window_, o_window_, p_precision_, o_precision_, p_deadband_, o_deadband_;
        double forward_angle_;
        double t_p_gain_, t_d_gain_, r_p_gain_, r_d_gain_;
        double vx, vy, vth;
        double prev_ex_, prev_ey_, prev_eth_;  //last error values
     
};
};

PLUGINLIB_EXPORT_CLASS(simple_local_planner::SimpleLocalPlanner, nav_core::BaseLocalPlanner)
