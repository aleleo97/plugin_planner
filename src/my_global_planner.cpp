
 #include "my_global_planner.h"
 #include <base_local_planner/costmap_model.h>
 #include <pluginlib/class_list_macros.h>
 #include <tf2/convert.h>
 #include <tf2/utils.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
 #include <nav_msgs/Path.h>
 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(my_global_planner::MyGlobalPlanner, nav_core::BaseGlobalPlanner)
 
 bool gProxyFlag ;

   void pathCallback(const nav_msgs::Path& msg){
       
       ROS_INFO("proxy called");
       printf("proxy called");
       gProxyFlag = true;

   }

 namespace my_global_planner {
 
   MyGlobalPlanner::MyGlobalPlanner()
   : costmap_ros_(NULL), initialized_(false){}
 
   MyGlobalPlanner::MyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
   : costmap_ros_(NULL), initialized_(false){
     initialize(name, costmap_ros);
   }

   void MyGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
     if(!initialized_){
       printf("Planner inizialised");
       ROS_INFO("This planner has been initialized");
       costmap_ros_ = costmap_ros;
       costmap_ = costmap_ros_->getCostmap();
 
       ros::NodeHandle private_nh("~/" + name);
       private_nh.param("step_size", step_size_, costmap_->getResolution());
       private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
       //world_model_ = new base_local_planner::CostmapModel(*costmap_); 

       ros::Subscriber sub = private_nh.subscribe("path_proxy", 1000, pathCallback);
       initialized_ = true;
       gProxyFlag = false;
     }
     else
       ROS_WARN("This planner has already been initialized... doing nothing");
   }
 

 
 
   bool MyGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
       const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
 
     if(!initialized_){
       ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
       return false;
     }
 
     ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
     ROS_INFO(gProxyFlag ? "true" : "false");
     //printf( gProxyFlag ? "true" : "false");//
     plan.clear();
     costmap_ = costmap_ros_->getCostmap();
 
     const double start_yaw = tf2::getYaw(start.pose.orientation);
     const double goal_yaw = tf2::getYaw(goal.pose.orientation);
 
     //we want to step back along the vector created by the robot's position and the goal pose until we find a legal cell
     double goal_x = goal.pose.position.x;
     double goal_y = goal.pose.position.y;
     double start_x = start.pose.position.x;
     double start_y = start.pose.position.y;
 
     double diff_x = goal_x - start_x;
     double diff_y = goal_y - start_y;
     double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);
 
     double target_x = goal_x;
     double target_y = goal_y;
     double target_yaw = goal_yaw;
 
     bool done = false;
     double scale = 1.0;
     double dScale = 0.01;

     while(gProxyFlag == false){
         ROS_INFO("Waiting a valid plan");
         ROS_INFO(gProxyFlag ? "true" : "false");
     }
     ROS_INFO(gProxyFlag ? "true" : "false");
     target_yaw = angles::normalize_angle(start_yaw + scale * diff_yaw);
 
     plan.push_back(start);
     geometry_msgs::PoseStamped new_goal = goal;
     tf2::Quaternion goal_quat;
     goal_quat.setRPY(0, 0, target_yaw);
 
     new_goal.pose.position.x = target_x;
     new_goal.pose.position.y = target_y;
 
     new_goal.pose.orientation.x = goal_quat.x();
     new_goal.pose.orientation.y = goal_quat.y();
     new_goal.pose.orientation.z = goal_quat.z();
     new_goal.pose.orientation.w = goal_quat.w();
 
     plan.push_back(new_goal);
     return (done);
   }
 
 };