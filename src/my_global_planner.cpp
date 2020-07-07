
 #include "my_global_planner.h"
 #include <base_local_planner/costmap_model.h>
 #include <pluginlib/class_list_macros.h>
 #include <tf2/convert.h>
 #include <tf2/utils.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
 #include <nav_msgs/Path.h>
 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(my_global_planner::MyGlobalPlanner, nav_core::BaseGlobalPlanner)
 
 nav_msgs::Path my_plan;
 bool gProxyFlag ;
   void pathCallback(const nav_msgs::Path& msg){
       ROS_INFO("proxy called");
       gProxyFlag = true;
       my_plan = msg;
   }
 
 void subProxy(){
       ros::Rate Rate(10);
       ros::NodeHandle nh;
       ros::Subscriber sub = nh.subscribe("path_proxy", 1, pathCallback);
       while(gProxyFlag == false){
         ROS_INFO("Waiting a valid plan");
         boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
     }
   }

 namespace my_global_planner {
   MyGlobalPlanner::MyGlobalPlanner()
   : costmap_ros_(NULL), initialized_(false){}
   MyGlobalPlanner::MyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
   : costmap_ros_(NULL), initialized_(false){
     int argc = 0;
     char** argv = NULL;
     ros::init(argc,argv,name);
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
     plan.clear();
     costmap_ = costmap_ros_->getCostmap();
     ROS_INFO("path created");
     boost::thread t1 (&subProxy);
     t1.join();
     plan.push_back(start);
     for(int i=0; i < (my_plan.poses.size());i++){
        geometry_msgs::PoseStamped new_goal = goal;

        new_goal.pose.position.x = my_plan.poses[i].pose.position.x;
        new_goal.pose.position.y = my_plan.poses[i].pose.position.y;

        new_goal.pose.orientation.x = my_plan.poses[i].pose.orientation.x;
        new_goal.pose.orientation.y = my_plan.poses[i].pose.orientation.y;
        new_goal.pose.orientation.z = my_plan.poses[i].pose.orientation.z;
        new_goal.pose.orientation.w = my_plan.poses[i].pose.orientation.w;

        plan.push_back(new_goal);
    }
    
    plan.push_back(goal);
    gProxyFlag = false;
    return true;
    }
   };