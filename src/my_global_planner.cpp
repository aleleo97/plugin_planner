
 #include "my_global_planner.h"
 #include <base_local_planner/costmap_model.h>
 #include <pluginlib/class_list_macros.h>
 #include <tf2/convert.h>
 #include <tf2/utils.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
 #include <nav_msgs/Path.h>
 #include <std_srvs/Empty.h>
 #include "geometry_msgs/PoseWithCovarianceStamped.h"
 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(my_global_planner::MyGlobalPlanner, nav_core::BaseGlobalPlanner)
 
 nav_msgs::Path my_plan;
 bool gProxyFlag ;
 geometry_msgs::PoseStamped global_goal;
 geometry_msgs::PoseWithCovarianceStamped global_init;

   void pathCallback(const nav_msgs::Path& msg){
       ROS_INFO("proxy called");
       gProxyFlag = true;
       my_plan = msg;
   }
 
 void subProxy(){
       ros::Rate Rate(10);
       ros::NodeHandle nh;
       ros::Subscriber sub = nh.subscribe("path_proxy", 1, pathCallback);
       ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("call_proxy");
       std_srvs::Empty rien;
       ros::Publisher pub1 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose", 1);
       ros::Publisher pub2 = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
       pub1.publish(global_init);
       pub2.publish(global_goal);
       if (client.call(rien)) // Send through the connection the name of the trajectory to execute
        {
            ROS_INFO("CALLED PROXY"); // Print the result given by the service called
        }
        else
        {
            ROS_ERROR("Failed to call service /trajectory_by_name");
        }

       while(gProxyFlag == false){
         ROS_WARN("NOT HAVE a valid plan ... proxy failed try again");
         boost::this_thread::sleep(boost::posix_time::milliseconds(10));
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
     global_goal = goal;
     global_init.header.frame_id = start.header.frame_id;
     global_init.pose.pose.position.x = start.pose.position.x;
     global_init.pose.pose.position.y = start.pose.position.y;
     global_init.pose.pose.orientation.x = start.pose.orientation.x;
     global_init.pose.pose.orientation.y = start.pose.orientation.y;
     global_init.pose.pose.orientation.z = start.pose.orientation.z;
     global_init.pose.pose.orientation.w = start.pose.orientation.w;
     boost::thread t1 (&subProxy);
     t1.join();
     ROS_INFO("path created");
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