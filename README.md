# plugin_planner
Requirements: 

- ROS installed correctly
- a publisher in path_proxy 
- move_base node correctly installed \
How to use:
- launch a gazebo simulation
- launch your proxy that pub the topic 
- launch move_base simulation withh this plugin as a global plunner 
=======
- ROS environment installed correctly
- a server that provide a path inside the topic path_proxy (there is the project related to this that will do this)
- a GAZEBO simulation fot the navigation process (you can find that inside the other project) 

# How to use:
- launch a gazebo simulation with a robot and a map 
&&roslaunch husky_navigation husky_playpen.launch
- launch your mapping algorithm (or use directly the map done by the file .yaml and use it vith the launch file of amcl.launch)
- launch your proxy service that pub the topic 
&&rosrun proxy_nav planner.py
- launch move_base simulation with this plugin as a global plunner 
&&roslaunch husky_navigation amcl_demo.launch
- USE rviz to provide a init pose and a goal pose
&&rosrun rviz rviz
# Comments:
this project wants to give the opportunity to edit the global planner that is entirly written en c++ with a pythone code.
This allows you to comunicate with a python proxy that can generate the trajectory.
From an optimization point it can be useless to use python but the python code allows us much more freedom to code.

>>>>>>> a5d088f851f4a33b5def6fcd657173f117b6035c
