# plugin_planner
Requirements: 
- ROS environment installed correctly
- a server that provide a path inside the topic path_proxy (there is the project related to this that will do this)
- a GAZEBO simulation fot the navigation process (you can find that inside the other project) 

# How to use:
- launch a gazebo simulation with a robot and a map 
- launch your mapping algorithm 
- launch your proxy service that pub the topic 
- launch move_base simulation with this plugin as a global plunner 
- USE rviz to provide a init pose and a goal pose
# Comments:
this project wants to give the opportunity to edit the global planner that is entirly written en c++ with a pythone code.
This allows you to comunicate with a python proxy that can generate the trajectory.
From an optimization point it can be useless to use python but the python code allows us much more freedom to code.

