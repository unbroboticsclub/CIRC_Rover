This branch will be to test all bridge node features to ensure they work before deploying them to the main branch.


rover_bringup - For running all nodes
rover_description - docs for visulization (what it is) (MAY REMOVE)
rover_interfaces - To define messages, services, actions
rover_hardware - nodes which directly interact with motors, sensors and MCUs
rover_control - nodes for motion control + teleoperations + safety logic
rover_perception - nodes to process camera + lidar + other sensors
rover_navigation - nodes to create new commands (using data generated from rover_perception)
rover_simulation - nodes and files for running simulations
