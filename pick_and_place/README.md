# README for TRINA2 Pick and Place

### Prerequisites
In order to run our simulation, all the prerequisites for the baseline HIRO TRINA2 environment need to be met.  In addition, an alternate version of the GazeboGraspFix plugin needs to be installed in the ros_kortex package.  The file "GazeboGraspFix.cpp" in ros_kortex/third_party/gazebo-pkgs/gazebo_grasp_plugin/src should be replaced with the version in this folder.  This is a temporary solution to an issue with using two grippers for manipulation.  A better design for a permanent fix will require some reorganization in the robot description and could be planned for future work.

### Startup

Launch hospital world with table and blocks:
roslaunch trina2_gazebo trina2_blocks.launch

Wait for TRINA to complete initialization (arms in home position, trajectory controllers complete).
Pause the simulation to ensure the block location reset works.  This functionality is maintained so that we can create a test environment in front of TRINA regardless of the robot's location in a world.

Start the service node:
roslaunch pick_and_place arm_to_block.launch

Unpause the simulation.

If the table and blocks are not properly positioned or need to be reset for some reason, call init_blocks manually:
rosrun pick_and_place init_blocks 
As long as all blocks have been successfully detached from the robot, you can leave the simulation running, shut down the arm_to_block console, and start again.  If the collision objects are in a faulty state, Gazebo has to be restarted to clear the errors.  This appears to be an issue with MoveIt collision objects and planning scenes - need to find a way to fully clear the planning scene, but the provided interfaces aren't working.  More work needed here.

### Manual pick and place ops

Call pick service, specifying which block via argument:
rosservice call /trina2_1/trina_pick "pick_obj: 'unit_box_2'" 

If the service returns without an error, TRINA will be "holding" (collision object attached in RViz, box attached in Gazebo) the block.

Currently, the pick service always terminates in an error state - future work to correct this will be needed.

To recover from a pick error, call the recovery service, specifying whether to continue (true) or cancel (false) via argument:
rosservice call /trina2_1/trina_recover "cont: true"

Call place service with (x,y) coordinates on tabletop:
rosservice call /trina2_1/trina_place "x: 0.7 y: 0.0"
  
(x,y) are in TRINA's base link frame.

### Demo to stack (more or less) three blocks 

Run the demo via:
rosrun pick_and_place pnp_demo

TRINA will attempt to pick and place all three blocks to make a stack.  Sometimes it works!  You can watch the communications with the trajectory planner and the state machine in the console during execution.

