# Ground Removal Laptesting

## How to build

Prerequisites:

- ROS Noetic http://wiki.ros.org/noetic/Installation
- PCL https://pointclouds.org/downloads/

1. Navigate to an existing catkin workspace or create a new one by running 'catkin_make' in a directory.
2. Clone this repo under the 'src' folder in your catkin workspace.
3. Run 'catkin_make' from the root directory of your catkin workspace.
4. Run 'source devel/local_setup.bash' from the root directory of your catkin workspace.
5. Run the 'run.bash' script in the repo, the program will read any data under /velodyne_points and play it.

(Be sure to source the setup scripts /opt/ros/noetic/local_setup.bash and <catkin_workspace>/devel/local_setup.bash in each new terminal instance)
