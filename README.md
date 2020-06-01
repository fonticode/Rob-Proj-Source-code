
#### Final Project: Hector Quadrotor Package Exploration
#### Authors: Ivan Fontana and Severin Husmann
#### Group D

In order to reproduce the results in the report, you can find 
the controllers, the maps as well the corresponding launch files
in the corresponding folder on GitHub. 

In the following list, you can find what to do when in order to get everything
set up and running. 

1. Install the hector quad rotor package as described in the report and
in the tutorial.
2. Copy the controllers into hector_quadrotor/hector_quadrotor_teleop
3. Copy the worlds into hector_gazebo/hector_gazebo_worlds/worlds
4. Copy the experiment.launch file into hector_gazebo/hector_gazebo_worlds/launch
5. Change name of desired world to be launched in experiment.launch
5. Change the world launch file name in hector_quadrotor/hector_quadrotor_demo/launch/
indoor_slam_gazebo.launch to experiments.launch
6. Launch with the following command: roslaunch hector_quadrotor_demo indoor_slam_gazebo.launch
7. Launch the desired controller with: rosrun hector_quadrotor_teleop desired_controller_name.launch

Note: The action_controller consists of a short path with x and y coordinate goals to demonstarte the client server relationship and if desired, should be launched in the willow garage or an empty world.
Finally, the presentation video can be found under: https://usi365-my.sharepoint.com/:v:/g/personal/husmas_usi_ch/EXJha9fj5ghAvriOWPftvvQBhn4kRJj-5oVPcDqtO7Bbww?e=KouKPq
