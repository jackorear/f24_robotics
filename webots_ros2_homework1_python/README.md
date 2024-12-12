# Running Ros2 Controller

1. Open a new terminal and source global ROS environment variables
<pre>
source /opt/ros/humble/setup.bash
</pre>

2. Navigate to the directory webots_ros2_homework1_python
<pre>
cd .../f24_robotics/webots_ros2_homework1_python
</pre>

3. Build and source the controller package
<pre>
colcon build
source install/setup.bash
</pre>

4. Launch the standalone controller node
<pre>
ros2 launch webots_ros2_homework1_python controller_launch.py
</pre>
