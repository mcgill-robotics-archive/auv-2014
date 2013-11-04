McGill RoboSub 2013-2014
===================

<h3>Installed OS & ROS</h3>
Ubuntu 12.04 LTS<br>
ROS Hydro

<h3>How to set up ROS</h3>
<ol>
	<li>Install ROS following <a href="http://wiki.ros.org/hydro/Installation/Ubuntu">Installation Guide</a></li>
	<li>Follow the required steps to set up <a href="http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment">Environment Variables</a></li>
	<li>
		Edit ~/.bashrc by adding following lines:<br>
			<pre><code>
				export ROS_IP=127.0.0.1
				export ROS_HOSTNAME=your_machine_name
			</code></pre>
	</li>
	<li>
		To add ROS workspaces, open ~/.bashrc and add path names under <i>"ROS_PACKAGE_PATH"</i><br>
		If there is no <i>"ROS_PACKAGE_PATH"</i>, add following line first:
		<pre><code>
			export ROS_PACKAGE_PATH=/opt/ros/hydro/stacks:/opt/ros/hydro/share
		</code></pre>
	</li>
</ol>

<h3> How to use/develop mock_data and front_end packages </h3>
<ol>
	<li>in your favorite folder: git clone git@github.com:McGill-Robotics/McGill_RoboSub_2014.git</li>
	<li>cd McGill_RoboSub_2014</li>
	<li>cd catkin_ws</li>
	<li>catkin_make</li>
	<li>source devel/setup.bash (this command needs to be run in all terminal windows/tabs, or put it in ~/.bashrc</li>
	<li>roscore</li>
	<li>rosrun mock_data posePublisher</li>
	<li>rosrun front_end poseSubscriber</li>
</ol>
