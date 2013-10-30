McGill LunarEx 2013-2014
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
