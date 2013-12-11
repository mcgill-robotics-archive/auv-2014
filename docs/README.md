<h1>How to set up ROS</h1>
Last Edited Dec 11, 2013, Bei Liu

<h4>Install ROS following Installation Guide and skip part 1.6:</h4>
http://wiki.ros.org/hydro/Installation/Ubuntu
	
<h4>Pull the McGill_RoboSub_2014 from github if you have not done so:</h4>
In terminal run the following command:
<pre><code>sudo apt-get install git
cd ~
mkdir McGill_RoboSub_2014
cd McGill_RoboSub_2014
git init
git remote add origin "https://github.com/McGill-Robotics/McGill_RoboSub_2014.git" 
git pull origin master
</code></pre>
		
	
<h4>Run the Symlink.sh in the dotfiles folder.</h4>
<p>This will copy configuration file for tmux and vim; copy roboticrc to your home directory and source it in .bashrc.</p><p>(You don't to read the rest if you don't want to).</p><p>The roboticrc will have several functions:</p> 
<ol>
<li>add some alias that are somewhat useful (ask Anass), you can add your own alias to this list; </li>
<li>sources /opt/ros/hydro/setup.bash so you can use ros command directly on any new terminal windows; </li>
<li>sources /catkin_ws/devel/setup.bash, this enables you to use the package that we have created (if you get error on opening terminal, be sure to run catkin_make in your catkin_ws);</li>
<li>export ROS_PATH and export ROS_WORKSPACE which are the ros environment variables. </li></ol>
