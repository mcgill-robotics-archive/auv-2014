How to set up ROS
Last Edited Dec 11, 2013, Bei Liu

Install ROS following Installation Guide and <>skip part 1.6:
	http://wiki.ros.org/hydro/Installation/Ubuntu
	
Pull the McGill_RoboSub_2014 from github if you have not done so:
	in terminal run the following command:
		sudo apt-get install git
		cd ~
		mkdir McGill_RoboSub_2014
		cd McGill_RoboSub_2014
		git init
		git remote add origin "https://github.com/McGill-Robotics/McGill_RoboSub_2014.git" 
		git pull origin master
		
	
Run the Symlink.sh in the dotfiles folder.
	this will copy roboticrc to your home directory and source it in .bashrc.  (You don't to read the rest if you don't want to)
	The roboticrc will have several functions: 
		It will add some alias that are somewhat useful (ask Anass), you can add your own alias to this list; 
		sources /opt/ros/hydro/setup.bash so you can use ros command directly on any new terminal windows; 
		sources /catkin_ws/devel/setup.bash, this enables you to use the package that we have created (if you get error on opening terminal, be sure to run catkin_make in you catkin_ws);
		export ROS_PATH and export ROS_WORKSPACE which are the ros environment variables. 
