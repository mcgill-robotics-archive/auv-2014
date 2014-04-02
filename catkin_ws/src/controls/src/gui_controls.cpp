#include "gui_controls.h" //should I make a different header?

/* 
GUI for controls gains
Makes sliders for each gain appear with integer and decimal control
Created by Matt Mayers Apr 2
*/

int main(int argc, char **argv)
{
	//Create ROS Node
	ros::init(argc,argv,"gui_controls");
	ros::NodeHandle n;

	float kp_xPos;
	float ki_xPos;
	float kd_xPos;
	float kp_yPos;
	float ki_yPos;
	float kd_yPos;
	float kp_Depth;
	float ki_Depth;
	float kd_Depth;
	float kp_Yaw;
	float ki_Yaw;
	float kd_Yaw;
	float kp_Pitch;
	float ki_Pitch;
	float kd_Pitch;


		// Variables used for the trackbars window.
		const bool isUsingControlTrackbarWindow_xPos = true;
		const std::string CONTROL_TRACKBARS_WINDOW_XPOS = "control_trackbars_window_xpos";

		const bool isUsingControlTrackbarWindow_yPos = false;
		const std::string CONTROL_TRACKBARS_WINDOW_YPOS = "control_trackbars_window_ypos";

		const bool isUsingControlTrackbarWindow_Depth = false;
		const std::string CONTROL_TRACKBARS_WINDOW_DEPTH = "control_trackbars_window_depth";

		const bool isUsingControlTrackbarWindow_Yaw = true;
		const std::string CONTROL_TRACKBARS_WINDOW_YAW = "control_trackbars_window_yaw";

		const bool isUsingControlTrackbarWindow_Pitch = true;
		const std::string CONTROL_TRACKBARS_WINDOW_PITCH = "control_trackbars_window_pitch";

		int kp_xPos_dec = 0;
	    int kp_xPos_int = 5;
	    int ki_xPos_dec = 0;
	    int ki_xPos_int = 5;
	    int kd_xPos_dec = 0;
	    int kd_xPos_int = 5;

	    int kp_yPos_dec = 0;
	    int kp_yPos_int = 5;
	    int ki_yPos_dec = 0;
	    int ki_yPos_int = 5;
	    int kd_yPos_dec = 0;
	    int kd_yPos_int = 5;

	    int kp_Depth_dec = 0;
	    int kp_Depth_int = 5;
	    int ki_Depth_dec = 0;
	    int ki_Depth_int = 5;
	    int kd_Depth_dec = 0;
	    int kd_Depth_int = 5;

	    int kp_Yaw_dec = 0;
	    int kp_Yaw_int = 5;
	    int ki_Yaw_dec = 0;
	    int ki_Yaw_int = 5;
	    int kd_Yaw_dec = 0;
	    int kd_Yaw_int = 5;

	    int kp_Pitch_dec = 0;
	    int kp_Pitch_int = 5;
	    int ki_Pitch_dec = 0;
	    int ki_Pitch_int = 5;
	    int kd_Pitch_dec = 0;
	    int kd_Pitch_int = 5;



 // Adds the variables that you want to modify with the trackbars.

	    if (isUsingControlTrackbarWindow_xPos) {
	    	// Instantiates the window object used for the trackbars.
	    	cv::namedWindow(CONTROL_TRACKBARS_WINDOW_XPOS, CV_WINDOW_KEEPRATIO);
	    	ROS_INFO("Controls::The trackbars window was created.");

	    	ROS_INFO("Controls::Adding trackbars to the trackbar window.");
	    	cv::createTrackbar("kp_xPos_dec", CONTROL_TRACKBARS_WINDOW_XPOS, &kp_xPos_dec, 10);
	    	cv::createTrackbar("kp_xPos_int", CONTROL_TRACKBARS_WINDOW_XPOS, &kp_xPos_int, 25);
	    	cv::createTrackbar("ki_xPos_dec", CONTROL_TRACKBARS_WINDOW_XPOS, &ki_xPos_dec, 10);
	    	cv::createTrackbar("ki_xPos_int", CONTROL_TRACKBARS_WINDOW_XPOS, &ki_xPos_int, 25);
	    	cv::createTrackbar("kd_xPos_dec", CONTROL_TRACKBARS_WINDOW_XPOS, &kd_xPos_dec, 10);
	    	cv::createTrackbar("kd_xPos_int", CONTROL_TRACKBARS_WINDOW_XPOS, &kd_xPos_int, 25);

	    }

		if (isUsingControlTrackbarWindow_yPos) {
	    	// Instantiates the window object used for the trackbars.
	    	cv::namedWindow(CONTROL_TRACKBARS_WINDOW_YPOS, CV_WINDOW_KEEPRATIO);
	    	ROS_INFO("Controls::The trackbars window was created.");

	    	ROS_INFO("Controls::Adding trackbars to the trackbar window.");
	    	cv::createTrackbar("kp_yPos_dec", CONTROL_TRACKBARS_WINDOW_YPOS, &kp_yPos_dec, 10);
	    	cv::createTrackbar("kp_yPos_int", CONTROL_TRACKBARS_WINDOW_YPOS, &kp_yPos_int, 25);
	    	cv::createTrackbar("ki_yPos_dec", CONTROL_TRACKBARS_WINDOW_YPOS, &ki_yPos_dec, 10);
	    	cv::createTrackbar("ki_yPos_int", CONTROL_TRACKBARS_WINDOW_YPOS, &ki_yPos_int, 25);
	    	cv::createTrackbar("kd_yPos_dec", CONTROL_TRACKBARS_WINDOW_YPOS, &kd_yPos_dec, 10);
	    	cv::createTrackbar("kd_yPos_int", CONTROL_TRACKBARS_WINDOW_YPOS, &kd_yPos_int, 25);

	    }	    

	    if (isUsingControlTrackbarWindow_Depth) {
	    	// Instantiates the window object used for the trackbars.
	    	cv::namedWindow(CONTROL_TRACKBARS_WINDOW_DEPTH, CV_WINDOW_KEEPRATIO);
	    	ROS_INFO("Controls::The trackbars window was created.");

	    	ROS_INFO("Controls::Adding trackbars to the trackbar window.");
	    	cv::createTrackbar("kp_Depth_dec", CONTROL_TRACKBARS_WINDOW_DEPTH, &kp_Depth_dec, 10);
	    	cv::createTrackbar("kp_Depth_int", CONTROL_TRACKBARS_WINDOW_DEPTH, &kp_Depth_int, 25);
	    	cv::createTrackbar("ki_Depth_dec", CONTROL_TRACKBARS_WINDOW_DEPTH, &ki_Depth_dec, 10);
	    	cv::createTrackbar("ki_Depth_int", CONTROL_TRACKBARS_WINDOW_DEPTH, &ki_Depth_int, 25);
	    	cv::createTrackbar("kd_Depth_dec", CONTROL_TRACKBARS_WINDOW_DEPTH, &kd_Depth_dec, 10);
	    	cv::createTrackbar("kd_Depth_int", CONTROL_TRACKBARS_WINDOW_DEPTH, &kd_Depth_int, 25);

	    }

	    if (isUsingControlTrackbarWindow_Yaw) {
	    	// Instantiates the window object used for the trackbars.
	    	cv::namedWindow(CONTROL_TRACKBARS_WINDOW_YAW, CV_WINDOW_KEEPRATIO);
	    	ROS_INFO("Controls::The trackbars window was created.");

	    	ROS_INFO("Controls::Adding trackbars to the trackbar window.");
	    	cv::createTrackbar("kp_Yaw_dec", CONTROL_TRACKBARS_WINDOW_YAW, &kp_Yaw_dec, 10);
	    	cv::createTrackbar("kp_Yaw_int", CONTROL_TRACKBARS_WINDOW_YAW, &kp_Yaw_int, 25);
	    	cv::createTrackbar("ki_Yaw_dec", CONTROL_TRACKBARS_WINDOW_YAW, &ki_Yaw_dec, 10);
	    	cv::createTrackbar("ki_Yaw_int", CONTROL_TRACKBARS_WINDOW_YAW, &ki_Yaw_int, 25);
	    	cv::createTrackbar("kd_Yaw_dec", CONTROL_TRACKBARS_WINDOW_YAW, &kd_Yaw_dec, 10);
	    	cv::createTrackbar("kd_Yaw_int", CONTROL_TRACKBARS_WINDOW_YAW, &kd_Yaw_int, 25);

	    }

	    if (isUsingControlTrackbarWindow_Pitch) {
	    	// Instantiates the window object used for the trackbars.
	    	cv::namedWindow(CONTROL_TRACKBARS_WINDOW_PITCH, CV_WINDOW_KEEPRATIO);
	    	ROS_INFO("Controls::The trackbars window was created.");

	    	ROS_INFO("Controls::Adding trackbars to the trackbar window.");
	    	cv::createTrackbar("kp_Pitch_dec", CONTROL_TRACKBARS_WINDOW_PITCH, &kp_Pitch_dec, 10);
	    	cv::createTrackbar("kp_Pitch_int", CONTROL_TRACKBARS_WINDOW_PITCH, &kp_Pitch_int, 25);
	    	cv::createTrackbar("ki_Pitch_dec", CONTROL_TRACKBARS_WINDOW_PITCH, &ki_Pitch_dec, 10);
	    	cv::createTrackbar("ki_Pitch_int", CONTROL_TRACKBARS_WINDOW_PITCH, &ki_Pitch_int, 25);
	    	cv::createTrackbar("kd_Pitch_dec", CONTROL_TRACKBARS_WINDOW_PITCH, &kd_Pitch_dec, 10);
	    	cv::createTrackbar("kd_Pitch_int", CONTROL_TRACKBARS_WINDOW_PITCH, &kd_Pitch_int, 25);

	    }
	    ROS_INFO("Controls::The window should be instantiated.");


	while(ros::ok())
	{
		ros::spinOnce();	//Updates all variables

	    kp_xPos = kp_xPos_int + kp_xPos_dec/10;
	    ki_xPos = ki_xPos_int + ki_xPos_dec/10;
	    kd_xPos = kd_xPos_int + kd_xPos_dec/10;


	    kp_yPos = kp_yPos_int + kp_yPos_dec/10;
	    ki_yPos = ki_yPos_int + ki_yPos_dec/10;
	    kd_yPos = kd_yPos_int + kd_yPos_dec/10;


	    kp_Depth = kp_Depth_int + kp_Depth_dec/10;
	    ki_Depth = ki_Depth_int + ki_Depth_dec/10;
	    kd_Depth = kd_Depth_int + kd_Depth_dec/10;

	    kp_Yaw = kp_Yaw_int + kp_Yaw_dec/10;
	    ki_Yaw = ki_Yaw_int + ki_Yaw_dec/10;
	    kd_Yaw = kd_Yaw_int + kd_Yaw_dec/10;

	    kp_Pitch = kp_Pitch_int + kp_Pitch_dec/10;
	    ki_Pitch = ki_Pitch_int + ki_Pitch_dec/10;
	    kd_Pitch = kd_Pitch_int + kd_Pitch_dec/10;

	    ROS_INFO(("Controls::kp_xPos value " + boost::lexical_cast<std::string>(kp_xPos)).c_str());

	    //Refreshes the window.
	    if (isUsingControlTrackbarWindow_xPos) {
	    	//Wait 1 milisecond to listen to events from trackbars
	    	cv::waitKey(1);
	    }	



	    n.setParam("/gains/kp_xPos", kp_xPos);
	    n.setParam("/gains/ki_xPos", ki_xPos);
	    n.setParam("/gains/kd_xPos", kd_xPos);

	    n.setParam("/gains/kp_yPos", kp_yPos);
	    n.setParam("/gains/ki_yPos", ki_yPos);
	    n.setParam("/gains/kd_yPos", kd_yPos);	    


	    n.setParam("/gains/kp_Depth", kp_Depth);
	    n.setParam("/gains/ki_Depth", ki_Depth);
	    n.setParam("/gains/kd_Depth", kd_Depth);	    

	    n.setParam("/gains/kp_Yaw", kp_Yaw);
	    n.setParam("/gains/ki_Yaw", ki_Yaw);
	    n.setParam("/gains/kd_Yaw", kd_Yaw);	    

	    n.setParam("/gains/kp_Pitch", kp_Pitch);
	    n.setParam("/gains/ki_Pitch", ki_Pitch);
	    n.setParam("/gains/kd_Pitch", kd_Pitch);	    


		if (isUsingControlTrackbarWindow_xPos) {
			ROS_INFO("Freeing memory used by the trackbar window.");
			// Frees the memory used by the instantiated window.
			cv::destroyWindow(CONTROL_TRACKBARS_WINDOW_XPOS);
		}

		if (isUsingControlTrackbarWindow_yPos) {
			ROS_INFO("Freeing memory used by the trackbar window.");
			// Frees the memory used by the instantiated window.
			cv::destroyWindow(CONTROL_TRACKBARS_WINDOW_YPOS);
		}

		if (isUsingControlTrackbarWindow_Depth) {
			ROS_INFO("Freeing memory used by the trackbar window.");
			// Frees the memory used by the instantiated window.
			cv::destroyWindow(CONTROL_TRACKBARS_WINDOW_DEPTH);
		}

		if (isUsingControlTrackbarWindow_Yaw) {
			ROS_INFO("Freeing memory used by the trackbar window.");
			// Frees the memory used by the instantiated window.
			cv::destroyWindow(CONTROL_TRACKBARS_WINDOW_YAW);
		}

		if (isUsingControlTrackbarWindow_Pitch) {
			ROS_INFO("Freeing memory used by the trackbar window.");
			// Frees the memory used by the instantiated window.
			cv::destroyWindow(CONTROL_TRACKBARS_WINDOW_PITCH);
		}

		
	}
	return 0;
}