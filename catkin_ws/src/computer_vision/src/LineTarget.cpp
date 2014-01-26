/*
 * LineTarget.cpp
 *
 *  Created on: Jan 16, 2014
 *      Author: thuy-anh
 */
#include "LineTarget.h"

#include "cv.h"      // include it to used Main OpenCV functions.
#include "highgui.h" //include it to use GUI functions.
#include <math.h>
int kernelSize = 3;
Mat src; Mat src_gray;
cv::Mat filteredImage;
int thresh = 300;
int max_thresh = 500;
RNG rng(12345);

//int main(int argc, char** argv){
//
//	VideoCapture video("run2-cropped.avi");
//	executeVideo (video);
//}

double xDistance = 0;
double yDistance = 0;
double yaw = 0;
bool visibility = false;

std::vector<computer_vision::VisibleObjectData*> LineTarget::retrieveObjectData(cv::Mat& currentFrame) {
        bool isVisible;
        double yawAngle = -90.0;
        double pitchAngle = 90.0;
        //double xDistance = 1;
        //double yDistance = 3;
        double zDistance = 4;
        std::vector<computer_vision::VisibleObjectData*> messagesToReturn;

        // Creates a pointer that will point to a computer_vision::VisibleObjectData object.
        computer_vision::VisibleObjectData* visibleObjectData;

        // Apply filters on the cv::Mat object.
        applyFilter(currentFrame);

        // Check if door is visible

        if (visibility) {
                // Get object data
                // [...]

                // Return gathered data to caller
                visibleObjectData->object_type = visibleObjectData->LANE;
                visibleObjectData->pitch_angle = 0;
                visibleObjectData->yaw_angle = yaw;
                visibleObjectData->x_distance = xDistance;
                visibleObjectData->y_distance = yDistance;
                visibleObjectData->z_distance = 0;

                messagesToReturn.push_back(visibleObjectData);

                return (messagesToReturn);
        } else {
                return (messagesToReturn);
        }
}
/**
 * Applies filter so we can detect the line
 * @param image Current frame given from video
 */

void applyFilter(cv::Mat& image){
		cv::Mat imageHSV;
	   src = image;
      cv::cvtColor(image, imageHSV,CV_BGR2HSV);
      cv::GaussianBlur(filteredImage, filteredImage, cv::Size(kernelSize, kernelSize),0,0);
      cv::inRange(imageHSV, cv::Scalar(5, 70, 30), cv::Scalar(15,350,180), filteredImage);


  /// Create Window
  char* source_window = "Source";

  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  imshow( source_window, src );

  createTrackbar( "Canny thresh:", "Source", &thresh, max_thresh, thresh_callback );
  thresh_callback( 0, 0 );
}

//void executeVideo(cv::VideoCapture cap) {
//        while (1) {
//                cv::Mat currentFrame;
//                cv::Mat hsvCurrentFrame;
//
//                bool readCurrentFrame = cap.read(currentFrame);
//                if (!readCurrentFrame) {
//                        std::cout << "[INFO] Looping back to the first frame." << std::endl;
//                        cap.set(CV_CAP_PROP_POS_FRAMES, 35);
//                        readCurrentFrame = cap.read(currentFrame);
//                }
//
//                //apply fliter to current frame
//                applyFilter(currentFrame);
//
//                // Manages the frquency at which the frames are being updated. And closes the windows whenever ESC is pressed.
//                if (cv::waitKey(150) == 27) {
//                        std::cout << "[WARNING] The user has pressed ESC. Terminating the process" << std::endl;
//                        exit(EXIT_SUCCESS);
//                }
//        }
//}

void thresh_callback(int, void* )
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  cv::Mat img = cv::Mat::zeros(500, 500, CV_8UC3);


  /// Detect edges using canny
  Canny( filteredImage, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Draw contours
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );

	  Point2f vertices[4];
	  cv::RotatedRect line;
	  std::vector<std::vector<cv::Point> > contours_poly( contours.size() );

	  	  //evaluates all found lines and returns largest line line that fits ratio
		  for( int i = 0; i < contours.size(); i++ ) {
		      approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 4, true );
		      std::vector<cv::Point> hull;
		       cv::convexHull(cv::Mat(contours_poly[i]),hull);
		       cv::Mat hull_points(hull);
		       cv::RotatedRect current = minAreaRect(hull_points);

		       if(current.size.area()<350){
		           continue;
		       }

		       current.points(vertices);
		       if (current.size.area() > line.size.area()){
		        line = minAreaRect(hull_points);
		       }
		  }

		  //draws located line
		  line.points(vertices);
		  for (int i = 0; i < 4; ++i){
			  cv::line(drawing, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 1, CV_AA);
		  }
		  int centerX = line.center.x;
		  int centerY = line.center.y;

		  cv::line(drawing, Point(centerX, centerY), Point(centerX,centerY), cv::Scalar(0, 0, 255),2,8, 0 );
		  cv::line(drawing, Point(drawing.size().width/2, drawing.size().height/2), Point(drawing.size().width/2, drawing.size().height/2), cv::Scalar(255, 0, 0),2,8, 0 );
		  visibility = isVisible(line);
		  yaw = relativeYaw(line);

		  //If output is opposite, then invert
		  xDistance = drawing.size().width/2 - centerX;
 		  yDistance = drawing.size().height/2 - centerY;
 		  cout << "xDistance: " << xDistance << "\n";
 		  cout << "yDistance: " << yDistance << "\n";


  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );
}
/**
 * Determines distance of robot from line
 * @param line Line found using contours
 * @return Distance of roboto from line
 */
double distance (cv::RotatedRect line){
	return 0.;
}

/**
 * Determines Whether there is currently a visible line
 * @param line Line found using contours
 * @return Whether line is visible or not
 */
bool isVisible(cv::RotatedRect line){
	if (line.size.height != 0 || line.size.width !=0)
		return true;
	return false;
}

/**
 * Evaluates found line and returns relative yaw between the sub and the line
 * @param line Line found using contours
 * @return Relative yaw between the sub and the line
 */
double relativeYaw(cv::RotatedRect line){
	double angle = line.angle;
	//re-adjust incorrect angles
	if (line.size.height <  line.size.width){
		angle = angle+90;
	}
	cout << "Relative Yaw Is " << angle << endl;
	cout<< " -----------------------------------------------" << endl;
	return angle;
}




