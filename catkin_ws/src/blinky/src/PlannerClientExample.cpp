#include <vector>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "BlinkyAPI.h"

blinky::RGB createRGB(int8_t r, int8_t g, int8_t b)
{
        blinky::RGB color;
        color.r = r;
        color.g = g;
        color.b = b;

        return color;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PlannerClientExample");
        ros::NodeHandle n;

        BlinkyClient btClient = BlinkyClient(n);

 	std::vector<blinky::RGB> colors;
        colors.push_back(createRGB(255,255,255));
        colors.push_back(createRGB(255,0,0));
        colors.push_back(createRGB(0,255,0));

        btClient.Planner_sendColors(colors);
	return 0;
}
