#include "ros/ros.h"
#include <cstdlib>
#include <vector>

// Include the required service and message headers
#include <blinky/UpdatePlannerLights.h>
#include <blinky/RGB.h>

// Send RGB vector colors to the planner segment
int8_t Planner_updateColors(const std::vector<blinky::RGB>& colors, ros::NodeHandle n)
{
    // Get access to the blinky server for the UpdatePlannerLights service
    ros::ServiceClient client = n.serviceClient<blinky::UpdatePlannerLights>("update_planner_lights");

    // The service request for UpdatePlannerLights service
    blinky::UpdatePlannerLights srv;
    srv.request.colors = colors;

    // Call the service with that request
    if (client.call(srv)) {
    	int8_t res = srv.response.success; // Get service response

    	if (res) {
    	    ROS_INFO("UpdatePlannerLights request unsuccessful: %d", res);
	}

	return res;
    } else {
	ROS_ERROR("Failed to call service UpdatePlannerLights");
	return -1;
    }

    return 0;
}

// Just for convenience, it returns an RGB with r, g and b components
blinky::RGB createRGB(uint8_t r, uint8_t g, uint8_t b)
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

    std::vector<blinky::RGB> colors;
    colors.push_back(createRGB(255,255,255));
    colors.push_back(createRGB(255,0,0));
    colors.push_back(createRGB(0,255,0));

    Planner_updateColors(colors, n);
    return 0;
}
