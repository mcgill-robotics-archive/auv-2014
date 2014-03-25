#include "ros/ros.h"
#include <cstdlib>
#include <vector>

#include <blinky/BlinkyTapeService.h>
#include <blinky/RGB.h>
#include "BlinkyAPI.h"

const int PLANNER_ID = 1;
const int BATTERY_ID = 2;

BlinkyClient::BlinkyClient(ros::NodeHandle node_handle)
{
        n = node_handle;
}

int8_t BlinkyClient::send_colorList(const std::vector<blinky::RGB>& colors, int8_t blinkyID)
{
	ros::ServiceClient client = n.serviceClient<blinky::BlinkyTapeService>("BlinkyDisplay");

	blinky::BlinkyTapeService srv;
	srv.request.btColors = colors;
	srv.request.blinkyID = blinkyID;

	if (client.call(srv)) {
		int8_t res = srv.response.success;

		if (res) {
			ROS_INFO("BlinkyDisplay request unsuccessful: %d", res);
		}

		return res;
	} else {
		ROS_ERROR("Failed to call service BlinkyDisplay");
		return -1;
	}

	return 0;
}

int8_t BlinkyClient::Planner_sendColors(const std::vector<blinky::RGB>& colors)
{
	return send_colorList(colors, PLANNER_ID);
}

int8_t BlinkyClient::Battery_sendColors(const std::vector<blinky::RGB>& colors)
{
	return send_colorList(colors, BATTERY_ID);
}
