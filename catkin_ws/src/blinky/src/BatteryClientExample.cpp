#include "ros/ros.h"
#include <cstdlib>
#include <vector>

// Include the required service and message headers
#include <blinky/UpdateBatteryLights.h>
#include <blinky/RGB.h>

// Send RGB vector colors to the battery segment
int8_t Battery_updateColors(const std::vector<blinky::RGB>& colors, ros::NodeHandle n)
{
    // Get access to the blinky server for the UpdateBatteryLights service
    ros::ServiceClient client = n.serviceClient<blinky::UpdateBatteryLights>("update_battery_lights");

    // The service request for UpdateBatteryLights service
    blinky::UpdateBatteryLights srv;
    srv.request.colors = colors;

    // Call the service with that request
    if (client.call(srv)) {
    	int8_t res = srv.response.success; // Get service response

    	if (res) {
    	    ROS_INFO("UpdateBatteryLights request unsuccessful: %d", res);
	}

	return res;
    } else {
	ROS_ERROR("Failed to call service UpdateBatteryLights");
	return -1;
    }

    return 0;
}


// Just for convenience. This returns an RGB color with r, g and b components
blinky::RGB createRGB(uint8_t r, uint8_t g, uint8_t b)
{
    blinky::RGB color;
    color.r = r;
    color.g = g;
    color.b = b;

    return color;
}

// Just an example. Returns array of colors representing n in binary.
// 1 is a random color and 0 is turning the led off.
std::vector<blinky::RGB> generate_colors(unsigned int n)
{
    unsigned int t = n;
    std::vector<blinky::RGB> colors;

    for (int i = 0; i < 30; i++) {
        if (n % 2 == 0)
            colors.push_back(createRGB(0,0,0));
        else
            colors.push_back(createRGB(rand() % 256, rand() % 256, rand() % 256));    
        
        n /= 2;
    }

    return colors;
}

int main(int argc, char **argv)
{
    unsigned int i = 0;
    ros::init(argc, argv, "BatteryClientExample");
    ros::NodeHandle n;

    srand(time(0));

    while (true) {
        Battery_updateColors(generate_colors(i), n);
        ++i;
        usleep(100000);
    }

    return 0;
}
