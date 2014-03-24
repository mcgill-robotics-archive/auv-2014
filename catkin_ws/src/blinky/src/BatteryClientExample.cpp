#include <vector>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "BlinkyAPI.h"

#include <ctime>
#include <cstdlib>
#include <unistd.h>

blinky::RGB createRGB(int8_t r, int8_t g, int8_t b)
{
        blinky::RGB color;
        color.r = r;
        color.g = g;
        color.b = b;

        return color;
}

std::vector<blinky::RGB> generate_colors(unsigned long int n)
{
        unsigned long int t = n;
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
	ros::init(argc, argv, "BatteryClientExample");
        ros::NodeHandle n;

        BlinkyClient btClient = BlinkyClient(n);

        srand(time(0));

        int i = 0;

        while (true) {
                btClient.Battery_sendColors(generate_colors(i));
                ++i;
                usleep(100000);
        }

	return 0;
}
