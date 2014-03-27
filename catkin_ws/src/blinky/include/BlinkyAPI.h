#ifndef BLINKYAPI_H
#define BLINKYAPI_H

#include <vector>
#include <blinky/RGB.h>

class BlinkyClient {

public:
    BlinkyClient(ros::NodeHandle n);
    int8_t Planner_sendColors(const std::vector<blinky::RGB>& colors);
    int8_t Battery_sendColors(const std::vector<blinky::RGB>& colors);

private:
    ros::NodeHandle n;

    int8_t send_colorList(const std::vector<blinky::RGB>& colors, int8_t blinkyID);
};

#endif
