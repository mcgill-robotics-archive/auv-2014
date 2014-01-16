killall roscore
killall python
killall xterm

. devel/setup.bash

catkin_make

sleep 2
echo "Done building catkin_ws"
echo "Starting roscore and simulator, then waiting 30 seconds to complete"
xterm -e roslaunch simulator simulator.launch &
sleep 30


echo "Starting Planner"
xterm -e rosrun planner Planner &

sleep 5
