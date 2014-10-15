# ROS Hydrophones Package

## Dependencies
This package requires the following libraries:
* pyaudio

## Launch
### Core
The hydrophones package can be run either in simulated audio or in real audio modes.
#### Real Audio
To run the package in real audio mode, run the following:
```
roslaunch hydrophones hydrophones.launch
```
#### Simulated Audio
To run the package in simulated audio mode, run the following:
```
roslaunch hydrophones sim-hydrophones.launch
```
This creates new simulation parameters under `/hydrophones/sim` which can be manipulated in real-time.

## Monitoring
To monitor the hydrophones package in real-time in a Textual User Interface (TUI) run the following:
```
roslaunch hydrophones visualizer.launch
```
## Nodes
The hydrophones package can spawn the following nodes:
### Core
The following nodes run regardless of the mode.
#### Parameter Manager
Wrapper to get all up to date parameters.
N.B. This node does not stay up for long.
Node name: /hydrophones/param
Topics: None
Subscribes to: None
#### Audio
Deals with acquiring audio data and publishing it.
Node name: /hydrophones/audio
Topics: /hydrophones/audio
Subscribes to: None
#### TDOA
Analyzes the incoming audio data and measures TDOAs if necessary.
Node name: /hydrophones/tdoa
Topics: /hydrophones/tdoa
Subscribes to: /hydrophones/audio
#### Solver
Estimates the location of the pinger given TDOAs.
Node name: /hydrophones/solver
Topics: /hydrophones/sol
Subscribes to: /hydrophones/tdoa

## MESSAGES
This package contains the following custom messages:
### channels
Timestamped four-channel buffer of audio data.
```
float32[] channel_0
float32[] channel_1
float32[] channel_2
float32[] channel_3
time      stamp
```
### tdoa
Time difference of arrival and whether this is the target pinger or not.
```
float64 tdoa_1
float64 tdoa_2
float64 tdoa_3
bool    target
```
### cartesian
Two-dimensional cartesian coordinates.
```
float64 x
float64 y
```
### polar
Polar coordinates.
```
float64 r
float64 theta
```
### solution
Pinger solution containing cartesian and polar coordinates and whether this is the target pinger.
```
cartesian   cartesian
polar       polar
bool        target
```
### freq
Frequency domain of channels.
```
complex channel_0
complex channel_1
complex channel_2
complex channel_3
```
### peaks
Peak frequency per channel.
```
float64 channel_0
float64 channel_1
float64 channel_2
float64 channel_3
```
### complex
Array of complex numbers.
```
float64[] real
float64[] imag
```
