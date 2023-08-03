<div align="center">
  <h1>Error-State Kalman Filter Based External Wrench Estimation for MAVs under a Cascaded Architecture</h1>
  <br>
  <img src="./assets/roped_flight.gif" width="700">
  <br>
</div>

## Quick Start

The project has been tested on Ubuntu 20.04(ROS Noetic).

### Prerequisites

Install `Eigen3`

```bash
sudo apt install libeigen3-dev
```

Install `yaml-cpp`

```bash
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build && cd build
cmake .. && make
sudo make install
```

Install `PlotJuggler` for visualization

```bash
sudo apt install ros-noetic-plotjuggler ros-noetic-plotjuggler-ros
```

### Build on ROS

```bash
cd ${YOUR_WORKSPACE_PATH}/src
git clone https://github.com/helloliver/ewe4mav.git
cd ${YOUR_WORKSPACE_PATH}
catkin_make
```

### RUN !

```bash
cd ${YOUR_WORKSPACE_PATH}
source devel/setup.bash
roslaunch ewe4mav roped_flight.launch
```

### Visualization

The estimation result can be visualized using `PlotJuggler`.
Open a new terminal, go to your workspace and run

```bash
rosrun plotjuggler plotjuggler -l src/ewe4mav/config/plot.xml
```

* Q: Start the previously used streaming plugin? ROS Topic Subscriber
  * A: SELECT `YES`
* Q: Select one or multiple topics
  * A: `CTRL+A`, then SELECT `OK`

## Licence

The source code is released under [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0) license.
