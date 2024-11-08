# STL STT Examples
* This repo is a spinoff to the [STL STT Toolbox](https://github.com/SnyprDragun/STL_SpatiotemporalTubes_Toolbox), focusing mainly on implementing a controller for robots to follow the STT for safety. Contents of this repo come in handy once you have the tubes and want to try it out on real world subject. There is documentation for both simulation and hardware setup to implement the tubes.
* Illustration consists of two cases:
  * [Mechanum Omnibot](#mechanum-omnibot)
  * [Quadcopter](#quadcopter)

## System Requirements
* Ubuntu 20.04 LTS
* ROS Noetic
* Python 3.8
* PyTorch

## Setup
* you can find the Mechanum Omnibot setup [here]()
* you can find the Quadcopter setup [here](https://github.com/SnyprDragun/PX4-MAVROS-Simulation-Setup).
* clone this repo to the `src` folder of your workspace. Subsequent sections walk you through both the `cpp` and `python` implementations.
* modify the `STT_Controller.py` file:
  * scroll down to the bottom and uncomment any one set of coefficients depending upon your robot (Mechanum Omnibot / Quadcopter).
  * make sure that you are calling the correct function of the class `STT_Controller()`. It should look like:
    * `STT_Controller(C, 2, start_time, end_time).omnibot_control()` for Mechanum Omnibot.
    * `STT_COntroller(C, 3, start_time, end_time).uav_control()` for Quadcopter.
    * second argument of the class is the dimension of the tubes.

## Mechanum Omnibot
-----
## Quadcopter
### Python
* Run `roslaunch px4 mavros_posix_sitl.launch` and then initialize takeoff using `>px4 commander takeoff`. You can also use the `cpp` files for purely takeoff purposes.
* Once takeoff is complete, you can directly run `STT_Controller.py`.
