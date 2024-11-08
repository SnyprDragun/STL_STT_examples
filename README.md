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

## Controller
* The controller is based on bounded control strategies and ensures two main things:
  * If inside the tube, stay inside the tube no matter what
  * If outside the tube, try to come inside asap.
* Tunable parameter `k` enables sharper turns for higher values. So If your STT has sharp turns, increment `k` to handle for them.
* Each dimension has its own tunable `k`.
* For more insight into the controller design for STTs, follow the publication listed [here](https://github.com/SnyprDragun/STL_SpatiotemporalTubes_Toolbox)

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
* Launch `MAVROS` environment:
  * For simulation, run `roslaunch px4 mavros_posix_sitl.launch` and then initialize takeoff using `>px4 commander takeoff`. You can also use the `cpp` files for purely takeoff purposes.
  * For hardware testing, run `roslaunch mavros px4.launch` instead, and takeoff using `cpp` scripts. 
* Once takeoff is complete, you can directly run `STT_Controller.py`. This initializes the controller for uav motion.
* After the tube ends, the uav lands automatically.
* Wait for a few minutes for the plots to be generated of the motion. You can retune the parameters based on the results obtained.

### Cpp
* Launch `MAVROS` environment same as before and initialize takeoff by running `rosrun <pkg name> root_controller`.
* [`cpp` executable is not yet released, pls follow `python` implementation or stay tuned.]

-----
