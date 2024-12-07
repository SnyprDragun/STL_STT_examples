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
* For more insight into the controller design for STTs, follow (no link) <!-- the publication listed [here](https://github.com/SnyprDragun/STL_SpatiotemporalTubes_Toolbox/#related-publication) -->

## Setup
* you can find the Mechanum Omnibot setup [here]()
* you can find the Quadcopter setup [here](https://github.com/SnyprDragun/PX4-MAVROS-Simulation-Setup).
* clone this repo to the `src` folder of your workspace. Subsequent sections walk you through both the `cpp` and `python` implementations.

## Mechanum Omnibot
-----
## Quadcopter
### Python
* modify the `STT_Controller.py` file:
  * scroll down to the bottom and uncomment any one set of coefficients depending upon your robot (Mechanum Omnibot / Quadcopter).
  * make sure that you are calling the correct function of the class `STT_Controller()`. It should look like:
    * `STT_Controller(C, 2, start_time, end_time).omnibot_control()` for Mechanum Omnibot.
    * `STT_Controller(C, 3, start_time, end_time).uav_control()` for Quadcopter.
    * second argument of the class is the dimension of the tubes.
* Launch `MAVROS` environment:
  * For simulation, run `roslaunch px4 mavros_posix_sitl.launch` and then initialize takeoff using `>px4 commander takeoff`. You can also use the `cpp` files for purely takeoff purposes.
  * For hardware testing, run `roslaunch mavros px4.launch` instead, and takeoff using `cpp` scripts. 
* Once takeoff is complete, you can directly run `STT_Controller.py`. This initializes the controller for uav motion.
* After the tube ends, the uav lands automatically.
* Wait for a few minutes for the plots to be generated of the motion. You can retune the parameters based on the results obtained.

### Cpp
* Launch `MAVROS` environment same as before:
  * Simulation: `roslaunch px4 mavros_posix_sitl.launch`
  * Hardware: `roslaunch mavros px4.launch`
* Initialize navigation by running `rosrun dd_stl_stt root_controller`.
* `root_controller` takes care of takeoff, following STT using the controller, and finally landing once tube ends.
* If you want uav to stay still once tube ends, just comment out the last few lines in `root_controller` and build the package.

### Motion Capture (Mocap) Setup
This section covers steps to fly your drone in indoor environments, using some kind of mocap setup. In our case, we have used PhaseSpace Active Markers Motion Capture System. The requirements and setup for making your drone compatible with mocap, are mentioned below:
* The way to go about this can be brocken down into following chunks
  * Disable external GPS dependency of pixhawk - For this, connect you pixhawk to a PC and open QGroundControl. Go to parameters and modify the following: [source](https://ardupilot.org/copter/docs/common-optitrack.html)
    * set `AHRS_EKF_TYPE` to `3` , `EK3_ENABLE` to `1` and `EK2_ENABLE` to `0`
    * set `COMPASS_USE`, `COMPASS_USE2`, `COMPASS_USE3` to `0`. It prevents ArduPilot from using compass, because there are many sources causing electromagnetic interference in indoor environment.
    * set `VISO_TYPE` to `1`
    * set `VISO_POS_M_NSE` to `0.3` or lower to increase the weighting of position measurements from motion capture system.
    * set `VISO_YAW_M_NSE` to `0.2` or lower
    * set `EK3_SRC1_POSXY` to `6`
    * set `EK3_SRC1_POSZ` to `6`
    * set `EK3_SRC1_YAW` to `6`
    * set `EK3_SRC1_VELXY` to `0`
    * set `EK3_SRC1_VELZ` to `0`
  * Enable the `fake_gps` plugin on mavros for mocap compatibility. Navigate to `~/catkin_w_s/src/mavros/mavros_extras/src/plugins` and open `mocap_pose_estimate.cpp`. Now depending on the type of feedback you are expecting/receiving from your mocap system (`PoseStamped` position and orientation data or `TransformStamped` tf data), change lines 47-56.
    * `PoseStamped` feedback - set `use_pose` as `true` in line 50 and set `/mavros/mocap/pose` as topic in line 56. (our case)
    * `TransformStamped` feedback - set `use_tf` as `true` in line 47 and set `/mavros/fake_gps/mocap/tf` as topic in line 53.
  * Run `roslaunch phasespace_bringup phasespace_mocap.launch` on mocap PC.
  * Create mocap environment on jetson either by running `export ROS_MASTER_URI="http://<mocap_pc_ip>:11311"` or just putting it in `~/.bashrc`
  * Remap the mocap data to mavros as feedback for pose and substitute for GPS. 
    * first get Phasespace feedback on rostopics by cloning [this repo]() into your workspace.
    * Next run the `phasespace_mocap.py` file to remap mocap data from phasespace rostopic to mavros mocap topics.
    * Now you should see something like this when you launch `px4.launch`:
      * `[ INFO] [1733224666.962037564]: FCU: EKF3 IMU0 is using external nav data`
      * `[ INFO] [1733224666.962291735]: FCU: EKF3 IMU0 initial pos NED = 0.8,-0.3,-0.1 (m)` (coordinates might differ)
      * Also observe `FCU: PreArm: VisOdom: not healthy` dissappears.
    * Order of commands is important.
  * Finally, to test STT, run `rosrun dd_stl_stt STT_Mocap.py`






