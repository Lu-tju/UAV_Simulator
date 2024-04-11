# UAV Simulator

This repo is modified from https://github.com/HKUST-Aerial-Robotics/Fast-Planner

Use:
```bash
cd ${YOUR_WORKSPACE_PATH}/src
git clone https://github.com/Lu-tju/UAV_Simulator
cd ../
catkin_make
source devel/setup.bash
roslaunch so3_quadrotor_simulator simulator.launch
```

**updata:**
+ add the log recorder
+ fine-tuning PID