A muti-uavs simulator, modified by Fast-Planner.

1. generate muti-uav launch file

```bash
python generate_launch.py
```

2. launch controller and simulator

```bash
catkin_make
source devel/setup.bash
roslaunch so3_quadrotor_simulator muti_simulator.launch
```

3. visualization

```bash
source devel/setup.bash
rviz -d src/swarm.rviz
```