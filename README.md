# USV controller

This repository is dedicated to the vereniki platform model used in 
[usv_simulation](https://github.com/g-poulos/usv_simulation). It contains 
an open-loop and a proportional controller and the simulation of the platform 
dynamic model. 

## Control

For both controllers to work, the usv_simulation needs to be started from the
launch file for the GZ and ROS topics to be bridged.

### Source the ROS installation

```
cd ~/ros2_ws/src/usv_controller
source ../../install/local_setup.bash
```

### Open-loop Controller 

Using the kinematics of the platform its movement can be controlled by defining the 
3x1 vector: $q_c = [f_x, f_y, n_z]^T$

The first two elements denote force (N) along the x and y axis of the platform
body-fixed frame. The third element denotes torque (Nm) about the z-axis of the
same frame.

```
ros2 run usv_controller vereniki_controller --ros-args -p thrust:="100, 0, 0" -p cmd_type := direction_only
```

+ `thrust`: the control vector $q_c$ ([100, 0, 0] if not defined)
+ `cmd_type`: **thrust_only** or **direction_only** (both if not defined)

### P-Controller

The proportional controller takes as input a point and a desired orientation. The 
platform moves towards the target with linear and angular speed proportional of its 
distance. While the controller is running the platform maintains the target position
and orientation.

```
# ros2 run usv_controller vereniki_p_controller --ros-args -p target:= "x_des, y_des, theta_des"
```

## Gazebo Sim Recording

`bag_writer.py` contains o ROS node that records simulation data while the 
usv_simulation runs. The records are then stored in a .bag file. To create a record 
pause the simulation and start the node. When the simulation is resumed the node will 
start recording for the given number of seconds. 

```
ros2 run usv_controller bag_writer --ros-args -p duration:=120.0
```

+ `duration`: recording duration in seconds 

## Dynamic Model Simulation

The dynamic model of the platform is simulated using Euler's Method. All the scripts
for the simulation are in the `sim_scripts/dynamic_model` directory.

```
cd ~/ros2_ws/src/usv_controller/sim_scripts/dynamic_model
python diff_calc.py
```
The simulation is configured by defining the inputs for the `run_simulation` function
in the `diff_calc.py` main.

```python
run_simulation(np.array([100, 20, 40]),
               duration=2,
               p_control=False,
               dist=True,
               dist_file=None,
               plot=True)
```

+ `input_vector`: The input vector $q_c$
+ `duration`: duration of the simulation in minutes
+ `p_control`: If **True** then the input vector is used as input for the P controller
+ `dist`: If **True** wind and ocean current are simulated
+ `dist_file`: File (.bag) with recorded disturbances. The duration must match the 
               simulation duration. If **None** and `dist`=**True** then the disturbances are
               generated as defined in the `run_simulation` function.
+ `plot`: Plots the results if **True**

The simulation results are stored in 4 CSV files: 

+ `dynamic_model_out.csv`: The position, velocity and acceleration of the platform
+ `hydrodynamics.csv`: The sum of the hydrodynamic forces and torque
+ `disturbance.csv`: The sum of the forces and torque due to wind and ocean current
+ `thrust`: The input vector $q_c$ for every simulation iteration

*`hydrodynamics` and `disturbance` files contain forces along the x and y axis and 
torque about the z-axis*


## Simulation results comparison
