# Kinova Interface

You can import the interface at `src/gazebo_kinova_interface/nodes/interface/kinova_interface.py`. 

You should first launch the gazebo simulator via
```sh
roslaunch gazebo_kinova_interface jaco_gazebo.launch
# or roslaunch gazebo_kinova_interface jaco_gazebo.launch rqt:=true
# where rqt provides a GUI for testing applying force at each joint
```

Then, the jaco arm can be control via
```python
interface = KinovaInterface()

interface.send_joint_effort([20, 30.5, 0, 0, None, None])
```
and joint state can be queried via
```python
joint_angles, joint_velocities = interface.get_current_state()
```

