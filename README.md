# Fetch-RIZE-example (ROS melodic)

### Step 1.- Launch RIZE (inside RIZE folder):

`npm run start`

### Step 1.- Launch ROS bridge:

```
source /opt/ros/melodic/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

### Step 2.- Launch gazeboo Fetch robot simulation:

```
source /opt/ros/melodic/setup.bash
roslaunch fetch_gazebo simulation.launch
```

### Step 3.- Launch moveit Fetch:

```
source /opt/ros/melodic/setup.bash
roslaunch fetch_moveit_config move_group.launch
```

### Step 4.- Execute Robot Action Engine (inside Fetch-RIZE-example folder)

`
python RobotExample.py 
`
