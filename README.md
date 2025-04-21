# Neura Technical Challenge Workspace

Workspace for Neura Technical Challenge. The simulation uses the neobotix ROX platform as mobile manipulator and is set up for ROS Jazzy with Modern Gazebo. This workspace uses the devcontainer feature from VS Code to provide an easy setup, but you can also setup the workspace manually. Check the Dockerfile in the .devcontainer folder for the required packages.

## Setup instructions

- Clone the workspace with all submodules:

```
git clone --recursive https://github.com/Eruvae/neura_task_ws.git
```

- Open the folder in VS Code. If you don't have it, install the Dev Containers extension
- VS Code should promt you to open the folder in the container. If not, press F1 and select "Dev Containers: Rebuild and Reopen in Container"
- The setup of the docker container and compilation of the workspace should happen automatically. When it is done, open new terminals to start the files for the task.

## Running the tasks

- First, launch the nodes required for simulation:
```
ros2 launch neura_task rox.launch.py
```
- Wait for Gazebo to fully load the the environment. It should look like this:
![Screenshot of the simulation environment with gazebo and rviz.](LINK_TODO)

- For Task 1, launch:
```
ros2 launch neura_task task1.launch.py
```
The robot will drive in a circle, with the arm swinging in a sine motion. You can try modifying the parameters in the launch file. The path for the base will be displayed as a green circle in rviz.

- For Task 2a, launch:
```
ros2 launch neura_task task2a.launch.py
```
or for different parameters:
```
ros2 launch neura_task task2a_with_base_move.launch.py
```
If the robots finds a cartesian path between the two poses, it will move the arm to the start position and execute the cartesian path. Otherwise, it will attempt to find a position for the robot base where a caresian path can be found, move there, and then execute the cartesian motion.

- For Task 2b, launch:
```
ros2 launch neura_task task2b_circle.launch.py
```
or
```
ros2 launch neura_task task2b_line.launch.py
```
In the first case, the robot will move in a circle like in task 1, while it tries to keep the arm in the center looking upwards.
In the second case, the robot will move between two points, while trying to keep the arm tool in the same point.
You can also try modifying the parameters.