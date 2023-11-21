# Motion Planning with RRT Connect

## Problem
Consider a planar robot $R$ manipulating objects in a 2D workspace. The robot has four links $l_0, l_1, l_2, l_3$ ($l_0$ is the base link) with the assumption that each joint is in the range of $[-\pi/2,\pi/2]$. Each pose in the workspace is represented with the format $(x,y,\alpha)$. Based on the assumptions, the robot is tasked to find a motion sequence $\Pi$ moving a target object $T$ from a start pose $S_T$ to a goal pose $S_G$. Each motion in $\Pi$ is required to be collision-free with other boxes in the environment.

![problem](https://github.com/gaokai15/motion_planning_cpp/assets/53358252/5105f846-521b-404f-adb0-48fee6ccbe03)

## Get Started

First, install necessary libraries: (1) nlohmann-json for reading configuration files; (2) opencv4 for visualization.

```bash
$ sudo apt-get update
$ sudo apt-get install libopencv-dev
$ sudo apt-get install nlohmann-json3-dev
```

Second, change the parameters in the ``config.json'' file as needed. Current supported parameters include (1) the start and goal pose of the target object; (2) Poses of obstacles in the environment, (3) the length and width of a link; (4) the width and height of an obstacle/target object; (5) Clearance to obstacles.

![config](https://github.com/gaokai15/motion_planning_cpp/assets/53358252/a91a5501-d139-4f10-8689-4c144510dc69)

Third, build the project with "make" command.
```bash
$ cd /path/to/repo/root/
$ make
```

Fourth, run the executable with
```bash
$ ./motion_planning
```

Fifth, when the planner finds a path, it plots an image of the initial state.

![initial_state](https://github.com/gaokai15/motion_planning_cpp/assets/53358252/c09ae2b5-8ced-4a8e-961b-e513dcd965a1)


Sixth, press any key to start the animation. Some demo videos are attached in the zip file.

![animation](https://github.com/gaokai15/motion_planning_cpp/assets/53358252/387c8b91-7f32-44f2-a3c4-e953da7c35ed)


## Technical Details
### Basic Version
The motion plan is computed by RRT-connect. IK is chosen to be the analytical solution with $q_2\geq 0$.

### Clearance Version (lastest)
In the basic version, RRT state feasibility is evaluated with polygon intersections. In this version, I evaluate polygon distance instead. A state is feasible if and only if all the robot links and the target object is at least $d$ away from obstacles, where $d$ is the defined clearance.

## Video Explanation
1. "basic-version-easy.mp4". The basic version planner in an easy test case.
2. "basic-version-narrow.mp4". The basic version planner in a challenging test case.
3. "clearance-version-5cm.mp4" Clearance version implementation with clearance=5cm.
3. "clearance-version-10cm.mp4" Clearance version implementation with clearance=10cm.


