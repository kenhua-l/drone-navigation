# Drone Navigation
This is the project to navigate a simple drone in a ROS simulation and the real world. This is currently a live documentation and the project is expected to complete by 10 April 2017.

## Requirements
In production, the code will run on Ubuntu (14.04) and ROS (indigo). You may use a different OS and ROS version for development but keep in mind the production will run on these two platforms.
The codes will also run in C++, thus a C++ compiler is needed (g++ is a good enough c++ compiler).

The following lists the dependencies needed on ROS that is either included in the full ROS installation or it can be installed independently on top of basic ROS.

1. Gazebo (which means you need CMake and stuffs)
2. hector_quadrotor (is provided in the files by the teaching team)

Because we are working on this project in agile mode, please have git installed as well.

## Development
There are two parts to the project.

1. Path Planning algorithm
2. Drone Trajectory

Consequently, in the spirit of modularity and separation of concern, there are two branches that each team should only focus on one. Only upon successful integration, the code can then be pushed into master.

### Getting Started
Regardless of team, let's get started first. Follow the instruction from the manual provided to create the workspace and the copying of source files (tutorial slide 24).
To save space, this repo should NOT have the source files (gazebo world and quadrotor) given by the instructor (Separation of concern). Instead, this repo lies as a separate folder in the src folder. Thus, you would need to git clone this repo into the src folder.

After catkin_make, go to uavWs/src.
```
git clone https://github.com/kenhua-l/drone-navigation.git
```

This is the ideal folder structure in your machine right now.
```
+-uavWs (name not important)
  +- build (Not part of the repo, created on catkin_make. Build stuffs in here.)
  +- devel (Not part of the repo, created on catkin_make. Devel stuffs in here.)
  +- (A BUNCH of script files written by the teaching team. Also not in this repo.)
  +- src
     +- CMakeLists.txt (Not part of this repo and its supposedly locked. Created on catkin_init_workspace)
     +- hector_gazebo (From the simlite folder given. Not part of this repo.)
     +- hector_quadrotor (From the simlite folder given. Not part of this repo.)
     +- drone_navigation (this repo itself)
        +- src (whatever's in the src folder of this repo - currently empty with dummy cpp file)
        +- CMakeLists.txt (part of this repo - configuration related)
        +- package.xml (part of this repo - configuration related)
        +- README.md (you are reading me now)
```

Now, check in to your respective branch. (you should be currently in the master branch)

### Path Planning team
```
cd uavWs/src/drone-navigation
git fetch
git checkout path-planning
git branch
```
If successful, you should now be in the path-planning branch. (current branch has a *).

You may now start working from here. Remember to push your code constantly. Depending on your team's workflow, you may also want to create separate branch from this branch. 

### Drone Trajectory team
```
cd drone-navigation
git fetch
git checkout trajectory
git branch
```
If successful, you should now be in the trajectory branch. (current branch has a *).

You may now start working from here. Remember to push your code constantly. Depending on your team's workflow, you may also want to create separate branch from this branch and integrate amongst the team first before merging to master.

### Testing
To-be-updated

### Integration
When the team is ready to merge code into master, please go through the integration test before merging. 
On github, make a pull request. One other person (preferably not from the same team) will need to test the integration before merging it into master. Update the team when a pull request is merged.

## Documentation
I'll try to update this documentation as and when needed promptly.