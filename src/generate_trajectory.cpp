#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include <math.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

class TrajectoryPlanner
{
	private:
		nav_msgs::OccupancyGrid occupancy_grid;

	public:
		TrajectoryPlanner()
		{

		}

		void putObstaclesOnGrid()
		{
			// Read obstacles from file
			// Create 60cm buffer around obstacles
		}

		nav_msgs::Path generatePath()
		{
			// As a first step, generate a path between a start and an end goal
			// The next step would be to generate a *smooth* path for multiple goals
		}

		void generateTrajectory(nav_msgs::Path path)
		{
			// Generate the velocity, acceleration profile and write it to a file
		}

		void writeTrajectoryToFile()
		{
			// Format: x y z vx vy vz ax ay az head headv
		}
};

int main(int argc, char **argv)
{

}
