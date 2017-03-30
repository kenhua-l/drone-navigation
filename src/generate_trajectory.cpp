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

		void convertToLocalCoords(float robot_x, float robot_y, float robot_yaw, float x, float y, float &local_x, float &local_y)
    {
      local_x = (x - robot_x) * cos(-robot_yaw) - (y - robot_y) * sin(-robot_yaw);
      local_y = (x - robot_x) * sin(-robot_yaw) + (y - robot_y) * cos(-robot_yaw);
    }

		void generateTrajectory(nav_msgs::Path path)
		{
			// Generate the velocity, acceleration profile and write it to a file
      if(path.poses.size() == 0)
      {
        return;
      }

			// Initialize robot coordinates and yaw
      float robot_x = current_x;
      float robot_y = current_y;
      float robot_yaw = current_yaw;
      //Find nearest point to robot
      int index = find_index_nearest(path);

      //Find the look ahead point
      geometry_msgs::Pose look_ahead_pose;
      int look_ahead = 2;
      if(index + look_ahead < path.poses.size())
      {
        look_ahead_pose = path.poses[index + look_ahead].pose;
        // std::cout << "index: " << index + look_ahead << std::endl;
      }
      else
      {
        look_ahead_pose = path.poses[path.poses.size() - 1].pose;
      }

      //Convert the look ahead point to the robot's local coordinates
      float look_ahead_local_x, look_ahead_local_y;
      convertToLocalCoords(robot_x, robot_y, robot_yaw, look_ahead_pose.position.x, look_ahead_pose.position.y, look_ahead_local_x, look_ahead_local_y);

			// Distance traveled in 50ms using current velocity
			// Use distance = radius * theta to find theta
			// Use rotate current x and y by theta around the origin of the circle to find the new local x and local y
			// Convert new local x and local y to global x and global y

			// Find velocity for next point
			// Find acceleration between current point and next point
			// Find yaw for next point
			// Find change in yaw between current point and next point
		}

		void writeTrajectoryToFile()
		{
			// Format: x y z vx vy vz ax ay az head headv
		}
};

int main(int argc, char **argv)
{

}
