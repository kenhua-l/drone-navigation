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

		float distance(float x1, float y1, float x2, float y2)
    {
      return sqrt( pow(x1 - x2, 2) + pow(y1 - y2, 2) );
    }

		int find_index_nearest(nav_msgs::Path &path, float robot_x, float robot_y)
    {
      static int previous_index = 0;

      float temp_distance, next_distance;
      geometry_msgs::Pose temp_pose, next_pose;
      for(int i = previous_index; i < path.poses.size() - 1; i++)
      {
        temp_pose = path.poses[i].pose;
        temp_distance = distance(robot_x, robot_y, temp_pose.position.x, temp_pose.position.y);

        next_pose = path.poses[i+1].pose;
        next_distance = distance(robot_x, robot_y, next_pose.position.x, next_pose.position.y);
        //std::cout << "index: " << i << " temp_distance: " << temp_distance << " previous_distance: " << previous_distance << std::endl;
        if(temp_distance < next_distance)
        {
          previous_index = i;
          return i;
        }
      }
      return path.poses.size() - 1;
    }

		void convertToLocalCoords(float robot_x, float robot_y, float robot_yaw, float x, float y, float &local_x, float &local_y)
    {
      local_x = (x - robot_x) * cos(-robot_yaw) - (y - robot_y) * sin(-robot_yaw);
      local_y = (x - robot_x) * sin(-robot_yaw) + (y - robot_y) * cos(-robot_yaw);
    }

		// Find the angle between the origin and a line formed by two points
		float findAngle(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
		{
			return atan2( pose2.position.y - pose1.position.y, pose2.position.x - pose1.position.x );
		}

		void rotate(float angle, float x, float, y)
		{

		}

		// Generate the velocity, acceleration profile and write it to a file
		void generateTrajectory(nav_msgs::Path path)
		{
      if(path.poses.size() == 0)
      {
        return;
      }

			// Initialize robot coordinates and yaw
      float robot_x = path.poses[0].pose.position.x;
      float robot_y = path.poses[0].pose.position.y;
      float robot_yaw = findAngle(path.poses[0].pose, path.poses[1].pose);
      //Find nearest point to robot
      int index = find_index_nearest(path, robot_x, robot_y);

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

			float current_vel = 0.0;
			float acceleration = 0.1;
			float PERIOD = 0.05; // 50ms
			float acc_per_50_ms = acceleration * PERIOD;
			// Distance traveled in 50ms using current velocity
			float distance_traveled = current_vel + 0.5 * pow(acc_per_50_ms, 2);
			// Use distance = radius * theta to find theta
			float radius = pow(distance(look_ahead_local_x, look_ahead_local_y, robot_x, robot_y), 2) / (2 * (look_ahead_local_x - robot_x) )
			float theta = distance_traveled / radius;
			// Rotate current x and y by theta around the origin of the circle to find the new local x and local y
			float temp_x;
			if(look_ahead_local_x > 0)
			{
				temp_x = -radius;
			}
			else
			{
				temp_x = radius
			}
			float temp_x = -radius;
			float temp_y = 0;
			rotate(theta, temp_x, temp_y);
			// Convert new local x and local y to global x and global y
			float new_robot_x = robot_x + temp_x;
			float new_robot_y = robot_y + temp_y;

			// Find velocity for next point
			float new_vel;
			if(new_vel < 0.3)
			{
				new_vel = current_vel + acc_per_50_ms;
			}
			else
			{
				new_vel = 0.3;
			}

			// Find acceleration between current point and next point
			// Find change in yaw between current point and next point
			float curvature = 1.0 / radius;
			float delta_yaw = robot_yaw + curvature * distance_traveled;
		}

		void writeTrajectoryToFile()
		{
			// Format: x y z vx vy vz ax ay az head headv
		}
};

int main(int argc, char **argv)
{

}
