#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include <math.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#define PI 3.14159
class TrajectoryPlanner
{
	public:
		double PERIOD;
		double ACCELERATION;
		double DECELERATION;
		double MAX_VEL;


		TrajectoryPlanner()
		{
			PERIOD = 0.05; // 50ms
			ACCELERATION = 0.1;
			DECELERATION = 0.15;
			MAX_VEL = 0.5;
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

		double distance(double x1, double y1, double x2, double y2)
    {
      return sqrt( pow(x1 - x2, 2) + pow(y1 - y2, 2) );
    }

		double distance(geometry_msgs::Pose a, geometry_msgs::Pose b)
    {
			return sqrt( pow(a.position.x - b.position.x, 2) + pow(a.position.y - b.position.y, 2));
    }

		int find_index_nearest(nav_msgs::Path &path, double robot_x, double robot_y)
    {
      static int previous_index = 0;

      double temp_distance, next_distance;
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

		void convertToLocalCoords(double robot_x, double robot_y, double robot_yaw, double x, double y, double &local_x, double &local_y)
    {
			// std::cout << "robot_x: " << robot_x << " robot_y: " << robot_y << std::endl;
			// std::cout << "x: " << x << " y: " << y << std::endl;
			// std::cout << "a: " << x - robot_x << std::endl;
			// std::cout << "b: " << cos(-robot_yaw) << std::endl;
			// std::cout << "c: " << y - robot_y << std::endl;
			// std::cout << "d: " << sin(-robot_yaw) << std::endl;
      local_x = (x - robot_x) * cos(-robot_yaw) - (y - robot_y) * sin(-robot_yaw);
      local_y = (x - robot_x) * sin(-robot_yaw) + (y - robot_y) * cos(-robot_yaw);
			// std::cout << "local_x: " << local_x << std::endl;
			// std::cout << "local_y: " << local_y << std::endl;
    }

		// Find the angle between the origin and a line formed by two points
		double findAngle(double x1, double y1, double x2, double y2)
		{
			return atan2( y2 - y1, x2 - x1 );
		}

		// Find the angle between the origin and a line formed by two points
		double findAngle(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
		{
			return atan2( pose2.position.y - pose1.position.y, pose2.position.x - pose1.position.x );
		}

		void rotate(double angle, double &x, double &y)
		{
			x = x * cos(angle) - y * sin(angle);
			y = x * sin(angle) + y * cos(angle);
		}

		double calcRadius(double look_ahead_local_x, double look_ahead_local_y, double robot_x, double robot_y)
		{
			return abs( ( pow(look_ahead_local_x, 2) + pow(look_ahead_local_y, 2) ) / (2 * look_ahead_local_y) );
		}

		void calcNextPoint(double robot_x, double robot_y, double robot_yaw, double look_ahead_local_x, double look_ahead_local_y,
											double current_vel, double &new_robot_x, double &new_robot_y, double &new_robot_yaw, double &new_vel)
		{
			if( (look_ahead_local_y < 0.0001) && (look_ahead_local_y > -0.0001) )
			{

				// new_robot_x = robot_x + distance_traveled * cos(robot_yaw);
				// new_robot_y = robot_y + distance_traveled * sin(robot_yaw);
				// distance_from_look_ahead = distance(robot_x, robot_y, look_ahead_local_x, look_ahead_local_y);
				// delta_yaw = 0;
			}
			else
			{
				// double radius = calcRadius(look_ahead_local_x, look_ahead_local_y, robot_x, robot_y);
				// double theta = distance_traveled / radius;
				// double delta_x = radius * sin(theta);
				// double delta_y = radius - radius * cos(theta);
				// rotate(robot_yaw, delta_x, delta_y);
				// new_robot_x = robot_x + delta_x;
				// new_robot_y = robot_y + delta_y;
				// if(look_ahead_local_y < 0)
				// {
				// 	delta_yaw = -theta;
				// 	std::cout << " delta_yaw: " << delta_yaw << std::endl;
				// }
				// else
				// {
				// 	delta_yaw = theta;
				// 	std::cout << " delta_yaw: " << delta_yaw << std::endl;
				// }

			}
		}

		// This function assumes that the robot will be close enough to the path
		double calcVelocity(double distance_from_look_ahead, nav_msgs::Path path, int look_ahead, double current_vel, bool &stop)
		{
			// std::cout << "distance_from_look_ahead: " << distance_from_look_ahead << std::endl;
			double braking_time = current_vel / DECELERATION;
			double braking_distance = current_vel * braking_time + 0.5 * DECELERATION * pow(braking_time, 2) + 0.15; //0.15m is the buffer

			double remaining_distance = distance_from_look_ahead;
			double temp_distance;
			bool slow_down = true;
			for(int i = look_ahead; i < path.poses.size() - 1; i++)
			{
				temp_distance = distance(path.poses[i].pose, path.poses[i+1].pose);
				remaining_distance += temp_distance;
				if(braking_distance - remaining_distance <= 0)
				{
					slow_down = false;
				}
			}

			double new_vel;
			if(slow_down)
			{
				// std::cout << "slow down" << std::endl;
				if(remaining_distance < 0.05)
				{
					new_vel = 0;
					stop = true;
					return new_vel;
				}
				else
				{
					double average_acc = -pow(current_vel, 2) / (2 * remaining_distance);
					double acc_per_50_ms = average_acc * PERIOD;
					new_vel = current_vel + acc_per_50_ms;
				}
			}
			else
			{
				double acc_per_50_ms = DECELERATION * PERIOD;
				new_vel = current_vel + acc_per_50_ms;
				if(new_vel > MAX_VEL)
				{
					new_vel = MAX_VEL;
				}
			}
			stop = false;
			return new_vel;
		}

		std::vector<double> calculatePathMaxVelocity(nav_msgs::Path path)
		{
			std::vector<double> result;
			double temp, calculated_vel, total_distance = 0, sum_distance = 0;

			for(int i = 0; i < path.poses.size() - 1; i++)
			{
				total_distance += distance(path.poses[i].pose, path.poses[i+1].pose);
			}
			double braking_distance = 1.0 / (2 * DECELERATION);
			double remaining_distance, max_curving_speed, max_braking_speed;
			for(int i = 0; i < path.poses.size() - 2; i++)
			// for(int i = 0; i < 10; i++)
			{
				remaining_distance = total_distance - sum_distance;
				// temp = distance(path.poses[i].pose, path.poses[i+1].pose) + distance(path.poses[i+1].pose, path.poses[i+2].pose);
				max_curving_speed = MAX_VEL * distance(path.poses[i].pose, path.poses[i+2].pose) / ( distance(path.poses[i].pose, path.poses[i+1].pose) + distance(path.poses[i+1].pose, path.poses[i+2].pose) );
				if(braking_distance >= remaining_distance)
				{
					max_braking_speed = sqrt(2 * DECELERATION * remaining_distance);
					std::cout << "max_braking_speed: " << max_braking_speed << " remaining_distance: " << remaining_distance << std::endl;
				}
				else
				{
					max_braking_speed = 1;
				}
				// std::cout << "curve_speed: " << max_curving_speed << " max_braking_speed: " << max_braking_speed << std::endl;
				calculated_vel = fmin(max_curving_speed, max_braking_speed);
				if(calculated_vel > 1)
				{
					calculated_vel = 1;
				}
				result.push_back(calculated_vel);
				sum_distance += distance(path.poses[i].pose, path.poses[i+1].pose);
			}

			remaining_distance = total_distance - sum_distance;
			max_braking_speed = sqrt(2 * DECELERATION * remaining_distance);
			// For second last point in path
			result.push_back(max_braking_speed);
			sum_distance += distance(path.poses[ path.poses.size() - 2 ].pose, path.poses[ path.poses.size() - 1].pose);
			// Last point in path has zero velocity
			remaining_distance = total_distance - sum_distance;
			max_braking_speed = sqrt(2 * DECELERATION * remaining_distance);
			result.push_back(max_braking_speed);

			return result;
		}

		// Generate the velocity, acceleration profile and write it to a file
		void generateTrajectory(nav_msgs::Path path)
		{
      if(path.poses.size() == 0)
      {
        return;
      }

			// Calculate maximum velocity for each point
			std::vector<double> path_max_vel = calculatePathMaxVelocity(path);

			// Initialize robot coordinates and yaw
      double robot_x = path.poses[0].pose.position.x;
      double robot_y = path.poses[0].pose.position.y;
      double robot_yaw = findAngle(path.poses[0].pose, path.poses[1].pose);

			double current_x_vel = 0;
			double current_y_vel = 0;
			double current_z_vel = 0;
			bool stop = false;
			for(int j = 0; j < 400; j++)
			{
				std::cout << robot_x << " " << robot_y << " " << "0" << " " << current_x_vel << " " << current_y_vel << " " << current_z_vel << " ";
				//Find nearest point to robot
	      int index = find_index_nearest(path, robot_x, robot_y);

	      //Find the look ahead point
	      geometry_msgs::Pose look_ahead_pose;
	      int look_ahead = 2;
	      if(index + look_ahead < path.poses.size())
	      {
	        look_ahead_pose = path.poses[index + look_ahead].pose;
	      }
	      else
	      {
	        look_ahead_pose = path.poses[path.poses.size() - 1].pose;
	      }

	      //Convert the look ahead point to the robot's local coordinates
	      double look_ahead_local_x, look_ahead_local_y;
	      convertToLocalCoords(robot_x, robot_y, robot_yaw, look_ahead_pose.position.x, look_ahead_pose.position.y, look_ahead_local_x, look_ahead_local_y);
				double current_vel = sqrt( pow(current_x_vel, 2) + pow(current_y_vel, 2) );

				double new_robot_x, new_robot_y, new_robot_yaw, new_vel;
				// Want to find new_vel, new_robot_x, new_robot_y, delta_yaw
				calcNextPoint(robot_x, robot_y, robot_yaw, look_ahead_local_x, look_ahead_local_y, current_vel, new_robot_x, new_robot_y, new_robot_yaw, new_vel);
				double new_x_vel = new_vel * cos(new_robot_yaw);
				double new_y_vel = new_vel * sin(new_robot_yaw);
				double new_z_vel = 0;

				double current_x_acc = (new_x_vel - current_x_vel) / PERIOD;
				double current_y_acc = (new_y_vel - current_y_vel) / PERIOD;
				double current_z_acc = (new_z_vel - current_z_vel) / PERIOD;

				// std::cout << current_x_acc << " " << current_y_acc << " " << current_z_acc << " " << robot_yaw << " " << delta_yaw << std::endl;
				robot_x = new_robot_x;
				robot_y = new_robot_y;
				robot_yaw = new_robot_yaw;
				current_x_vel = new_x_vel;
				current_y_vel = new_y_vel;
				current_z_vel = new_z_vel;
				std::cout << std::endl;
			}
			std::cout << std::endl << std::endl;
		}

		void writeTrajectoryToFile()
		{
			// Format: x y z vx vy vz ax ay az head headv
		}

	private:
		nav_msgs::OccupancyGrid occupancy_grid;

};

void add_pose_to_path(nav_msgs::Path &desired_path, float x, float y)
{
  static int seq = 1;
  geometry_msgs::PoseStamped ps;
  ps.header.seq = seq;
  seq++;
  ps.header.stamp = ros::Time::now();
  ps.header.frame_id = "base_footprint";
  ps.pose.position.x = x;
  ps.pose.position.y = y;

  desired_path.poses.push_back(ps);
}

nav_msgs::Path set_up_test_case()
{
  nav_msgs::Path desired_path;
  add_pose_to_path(desired_path, 0.5, 0.5);
  add_pose_to_path(desired_path, 0.5, 0.7);
  add_pose_to_path(desired_path, 0.5, 0.9);
  add_pose_to_path(desired_path, 0.5, 1.1);
  add_pose_to_path(desired_path, 0.5, 1.3);
  add_pose_to_path(desired_path, 0.5, 1.5);
  add_pose_to_path(desired_path, 0.5, 1.7);
  add_pose_to_path(desired_path, 0.5, 1.9);
  add_pose_to_path(desired_path, 0.5, 2.1);
  add_pose_to_path(desired_path, 0.5, 2.3);
  add_pose_to_path(desired_path, 0.5, 2.5);
  add_pose_to_path(desired_path, 0.5, 2.7);
  add_pose_to_path(desired_path, 0.5, 2.9);
  add_pose_to_path(desired_path, 0.5, 3.1);
  add_pose_to_path(desired_path, 0.5, 3.3);
  add_pose_to_path(desired_path, 0.5, 3.5);
  add_pose_to_path(desired_path, 0.5, 3.7);
  add_pose_to_path(desired_path, 0.5, 3.9);
  add_pose_to_path(desired_path, 0.5, 4.1);
  add_pose_to_path(desired_path, 0.5, 4.3);
  add_pose_to_path(desired_path, 0.5, 4.5);
  add_pose_to_path(desired_path, 0.5, 4.7);
  add_pose_to_path(desired_path, 0.5, 4.9);

  add_pose_to_path(desired_path, 0.503, 5.1);
  add_pose_to_path(desired_path, 0.513, 5.197);
  add_pose_to_path(desired_path, 0.556, 5.406);
  add_pose_to_path(desired_path, 0.627, 5.604);
  add_pose_to_path(desired_path, 0.733, 5.803);
  add_pose_to_path(desired_path, 0.84, 5.951);
  add_pose_to_path(desired_path, 0.975, 6.095);
  add_pose_to_path(desired_path, 1.125, 6.218);
  add_pose_to_path(desired_path, 1.28, 6.316);
  add_pose_to_path(desired_path, 1.456, 6.398);
  add_pose_to_path(desired_path, 1.645, 6.457);
  add_pose_to_path(desired_path, 1.82, 6.489);
  add_pose_to_path(desired_path, 2, 6.5);

  add_pose_to_path(desired_path, 2, 6.5);
  add_pose_to_path(desired_path, 2.2, 6.5);
  add_pose_to_path(desired_path, 2.4, 6.5);
  add_pose_to_path(desired_path, 2.6, 6.5);
  add_pose_to_path(desired_path, 2.8, 6.5);
  add_pose_to_path(desired_path, 3.0, 6.5);
  add_pose_to_path(desired_path, 3.2, 6.5);
  add_pose_to_path(desired_path, 3.4, 6.5);
  add_pose_to_path(desired_path, 3.6, 6.5);
  add_pose_to_path(desired_path, 3.8, 6.5);
  add_pose_to_path(desired_path, 4.0, 6.5);
  add_pose_to_path(desired_path, 4.2, 6.5);
  add_pose_to_path(desired_path, 4.4, 6.5);
  add_pose_to_path(desired_path, 4.6, 6.5);
  add_pose_to_path(desired_path, 4.8, 6.5);
  add_pose_to_path(desired_path, 5.0, 6.5);
  add_pose_to_path(desired_path, 5.2, 6.5);
  add_pose_to_path(desired_path, 5.4, 6.5);
  add_pose_to_path(desired_path, 5.6, 6.5);
  add_pose_to_path(desired_path, 5.8, 6.5);
  add_pose_to_path(desired_path, 6.0, 6.5);
  add_pose_to_path(desired_path, 6.2, 6.5);
  add_pose_to_path(desired_path, 6.4, 6.5);
  add_pose_to_path(desired_path, 6.6, 6.5);

  add_pose_to_path(desired_path, 6.6, 6.3);
  add_pose_to_path(desired_path, 6.6, 6.1);
  add_pose_to_path(desired_path, 6.6, 5.9);
  add_pose_to_path(desired_path, 6.6, 5.7);
  add_pose_to_path(desired_path, 6.6, 5.5);
  add_pose_to_path(desired_path, 6.6, 5.3);
  add_pose_to_path(desired_path, 6.6, 5.1);
  add_pose_to_path(desired_path, 6.6, 4.9);
  add_pose_to_path(desired_path, 6.6, 4.7);
  add_pose_to_path(desired_path, 6.6, 4.5);
  add_pose_to_path(desired_path, 6.6, 4.3);
  add_pose_to_path(desired_path, 6.6, 4.1);
  add_pose_to_path(desired_path, 6.6, 3.9);
  add_pose_to_path(desired_path, 6.6, 3.7);
  add_pose_to_path(desired_path, 6.6, 3.5);
  add_pose_to_path(desired_path, 6.6, 3.3);
  add_pose_to_path(desired_path, 6.6, 3.1);
  add_pose_to_path(desired_path, 6.6, 2.9);
  add_pose_to_path(desired_path, 6.6, 2.7);
  add_pose_to_path(desired_path, 6.6, 2.5);
  add_pose_to_path(desired_path, 6.6, 2.3);
  add_pose_to_path(desired_path, 6.6, 2.1);
  add_pose_to_path(desired_path, 6.6, 1.9);
  add_pose_to_path(desired_path, 6.6, 1.7);
  add_pose_to_path(desired_path, 6.6, 1.5);
  add_pose_to_path(desired_path, 6.6, 1.3);

  add_pose_to_path(desired_path, 6.63, 1.163);
  add_pose_to_path(desired_path, 6.41, 1.006);
  add_pose_to_path(desired_path, 6.08, 0.76);
  add_pose_to_path(desired_path, 5.86, 0.684);
  add_pose_to_path(desired_path, 5.6, 0.65);
  add_pose_to_path(desired_path, 5.39, 0.67);
  add_pose_to_path(desired_path, 5.2, 0.725);
  add_pose_to_path(desired_path, 5.01, 0.822);
  add_pose_to_path(desired_path, 4.8, 1);

  add_pose_to_path(desired_path, 4.8, 1.2);
  add_pose_to_path(desired_path, 4.8, 1.4);
  add_pose_to_path(desired_path, 4.8, 1.6);
  add_pose_to_path(desired_path, 4.8, 1.8);
  add_pose_to_path(desired_path, 4.8, 2.0);
  add_pose_to_path(desired_path, 4.8, 2.2);
  add_pose_to_path(desired_path, 4.8, 2.4);
  add_pose_to_path(desired_path, 4.8, 2.6);
  add_pose_to_path(desired_path, 4.8, 2.8);
  add_pose_to_path(desired_path, 4.8, 3.0);
  add_pose_to_path(desired_path, 4.8, 3.2);
  add_pose_to_path(desired_path, 4.8, 3.4);
  add_pose_to_path(desired_path, 4.8, 3.6);
  add_pose_to_path(desired_path, 4.8, 3.8);
  add_pose_to_path(desired_path, 4.8, 4.0);
  add_pose_to_path(desired_path, 4.8, 4.2);
  add_pose_to_path(desired_path, 4.8, 4.4);

  desired_path.header.stamp = ros::Time::now();
  desired_path.header.frame_id = "base_footprint";
  return desired_path;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robotController");
	ros::NodeHandle nh;

	TrajectoryPlanner tp;
	nav_msgs::Path path = set_up_test_case();
	// tp.generateTrajectory(path);
	std::vector<double> path_max_vel = tp.calculatePathMaxVelocity(path);
	for(int i = 0; i < path_max_vel.size(); i++)
	{
		std::cout << path_max_vel[i] << std::endl;
	}
}
