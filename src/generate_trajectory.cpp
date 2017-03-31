#include <iostream>
#include <iomanip>  // this included for the decimal point printing
#include <stdlib.h>
#include "ros/ros.h"
#include <math.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#define PI 3.14159
class TrajectoryPlanner
{
<<<<<<< Updated upstream
=======
	private:
		nav_msgs::OccupancyGrid occu  ncy_grid;

>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
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
			// std::cout << "robot_x: " << robot_x << " robot_y: " << std::endl;
			// std::cout << "x: " << x << " y: " << y << std::endl;
			// std::cout << "a: " << x - robot_x << std::endl;
			// std::cout << "b: " << cos(-robot_yaw) << std::endl;
			// std::cout << "c: " << y - robot_y << std::endl;
			// std::cout << "d: " << sin(-robot_yaw) << std::endl;
      local_x = (x - robot_x) * cos(-robot_yaw) - (y - robot_y) * sin(-robot_yaw);
      local_y = (x - robot_x) * sin(-robot_yaw) + (y - robot_y) * cos(-robot_yaw);
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

		void rotate(double angle, double x, double y, double &new_x, double &new_y)
		{
			new_x = x * cos(-angle) - y * sin(-angle);
			new_y = x * sin(-angle) + y * cos(-angle);
		}

		double calcRadius(double look_ahead_local_x, double look_ahead_local_y, double robot_x, double robot_y)
		{
			// std::cout << "denominator: " << 2 * (look_ahead_local_x - robot_x) << std::endl;
			return pow( distance(look_ahead_local_x, look_ahead_local_y, robot_x, robot_y), 2) / (2 * (look_ahead_local_x - robot_x));
		}

		void calcDestinationCoords(double robot_x, double robot_y, double robot_yaw, double look_ahead_local_x,
															double look_ahead_local_y, double distance_traveled, double &new_robot_x,
															double &new_robot_y, double &distance_from_look_ahead)
		{
			// std::cout << "look_ahead_local_x: " << look_ahead_local_x << " robot_y: " << robot_y << std::endl;
			if( (look_ahead_local_x - robot_x < 0.0001) && (look_ahead_local_x - robot_x > -0.0001) )
			{
				new_robot_x = robot_x + distance_traveled * cos(robot_yaw);
				new_robot_y = robot_y + distance_traveled * sin(robot_yaw);
				distance_from_look_ahead = distance(robot_x, robot_y, look_ahead_local_x, look_ahead_local_y);
				// std::cout << "distance_from_look_ahead_aaaa: " << distance_from_look_ahead << std::endl;
			}
			else
			{
				double radius = calcRadius(look_ahead_local_x, look_ahead_local_y, robot_x, robot_y);
				// std::cout << "radius: " << radius << std::endl;
				double theta = distance_traveled / radius;
				// std::cout << "theta: " << theta << std::endl;
				double temp_x, temp_y = 0;
				if(look_ahead_local_x > 0)
				{
					temp_x = -radius;
				}
				else
				{
					temp_x = radius;
				}
				double new_x, new_y;
				rotate(theta, temp_x, temp_y, new_x, new_y);
				new_x = new_x - temp_x + robot_x;
				new_y = new_y + robot_y;
				rotate(robot_yaw + PI/2, new_x, new_y, new_x, new_y);
				new_robot_x = new_x;
				new_robot_y = new_y;

				// Calculate angle from robot to origin of circle to look ahead point
				double temp_angle = asin(look_ahead_local_y / radius);
				// std::cout << "temp_angle: " << temp_angle << std::endl;
				distance_from_look_ahead = radius * temp_angle;
				// std::cout << "distance_from_look_ahead_bbbb: " << distance_from_look_ahead << std::endl;
			}

			/* Test case for this function
			TrajectoryPlanner tp;
			double x, y, d;
			tp.calcDestinationCoords(0, 0, 0, 1, 1, 1, x, y, d);
			std::cout << "x: " << x << " y: " << y << " d: " << d << std::endl;
			tp.calcDestinationCoords(0, 0, PI, 1, 1, 1, x, y, d);
			std::cout << "x: " << x << " y: " << y << " d: " << d << std::endl;
			tp.calcDestinationCoords(0, 0, 0, 0, 1, 1, x, y, d);
			std::cout << "x: " << x << " y: " << y << " d: " << d << std::endl;
			tp.calcDestinationCoords(0, 0, PI, 0, 1, 1, x, y, d);
			std::cout << "x: " << x << " y: " << y << " d: " << d << std::endl;
			tp.calcDestinationCoords(0, 0, -PI/4, 0, 1, 1, x, y, d);
			std::cout << "x: " << x << " y: " << y << " d: " << d << std::endl;
			*/
		}

		// This function assumes that the robot will be close enough to the path
		double calcVelocity(double distance_from_look_ahead, nav_msgs::Path path, int look_ahead, double current_vel, bool &stop)
		{
			// std::cout << "distance_from_look_ahead: " << distance_from_look_ahead << std::endl;
			double braking_time = current_vel / DECELERATION;
			double braking_distance = current_vel * PERIOD + 0.5 * DECELERATION * pow(braking_time, 2) + 0.15; //0.15m is the buffer

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
			// 	std::cout << "slow down" << std::endl;
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
				// std::cout << "acc: " << acc_per_50_ms << std::endl;
				new_vel = current_vel + acc_per_50_ms;
				// std::cout << "new_vel in calcVelocity: " << new_vel << std::endl;
				// std::cout << "speed up" << std::endl;
				if(new_vel > MAX_VEL)
				{
					// std::cout << "max speed" << std::endl;
					new_vel = MAX_VEL;
				}
			}
			// std::cout << "end" << std::endl;
			stop = false;
			return new_vel;
		}

		// Generate the velocity, acceleration profile and write it to a file
=======
		void takeoff()
		{
		  float deceleration = -0.5;
		  float acceleration = 0.5;

		  float velocity = 0.0;

		  float targetDist = 1.0 ;
		  float middleDist = targetDist / 2;

		  float coorX = -1.500;
		  float coorY = -1.500;
		  float currentZ = 0.0 ;

		  float liftof_time = 0.0;
		  float interval = 0.05;

		  float timecount= 0;
		  int buffer =400;

			while(currentZ <= 0.3)
			{
				velocity = velocity + (acceleration * interval);
				currentZ = currentZ + (velocity * interval);
				std::cout <<std::fixed << std::setprecision(3) <<coorX << " " << coorY<< " "<<currentZ << " 0.000 0.000 "<< velocity << " 0.000 0.000 "<< acceleration << " 0.000 0.000"<<std::endl;
				timecount=timecount+interval;
			}


			while(buffer > 0)
			{
				std::cout <<std::fixed << std::setprecision(3) <<coorX << " " << coorY<< " "<<currentZ << " 0.000 0.000 "<< velocity << " 0.000 0.000 0.000 0.000 0.000"<<std::endl;
				buffer = buffer - 1;
			}

			while(velocity >= 0.0)
			{


				if(currentZ <= middleDist)
				{
					velocity = velocity + (acceleration * interval);
					currentZ = currentZ + (velocity * interval);
					std::cout <<std::fixed << std::setprecision(3) <<coorX << " " << coorY<< " "<<currentZ << " 0.000 0.000 "<< velocity << " 0.000 0.000 "<< acceleration << " 0.000 0.000"<<std::endl;
				}
				else
				{
					velocity = velocity + (deceleration * interval);
					currentZ = currentZ + (velocity * interval);
					std::cout <<std::fixed << std::setprecision(3) <<coorX << " " << coorY<< " "<<currentZ << " 0.000 0.000 "<< velocity << " 0.000 0.000 "<< deceleration << " 0.000 0.000"<<std::endl;
				}

				timecount=timecount+interval;
			}
			buffer = 400;
			while(buffer > 0)
			{
				std::cout <<std::fixed << std::setprecision(3) <<coorX << " " << coorY<< " "<<currentZ << " 0.000 0.000 "<< velocity << " 0.000 0.000 0.000 0.000 0.000"<<std::endl;
				buffer = buffer - 1;
			}

		}

		void landing()
		{
		  float acceleration = 0.3;
		  float deceleration = -0.3;

		  float velocity = 0.0;

		  float land_targetDist = 0.0;
		  float targetDist = 1.0 ;
		  float middleDist = targetDist / 2;

		  float coorX = -1.500;
		  float coorY = -1.500;
		  float landing_currentZ = 0.98;
		  float currentZ = 0.0;

		  float liftof_time = 0.0;
		  float interval = 0.05;

		  float timecount= 0;
		  while (velocity <= 0.0)
		  {
		    //if(currentZ <= middleDist)
		    if(landing_currentZ >= middleDist )
		    {
		      //velocity = velocity + (acceleration * interval);
		      velocity = velocity + (deceleration * interval);
		      landing_currentZ = landing_currentZ + (velocity * interval);
		          std::cout <<std::fixed << std::setprecision(3) <<coorX << " " << coorY<< " "<<landing_currentZ << " 0.000 0.000 "<< velocity << " 0.000 0.000 "<< deceleration << " 0.000 0.000"<<std::endl;
		    }
		    else
		    {
		      //velocity = velocity + (deceleration * interval);
		      velocity = velocity + (acceleration * interval);
		      landing_currentZ = landing_currentZ + (velocity * interval);
		          std::cout <<std::fixed << std::setprecision(3) <<coorX << " " << coorY<< " "<<landing_currentZ << " 0.000 0.000 "<< velocity << " 0.000 0.000 "<< acceleration << " 0.000 0.000"<<std::endl;
		    }

		    timecount=timecount+interval;
		  }
		  //Print out steady state
		  std::cout <<std::fixed << std::setprecision(3) <<coorX << " " << coorY<< " "<<"0.000" << " 0.000 0.000 "<< velocity << " 0.000 0.000 "<< "0.000" << " 0.000 0.000"<<std::endl;
		    //Print out to stop the propeller
		  std::cout <<std::fixed << std::setprecision(3) <<coorX << " " << coorY<< " "<<"-1.000" << " 0.000 0.000 "<< velocity << " 0.000 0.000 "<< "0.000" << " 0.000 0.000"<<std::endl;
		}

>>>>>>> Stashed changes
		void generateTrajectory(nav_msgs::Path path)
		{
      if(path.poses.size() == 0)
      {
        return;
      }

			// Initialize robot coordinates and yaw
      double robot_x = path.poses[0].pose.position.x;
      double robot_y = path.poses[0].pose.position.y;
      double robot_yaw = findAngle(path.poses[0].pose, path.poses[1].pose);

			double current_x_vel = 0;
			double current_y_vel = 0;
			double current_z_vel = 0;
			bool stop = false;
			while(!stop && ros::ok())
			{
				// std::cout << robot_x << " " << robot_y << " " << "0" << " " << current_x_vel << " " << current_y_vel << " " << current_z_vel << " ";
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
	      double look_ahead_local_x, look_ahead_local_y;
				// std::cout << "robot_x_main: " << robot_x << " robot_y_main: " << robot_y << std::endl;
				// std::cout << "look_ahead_x: " << look_ahead_pose.position.x << " look_ahead_y: " << look_ahead_pose.position.y << std::endl;
	      convertToLocalCoords(robot_x, robot_y, robot_yaw, look_ahead_pose.position.x, look_ahead_pose.position.y, look_ahead_local_x, look_ahead_local_y);
				// std::cout << "look_ahead_local_x_main: " << look_ahead_local_x << " look_ahead_local_y_main: " << look_ahead_local_y << std::endl;
				double current_vel = sqrt( pow(current_x_vel, 2) + pow(current_y_vel, 2) );

				double acc_per_50_ms = ACCELERATION * PERIOD;
				// Distance traveled in 50ms using current velocity
				double distance_traveled = current_vel + 0.5 * pow(acc_per_50_ms, 2);
				// Find the next point that the robot travelled to using distance_traveled
				double new_robot_x, new_robot_y, distance_from_look_ahead;
				calcDestinationCoords(robot_x, robot_y, robot_yaw, distance_traveled, look_ahead_local_x, look_ahead_local_y, new_robot_x, new_robot_y, distance_from_look_ahead);

				// Find velocity for next point
				double new_vel = calcVelocity(distance_from_look_ahead, path, look_ahead, current_vel, stop);
				// std::cout << "new_vel: " << new_vel << std::endl;
				// Find change in yaw between current point and next point
				double delta_yaw = findAngle(new_robot_x, new_robot_y, path.poses[index].pose.position.x, path.poses[index].pose.position.y);

				// std::cout << "robot_yaw: " << robot_yaw << " sin: " << sin(robot_yaw) << " cos: " << cos(robot_yaw) << std::endl;
				double new_x_vel = new_vel * cos(robot_yaw);
				double new_y_vel = new_vel * sin(robot_yaw);
				// std::cout << "new_x_vel: " << new_x_vel << " new_y_vel: " << new_y_vel << std::endl;
				double new_z_vel = 0;

				double current_x_acc = (new_x_vel - current_x_vel) / PERIOD;
				double current_y_acc = (new_y_vel - current_y_vel) / PERIOD;
				double current_z_acc = (new_z_vel - current_z_vel) / PERIOD;

				// std::cout << current_x_acc << " " << current_y_acc << " " << current_z_acc << " " << robot_yaw << " " << delta_yaw << std::endl;
				std::cout << "new_robot_x: " << new_robot_x << std::endl;
				std::cout << "new_robot_y: " << new_robot_y << std::endl;
				robot_x = new_robot_x;
				robot_y = new_robot_y;
				robot_y += delta_yaw;
				current_x_vel = new_x_vel;
				current_y_vel = new_y_vel;
				current_z_vel = new_z_vel;
				// std::cout << current_x_vel << " " << current_y_vel << " " << current_z_vel << std::endl;
			}

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
	tp.generateTrajectory(path);
}
