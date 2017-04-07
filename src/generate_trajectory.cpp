#include <iostream>
#include <tuple>
#include <fstream>
#include <iomanip>

#include <stdlib.h>
#include <math.h>
#include "ros/ros.h"

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

// Map definitions
#define MAP_SIZE        5
#define MAP_START_X			-2.5
#define MAP_START_Y			-2.5
#define GRID_STEPS      12
#define GRID_RESOLUTION (1.0/GRID_STEPS)
#define GRID_LENGTH     (MAP_SIZE * GRID_STEPS + 1)
#define GRID_N          (GRID_LENGTH * GRID_LENGTH)
// Problem definitions
#define OBS_FILE 				"/home/mervyn/Desktop/EE4308-2/obstacles.txt"
#define OBS_RADIUS			0.71
#define START_X					-1.5
#define START_Y					1.5
#define GOAL_X					-1.5
#define GOAL_Y					-1.5
#define CHECKPT_X				2
#define CHECKPT_Y				0
// Directional A-star definitions
#define MAX_DIST 							999999
#define PI 										3.14159
#define DRONE_TURN_RADIUS 		0.5
#define DRONE_TURN_ARC 				(PI / 2.0 * DRONE_TURN_RADIUS)
#define DRONE_TURN_STEPS 			(int)(DRONE_TURN_ARC / GRID_RESOLUTION)
#define DRONE_TURN_STEP_RAD 	(GRID_RESOLUTION / DRONE_TURN_RADIUS)
#define NUM_DIRECTIONS 				8
#define NUM_NEIGHBOURS_RIGHT 	DRONE_TURN_STEPS
#define NUM_NEIGHBOURS 				(2 * DRONE_TURN_STEPS)

class TrajectoryPlanner
{
private:
	// PP variables
	nav_msgs::OccupancyGrid occupancy_grid;
	nav_msgs::Path a_star_path;
	ros::Publisher grid_pub;
	ros::Publisher path_pub;
	int neighbours_offset[NUM_DIRECTIONS][NUM_NEIGHBOURS][3];
	float neighbours_offset_dist[NUM_DIRECTIONS][NUM_NEIGHBOURS];
	// Trajectory variables
	double PERIOD;
	double ACCELERATION;
	double DECELERATION;
	double MAX_VEL;
	double ANGULAR_VEL;

	double robot_x;
	double robot_y;
	double robot_z;

	// Class used for path-finding
	class MapCell
	{
	public:
		int x;
		int y;
		float distance;
		int heading;

		MapCell(int x, int y, float distance, int heading)
		{
			this->x = x;
			this->y = y;
			this->distance = distance;
			this->heading = heading;
		}

		bool operator<(MapCell other) const
		{
			return distance < other.distance;
		}

		void setHeading(int heading)
		{
			this->heading = heading;
		}
	};


public:
	TrajectoryPlanner(ros::NodeHandle &nh, double x, double y)
	{

		PERIOD = 0.05; // 50ms
		ACCELERATION = 0.1;
		DECELERATION = 0.15;
		MAX_VEL = 0.5;
		ANGULAR_VEL = PI / 4; // radians per second

		robot_x = x;
		robot_y = y;
		robot_z = 0;

		// Publish grid and path data for display in rviz
		grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid", 0);
		path_pub = nh.advertise<nav_msgs::Path>("/a_star_path", 0);

		// Initialize Occupancy Grid
		std::vector<int8_t> grid_data(GRID_N, -1);
		occupancy_grid.info.map_load_time = ros::Time::now();
		occupancy_grid.info.resolution = GRID_RESOLUTION;
		occupancy_grid.info.width = GRID_LENGTH;
		occupancy_grid.info.height = GRID_LENGTH;
		occupancy_grid.info.origin.position.x = MAP_START_X;
		occupancy_grid.info.origin.position.y = MAP_START_Y;
		occupancy_grid.data = grid_data;

		initNeighbours();
		putObstaclesOnGrid(3);
		// generatePath();

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

		std::pair<double, double> convertToLocalCoords(double robot_x, double robot_y, double robot_yaw, double x, double y)
    {
    	double local_x = (x - robot_x) * cos(-robot_yaw) - (y - robot_y) * sin(-robot_yaw);
      double local_y = (x - robot_x) * sin(-robot_yaw) + (y - robot_y) * cos(-robot_yaw);
			return std::make_pair(local_x, local_y);
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

		void applyRotationMatrix(double angle, double &x, double &y)
		{
			// std::cout << "x: " << x << " y: " << y << " angle: " << angle << std::endl;
			double temp_x = x, temp_y = y;
			x = temp_x * cos(angle) - temp_y * sin(angle);
			y = temp_x * sin(angle) + temp_y * cos(angle);
			// std::cout << "x: " << x << " y: " << y << std::endl;
		}

		double calcRadius(double look_ahead_local_x, double look_ahead_local_y)
		{
			// std::cout << "!!!x: " << look_ahead_local_x << " !!!y: " << look_ahead_local_y << std::endl;
			// std::cout << "temp1: " << pow(look_ahead_local_x, 2) << " temp2: " << pow(look_ahead_local_y, 2) << std::endl;
			// std::cout << "temp3: " << (2 * look_ahead_local_y) << std::endl;
			// std::cout << "temppp: " << ( pow(look_ahead_local_x, 2) + pow(look_ahead_local_y, 2) ) / (2 * look_ahead_local_y)<< std::endl;
			return fabs( ( pow(look_ahead_local_x, 2) + pow(look_ahead_local_y, 2) ) / (2 * look_ahead_local_y) );
		}

		void calcNextPoint(double robot_x, double robot_y, double robot_yaw, double current_vel, std::pair<double, double> local_look_ahead_point,
											double look_ahead_max_vel, double &new_robot_x, double &new_robot_y, double &new_robot_yaw, double &new_vel)
		{
			// std::cout << "look_ahead_max_vel: " << look_ahead_max_vel << std::endl;
			if( (local_look_ahead_point.second < 0.0001) && (local_look_ahead_point.second > -0.0001) )
			{
				// std::cout << "straight" << std::endl;
				// Find distance
				double distance_to_look_ahead = distance(local_look_ahead_point.first, local_look_ahead_point.second, 0, 0);
				// std::cout << "numerator: " << pow(look_ahead_max_vel, 2) - pow(current_vel, 2) << " denominator: " << 2 * distance_to_look_ahead << std::endl;
				double acc_per_50_ms = PERIOD * ( ( pow(look_ahead_max_vel, 2) - pow(current_vel, 2) ) / ( 2 * distance_to_look_ahead ) );
				// std::cout << "acc_per_50_ms: " << acc_per_50_ms << std::endl;
				new_vel = current_vel + acc_per_50_ms;
				if(new_vel > 1)
				{
					new_vel = 1;
				}
				double distance_traveled = (new_vel + current_vel) / 2 * PERIOD;
				new_robot_x = robot_x + distance_traveled * cos(robot_yaw);
				new_robot_y = robot_y + distance_traveled * sin(robot_yaw);
				new_robot_yaw = robot_yaw;
			}
			else
			{
				// std::cout << "curve" << std::endl;
				// std::cout << "local_look_ahead_point_x: " << local_look_ahead_point.first << " local_look_ahead_point_y: " << local_look_ahead_point.second << std::endl;
				double radius = calcRadius(local_look_ahead_point.first, local_look_ahead_point.second);
				// std::cout << "radius: " << radius << std::endl;
				// theta is the angle between robot position and look ahead point
				double look_ahead_theta = 2 * asin( distance(local_look_ahead_point.first, local_look_ahead_point.second, 0, 0) / (2 * radius) );
				// std::cout << "x1: " << local_look_ahead_point.first << " y1: " << local_look_ahead_point.second << " x2: " << robot_x << " y2: " << robot_y << std::endl;
				// std::cout << "distance: " << distance(local_look_ahead_point.first, local_look_ahead_point.second, 0, 0) << std::endl;
				// std::cout << "temp: " << distance(local_look_ahead_point.first, local_look_ahead_point.second, 0, 0) / (2 * radius) << std::endl;
				// std::cout << "look_ahead_theta: " << look_ahead_theta << std::endl;
				double arc_distance_to_look_ahead = radius * look_ahead_theta;
				double acc_per_50_ms = ( pow(look_ahead_max_vel, 2) - pow(current_vel, 2) ) / ( 2 * arc_distance_to_look_ahead ) * PERIOD;
				// std::cout << "acc_per_50_ms: " << acc_per_50_ms << std::endl;
				// std::cout << "look_ahead_max_vel: " << look_ahead_max_vel << std::endl;
				// std::cout << "arc_distance_to_look_ahead: " << arc_distance_to_look_ahead << std::endl;
				new_vel = current_vel + acc_per_50_ms;
				if(new_vel > 1)
				{
					new_vel = 1;
				}
				// double distance_traveled = ( pow(look_ahead_max_vel, 2) - pow(current_vel, 2) ) / ( 2 * acc_per_50_ms );
				double distance_traveled = (new_vel + current_vel) / 2 * PERIOD;
				// std::cout << "distance_traveled: " << distance_traveled << " new_vel: " << new_vel << " current_vel: " << current_vel << std::endl;
				double traveled_theta = distance_traveled / radius;
				// std::cout << "radius: " << radius << " theta: " << traveled_theta << " sin(theta): " << sin(traveled_theta) << " cos(theta): " << cos(traveled_theta) << std::endl;
				double delta_x = radius * sin(traveled_theta);
				double delta_y = radius - radius * cos(traveled_theta);
				// std::cout << "look_ahead_local_y: " << local_look_ahead_point.second << std::endl;
				// std::cout << "(before rotation) delta_x: " << delta_x << " delta_y: " << delta_y << std::endl;
				applyRotationMatrix(robot_yaw, delta_x, delta_y);
				new_robot_x = robot_x + delta_x;
				new_robot_y = robot_y + delta_y;
				// std::cout << "(after rotation) delta_x: " << delta_x << " delta_y: " << delta_y << std::endl;
				// std::cout << "new_robot_x: " << new_robot_x << " new_robot_y: " << new_robot_y << std::endl;
				if(local_look_ahead_point.second < 0)
				{
					new_robot_yaw = robot_yaw - traveled_theta;
				}
				else
				{
					new_robot_yaw = robot_yaw + traveled_theta;
				}
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
				}
				else
				{
					max_braking_speed = 1;
				}
				// std::cout << "index: " << i << " curve_speed: " << max_curving_speed << " max_braking_speed: " << max_braking_speed << std::endl;
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
			// std::cout << "index: " << result.size() - 1 << " remaining_distance: " << remaining_distance << " max_braking_speed: " << max_braking_speed << std::endl;

			return result;
		}

		// Generate the velocity, acceleration profile and write it to a file
		void generateFlightTrajectory(nav_msgs::Path &path)
		{
      if(path.poses.size() == 0)
      {
        return;
      }

			// Calculate maximum velocity for each point
			std::vector<double> path_max_vel = calculatePathMaxVelocity(path);

			// Initialize robot coordinates and yaw
      // double robot_x = path.poses[0].pose.position.x;
      // double robot_y = path.poses[0].pose.position.y;
      double robot_yaw = findAngle(path.poses[0].pose, path.poses[1].pose);

			double current_x_vel = 0;
			double current_y_vel = 0;
			double current_z_vel = 0;
			bool stop = false;
			while(!stop)
			{
				std::cout << std::setprecision(3) << std::fixed << robot_x << " " << robot_y << " " << robot_z << " " << current_x_vel << " " << current_y_vel << " " << current_z_vel << " ";
				// std::cout << std::setprecision(3) << std::fixed << robot_x << " " << robot_y << " " << "0" << " " << current_x_vel << " " << current_y_vel << " " << current_z_vel << " " << std::endl;
				//Find nearest point to robot
	      int index = find_index_nearest(path, robot_x, robot_y);

	      //Find the look ahead point

	      int look_ahead = 1;
				int look_ahead_index = index + look_ahead;
				if(look_ahead_index >= path.poses.size())
				{
					look_ahead_index = path.poses.size() - 1;
				}
				geometry_msgs::Pose look_ahead_pose = path.poses[look_ahead_index].pose;
				// std::cout << "index: " << index << " look_ahead_index: " << look_ahead_index << " robot_x: " << robot_x << " robot_y: " << robot_y << " look_ahead_x: " << look_ahead_pose.position.x << " look_ahead_y: " << look_ahead_pose.position.y << std::endl;
	      //Convert the look ahead point to the robot's local coordinates
	      std::pair<double, double> local_look_ahead_point = convertToLocalCoords(robot_x, robot_y, robot_yaw, look_ahead_pose.position.x, look_ahead_pose.position.y);
				double current_vel = sqrt( pow(current_x_vel, 2) + pow(current_y_vel, 2) );

				double look_ahead_max_vel = path_max_vel[look_ahead_index];
				double new_robot_x, new_robot_y, new_robot_yaw, new_vel;
				// Want to find new_robot_x, new_robot_y, new_robot_yaw, new_vel
				calcNextPoint(robot_x, robot_y, robot_yaw, current_vel, local_look_ahead_point, look_ahead_max_vel, new_robot_x, new_robot_y, new_robot_yaw, new_vel);
				double new_x_vel = new_vel * cos(new_robot_yaw);
				double new_y_vel = new_vel * sin(new_robot_yaw);
				// std::cout << "new_vel: " << new_vel << " new_robot_yaw: " << new_robot_yaw << " new_x_vel: " << new_x_vel << " new_y_vel: " << new_y_vel << std::endl;
				double new_z_vel = 0;

				double current_x_acc = (new_x_vel - current_x_vel) / PERIOD;
				double current_y_acc = (new_y_vel - current_y_vel) / PERIOD;
				double current_z_acc = (new_z_vel - current_z_vel) / PERIOD;

				double delta_yaw = new_robot_yaw - robot_yaw;

				std::cout << current_x_acc << " " << current_y_acc << " " << current_z_acc << " " << robot_yaw << " " << delta_yaw << std::endl;
				// std::cout << current_x_acc << " " << current_y_acc << " " << current_z_acc << " " << robot_yaw << " " << delta_yaw << std::endl << std::endl;

				robot_x = new_robot_x;
				robot_y = new_robot_y;
				robot_yaw = new_robot_yaw;
				current_x_vel = new_x_vel;
				current_y_vel = new_y_vel;
				current_z_vel = new_z_vel;

				if(index == look_ahead_index && fabs(new_vel) < 0.02)
				{
					stop = true;
				}
			}
		}

		void takeoff()
		{
		  float deceleration = -0.5;
		  float acceleration = 0.5;

		  float velocity = 0.0;

		  float targetDist = 1.0;
		  float middleDist = targetDist / 2;

		  float liftof_time = 0.0;
		  float interval = 0.05;

		  float timecount= 0;
		  int buffer = 400;

			while(robot_z <= 0.3)
			{
				velocity = velocity + (acceleration * interval);
				robot_z = robot_z + (velocity * interval);
				std::cout <<std::fixed << std::setprecision(3) << robot_x << " " << robot_y << " "<< robot_z << " 0.000 0.000 "<< velocity << " 0.000 0.000 "<< acceleration << " 0.000 0.000"<<std::endl;
				timecount=timecount+interval;
			}

			while(buffer > 0)
			{
				std::cout <<std::fixed << std::setprecision(3) << robot_x << " " << robot_y << " "<< robot_z << " 0.000 0.000 "<< velocity << " 0.000 0.000 0.000 0.000 0.000"<<std::endl;
				buffer = buffer - 1;
			}

			while(velocity >= 0.0)
			{
				if(robot_z <= middleDist)
				{
					velocity = velocity + (acceleration * interval);
					robot_z = robot_z + (velocity * interval);
					std::cout <<std::fixed << std::setprecision(3) << robot_x << " " << robot_y << " "<< robot_z << " 0.000 0.000 "<< velocity << " 0.000 0.000 "<< acceleration << " 0.000 0.000"<<std::endl;
				}
				else
				{
					velocity = velocity + (deceleration * interval);
					robot_z = robot_z + (velocity * interval);
					std::cout <<std::fixed << std::setprecision(3) << robot_x << " " << robot_y << " "<< robot_z << " 0.000 0.000 "<< velocity << " 0.000 0.000 "<< deceleration << " 0.000 0.000"<<std::endl;
				}
				timecount=timecount+interval;
			}
			buffer = 400;
			while(buffer > 0)
			{
				std::cout <<std::fixed << std::setprecision(3) << robot_x << " " << robot_y << " "<< robot_z << " 0.000 0.000 "<< velocity << " 0.000 0.000 0.000 0.000 0.000"<<std::endl;
				buffer = buffer - 1;
			}

		}

		// void rotate(double robot_x, double robot_y, double start_yaw, double end_yaw)
		void rotate(double robot_x, double robot_y, double robot_z, double robot_yaw, double end_yaw)
		{
			double temp_start_yaw = robot_yaw, temp_end_yaw = end_yaw;
			if(temp_start_yaw < 0)
			{
				// temp_start_yaw = 360 + robot_yaw;
				temp_start_yaw = 2 * PI + robot_yaw;
			}
			if(temp_end_yaw < 0)
			{
				temp_end_yaw = 2 * PI + end_yaw;
				// temp_end_yaw = 360 + end_yaw;
			}

			double turn_angle;
			double diff1 = temp_end_yaw - temp_start_yaw;
			// std::cout << diff1 << std::endl;
			if(diff1 < 0)
			{
				// double diff2 = 360 + diff1;
				double diff2 = 2 * PI + diff1;
				if( fabs(diff1) < fabs(diff2) )
				{
					turn_angle = diff1;
				}
				else
				{
					turn_angle = diff2;
				}
			}
			else
			{
				// double diff2 = 360 - diff1;
				double diff2 = 2 * PI - diff1;
				if( fabs(diff1) < fabs(diff2) )
				{
					turn_angle = diff1;
				}
				else
				{
					turn_angle = -diff2;
				}
			}
			double sum_yaw = 0;
			double angular_vel_per_50ms, angular_vel_per_sec;
			if(turn_angle < 0)
			{
				angular_vel_per_50ms = -ANGULAR_VEL * PERIOD;
				angular_vel_per_sec = -ANGULAR_VEL;
			}
			else
			{
				angular_vel_per_50ms = ANGULAR_VEL * PERIOD;
				angular_vel_per_sec = ANGULAR_VEL;
			}
			std::cout << std::setprecision(3) << std::fixed << robot_x << " " << robot_y << " " << robot_z << " 0.000 0.000 0.000 0.000 0.000 0.000 " << robot_yaw << " " << angular_vel_per_sec << std::endl;
			while( sum_yaw < fabs(turn_angle) )
			{
				robot_yaw += angular_vel_per_50ms;
				std::cout << robot_x << " " << robot_y << " " << robot_z << " 0.000 0.000 0.000 0.000 0.000 0.000 " << robot_yaw << " " << angular_vel_per_sec << std::endl;
				sum_yaw += fabs(angular_vel_per_50ms);
			}
			std::cout << robot_x << " " << robot_y << " " << robot_z << " 0.000 0.000 0.000 0.000 0.000 0.000 " << robot_yaw << " " << angular_vel_per_sec << std::endl;
		}

			void landing()
		{
		  float acceleration = 0.3;
		  float deceleration = -0.3;

		  float velocity = 0.0;

		  float land_targetDist = 0.0;
		  float middleDist = robot_z / 2;

		  float liftof_time = 0.0;
		  float interval = 0.05;

		  float timecount= 0;
		  while (velocity <= 0.0)
		  {
		    //if(currentZ <= middleDist)
		    if(robot_z >= middleDist )
		    {
		      //velocity = velocity + (acceleration * interval);
		      velocity = velocity + (deceleration * interval);
		      robot_z = robot_z + (velocity * interval);
		      std::cout <<std::fixed << std::setprecision(3) << robot_x << " " << robot_y << " "<< robot_z << " 0.000 0.000 "<< velocity << " 0.000 0.000 "<< deceleration << " 0.000 0.000"<<std::endl;
		    }
		    else
		    {
		      //velocity = velocity + (deceleration * interval);
		      velocity = velocity + (acceleration * interval);
		      robot_z = robot_z + (velocity * interval);
		      std::cout <<std::fixed << std::setprecision(3) << robot_x << " " << robot_y << " "<< robot_z << " 0.000 0.000 "<< velocity << " 0.000 0.000 "<< acceleration << " 0.000 0.000"<<std::endl;
		    }

		    timecount=timecount+interval;
		  }
		  //Print out steady state
		  std::cout <<std::fixed << std::setprecision(3) << robot_x << " " << robot_y << " "<<"0.000" << " 0.000 0.000 "<< velocity << " 0.000 0.000 "<< "0.000" << " 0.000 0.000"<<std::endl;
		    //Print out to stop the propeller
		  std::cout <<std::fixed << std::setprecision(3) << robot_x << " " << robot_y << " "<<"-1.000" << " 0.000 0.000 "<< velocity << " 0.000 0.000 "<< "0.000" << " 0.000 0.000"<<std::endl;
		}

		void generateTrajectory(nav_msgs::Path &path)
		{
			std::ofstream myfile;
		  myfile.open("ideal_path.txt");
		  for(int i = 0; i < path.poses.size(); i++)
			{
				myfile << path.poses[i].pose.position.x << "," << path.poses[i].pose.position.y << std::endl;
			}
		  myfile.close();
			takeoff();
			// generateFlightTrajectory(path);
			landing();
		}

	/* ----- PATH PLANNER FUNCTIONS ----- */

	// Just continously publish data for display in rviz
	void loopActivity()
	{
		static int seq = 1;

		occupancy_grid.header.seq = seq;
		occupancy_grid.header.stamp = ros::Time::now();
		occupancy_grid.header.frame_id = "base_footprint";
		grid_pub.publish(occupancy_grid);
		path_pub.publish(a_star_path);

		seq++;
	}

	void putObstaclesOnGrid(int n)
	{
		// Read obstacles from file and mark on grid
		std::ifstream obsFile;
		obsFile.open(OBS_FILE);
		float obsIn[2];
		int size;
		obsFile >> size;
		for (int i=0; i<size; i++) {
			if (obsFile >> obsIn[0] >> obsIn[1]) {
				putCircleOnGrid(obsIn[0],obsIn[1]);
			}
		}
		obsFile.close();
	}

	void putCircleOnGrid(float x, float y) {
		int grid_x, grid_y;
		mapXyToGridXy(grid_x, grid_y, x, y);
		putCircleOnGrid(grid_x, grid_y, OBS_RADIUS);
	}

	void putCircleOnGrid(int x, int y, float radius)
	{
		int grid_radius = (int) (radius / GRID_RESOLUTION);
		for (int i = -grid_radius; i < grid_radius; i++)
		{
			for (int j = -grid_radius; j < grid_radius; j++)
			{
				int curr_x = x + i;
				int curr_y = y + j;
				if (validGridXy(curr_x,curr_y) && i*i+j*j<=grid_radius*grid_radius)
				{
					occupancy_grid.data[gridXyToGridI(curr_x,curr_y)] = 100;
				}
			}
		}
	}

	nav_msgs::Path generatePath()
	{
		// As a first step, generate a path between a start and an end goal
		// The next step would be to generate a *smooth* path for multiple goals

		// Get path(s)
		// TODO: Not hardcoded directions?
		// TODO: options

		a_star_path = a_star_search(CHECKPT_X,CHECKPT_Y, 0, START_X, START_Y);
		nav_msgs::Path a_star_path2 = a_star_search(CHECKPT_X,CHECKPT_Y, 4, GOAL_X, GOAL_Y);
		reversePathAndRenumber(a_star_path2, a_star_path.poses.size()+1);
		a_star_path.poses.insert(a_star_path.poses.end(), a_star_path2.poses.begin(), a_star_path2.poses.end());
		return a_star_path;
	}

	void initNeighbours() {
		// NOTE: direction taken according to north (y)
		float orig_x = GRID_RESOLUTION/2.0;
		float orig_y = GRID_RESOLUTION/2.0;

		for(int dir=0; dir<8; dir++){
			float direction_theta = PI/4 * dir;
			// Right turn neighbours
			for (int i=0; i<NUM_NEIGHBOURS_RIGHT; i++) {
				float theta = (i+1) * DRONE_TURN_STEP_RAD;
				float base_offset_x = DRONE_TURN_RADIUS * (1.0 - cos(theta));
				float base_offset_y = DRONE_TURN_RADIUS * sin(theta);
				float offset_x = (base_offset_y * sin(direction_theta)) + (base_offset_x * cos(direction_theta));
				float offset_y = (base_offset_y * cos(direction_theta)) - (base_offset_x * sin(direction_theta));

				neighbours_offset[dir][i][0] = (int) ((offset_x + orig_x)/GRID_RESOLUTION);
				neighbours_offset[dir][i][1] = (int) ((offset_y + orig_y)/GRID_RESOLUTION);
				neighbours_offset[dir][i][2] = (int) dir + (theta / (PI / 4));
				neighbours_offset_dist[dir][i] = estimateDistance(0,0,offset_x, offset_y);
			}
			// Left turn neighbours
			for (int i=NUM_NEIGHBOURS_RIGHT; i<NUM_NEIGHBOURS; i++) {
				float theta = (i-NUM_NEIGHBOURS_RIGHT+1) * DRONE_TURN_STEP_RAD;
				float base_offset_x = -(DRONE_TURN_RADIUS * (1.0 - cos(theta)));
				float base_offset_y = DRONE_TURN_RADIUS * sin(theta);
				float offset_x = (base_offset_y * sin(direction_theta)) + (base_offset_x * cos(direction_theta));
				float offset_y = (base_offset_y * cos(direction_theta)) - (base_offset_x * sin(direction_theta));

				neighbours_offset[dir][i][0] = (int) ((offset_x + orig_x)/GRID_RESOLUTION);
				neighbours_offset[dir][i][1] = (int) ((offset_y + orig_y)/GRID_RESOLUTION);
				neighbours_offset[dir][i][2] = (8 + dir - (int) (theta / (PI/4)))%8;
				neighbours_offset_dist[dir][i] = estimateDistance(0,0,offset_x, offset_y);
			}
		}
	}

	float arcDistance(float xOffset, float yOffset){
		// s = r*theta
		float alpha = atan(yOffset/xOffset);
		float theta = 180.0 - 2.0*alpha;
		return DRONE_TURN_RADIUS * theta;
	}

	nav_msgs::Path formPath(int sx, int sy, int sdir, int ex, int ey, int edir, std::vector< std::vector< std::vector<std::tuple<int, int, int> > > > parents) {
		// Path object setup
		static int path_header_seq = 1;
		nav_msgs::Path path;
		path.header.seq = path_header_seq++;
		path.header.stamp = ros::Time::now();
		path.header.frame_id = "base_footprint";

		int temp_x = ex;
		int temp_y = ey;
		int temp_dir = edir;
		int seq = 1;
		// generate the path using the parents table
		geometry_msgs::PoseStamped ps0;
		ps0.header.seq = seq;
		ps0.header.stamp = ros::Time::now();
		ps0.header.frame_id = "base_footprint";

		float temp_map_x0, temp_map_y0;
		gridXyToMapXy(temp_map_x0, temp_map_y0, temp_x, temp_y);
		ps0.pose.position.x = temp_map_x0;
		ps0.pose.position.y = temp_map_y0;
		path.poses.push_back(ps0);
		seq++;
		while( !(temp_x == sx && temp_y == sy && temp_dir == sdir) )
		{
			std::tuple<int, int, int> temp = parents[temp_x][temp_y][temp_dir];
			temp_x = std::get<0>(temp);
			temp_y = std::get<1>(temp);
			temp_dir = std::get<2>(temp);

			geometry_msgs::PoseStamped ps;
			ps.header.seq = seq;
			ps.header.stamp = ros::Time::now();
			ps.header.frame_id = "base_footprint";

			float temp_map_x, temp_map_y;
			gridXyToMapXy(temp_map_x, temp_map_y, temp_x, temp_y);
			ps.pose.position.x = temp_map_x;
			ps.pose.position.y = temp_map_y;
			path.poses.push_back(ps);
			seq++;
		}
		return path;
	}

	void reversePathAndRenumber(nav_msgs::Path &path, int start_renumber){
		int size = path.poses.size();
		for (int i=0; i<size/2; i++) {
			float temp = path.poses[i].pose.position.x;
			path.poses[i].pose.position.x = path.poses[size-i-1].pose.position.x;
			path.poses[size-i-1].pose.position.x = temp;
			temp = path.poses[i].pose.position.y;
			path.poses[i].pose.position.y = path.poses[size-i-1].pose.position.y;
			path.poses[size-i-1].pose.position.y = temp;
		}
	}


	bool isBlocked(MapCell arr[]) {
		// Check if the generated path is blocked by the obstacles
		for (int i=0; i<sizeof(arr)-1; i++) {
			int x = arr[i].x;
			int y = arr[i].y;
			if (isWall(x, y)) {
				return true;
			}
		}
		return false;
	}

	// Probably should clean this up. It's annoyingly long.
	nav_msgs::Path a_star_search(float map_start_x, float map_start_y, int start_direction, float map_end_x, float map_end_y) {
		// For ease of use, inputs are floats
		int start_x, start_y;
		int end_x, end_y;
		mapXyToGridXy(start_x, start_y, map_start_x, map_start_y);
		mapXyToGridXy(end_x, end_y, map_end_x, map_end_y);

		// A* search setup
		std::set<MapCell> pq;

		std::vector< std::vector< std::vector<float> > > distances(GRID_LENGTH, std::vector< std::vector<float> >(GRID_LENGTH, std::vector<float>(NUM_DIRECTIONS, MAX_DIST)));

		std::vector< std::vector< std::vector<std::tuple<int, int, int> > > > parents(GRID_LENGTH, std::vector< std::vector< std::tuple<int, int, int> > >(GRID_LENGTH, std::vector<std::tuple<int, int, int> >(NUM_DIRECTIONS, std::make_tuple(-1,-1,-1) )));

		// Start at the given direction.
		// Use this to control headings at checkpoint
		distances[start_x][start_y][start_direction] = 0;
		pq.insert(MapCell(start_x, start_y, 0, start_direction));

		while(!pq.empty()) {
			// Dequeue
			std::cout << "before fdosidforiwgr" << std::endl;
			MapCell current = *pq.begin();
			pq.erase(pq.begin());
			std::cout << "after fdosidforiwgr" << std::endl;

			// Check for goal
			if (current.x == end_x && current.y == end_y) {
				return formPath(start_x, start_y, start_direction, current.x, current.y, current.heading, parents);
			}

			// Add neighbours to queue
			// check right (break if blocked)
			for (int i=0; i<NUM_NEIGHBOURS_RIGHT; i++) {
				if (!checkIfWallAndEnqueueNeighbour(current, i, end_x, end_y, pq, distances, parents)) {
					break;
				}
			}

			// check left (break if blocked)
			for (int i=NUM_NEIGHBOURS_RIGHT; i<NUM_NEIGHBOURS; i++) {
				if (!checkIfWallAndEnqueueNeighbour(current, i, end_x, end_y, pq, distances, parents)) {
					break;
				}
			}
		}
		return formPath(start_x, start_y, start_direction, start_x, start_y, start_direction, parents);
	}

	// Returns false if invalid or neighbour is a wall
	bool checkIfWallAndEnqueueNeighbour(MapCell current, int neighbour_i, int end_x, int end_y, std::set<MapCell> &pq, std::vector< std::vector< std::vector<float> > > &distances, std::vector< std::vector< std::vector<std::tuple<int, int, int> > > > &parents) {
		int new_x = current.x + neighbours_offset[current.heading][neighbour_i][0];
		int new_y = current.y + neighbours_offset[current.heading][neighbour_i][1];
		int new_heading = neighbours_offset[current.heading][neighbour_i][2];
		float new_path_cost = current.distance + neighbours_offset_dist[current.heading][neighbour_i];
		float new_est_cost = new_path_cost + estimateDistance(new_x, new_y, end_x, end_y);

		if (validGridXy(new_x, new_y) && !isWall(new_x, new_y) && distances[new_x][new_y][new_heading]>new_path_cost) {
			pq.erase(MapCell(new_x, new_y, distances[new_x][new_y][new_heading], new_heading));
			distances[new_x][new_y][new_heading] = new_path_cost;
			parents[new_x][new_y][new_heading] = std::make_tuple(current.x,current.y,current.heading);

			pq.insert(MapCell(new_x, new_y, new_path_cost, new_heading));
		}

		return !validGridXy(new_x, new_y) || !isWall(new_x, new_y);
	}

	// Aux functions
	int gridXyToGridI(int x, int y)
	{
		return y*GRID_LENGTH + x;
	}
	void gridIToGridXy(int &grid_x, int &grid_y, int i)
	{
		// occupancy_grid.data uses a row-major order, so we need to convert
		grid_x = (int) (i % occupancy_grid.info.width);
		grid_y = (int) (i / occupancy_grid.info.width);
	}

	void mapXyToGridXy(int &grid_x, int &grid_y, float map_x, float map_y)
	{
		grid_x = (int)((map_x - occupancy_grid.info.origin.position.x)/GRID_RESOLUTION);
		grid_y = (int)((map_y - occupancy_grid.info.origin.position.y)/GRID_RESOLUTION);
	}
	void gridXyToMapXy(float &map_x, float &map_y, int grid_x, int grid_y)
	{
		map_x = occupancy_grid.info.origin.position.x + (grid_x + 0.5) * GRID_RESOLUTION;
		map_y = occupancy_grid.info.origin.position.y + (grid_y + 0.5) * GRID_RESOLUTION;
	}
	bool isWall(int x, int y)
	{
		int index = gridXyToGridI(x, y);
		int value = occupancy_grid.data[index];
		return (value == 100);
	}
	bool validGridXy(int x, int y)
	{
		return (x >= 0 && y >= 0 && x < GRID_LENGTH && y < GRID_LENGTH);
	}
	float estimateDistance(float x1, float y1, float x2, float y2)
	{
		return sqrt( pow(x1 - x2, 2) + pow(y1 - y2, 2) );
	}

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
	add_pose_to_path(desired_path, 0.0, 0.0);
	add_pose_to_path(desired_path, 0.1, 0.1);
	add_pose_to_path(desired_path, 0.2, 0.2);
	add_pose_to_path(desired_path, 0.3, 0.3);
	add_pose_to_path(desired_path, 0.4, 0.4);
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
	ros::init(argc, argv, "TrajectoryPlanner");
	ros::NodeHandle nh;
	TrajectoryPlanner tp(nh, START_X, START_Y);

	// nav_msgs::Path path = set_up_test_case();
	nav_msgs::Path path = tp.generatePath();
	tp.generateTrajectory(path);

	// ros::Rate loop_rate(10);
	// while(ros::ok())
	// {
	// 	tp.loopActivity();
	// 	ros::spinOnce();
	// 	loop_rate.sleep();
	// }
	// return 0;
}
