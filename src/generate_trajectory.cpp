#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include <math.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

// Map definitions
#define MAP_SIZE        5
#define MAP_START_X			-2.5
#define MAP_START_Y			-2.5
#define GRID_STEPS      10
#define GRID_RESOLUTION (1.0/GRID_STEPS)
#define GRID_LENGTH     (MAP_SIZE * GRID_STEPS)
#define GRID_N          (GRID_LENGTH * GRID_LENGTH)
// Problem definitions
#define OBS_RADIUS			0.60
#define START_X					-1.5
#define START_Y					1.5
#define CHECKPT_X				2
#define CHECKPT_Y				0
#define GOAL_X					-1.5
#define GOAL_Y					-1.5

class TrajectoryPlanner
{
	private:
		nav_msgs::OccupancyGrid occupancy_grid;
		nav_msgs::Path a_star_path;
    ros::Publisher grid_pub;
    ros::Publisher path_pub;

    // Class used for path-finding
    class MapCell
    {
      public:
        int x;
        int y;
				float distance;

        MapCell(int x, int y, float distance)
        {
          this->x = x;
          this->y = y;
          this->distance = distance;
        }

        bool operator<(MapCell other) const
        {
          return distance < other.distance;
        }
  	};

	public:
		TrajectoryPlanner(ros::NodeHandle &nh)
		{
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

			// Some setup
			putObstaclesOnGrid();
			a_star_path = a_star_search(START_X,START_Y,CHECKPT_X,CHECKPT_Y);
			nav_msgs::Path a_star_path2 = a_star_search(CHECKPT_X, CHECKPT_Y, GOAL_X, GOAL_Y);
			a_star_path.poses.insert(a_star_path.poses.end(), a_star_path2.poses.begin(), a_star_path2.poses.end());
		}

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

		void putObstaclesOnGrid()
		{
			// TODO: Read obstacles from file
			float obs[][2] = {{1,1},{1,-1},{-0.5,-0.5}};
			// Create 60cm buffer around obstacles
			for (int i=0; i<3; i++) {
				putCircleOnGrid(obs[i][0],obs[i][1],OBS_RADIUS);
			}
		}

		void putCircleOnGrid(float map_x, float map_y, float radius)
		{
			int x, y;
			mapXyToGridXy(x,y,map_x,map_y);

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

		// For testing only.
		void putRectOnGrid(int x1, int y1, int x2, int y2)
		{
			for (int i=x1; i<x2; i++)
			{
				for (int j=y1; j<y2; j++)
				{
					occupancy_grid.data[gridXyToGridI(i,j)] = 100;
				}
			}
		}

		nav_msgs::Path generatePath()
		{
			// As a first step, generate a path between a start and an end goal
			// The next step would be to generate a *smooth* path for multiple goals
		}

    // Probably should clean this up. It's annoyingly long.
    nav_msgs::Path a_star_search(float map_start_x, float map_start_y, float map_end_x, float map_end_y)
    {
			// Convert from map to grid
			int start_x, start_y;
			int end_x, end_y;
			mapXyToGridXy(start_x, start_y, map_start_x, map_start_y);
			mapXyToGridXy(end_x, end_y, map_end_x, map_end_y);

      // Path object setup
      static int path_header_seq = 1;
      nav_msgs::Path path;
      path.header.seq = path_header_seq++;
      path.header.stamp = ros::Time::now();
      path.header.frame_id = "base_footprint";

      // A* search setup
      std::set<MapCell> pq;
      std::vector< std::vector<float> > distances(GRID_LENGTH, std::vector<float>(GRID_LENGTH, 99999));
      std::vector< std::vector< std::pair<int, int> > > parents(GRID_LENGTH, std::vector< std::pair<int, int> >(GRID_LENGTH, std::make_pair(-1,-1) ));
      distances[end_x][end_y] = 0;
      pq.insert(MapCell(end_x, end_y, estimateDistance(start_x, start_y, end_x, end_y)));

      bool found = false;
      while(!pq.empty() && found == false)
      {
        // Dequeue
        MapCell current = *pq.begin();
        pq.erase(pq.begin());

        // Check all neighbours
        for(int i = -1; i < 2; i++)
        {
          for(int j = -1; j < 2; j++)
          {
            int temp_x = current.x + i;
            int temp_y = current.y + j;

            if(validGridXy(temp_x, temp_y))
            {
							// Using estimateDistance gives realistic path cost,
							// penalizing diagonal moves.
							float new_path_dist = distances[current.x][current.y] + estimateDistance(0,0,i,j);
              float new_distance = new_path_dist + estimateDistance(temp_x, temp_y, start_x, start_y);
              // Relax neighbour
              if(isWall(temp_x, temp_y))
              {
                distances[temp_x][temp_y] = -1;
              }
              else
              {
                if(distances[temp_x][temp_y] > new_path_dist)
                {
                  // If new_distance is shorter, replace it in the queue
                  pq.erase(MapCell(temp_x, temp_y, distances[temp_x][temp_y]));
                  distances[temp_x][temp_y] = new_path_dist;
                  parents[temp_x][temp_y] = std::make_pair(current.x, current.y);

                  MapCell temp = MapCell(temp_x, temp_y, new_distance);
                  pq.insert(temp);

                  // If at the start node (goal node)
                  // generate the path using the parents table
                  if(start_x == temp_x && start_y == temp_y)
                  {
                    int seq = 1;
                    while( !(temp_x == end_x && temp_y == end_y) )
                    {
                      std::pair<int, int> temp = parents[temp_x][temp_y];
                      temp_x = temp.first;
                      temp_y = temp.second;

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
                    if (seq>1)
                    {
                      // The first node should be the robot's position
                      // which is unnecessary
                      path.poses.erase(path.poses.begin());
                    }
                    return path;
                  }
                }
              }
            }
          }
        }
      }
      return path;
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
			// Adds 0.5 to get center of grid cell
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



		void generateTrajectory(nav_msgs::Path path)
		{
			// Generate the velocity, acceleration profile and write it to a file
		}

		void writeTrajectoryToFile()
		{
			// Format: x y z vx vy vz ax ay az head headv
		}

		bool isBlocked(MapCell arr[]) {
			// Check if the generated path is blocked by the obstacles
			for (int i=0; i<arr.length-1; i++) {
				int x = arr[i].x;
				int y = arr[i].y;
				if (isWall(x, y)) {
					return true;
				}
			}
			return false;
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "TrajectoryPlanner");
	ros::NodeHandle nh;
	TrajectoryPlanner tp(nh);

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
		tp.loopActivity();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
