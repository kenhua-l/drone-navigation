#include <iostream>
#include <tuple>
#include <fstream>

#include <stdlib.h>
#include <math.h>
#include "ros/ros.h"

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

// Map definitions
#define MAP_SIZE        5
#define MAP_START_X			0
#define MAP_START_Y			0
#define GRID_STEPS      10
#define GRID_RESOLUTION (1.0/GRID_STEPS)
#define GRID_LENGTH     (MAP_SIZE * GRID_STEPS + 1)
#define GRID_N          (GRID_LENGTH * GRID_LENGTH)
// Problem definitions
#define OBS_FILE 				"/home/yzxj/Part2/obs.txt"
#define OBS_RADIUS			0.60
#define START_X					1.5
#define START_Y					1.5
#define GOAL_X					3.5
#define GOAL_Y					3.5
#define CHECKPT_X				2.5
#define CHECKPT_Y				3.5
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
	nav_msgs::OccupancyGrid occupancy_grid;
	nav_msgs::Path a_star_path;
	ros::Publisher grid_pub;
	ros::Publisher path_pub;
	int neighbours_offset[NUM_DIRECTIONS][NUM_NEIGHBOURS][3];
	float neighbours_offset_dist[NUM_DIRECTIONS][NUM_NEIGHBOURS];

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
		initNeighbours();
		putObstaclesOnGrid(3);

		// Get path(s)
		a_star_path = a_star_search(CHECKPT_X,CHECKPT_Y, 6, START_X, START_Y);
		// nav_msgs::Path a_star_path2 = a_star_search(CHECKPT_X,CHECKPT_Y, 4, GOAL_X, GOAL_Y);
		// reversePath(a_star_path);
		// a_star_path.poses.insert(a_star_path.poses.end(), a_star_path2.poses.begin(), a_star_path2.poses.end());
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
		std::cout << "PUBLISH" << std::endl;

		seq++;
	}

	void putObstaclesOnGrid(int n)
	{
		// Read obstacles from file and mark on grid
		std::ifstream obsFile;
		obsFile.open(OBS_FILE);
		float obsIn[2];
		for (int i=0; i<n; i++) {
			if (obsFile >> obsIn[0] >> obsIn[1]) {
				putCircleOnGrid(obsIn[0],obsIn[1],OBS_RADIUS);
			}
		}
		obsFile.close();
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
				neighbours_offset_dist[dir][i] = arcDistance(offset_x, offset_y);
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
				neighbours_offset_dist[dir][i] = arcDistance(offset_x, offset_y);
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
			path.poses.insert(path.poses.begin(),ps);
			seq++;
		}
		return path;
	}

	void reversePath(nav_msgs::Path path){
		std::reverse(path.poses.begin(), path.poses.end());
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
		// TODO: Implement reversePath()
		distances[start_x][start_y][start_direction] = 0;
		pq.insert(MapCell(start_x, start_y, 0, start_direction));

		while(!pq.empty()) {
			// Dequeue
			MapCell current = *pq.begin();
			pq.erase(pq.begin());

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

	// Returns false if neighbour is a wall
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

		return !isWall(new_x, new_y);
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
