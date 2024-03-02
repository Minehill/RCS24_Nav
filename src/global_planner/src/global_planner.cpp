#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <queue>

const int cost = 1;

std::vector<std::vector<int>> map;

int width, height;
float resolution;
int origin_x, origin_y;

struct Point{
    int x,y;
    int g,h,f;
    Point* parent;

    Point(int x = -1, int y = -1, int g = 0, int h = 0, int f = 0) 
        : x(x), y(y), g(g), h(h), f(f), parent(nullptr) {}
};
Point startPoint,endPoint;

void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    ROS_INFO("Map is loading...");
    width = msg->info.width;
    height = msg->info.height;
    resolution = msg->info.resolution;
    origin_x = -msg->info.origin.position.y / resolution;
    origin_y = -msg->info.origin.position.x / resolution;
    for(int i=0;i<width;i++) map.push_back(std::vector<int>());
	for(int i=0;i<width;i++){
		for(int j=0;j<height;j++){
			map[i].push_back(msg->data[j*width+i]);
		}
	}
    ROS_INFO("Map is loaded! Map info: width = %d, height = %d, resolution = %f, origin_x = %d, origin_y = %d", width, height, resolution, origin_x, origin_y);
}

std::vector<Point*> getNeighbors(Point* p){
    std::vector<Point*> neighbors;
    for(int i=-1;i<=1;i++){
        for(int j=-1;j<=1;j++){
            if(i == 0 && j == 0) continue;
            if(i != 0 && j != 0) continue;
            neighbors.push_back(new Point(p->x + i, p->y + j));
        }
    }
    return neighbors;
}

std::vector<Point*> getPath(Point* start, Point* end){
    // 初始化table，0-未访问，-1-已访问（在closelist中），>0-代价g
    std::vector<std::vector<int>> table(width, std::vector<int>(height));
    for(auto& row : table) {
        std::fill(row.begin(), row.end(), 0);
    }

    std::priority_queue<Point*, std::vector<Point*>, bool(*)(Point*, Point*)> openList([](Point* a, Point* b){return a->f > b->f;});
    openList.push(start);
    table[start->x][start->y] = start->g;

    while(!openList.empty()){
        // 取出f值最小的点
        Point* current = openList.top();
        
        // 将当前点放入closelist中
        openList.pop();
        table[current->x][current->y] = -1;

        // 判断当前点是否目标点
        if(current->x == end->x && current->y == end->y){
            ROS_INFO("Path found!");
            // 回溯路径
            end->parent = current;
            std::vector<Point*> path;
            Point* p = end;
            while(p){
                path.push_back(p);
                p = p->parent;
            }
            ROS_INFO("path's length is : %ld", path.size());
            return path;
        }

        // 获取周围点
        std::vector<Point*> neighbors = getNeighbors(current);

        // 遍历周围点
        for(auto neighbor : neighbors){
            // 判断是否在网格内
            if(neighbor->x < 0 || neighbor->x >= width || neighbor->y < 0 || neighbor->y >= height) continue;

            // 判断是否是障碍物
            if(map[neighbor->y][neighbor->x] == 100) continue;

            // 判断是否在closelist中
            if(table[neighbor->x][neighbor->y] == -1) continue;

            // 计算代价
            neighbor->g = current->g + cost;
            neighbor->h = abs(neighbor->x - end->x) + abs(neighbor->y - end->y);
            neighbor->f = neighbor->g + neighbor->h;
            neighbor->parent = current;

            // 判断否在openlist中，如果在，需要比较当前是否更优
            if(table[neighbor->x][neighbor->y] > 0){
                if(neighbor->g < table[neighbor->x][neighbor->y]){
                    table[neighbor->x][neighbor->y] = neighbor->g;
                    neighbor->parent = current;
                }
            }
            // 如果不在openlist中，将其放入openlist
            else{
                openList.push(neighbor);
                table[neighbor->x][neighbor->y] = neighbor->g;
            }

        }
        
    }
    ROS_INFO("Path not found!");
    return {};
}

void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped msg){
    startPoint.x = msg.pose.pose.position.y / resolution + origin_x;
    startPoint.y = msg.pose.pose.position.x / resolution + origin_y;
    ROS_INFO("Init pose: x = %d, y = %d", startPoint.x, startPoint.y);

}

void GoalPoseCallback(const geometry_msgs::PoseStamped msg){
    endPoint.x = msg.pose.position.y / resolution + origin_x;
    endPoint.y = msg.pose.position.x / resolution + origin_y;
    startPoint.h = abs(startPoint.x - endPoint.x) + abs(startPoint.y - endPoint.y);
    ROS_INFO("Goal pose: x = %d, y = %d", endPoint.x, endPoint.y);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "global_planner");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("/map", 1, MapCallback);
    ros::Subscriber init_pose_sub = nh.subscribe("/initialpose", 1, InitPoseCallback);
    ros::Subscriber goal_pose_sub = nh.subscribe("/move_base_simple/goal", 1, GoalPoseCallback);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/global_path", 1);

    ros::Rate loop_rate(10);
    while(ros::ok()){
        if(map.size() > 0){
            if(startPoint.x != -1 && startPoint.y != -1 && endPoint.x != -1 && endPoint.y != -1){
                std::vector<Point*> path = getPath(&startPoint, &endPoint);
                if(path.size() > 0){
                    nav_msgs::Path path_msg;
                    path_msg.header.stamp = ros::Time::now();
                    path_msg.header.frame_id = "map";
                    for(auto it = path.rbegin(); it != path.rend(); it++){
                        geometry_msgs::PoseStamped pose;
                        pose.pose.position.x = ((*it)->y - origin_y) * resolution;
                        pose.pose.position.y = ((*it)->x - origin_x) * resolution;
                        pose.pose.orientation.w = 1;
                        path_msg.poses.push_back(pose);
                    }
                    path_pub.publish(path_msg);
                }
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
