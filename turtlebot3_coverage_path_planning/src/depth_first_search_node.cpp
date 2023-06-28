#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <stack>
#include <nav_msgs/Path.h>

bool NewMapUpdate = false;
static nav_msgs::OccupancyGrid map_update;
void callback_map_update(const nav_msgs::OccupancyGrid &map_msg)
{
    map_update = map_msg;
    NewMapUpdate = true;
}

bool NewPath = false;
static nav_msgs::Path path_gmapping;
void path_gmapping_callback(const nav_msgs::Path &path_msg)
{
    path_gmapping = path_msg;
    NewPath = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_first_search_node");
    ros::NodeHandle nh;

    ros::Subscriber subscriber_map_update  = nh.subscribe("/map_update",  50, callback_map_update);
    ros::Subscriber subscriber_path_gmapping = nh.subscribe("/path_gmapping", 50, path_gmapping_callback);

    ros::Publisher publisher_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 50);
    ros::Publisher publisher_map_D_visit = nh.advertise<nav_msgs::OccupancyGrid>("/map_visit_D", 50);

    geometry_msgs::PoseStamped pose_goal;
    pose_goal.header.frame_id = "map";
    pose_goal.pose.position.z = 0;
    pose_goal.pose.orientation.w = 0.995;
    pose_goal.pose.orientation.x = 0;
    pose_goal.pose.orientation.y = 0;
    pose_goal.pose.orientation.z = 0.0896;

    std::stack <unsigned int> stack_x_cell;
    std::stack <unsigned int> stack_y_cell;

    unsigned int x_cell_current = 0;
    unsigned int y_cell_current = 0;
    unsigned int x_cell_goal = 0;
    unsigned int y_cell_goal = 0;
    unsigned int x_cell_old = 0;
    unsigned int y_cell_old = 0;

    nav_msgs::OccupancyGrid map_D_visit;
    map_D_visit.header.frame_id = "map";
    map_D_visit.info.resolution = 0.35;
    map_D_visit.info.width = 50;
    map_D_visit.info.height = 50;
    map_D_visit.info.origin.position.x = -map_D_visit.info.resolution * map_D_visit.info.width / 2;
    map_D_visit.info.origin.position.y = -map_D_visit.info.resolution * map_D_visit.info.height / 2;
    map_D_visit.data.resize(map_D_visit.info.width * map_D_visit.info.height);

    for (unsigned int i = 0; i < map_D_visit.info.width * map_D_visit.info.height; i++)
    {
       map_D_visit.data[i] = 50;
    }

    ros::Rate r(10);

    bool FlagStart = true;

    while (nh.ok())
    {
        if (NewPath && NewMapUpdate)
        {
            x_cell_current = (path_gmapping.poses.back().pose.position.x - map_update.info.origin.position.x) / map_update.info.resolution;
            y_cell_current = (path_gmapping.poses.back().pose.position.y - map_update.info.origin.position.y) / map_update.info.resolution;

            if (FlagStart)
            {
                if (map_update.data[(y_cell_current - 1) * map_update.info.width + x_cell_current] == 0 && map_D_visit.data[(y_cell_current - 1) * map_D_visit.info.width + x_cell_current] != -2)
                {
                    stack_x_cell.push(x_cell_current);
                    stack_y_cell.push(y_cell_current);
                    x_cell_goal = x_cell_current;
                    y_cell_goal = y_cell_current - 1;
                    x_cell_old = x_cell_current;
                    y_cell_old = y_cell_current;
                    pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;
                    pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;

                    map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                    publisher_goal.publish(pose_goal);
                }

                else if (map_update.data[y_cell_current * map_update.info.width + x_cell_current + 1] == 0 && map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current + 1] != -2)
                {
                    stack_x_cell.push(x_cell_current);
                    stack_y_cell.push(y_cell_current);

                    x_cell_goal = x_cell_current + 1;
                    y_cell_goal = y_cell_current;
                    x_cell_old = x_cell_current;
                    y_cell_old = y_cell_current;

                    pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;
                    pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;

                    map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                    publisher_goal.publish(pose_goal);
                }


                else if (map_update.data[(y_cell_current + 1) * map_update.info.width + x_cell_current] == 0 && map_D_visit.data[(y_cell_current + 1) * map_D_visit.info.width + x_cell_current] != -2)
                {   
                    stack_x_cell.push(x_cell_current);
                    stack_y_cell.push(y_cell_current);

                    x_cell_goal = x_cell_current;
                    y_cell_goal = y_cell_current + 1;
                    x_cell_old = x_cell_current;
                    y_cell_old = y_cell_current;

                    pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;
                    pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;

                    map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                    publisher_goal.publish(pose_goal);
                }

                else if (map_update.data[y_cell_current * map_update.info.width + x_cell_current - 1] == 0 && map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current - 1] != -2)
                {
                    stack_x_cell.push(x_cell_current);
                    stack_y_cell.push(y_cell_current);

                    x_cell_goal = x_cell_current - 1;
                    y_cell_goal = y_cell_current;
                    x_cell_old = x_cell_current;
                    y_cell_old = y_cell_current;

                    pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;;
                    pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;;

                    map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                    publisher_goal.publish(pose_goal);
                }  
                FlagStart = false;
            }

            else if (x_cell_current == x_cell_goal && y_cell_current == y_cell_goal && !stack_x_cell.empty() && !stack_y_cell.empty())
            {
                if (x_cell_current == x_cell_old + 1 && y_cell_current == y_cell_old)
                {
                    if (map_update.data[(y_cell_current - 1) * map_update.info.width + x_cell_current] == 0 && map_D_visit.data[(y_cell_current - 1) * map_D_visit.info.width + x_cell_current] != -2)
                    {
                        stack_x_cell.push(x_cell_current);
                        stack_y_cell.push(y_cell_current);

                        x_cell_goal = x_cell_current;
                        y_cell_goal = y_cell_current - 1;
                        x_cell_old = x_cell_current;
                        y_cell_old = y_cell_current;

                        pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;
                        pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;

                        map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                        publisher_goal.publish(pose_goal);
                    }

                    else if (map_update.data[y_cell_current * map_update.info.width + x_cell_current + 1] == 0 && map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current + 1] != -2)
                    {
                        stack_x_cell.push(x_cell_current);
                        stack_y_cell.push(y_cell_current);

                        x_cell_goal = x_cell_current + 1;
                        y_cell_goal = y_cell_current;
                        x_cell_old = x_cell_current;
                        y_cell_old = y_cell_current;

                        pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;
                        pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;

                        map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                        publisher_goal.publish(pose_goal);
                    }


                    else if (map_update.data[(y_cell_current + 1) * map_update.info.width + x_cell_current] == 0 && map_D_visit.data[(y_cell_current + 1) * map_D_visit.info.width + x_cell_current] != -2)
                    {   
                        stack_x_cell.push(x_cell_current);
                        stack_y_cell.push(y_cell_current);

                        x_cell_goal = x_cell_current;
                        y_cell_goal = y_cell_current + 1;
                        x_cell_old = x_cell_current;
                        y_cell_old = y_cell_current;

                        pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;
                        pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;

                        map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                        publisher_goal.publish(pose_goal);
                    }

                    else
                    {
                        x_cell_goal = stack_x_cell.top();
                        y_cell_goal = stack_y_cell.top();
                        x_cell_old = x_cell_current;
                        y_cell_old = y_cell_current;

                        stack_x_cell.pop();
                        stack_y_cell.pop();

                        pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;
                        pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;

                        map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                        publisher_goal.publish(pose_goal);
                    }
                }
//-------------------------------------------------------------------------------------------------------------------------------------------------------
                else if (x_cell_current == x_cell_old  && y_cell_current == y_cell_old - 1)
                {
                    if (map_update.data[y_cell_current * map_update.info.width + x_cell_current - 1] == 0 && map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current - 1] != -2)
                    {
                        stack_x_cell.push(x_cell_current);
                        stack_y_cell.push(y_cell_current);

                        x_cell_goal = x_cell_current - 1;
                        y_cell_goal = y_cell_current;
                        x_cell_old = x_cell_current;
                        y_cell_old = y_cell_current;

                        pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;;
                        pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;;

                        map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                        publisher_goal.publish(pose_goal);
                    }

                    else if (map_update.data[(y_cell_current - 1) * map_update.info.width + x_cell_current] == 0 && map_D_visit.data[(y_cell_current - 1) * map_D_visit.info.width + x_cell_current] != -2)
                    {
                        stack_x_cell.push(x_cell_current);
                        stack_y_cell.push(y_cell_current);

                        x_cell_goal = x_cell_current;
                        y_cell_goal = y_cell_current - 1;
                        x_cell_old = x_cell_current;
                        y_cell_old = y_cell_current;

                        pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;
                        pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;

                        map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                        publisher_goal.publish(pose_goal);
                    }

                    else if (map_update.data[y_cell_current * map_update.info.width + x_cell_current + 1] == 0 && map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current + 1] != -2)
                    {
                        stack_x_cell.push(x_cell_current);
                        stack_y_cell.push(y_cell_current);

                        x_cell_goal = x_cell_current + 1;
                        y_cell_goal = y_cell_current;
                        x_cell_old = x_cell_current;
                        y_cell_old = y_cell_current;

                        pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;
                        pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;

                        map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                        publisher_goal.publish(pose_goal);
                    }

                    else
                    {
                        x_cell_goal = stack_x_cell.top();
                        y_cell_goal = stack_y_cell.top();
                        x_cell_old = x_cell_current;
                        y_cell_old = y_cell_current;

                        stack_x_cell.pop();
                        stack_y_cell.pop();

                        pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;
                        pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;

                        map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                        publisher_goal.publish(pose_goal);
                    }
                }
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
                else if (x_cell_current == x_cell_old - 1  && y_cell_current == y_cell_old)
                {
                    if (map_update.data[(y_cell_current + 1) * map_update.info.width + x_cell_current] == 0 && map_D_visit.data[(y_cell_current + 1) * map_D_visit.info.width + x_cell_current] != -2)
                    {
                        stack_x_cell.push(x_cell_current);
                        stack_y_cell.push(y_cell_current);

                        x_cell_goal = x_cell_current;
                        y_cell_goal = y_cell_current + 1;
                        x_cell_old = x_cell_current;
                        y_cell_old = y_cell_current;

                        pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;;
                        pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;;

                        map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                        publisher_goal.publish(pose_goal);
                    }

                    else if (map_update.data[y_cell_current * map_update.info.width + x_cell_current - 1] == 0 && map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current - 1] != -2)
                    {
                        stack_x_cell.push(x_cell_current);
                        stack_y_cell.push(y_cell_current);

                        x_cell_goal = x_cell_current - 1;
                        y_cell_goal = y_cell_current;
                        x_cell_old = x_cell_current;
                        y_cell_old = y_cell_current;

                        pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;
                        pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;

                        map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                        publisher_goal.publish(pose_goal);
                    }

                    else if (map_update.data[(y_cell_current - 1) * map_update.info.width + x_cell_current] == 0 && map_D_visit.data[(y_cell_current - 1) * map_D_visit.info.width + x_cell_current] != -2)
                    {
                        stack_x_cell.push(x_cell_current);
                        stack_y_cell.push(y_cell_current);

                        x_cell_goal = x_cell_current;
                        y_cell_goal = y_cell_current - 1;
                        x_cell_old = x_cell_current;
                        y_cell_old = y_cell_current;

                        pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;
                        pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;

                        map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                        publisher_goal.publish(pose_goal);
                    }

                    else
                    {
                        x_cell_goal = stack_x_cell.top();
                        y_cell_goal = stack_y_cell.top();
                        x_cell_old = x_cell_current;
                        y_cell_old = y_cell_current;

                        stack_x_cell.pop();
                        stack_y_cell.pop();

                        pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;
                        pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;

                        map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                        publisher_goal.publish(pose_goal);
                    }
                }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                else if (x_cell_current == x_cell_old && y_cell_current == y_cell_old + 1)
                {
                    if (map_update.data[y_cell_current * map_update.info.width + x_cell_current + 1] == 0 && map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current + 1] != -2)
                    {
                        stack_x_cell.push(x_cell_current);
                        stack_y_cell.push(y_cell_current);

                        x_cell_goal = x_cell_current + 1;
                        y_cell_goal = y_cell_current;
                        x_cell_old = x_cell_current;
                        y_cell_old = y_cell_current;

                        pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;
                        pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;

                        map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                        publisher_goal.publish(pose_goal);

                    }

                    else if (map_update.data[(y_cell_current + 1) * map_update.info.width + x_cell_current] == 0 && map_D_visit.data[(y_cell_current + 1) * map_D_visit.info.width + x_cell_current] != -2)
                    {
                        stack_x_cell.push(x_cell_current);
                        stack_y_cell.push(y_cell_current);

                        x_cell_goal = x_cell_current;
                        y_cell_goal = y_cell_current + 1;
                        x_cell_old = x_cell_current;
                        y_cell_old = y_cell_current;

                        pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;;
                        pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;;

                        map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                        publisher_goal.publish(pose_goal);
                    }

                    else if (map_update.data[y_cell_current * map_update.info.width + x_cell_current - 1] == 0 && map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current - 1] != -2)
                    {
                        stack_x_cell.push(x_cell_current);
                        stack_y_cell.push(y_cell_current);

                        x_cell_goal = x_cell_current - 1;
                        y_cell_goal = y_cell_current;
                        x_cell_old = x_cell_current;
                        y_cell_old = y_cell_current;

                        pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;;
                        pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;;

                        map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;   

                        publisher_goal.publish(pose_goal);
                    }

                    else
                    {
                        x_cell_goal = stack_x_cell.top();
                        y_cell_goal = stack_y_cell.top();
                        x_cell_old = x_cell_current;
                        y_cell_old = y_cell_current;

                        stack_x_cell.pop();
                        stack_y_cell.pop();

                        pose_goal.pose.position.x = ((float)x_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;
                        pose_goal.pose.position.y = ((float)y_cell_goal - map_update.info.width / 2) * map_update.info.resolution + 0.175;

                        map_D_visit.data[y_cell_current * map_D_visit.info.width + x_cell_current] = -2;

                        publisher_goal.publish(pose_goal);
                    }
                }
            }

            publisher_map_D_visit.publish(map_D_visit);
            NewPath = false;
            NewMapUpdate = false;
        }
        ros::spinOnce();
        r.sleep();
    }
}
