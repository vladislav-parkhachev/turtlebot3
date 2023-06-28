#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

bool NewMapData = false;
nav_msgs::OccupancyGrid map_SLAM;
void message_callback_map_SLAM(const nav_msgs::OccupancyGrid &map_msg)
{
    map_SLAM = map_msg;
    NewMapData = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_update_node");
    ros::NodeHandle nh;
    ros::Subscriber subscriber_map_SLAM = nh.subscribe("/map", 50, message_callback_map_SLAM);
    ros::Publisher publisher_map_update = nh.advertise<nav_msgs::OccupancyGrid>("/map_update", 50);

    nav_msgs::OccupancyGrid map_passable;
    map_passable.header.frame_id = "map";
    map_passable.info.resolution = 0.35;
    map_passable.info.width = 50;
    map_passable.info.height = 50;
    map_passable.info.origin.position.x = -map_passable.info.resolution * map_passable.info.width / 2;
    map_passable.info.origin.position.y = -map_passable.info.resolution * map_passable.info.height / 2;
    map_passable.data.resize(map_passable.info.width * map_passable.info.height);

    for (unsigned int x_current = 0; x_current < map_passable.info.width; x_current++)
    {
        for (unsigned int y_current = 0; y_current < map_passable.info.height; y_current++)
        {
            map_passable.data[y_current * map_passable.info.width + x_current] = 50;
        }
    }

    nav_msgs::OccupancyGrid map_not_passable = map_passable;
    nav_msgs::OccupancyGrid map_update = map_passable;

    ros::Rate r(1);

    while (nh.ok())
    {
        if (NewMapData)
        {
            for (unsigned int x_current = 0; x_current < map_update.info.width; x_current++)
            {
                for (unsigned int y_current = 0; y_current < map_update.info.height; y_current++)
                {
                    map_passable.data[y_current * map_passable.info.width + x_current] = 50;
                    map_not_passable.data[y_current * map_not_passable.info.width + x_current] = 50;
                    map_update.data[y_current * map_update.info.width + x_current] = 50;
                }
            }

            for (unsigned int x_current = 0; x_current < map_SLAM.info.width; x_current++)
            {
                for (unsigned int y_current = 0; y_current < map_SLAM.info.height; y_current++)
                {

                    if(map_SLAM.data[y_current * map_SLAM.info.width + x_current] == 0)
                    {
                        float x_coordinat = x_current * map_SLAM.info.resolution + map_SLAM.info.origin.position.x;
                        float y_coordinat = y_current * map_SLAM.info.resolution + map_SLAM.info.origin.position.y;

                        unsigned int x_cell_D = (x_coordinat - map_passable.info.origin.position.x) / map_passable.info.resolution;
                        unsigned int y_cell_D = (y_coordinat - map_passable.info.origin.position.y) / map_passable.info.resolution;

                        map_passable.data[y_cell_D * map_passable.info.width + x_cell_D] = 0;
                    }

                    if(map_SLAM.data[y_current * map_SLAM.info.width + x_current] == 100)
                    {
                        float x_coordinat = x_current * map_SLAM.info.resolution + map_SLAM.info.origin.position.x;
                        float y_coordinat = y_current * map_SLAM.info.resolution + map_SLAM.info.origin.position.y;

                        unsigned int x_cell_D = (x_coordinat - map_passable.info.origin.position.x) / map_passable.info.resolution;
                        unsigned int y_cell_D = (y_coordinat - map_passable.info.origin.position.y) / map_passable.info.resolution;

                        map_not_passable.data[y_cell_D * map_not_passable.info.width + x_cell_D] = 100;
                    }
                }
            }

            for (unsigned int x_current = 0; x_current < map_update.info.width; x_current++)
            {
                for (unsigned int y_current = 0; y_current < map_update.info.height; y_current++)
                {
                    if (map_passable.data[y_current * map_passable.info.width + x_current] == 0  && map_not_passable.data[y_current * map_not_passable.info.width + x_current] != 100)
                    {
                        map_update.data[y_current * map_passable.info.width + x_current] = 0;
                    }
                }
            }

            publisher_map_update.publish(map_update);
            NewMapData = false;
        }
        ros::spinOnce();
        r.sleep();
    }
}
