#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped uav_pose;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_poses");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);

    std::ofstream file_uav_poses, file_uav_poses_times;
    file_uav_poses.open("/home/hector/matlab_ws/TFM/_LazyTheta/uav_poses");
    file_uav_poses_times.open("/home/hector/matlab_ws/TFM/_LazyTheta/uav_poses_times");
    int cont_start_time = 0;
    double start_time;

    ros::Rate rate(1);
    while (ros::ok())
    {
        if (cont_start_time == 0)
        {
            start_time = ros::Time::now().toSec();
            cont_start_time++;
        }
        file_uav_poses << uav_pose.pose.position.x << " " << uav_pose.pose.position.y << " " << uav_pose.pose.position.z << " "
                       << uav_pose.pose.orientation.x << " " << uav_pose.pose.orientation.y << " " << uav_pose.pose.orientation.z << " "
                       << uav_pose.pose.orientation.w << std::endl;

        file_uav_poses_times << ros::Time::now().toSec() - start_time << std::endl;
        ros::spinOnce();
        rate.sleep();
    }
    file_uav_poses.close();
    file_uav_poses_times.close();
}
