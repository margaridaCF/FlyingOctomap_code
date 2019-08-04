#include <gtest/gtest.h>
#include <math.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <fstream>
#include <string>
#include <thread>
#include <ltStar_lib_ortho.h>

// terminal: catkin build --verbose --catkin-make-args run_tests | sed -n '/\[==========\]/,/\[==========\]/p'

namespace LazyThetaStarOctree
{
    ros::Publisher ltstar_request_pub;

    class MyTestSuite : public ::testing::Test {
       public:
        MyTestSuite() {

            geofence_min.x= -40;
            geofence_max.x= 30;

            geofence_min.y= -18;
            geofence_max.y= 20;

            geofence_min.z= 4;
            geofence_max.z= 35;

            max_time_secs = 1;
            ltstar_safety_margin = 5;
        }
        ~MyTestSuite() {}

        int max_time_secs;
        double ltstar_safety_margin;
        geometry_msgs::Point geofence_min, geofence_max;

        
    };




    void askForObstacleAvoidingPath(geometry_msgs::Point start, geometry_msgs::Point goal, int max_time_secs, double ltstar_safety_margin)
    {
        lazy_theta_star_msgs::LTStarRequest request;
        request.request_id = 0;
        request.header.frame_id = "world";
        request.start = start;
        request.goal  = goal;
        request.max_time_secs = max_time_secs;
        request.safety_margin = ltstar_safety_margin;
        ROS_INFO_STREAM ("[State manager] Requesting path from " << request.start << " to " << request.goal);
        ltstar_request_pub.publish(request);
    }

    TEST_F(MyTestSuite, ltstar) {
        geometry_msgs::Point start, goal;
        start.x = 0;
        start.y = 0;
        start.z = 2;
        goal.x = 0;
        goal.y = 0;
        goal.z = 2;
        geofence_min.x= -40;
        geofence_max.x= 30;

        geofence_min.y= -18;
        geofence_max.y= 20;

        geofence_min.z= 4;
        geofence_max.z= 35;
        askForObstacleAvoidingPath(start, goal, max_time_secs, ltstar_safety_margin);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tests_node");
    ros::NodeHandle nh;


    LazyThetaStarOctree::ltstar_request_pub = nh.advertise<lazy_theta_star_msgs::LTStarRequest>("ltstar_request", 10);

    testing::InitGoogleTest(&argc, argv);

    std::thread t([] {while(ros::ok()) ros::spin(); });

    auto res = RUN_ALL_TESTS();

    ros::shutdown();

    return res;
}
