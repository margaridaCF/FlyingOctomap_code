#include <gtest/gtest.h>
#include <goal_state_machine.h>


namespace goal_state_machine
{
    TEST(GoalStateMachine, UnobservableTest)
    {
    frontiers_msgs::FrontierReply frontiers_msg;
    double distance_inFront = 2;
    double distance_behind = 2;
    double ltstar_safety_margin = 5;
    int circle_divisions = 12;
    geometry_msgs::Point geofence_min_point, geofence_max_point;
    geofence_min_point.x = 0;
    geofence_min_point.y = 0;
    geofence_min_point.z = 0;
    geofence_max_point.x = 0;
    geofence_max_point.y = 0;
    geofence_max_point.z = 0;
    ros::Publisher marker_pub;
    rviz_interface::PublishingInput pi (marker_pub);
    ros::ServiceClient check_flightCorridor_client;
    GoalStateMachine goal_state_machine (frontiers_msg, distance_inFront, distance_behind, circle_divisions, geofence_min_point, geofence_max_point, pi, check_flightCorridor_client, ltstar_safety_margin);

    Eigen::Vector3d unobservable(1, 2, 3);
    Eigen::Vector3d viewpoint   (0, 0, 0);


    goal_state_machine.DeclareUnobservable(unobservable, viewpoint);
    ASSERT_TRUE( goal_state_machine.IsUnobservable(unobservable, viewpoint));
     
    }

  TEST(GoalStateMachine, UnobservableTest_Tolerance)
  {
    frontiers_msgs::FrontierReply frontiers_msg;
    double distance_inFront = 2;
    double distance_behind = 2;
    double ltstar_safety_margin = 5;
    int circle_divisions = 12;
    geometry_msgs::Point geofence_min_point, geofence_max_point;
    geofence_min_point.x = 0;
    geofence_min_point.y = 0;
    geofence_min_point.z = 0;
    geofence_max_point.x = 0;
    geofence_max_point.y = 0;
    geofence_max_point.z = 0;
    ros::Publisher marker_pub;
    rviz_interface::PublishingInput pi (marker_pub);
    ros::ServiceClient check_flightCorridor_client;
    GoalStateMachine goal_state_machine (frontiers_msg, distance_inFront, distance_behind, circle_divisions, geofence_min_point, geofence_max_point, pi, check_flightCorridor_client, ltstar_safety_margin);

    Eigen::Vector3d unobservable(1, 2, 3);
    Eigen::Vector3d viewpoint   (0, 0, 0);
    Eigen::Vector3d viewpoint_similar_x    (0.01, 0, 0);
    Eigen::Vector3d viewpoint_similar_y    (0, -0.01, 0);
    Eigen::Vector3d viewpoint_similar_z    (0, 0, 0.04);
    Eigen::Vector3d unobservable_similar_x (1.05, 2, 3);
    Eigen::Vector3d unobservable_similar_y (1, 1.6, 3);
    Eigen::Vector3d unobservable_similar_z (1, 2, 4);

    goal_state_machine.DeclareUnobservable(unobservable, viewpoint);
    // Too close
    ASSERT_TRUE( goal_state_machine.IsUnobservable(unobservable, viewpoint));
    ASSERT_TRUE( goal_state_machine.IsUnobservable(unobservable, viewpoint_similar_x));
    ASSERT_TRUE( goal_state_machine.IsUnobservable(unobservable, viewpoint_similar_y));
    ASSERT_TRUE( goal_state_machine.IsUnobservable(unobservable, viewpoint_similar_z));
    ASSERT_TRUE( goal_state_machine.IsUnobservable(unobservable_similar_x, viewpoint));
    ASSERT_TRUE( goal_state_machine.IsUnobservable(unobservable_similar_y, viewpoint));
    ASSERT_TRUE( goal_state_machine.IsUnobservable(unobservable_similar_z, viewpoint));
   
    // Far enough
    Eigen::Vector3d unobservable_different (1, 2, 4.01);
    ASSERT_FALSE( goal_state_machine.IsUnobservable(unobservable_different, viewpoint));
  }

  TEST(GoalStateMachine, ObservableTest)
  {
    frontiers_msgs::FrontierReply frontiers_msg;
    double distance_inFront = 2;
    double distance_behind = 2;
    double ltstar_safety_margin = 5;
    int circle_divisions = 12;
    geometry_msgs::Point geofence_min_point, geofence_max_point;
    geofence_min_point.x = 0;
    geofence_min_point.y = 0;
    geofence_min_point.z = 0;
    geofence_max_point.x = 0;
    geofence_max_point.y = 0;
    geofence_max_point.z = 0;
    ros::Publisher marker_pub;
    rviz_interface::PublishingInput pi (marker_pub);
    ros::ServiceClient check_flightCorridor_client;
    GoalStateMachine goal_state_machine (frontiers_msg, distance_inFront, distance_behind, circle_divisions, geofence_min_point, geofence_max_point, pi, check_flightCorridor_client, ltstar_safety_margin);

    Eigen::Vector3d unobservable(1, 2, 3);
    Eigen::Vector3d unobservable_x(5, 2, 3);
    Eigen::Vector3d unobservable_y(1, 5, 3);
    Eigen::Vector3d unobservable_z(1, 2, 5);
    Eigen::Vector3d viewpoint_x (5, 2, 3);
    Eigen::Vector3d viewpoint_y (1, 5, 3);
    Eigen::Vector3d viewpoint_z (1, 2, 5);
    Eigen::Vector3d viewpoint   (0, 0, 0);

    goal_state_machine.DeclareUnobservable(unobservable, viewpoint);
    ASSERT_FALSE( goal_state_machine.IsUnobservable(unobservable, viewpoint_x));
    ASSERT_FALSE( goal_state_machine.IsUnobservable(unobservable, viewpoint_y));
    ASSERT_FALSE( goal_state_machine.IsUnobservable(unobservable, viewpoint_z));
    ASSERT_FALSE( goal_state_machine.IsUnobservable(unobservable_x, viewpoint));
    ASSERT_FALSE( goal_state_machine.IsUnobservable(unobservable_y, viewpoint));
    ASSERT_FALSE( goal_state_machine.IsUnobservable(unobservable_z, viewpoint));
    ASSERT_FALSE( goal_state_machine.IsUnobservable(unobservable_z, viewpoint_x));
   
  }

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}