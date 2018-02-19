#include <ltStar_dispatcher.h>

namespace LazyThetaStarOctree{

  LazyThetaStarDispatcher::LazyThetaStarDispatcher(ros::NodeHandle nh)
    : marker_pub_(nh.advertise<LazyThetaStarOctree::RVizMarker>( "ltstar_path", 1)), 
      octomap_pub (nh.advertise<octomap_msgs::Octomap>("octomap_binary", 1)),
      in_coordinates_sub( nh.subscribe("chatter", 1000,    &LazyThetaStarDispatcher::chatterCallback, this) )
  {

  }
  LazyThetaStarDispatcher::~LazyThetaStarDispatcher(){}

  RVizMarker LazyThetaStarDispatcher::createMarker(int id)
  {

    double step = 0.2;
    RVizMarker marker;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "input";
    marker.id = id;
    marker.type = RVizMarker::LINE_STRIP;
    marker.action = RVizMarker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0*step;
    marker.scale.y = 1.0*step;
    marker.scale.z = 1.0*step;
    marker.color.a = 1.0;
    marker.color.r = 248;
    marker.color.g = 50;
    marker.color.b = 50;
    marker.header.seq++;
    return marker;
  }

  std::list<octomath::Vector3> LazyThetaStarDispatcher::extractResults (octomap::OcTree octree, octomath::Vector3 disc_initial, octomath::Vector3 disc_final, std::string dataset_name, int max_search_iterations = 500)
  {
    octomath::Vector3 direction =  disc_final-disc_initial;
    octomath::Vector3 return_value;

    bool initial_condidions = true;

    octomap::OcTreeNode* originNode = octree.search(disc_initial);
    if(originNode)
    {
      if(octree.isNodeOccupied(originNode) )
      {

        ROS_ERROR_STREAM( "[A] Initial point is known obstacle space. ");
        initial_condidions = false;
      }
      else
      { 
        ROS_WARN_STREAM("[A] Initial point is free space");
      }
    }
    else
    {
      ROS_ERROR_STREAM( "[A] Initial point is in unknown space");
      initial_condidions = false;
    }
    originNode = octree.search(disc_final);
    if(originNode)
    {
      if(octree.isNodeOccupied(originNode) )
      {

        ROS_ERROR_STREAM( "[A] Goal is known obstacle space. ");
        initial_condidions = false;
      }
      else
      { 
        ROS_WARN_STREAM("[A] Goal is free space");
      }
    }
    else
    {
      ROS_ERROR_STREAM( "[A] Goal is in unknown space");
      initial_condidions = false;
    }


    bool occupied_cell_was_hit = octree.castRay(disc_final, direction, return_value, false, direction.norm());
    if(occupied_cell_was_hit)
    {
      originNode = octree.search(return_value);
      if (octree.isNodeOccupied(originNode))
      {
        ROS_ERROR_STREAM("[B] There is an obstacle from " << disc_initial <<  " to " <<  disc_final << "at" << return_value);
      }
      else
      {
        ROS_ERROR_STREAM( "[B] There is unknown space between " << disc_initial <<  " and " <<  disc_final << "at" << return_value);
        initial_condidions = false;
      }
    }
    else
    {
      ROS_WARN_STREAM("[B] There is line of sight between " << disc_initial <<  " and " <<  disc_final);
    }

    if(!initial_condidions)
    {
      ROS_ERROR_STREAM("Find another sample.");
      return std::list<octomath::Vector3>();
    }


    ResultSet statistical_data;
    // timespec time1, time2;
    // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
    return lazyThetaStar_(octree, disc_initial, disc_final, statistical_data, max_search_iterations);
    // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
    // double total_nSecs_overall = diff(time1,time2).tv_nsec;
  }

  void LazyThetaStarDispatcher::chatterCallback(const std_msgs::String::ConstPtr& msg)
  {
    // == run 2 ==
    octomath::Vector3 disc_initial(0, 5, 1.5); 
    octomath::Vector3 disc_final  (2, -5, 1.5); 
    octomap::OcTree octree ("/ros_ws/src/path_planning/test/data/fr_campus.bt");
    std::string dataset_name = "freiburg campus";

    // // == LazyThetaStar_CoreDumped_Test ==
    // octomap::OcTree octree ("/ros_ws/src/path_planning/test/data/offShoreOil_1m.bt");
    // octomath::Vector3 disc_initial(-8.3, -8.3, 0.5);
    // octomath::Vector3 disc_final  (-7.3, -8.3, 0.5);
    // std::string dataset_name = "CoreDumped Test with offShoreOil_1m.bt from (-8.3, -8.3, 0.5) to (-7.3, -8.3, 0.5)";
    // // == LazyThetaStar_StraighLine_Test ==
    // octomap::OcTree octree ("/ros_ws/src/path_planning/test/data/offShoreOil_1m.bt");
    // octomath::Vector3 disc_initial(-6.9, -9.7, 0.5);
    // octomath::Vector3 disc_final  (-5.9, -9.7, 0.5);
    // std::string dataset_name = "StraighLine_Test with offShoreOil_1m.bt from (-6.9, -9.7, 0.5) to (-5.9, -9.7, 0.5)";
    // // == DepthSizeTest.hasLineOfSightTest ==
    // octomap::OcTree octree ("/ros_ws/src/path_planning/test/data/offShoreOil_1m.bt");
    // octomath::Vector3 disc_initial(-5.500001, -3.1, 0.5);
    // octomath::Vector3 disc_final  (-5.4, -2.6, 0.6);
    // std::string dataset_name = "hasLineOfSightTest with offShoreOil_1m.bt from (-5.500001, -3.1, 0.5) to (-5.4, -2.6, 0.6)";


    ROS_INFO_STREAM("Request is going from " << disc_initial << " to " << disc_final);
    octomap_msgs::Octomap octomap;
    octomap_msgs::binaryMapToMsg(octree, octomap);
    octomap_pub.publish(octomap);

    RVizMarker marker = createMarker(3);
    geometry_msgs::Point p;
    p.x = disc_initial.x();
    p.y = disc_initial.y();
    p.z = disc_initial.z();
    marker.points.push_back(p);
    p.x = disc_final.x();
    p.y = disc_final.y();
    p.z = disc_final.z();
    marker.points.push_back(p);
    marker_pub_.publish( marker );

    int max_search_iterations = 1000;
    std::list<octomath::Vector3> resulting_path = extractResults(octree, disc_initial, disc_final, dataset_name, max_search_iterations);
    std::ofstream waypoints_file;
    waypoints_file.open("/home/mfaria/Margarida/20170802_lazyThetaStar/experimental data/euroc_compare/newImplementation.log", std::ios_base::app);
    waypoints_file << " ===== " << dataset_name << " ===== " << std::endl;
    // waypoints_file << " iterations used: " << statistical_data.iterations_used  << "; Took " << total_nSecs_overall << " nano seconds." << std::endl;
    marker = createMarker(4);
    marker.color.g = 50;
    marker.color.b = 167;
    marker.ns = "output";
    for (octomath::Vector3 waypoint : resulting_path)
    {
      waypoints_file << waypoint << std::endl;
      
      p.x = waypoint.x();
      p.y = waypoint.y();
      p.z = waypoint.z();
      marker.points.push_back(p);
    }
    marker_pub_.publish( marker );
    waypoints_file << std::endl;
    waypoints_file.close();
    
    ROS_ERROR_STREAM( "Finished");
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;

  // Init
  ros::Publisher marker_pub_;
  marker_pub_ = nh.advertise<LazyThetaStarOctree::RVizMarker>( "ltstar_path", 1);
  LazyThetaStarOctree::LazyThetaStarDispatcher work_horse(nh);
  ros::spin();

  return 0;
}