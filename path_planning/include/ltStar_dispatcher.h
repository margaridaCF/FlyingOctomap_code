#ifndef LAZYTHETASTAR_H
#define LAZYTHETASTAR_H
#include <ltStar_temp.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/Point.h>
#include <path_planning/StartGoal.h>


namespace LazyThetaStarOctree{
    typedef visualization_msgs::Marker RVizMarker;
    class LazyThetaStarDispatcher
    {
    public:
        LazyThetaStarDispatcher(ros::NodeHandle nh);
        ~LazyThetaStarDispatcher();
        std::list<octomath::Vector3> extractResults (octomap::OcTree octree, octomath::Vector3 disc_initial, octomath::Vector3 disc_final, std::string dataset_name, int max_time_secs);
        RVizMarker createMarker(int id);
        void chatterCallback(const path_planning::StartGoal::ConstPtr& msg);
    private:
        ros::Publisher marker_pub_;
        ros::Publisher octomap_pub;
        ros::Subscriber in_coordinates_sub;
    };
}

#endif // LAZYTHETASTAR_H
