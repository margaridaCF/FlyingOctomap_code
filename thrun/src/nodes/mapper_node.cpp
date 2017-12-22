
#include <mapper_node.h>

#include <geometry_msgs/Vector3.h>
#include <octomap_msgs/conversions.h>
#include <iostream>

#include <argument_parser.h>

namespace mapper {

    MapperNode::MapperNode(ros::NodeHandle nh, std::string name)
        :sub_octomap(nh.subscribe("/octomap_binary", 20, &MapperNode::callback,this)),  tfListener(tfBuffer)
    {  
        
    }

    void MapperNode::callback(const octomap_msgs::Octomap::ConstPtr& octomapBinary){
        octomap::OcTree* octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*octomapBinary);
        geometry_msgs::TransformStamped transformStamped;
        try{
            /// ---- GET CURRENT UAV POSITION ----
            getRobotTransform(transformStamped);
            geometry_msgs::Vector3 translation = transformStamped.transform.translation;

            /// PRINTING THE Z PLANE
            Mapper2D::printOccupancyMatrix(translation, octree);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
        }


        // must delete octree in the end
        //delete [] octree;
    }


    void MapperNode::getRobotTransform(geometry_msgs::TransformStamped& transformStamped)
    {
        transformStamped = tfBuffer.lookupTransform("world", "velodyne_down", ros::Time(0));
    }


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapper");
    ros::NodeHandle n;

    //Init
    grvc::utils::ArgumentParser args_(argc, argv);
    // std::string point_cloud_topic =
    //         args_.getArgument("point_cloud_topic",
    //                           std::string("default_waypoint_topic"));
    mapper::MapperNode mapperNode (n, "mapper");

    ros::Rate loop_rate(1);
    ros::spin();

    return 0;
}
