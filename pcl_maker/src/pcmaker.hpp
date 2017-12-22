/**
 * @file pcmaker.hpp
 * @brief Generation of point cloud out of depth images
 * @author Fernando Caballero, fcaballero@us.es
 * @author Francisco J Perez-Grau, fjperez@catec.aero
 * @date October 2016
 */

#ifndef __PCMAKER_HPP__
#define __PCMAKER_HPP__

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>


/**
 * @class PcMaker
 * @brief Generates 3D point clouds from RGB-D data
 */
class PcMaker
{
public:
    /**
     * @brief Constructor
     * @param nodeName Node name for publishing topics
     * @param cameraTopic Name of the camera
     */
    PcMaker(std::string &nodeName, std::string &cameraTopic):
        it_(nh_),
        calibInit_(false)
	{
		// Topic subscription
        depthSub_ = it_.subscribe(cameraTopic + "/depth_registered/image_raw", 1, &PcMaker::imageCallback, this);
        cInfoSub_ = nh_.subscribe(cameraTopic + "/rgb/camera_info", 1, &PcMaker::cInfoCallback, this);

		// Load node parameters
        ros::NodeHandle lnh("~");
        if(!lnh.getParam("apply_downsampling", applyPcDown_))
            applyPcDown_ = false;
        if(!lnh.getParam("pc_downsampling", pcDown_))
            pcDown_ = 0.1;
        if(!lnh.getParam("apply_crop", applyPcCrop_))
            applyPcCrop_ = false;
        if(!lnh.getParam("pc_crop_dist", pcCropDist_))
            pcCropDist_ = 5.0;
        if(!lnh.getParam("apply_outlier", applyPcOutlier_))
            applyPcOutlier_ = false;
        if(!lnh.getParam("pc_outlier_radius", pcOutRadius_))
            pcOutRadius_ = 0.4;
        if(!lnh.getParam("pc_outlier_neighbors", pcOutNeighbors_))
            pcOutNeighbors_ = 10;

        ROS_INFO_STREAM("[PcMaker] Subscribed to:\n\t" << depthSub_.getTopic()
                        << "\n\t" << cInfoSub_.getTopic());

        pcPub_ = nh_.advertise<sensor_msgs::PointCloud2>(nodeName+"/point_cloud", 1);
    }

    /** @brief Destructor */
    ~PcMaker(void)
    {
    }

private:
	

    /**
     * @brief Convert images from ROS message to OpenCV format (cv::Mat)
     * @param rgbMsg RGB image in ROS format
     * @param depthMsg Depth image in ROS format
     * @param rgbMat RGB image in OpenCV format
     * @param depthMat Depth image in OpenCV format
     * @return
     */
    bool convertImage(const sensor_msgs::ImageConstPtr& depthMsg, cv::Mat& depthMat)
    {
        // Convert to OpenCV format without copy
        cv_bridge::CvImageConstPtr cvbDepth;
        try
        {
            cvbDepth = cv_bridge::toCvCopy(depthMsg);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return false;
        }

        cvbDepth->image.copyTo(depthMat);

        return true;
    }

    /**
     * @brief Filter dense point clouds before publishing
     * @param src
     * @param dst
     */
    void filterCloud(sensor_msgs::PointCloud2 &src, sensor_msgs::PointCloud2 &dst)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outrem (new pcl::PointCloud<pcl::PointXYZ>);

      // Pointcloud filters
      pcl::PassThrough<pcl::PointXYZ> pass_filter;
      pcl::VoxelGrid<pcl::PointXYZ> vox_filter;
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> out_filter;

      pcl::fromROSMsg(src, *cloud_src);

      if(applyPcCrop_)
      {
          pass_filter.setInputCloud(cloud_src);
          pass_filter.setFilterFieldName("z");
          pass_filter.setFilterLimits(0.0, pcCropDist_);
          pass_filter.filter(*cloud_pass);
      }
      else
          *cloud_pass = *cloud_src;

      if(applyPcDown_)
      {
        vox_filter.setInputCloud(cloud_pass);
        vox_filter.setLeafSize(pcDown_, pcDown_, pcDown_);
        vox_filter.filter(*cloud_down);
      }
      else
        *cloud_down = *cloud_pass;

      if(applyPcOutlier_)
      {
        out_filter.setInputCloud(cloud_down);
        out_filter.setRadiusSearch(pcOutRadius_);
        out_filter.setMinNeighborsInRadius(pcOutNeighbors_);
        out_filter.filter(*cloud_outrem);
      }
      else
        *cloud_outrem = *cloud_down;

      cloud_outrem->header = cloud_src->header;
      pcl::toROSMsg(*cloud_outrem, dst);
    }

    /**
     * @brief Publishes full sensor point cloud in the RGB camera frame
     * @param img Input depth image
     * @param frame_id Message frame name
     * @param stamp Time stamp to associate to the point cloud message
     */
    void publishPointCloud(const cv::Mat img, std::string frame_id, ros::Time stamp)
    {
        sensor_msgs::PointCloud outCloud;
        sensor_msgs::PointCloud2 outCloud2, outCloudFiltered;

        geometry_msgs::Point32 pt;
        outCloud.header.frame_id = frame_id;
        outCloud.header.stamp = stamp;
        outCloud.points.clear();
        float kx = 1.0/K_.at<double>(0,0), cx = K_.at<double>(0,2), ky = 1.0/K_.at<double>(1,1), cy = K_.at<double>(1,2);
        for(int v=0; v<img.rows; v+=3)
        {
            for(int u=0; u<img.cols; u+=3)
            {
                pt.z = img.at<float>(v, u);
                if(pt.z > 0.3 && pt.z < 10.0)
                {
                    pt.x = (float)(u-cx) * pt.z * kx;
                    pt.y = (float)(v-cy) * pt.z * ky;
                    outCloud.points.push_back(pt);
                }
            }
        }

        sensor_msgs::convertPointCloudToPointCloud2(outCloud, outCloud2);

        // Apply filtering
        filterCloud(outCloud2, outCloudFiltered);
        pcPub_.publish(outCloudFiltered);
    }

    /**
     * @brief Synchronized RGB and Depth images callback
     * @param imgMsg RGB image message
     * @param depthMsg Depth image message
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& depthMsg)
    {
        //Clock t;
									
        // Processing does not start until first calibration is received
        if(!calibInit_)
        {
            ROS_WARN("No camera calibration received yet, skipping image processing");
            return;
        }

        // Convert to OpenCV format
        cv::Mat depthImg;
        if(!convertImage(depthMsg, depthImg))
            return;

        publishPointCloud(depthImg, depthMsg->header.frame_id, depthMsg->header.stamp);
    }

    /**
     * @brief Camera calibration info callback
     * @param cInfoMsg Camera calibration message
     */
    void cInfoCallback(const sensor_msgs::CameraInfoConstPtr& cInfoMsg)
    {
        if(!calibInit_)
        {
            // Set calibration flag
            calibInit_ = true;

            // Store RGB camera parameters
            K_ = cv::Mat(3, 3, CV_64FC1, (void *)cInfoMsg->K.elems).clone();
            D_ = cv::Mat(cInfoMsg->D.size(), 1, CV_64FC1, (void *)cInfoMsg->D.data()).clone();
        }
    }

    ros::NodeHandle nh_;                        /**< ROS node handler*/
    image_transport::ImageTransport it_;        /**< Image transport*/
    image_transport::Subscriber depthSub_;    /**< Depth subscriber*/
    ros::Subscriber cInfoSub_;                      /**< Camera info subscriber*/
    ros::Publisher pcPub_;                /**< Point cloud publishers*/

    bool calibInit_;        /**< Flag indicating if we have calibration data*/

    // Node params
    cv::Mat K_;                 /**< Camera intrinsic parameters matrix*/
    cv::Mat D_;                 /**< Camera distortion parameters matrix*/

    // Pc filter stuff
    bool applyPcDown_;
    double pcDown_;
    bool applyPcOutlier_;
    double pcOutRadius_;
    double pcOutNeighbors_;
    bool applyPcCrop_;
    double pcCropDist_;
};

#endif
