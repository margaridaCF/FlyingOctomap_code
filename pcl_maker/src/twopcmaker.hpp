/**
 * @file pcmaker.hpp
 * @brief Generation of point cloud out of depth images
 * @author Fernando Caballero, fcaballero@us.es
 * @author Francisco J Perez-Grau, fjperez@catec.aero
 * @date October 2016
 */

#ifndef __TWOPCMAKER_HPP__
#define __TWOPCMAKER_HPP__

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;

/**
 * @class TwoPcMaker
 * @brief Generates 3D point clouds from RGB-D data
 */
class TwoPcMaker
{
public:
    /**
     * @brief Constructor
     * @param nodeName Node name for publishing topics
     * @param cameraTopic Name of the camera
     */
    TwoPcMaker(std::string &nodeName, std::string &frontCameraTopic, std::string &backCameraTopic):
        it_(nh_),
        frontImageSub_(it_, frontCameraTopic + "/depth_registered/image_raw", 1),
        backImageSub_(it_, backCameraTopic + "/depth_registered/image_raw", 1),
        imageSync_(syncPolicy(10), frontImageSub_, backImageSub_),
        frontCalibInit_(false),
        backCalibInit_(false)
	{
		// Topic subscription
        frontCInfoSub_ = nh_.subscribe(frontCameraTopic + "/depth_registered/camera_info", 1, &TwoPcMaker::frontCInfoCallback, this);
        backCInfoSub_ = nh_.subscribe(backCameraTopic + "/depth_registered/camera_info", 1, &TwoPcMaker::backCInfoCallback, this);
        imageSync_.registerCallback(boost::bind(&TwoPcMaker::syncImageCallback, this, _1, _2));

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

        ROS_INFO_STREAM("[PcMaker] Subscribed to:\n\t" << frontImageSub_.getTopic()
                        << "\n\t" << frontCInfoSub_.getTopic()
                        << "\n\t" << backImageSub_.getTopic()
                        << "\n\t" << backCInfoSub_.getTopic());

        pcPub_ = nh_.advertise<sensor_msgs::PointCloud2>(nodeName+"/point_cloud", 1);

        Tback2front_.setIdentity();
    }

    /** @brief Destructor */
    ~TwoPcMaker(void)
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
          pass_filter.setFilterLimits(-pcCropDist_, pcCropDist_);
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
     * @brief Pre-cache transform between cameras
     * @param tf[out] Transform
     */
    bool getBack2Front(const ros::Time& stamp, tf::StampedTransform& tf)
    {
        try
        {
            tfListener_.waitForTransform("front_rgb_optical_frame", "back_rgb_optical_frame", stamp, ros::Duration(0.2));
            tfListener_.lookupTransform("front_rgb_optical_frame", "back_rgb_optical_frame", stamp, tf);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            return false;
        }
        return true;
    }

    /**
     * @brief Publishes full sensor point cloud in the RGB camera frame
     * @param img Input depth image
     * @param frame_id Message frame name
     * @param stamp Time stamp to associate to the point cloud message
     */
    void publishPointCloud(const cv::Mat frontImg, const cv::Mat backImg, std::string frame_id, ros::Time stamp)
    {
        sensor_msgs::PointCloud outCloud;
        sensor_msgs::PointCloud2 outCloud2, outCloudFiltered;

        geometry_msgs::Point32 pt;
        outCloud.header.frame_id = frame_id;
        outCloud.header.stamp = stamp;
        outCloud.points.clear();

        //Front image
        float front_kx = 1.0/frontK_.at<double>(0,0), front_cx = frontK_.at<double>(0,2);
        float front_ky = 1.0/frontK_.at<double>(1,1), front_cy = frontK_.at<double>(1,2);
        for(int v=0; v<frontImg.rows; v+=3)
        {
            for(int u=0; u<frontImg.cols; u+=3)
            {
                pt.z = frontImg.at<float>(v, u);
                if(pt.z > 0.3 && pt.z < 8.0)
                {
                    pt.x = (float)(u-front_cx) * pt.z * front_kx;
                    pt.y = (float)(v-front_cy) * pt.z * front_ky;
                    outCloud.points.push_back(pt);
                }
            }
        }

        //Back image
        float back_kx = 1.0/backK_.at<double>(0,0), back_cx = backK_.at<double>(0,2);
        float back_ky = 1.0/backK_.at<double>(1,1), back_cy = backK_.at<double>(1,2);
        tf::Vector3 origin = Tback2front_.getOrigin();
        tf::Matrix3x3 basis = Tback2front_.getBasis();
        for(int v=0; v<backImg.rows; v+=3)
        {
            for(int u=0; u<backImg.cols; u+=3)
            {
                pt.z = backImg.at<float>(v, u);
                if(pt.z > 0.3 && pt.z < 8.0)
                {
                    pt.x = (float)(u-back_cx) * pt.z * back_kx;
                    pt.y = (float)(v-back_cy) * pt.z * back_ky;
                    geometry_msgs::Point32 ptFront;
                    ptFront.x = basis[0].x() * pt.x + basis[0].y() * pt.y + basis[0].z() * pt.z + origin.x();
                    ptFront.y = basis[1].x() * pt.x + basis[1].y() * pt.y + basis[1].z() * pt.z + origin.y();
                    ptFront.z = basis[2].x() * pt.x + basis[2].y() * pt.y + basis[2].z() * pt.z + origin.z();
                    outCloud.points.push_back(ptFront);
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
    void syncImageCallback(const sensor_msgs::ImageConstPtr& frontMsg,
                           const sensor_msgs::ImageConstPtr& backMsg)
    {
        //Clock t;
									
        // Processing does not start until first calibration is received
        if(!frontCalibInit_ || !backCalibInit_)
        {
            ROS_WARN("No camera calibration received yet, skipping image processing");
            return;
        }

        if(!getBack2Front(frontMsg->header.stamp, Tback2front_))
        {
            ROS_WARN("No transformation found between the cameras");
            return;
        }

        // Convert to OpenCV format
        cv::Mat frontDepthImg, backDepthImg;
        if(!convertImage(frontMsg, frontDepthImg) || !convertImage(backMsg, backDepthImg))
            return;

        publishPointCloud(frontDepthImg, backDepthImg, frontMsg->header.frame_id, frontMsg->header.stamp);
    }

    /**
     * @brief Camera calibration info callback
     * @param cInfoMsg Camera calibration message
     */
    void frontCInfoCallback(const sensor_msgs::CameraInfoConstPtr& cInfoMsg)
    {
        if(!frontCalibInit_)
        {
            // Set calibration flag
            frontCalibInit_ = true;

            // Store RGB camera parameters
            frontK_ = cv::Mat(3, 3, CV_64FC1, (void *)cInfoMsg->K.elems).clone();
            frontD_ = cv::Mat(cInfoMsg->D.size(), 1, CV_64FC1, (void *)cInfoMsg->D.data()).clone();
        }
    }

    /**
     * @brief Camera calibration info callback
     * @param cInfoMsg Camera calibration message
     */
    void backCInfoCallback(const sensor_msgs::CameraInfoConstPtr& cInfoMsg)
    {
        if(!backCalibInit_)
        {
            // Set calibration flag
            backCalibInit_ = true;

            // Store RGB camera parameters
            backK_ = cv::Mat(3, 3, CV_64FC1, (void *)cInfoMsg->K.elems).clone();
            backD_ = cv::Mat(cInfoMsg->D.size(), 1, CV_64FC1, (void *)cInfoMsg->D.data()).clone();
        }
    }

    ros::NodeHandle nh_;                        /**< ROS node handler*/
    image_transport::ImageTransport it_;        /**< Image transport*/
    image_transport::SubscriberFilter frontImageSub_, backImageSub_;    /**< Image subscriber*/
    message_filters::Synchronizer<syncPolicy> imageSync_;   /**< Time synchronizer filter*/

    image_transport::Subscriber depthSub_;    /**< Depth subscriber*/
    ros::Subscriber frontCInfoSub_, backCInfoSub_;  /**< Camera info subscriber*/
    ros::Publisher pcPub_;                          /**< Point cloud publishers*/

    bool frontCalibInit_, backCalibInit_;        /**< Flag indicating if we have calibration data*/

    tf::TransformListener tfListener_;
    tf::StampedTransform Tback2front_;

    // Node params
    cv::Mat frontK_, backK_;                 /**< Camera intrinsic parameters matrix*/
    cv::Mat frontD_, backD_;                 /**< Camera distortion parameters matrix*/

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
