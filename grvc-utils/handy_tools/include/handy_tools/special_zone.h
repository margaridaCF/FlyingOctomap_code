//----------------------------------------------------------------------------------------------------------------------
// GRVC Utils
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2016 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
#ifndef HANDY_TOOLS_SPECIAL_ZONE_H
#define HANDY_TOOLS_SPECIAL_ZONE_H

#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

namespace grvc { namespace utils {

/**
 *  Class for special zones. Special zones of any polygonal shape can be defined with this class.
 */
class special_zone {
    public:
        special_zone(std::string _frame_id, std::vector<geometry_msgs::Point> _vertices) {
            frame_id_ = _frame_id;
            vertices_ = _vertices;
            if (_vertices.size() < 3) {
                ROS_ERROR("The minimum number of vertices for a polygon is 3.");
                exit(0);
            }
        }

        ~special_zone(){};

        /**
        *  Checks if a certain point is within the special zone.
        *  @param _point the point to check
        *  @return true if the point is IN the zone, false if it is OUT
        */
        bool isIn(geometry_msgs::PointStamped _point) {
            geometry_msgs::Point test_point;
            std::string point_frame_id = tf2::getFrameId(_point);
            
            // Check reference frames
            if ( point_frame_id == "" || point_frame_id == frame_id_ ) {
                test_point = _point.point;
            }
            else {
                geometry_msgs::TransformStamped transformToZoneFrame;

                if ( cached_transforms_.find(point_frame_id) == cached_transforms_.end() ) {
                    // point_frame_id not found in cached_transforms_
                    tf2_ros::Buffer tfBuffer;
                    tf2_ros::TransformListener tfListener(tfBuffer);
                    transformToZoneFrame = tfBuffer.lookupTransform(frame_id_, point_frame_id, ros::Time(0), ros::Duration(1.0));
                    cached_transforms_[point_frame_id] = transformToZoneFrame; // Save transform in cache
                } else {
                    // found in cache
                    transformToZoneFrame = cached_transforms_[point_frame_id];
                }

                geometry_msgs::PointStamped aux_point;
                tf2::doTransform(_point, aux_point, transformToZoneFrame);
                test_point = aux_point.point;
            }
            
            // Do the maths
            bool is_inside = false;
            int i, j = 0;
            for (i = 0, j = vertices_.size()-1; i < vertices_.size(); j = i++) {
                if ( ((vertices_[i].y>test_point.y) != (vertices_[j].y>test_point.y)) &&
                (test_point.x < (vertices_[j].x-vertices_[i].x) * (test_point.y-vertices_[i].y) / (vertices_[j].y-vertices_[i].y) + vertices_[i].x) )
                    is_inside = !is_inside;
            }

            return is_inside;
        }
    private:
        std::string frame_id_;
        std::vector<geometry_msgs::Point> vertices_;
        std::map <std::string, geometry_msgs::TransformStamped> cached_transforms_;
};

}} // namespace grvc::utils

#endif // HANDY_TOOLS_SPECIAL_ZONE_H