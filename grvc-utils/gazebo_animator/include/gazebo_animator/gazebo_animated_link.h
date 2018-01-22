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
#ifndef GAZEBO_ANIMATOR_GAZEBO_ANIMATED_LINK
#define GAZEBO_ANIMATOR_GAZEBO_ANIMATED_LINK

#include <list>
#include <iterator>
#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_animator/frame.h>
#include <gazebo_animator/key_frame.h>

#define ANIMATION_FPS 25.0

namespace grvc { namespace utils {

class GazeboAnimatedLink {
public:
    GazeboAnimatedLink(const std::string& _link_name, const std::string& _reference_frame = "map") {
        link_name_ = _link_name;
        reference_frame_ = _reference_frame;
        std::string link_state_pub_topic = "/gazebo/set_link_state";
        link_state_publisher_ = nh_.advertise<gazebo_msgs::LinkState>(link_state_pub_topic, 1);
    }

    //GazeboAnimatedLink(const GazeboAnimatedLink&) = default;

    void addKeyFrame(const KeyFrame& _key_frame) {
        key_frames_.push_back(_key_frame);
    }

    void playOnce() {
        if (playing_) {
            std::cerr << "Stop before playing again!" << std::endl;
        } else {
            playing_ = true;
            playing_thread_ = std::thread([&](){
                play(ANIMATION_FPS);
            });
        }
    }

    void playLoop() {
        if (playing_) {
            std::cerr << "Stop before playing again!" << std::endl;
        } else {
            playing_ = true;
            playing_thread_ = std::thread([&](){
                while(playing_ && ros::ok()) {
                    play(ANIMATION_FPS);
                }
            });
        }
    }

    bool isPlaying() { return playing_; }

    void stop() {
        playing_ = false;
        playing_thread_.join();
    }

protected:
    std::string link_name_;
    std::string reference_frame_;
    std::list<KeyFrame> key_frames_;
    std::thread playing_thread_;
    bool playing_ = false;
    ros::NodeHandle nh_;
    ros::Publisher link_state_publisher_;

    void play(double _fps) {
        if (key_frames_.size() < 2) {
            std::cerr << "At least two key frames needed!" << std::endl;  // TODO: Throw?
            return;
        }
        //playing_ = true;
        key_frames_.sort();
        std::list<KeyFrame>::iterator next_frame = key_frames_.begin();
        // Wait for simulation time initialization
        while (ros::Time::now() == ros::Time(0)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        ros::Time play_started = ros::Time::now();
        ros::Rate rate(_fps);
        while (playing_ && ros::ok()) {
            ros::Duration elapsed = ros::Time::now() - play_started;
            //ROS_INFO("elapsed = %f", elapsed.toSec());
            while (next_frame->time.toSec() < elapsed.toSec()) {
                next_frame++;
                if (next_frame == key_frames_.end()) {
                    //playing_ = false;
                    return;
                }
            }
            Frame curr_frame = next_frame->frame;
            if (next_frame != key_frames_.begin()) {
                // Interpolate!
                auto prev_frame = std::prev(next_frame);
                curr_frame = KeyFrame::interpolate(*prev_frame, *next_frame, elapsed);
            }
            gazebo_msgs::LinkState current;
            current.link_name = link_name_;
            current.pose.position.x = curr_frame.position.x;
            current.pose.position.y = curr_frame.position.y;
            current.pose.position.z = curr_frame.position.z;
            current.reference_frame = reference_frame_;
            link_state_publisher_.publish(current);
            ros::spinOnce();
            rate.sleep();
        }
    }
};

}}  // grvc::utils

#endif  // GAZEBO_ANIMATOR_GAZEBO_ANIMATED_LINK
