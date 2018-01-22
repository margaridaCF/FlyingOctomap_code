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
#include <ros/ros.h>
#include <gazebo_animator/frame.h>
#include <gazebo_animator/key_frame.h>
#include <gazebo_animator/gazebo_animated_link.h>
#include <argument_parser/argument_parser.h>

using grvc::utils::Frame;
using grvc::utils::KeyFrame;
using grvc::utils::GazeboAnimatedLink;
using grvc::utils::ArgumentParser;

int main(int _argc, char** _argv) {
    ros::init(_argc, _argv, "test_gazebo_animator");
    ROS_INFO("Starting test_gazebo_animator");

    ArgumentParser options(_argc, _argv);
    std::string link_name = options.getArgument("link_name", std::string("yellow_cylinder::sample_link"));
    GazeboAnimatedLink link(link_name);
    // Define trajectory key frames (TODO: from file?)
    link.addKeyFrame(KeyFrame(Frame( 0.0,  3.0, 0.0), ros::Time(0.0)));
    link.addKeyFrame(KeyFrame(Frame( 4.0,  5.0, 0.0), ros::Time(5.0)));
    link.addKeyFrame(KeyFrame(Frame( 6.0,  1.0, 0.0), ros::Time(10.0)));
    link.addKeyFrame(KeyFrame(Frame( 0.0, -6.0, 0.0), ros::Time(18.0)));
    link.addKeyFrame(KeyFrame(Frame(-6.0,  1.0, 0.0), ros::Time(26.0)));
    link.addKeyFrame(KeyFrame(Frame(-4.0,  5.0, 0.0), ros::Time(31.0)));
    link.addKeyFrame(KeyFrame(Frame( 0.0,  3.0, 0.0), ros::Time(36.0)));

    // Animate link!
    //link.playOnce();
    link.playLoop();

    // Wait here somehow...
    ros::spin();  // Not really needed, just loop until ctrl+C

    // Stop link
    link.stop();
    return 0;
}
