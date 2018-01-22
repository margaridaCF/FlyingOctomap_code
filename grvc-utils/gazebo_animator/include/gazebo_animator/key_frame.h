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
#ifndef GAZEBO_ANIMATOR_KEY_FRAME_H
#define GAZEBO_ANIMATOR_KEY_FRAME_H

#include <ros/ros.h>
#include <gazebo_animator/frame.h>

namespace grvc { namespace utils {

struct KeyFrame {
    KeyFrame(Frame _frame, ros::Time _time): frame(_frame) {
        time = _time;
    }
    Frame frame;
    ros::Time time;
 
    bool operator<(const KeyFrame& _key) { return time < _key.time; }
 
    static Frame interpolate(const KeyFrame& _prev, const KeyFrame& _next, const ros::Duration& _elapsed) {
        double t_prev = _prev.time.toSec();
        double t_next = _next.time.toSec();
        double t_curr = _elapsed.toSec();
        double t = (t_curr - t_prev) / (t_next - t_prev);
        double x = (1.0 - t) * _prev.frame.position.x + t * _next.frame.position.x;
        double y = (1.0 - t) * _prev.frame.position.y + t * _next.frame.position.y;
        double z = (1.0 - t) * _prev.frame.position.z + t * _next.frame.position.z;
        return Frame(x, y, z);
    }
};

}}  // grvc::utils

#endif  // GAZEBO_ANIMATOR_KEY_FRAME_H
