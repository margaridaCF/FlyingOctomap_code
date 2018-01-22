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

#include <iostream>
#include <handy_tools/special_zone.h>
#include <ros/ros.h>

using namespace grvc::utils;

int main(int argc, char** argv) {
    ros::init(argc,argv,"test_special_zone");
    ros::NodeHandle nh;
    int nvert;
    std::vector<geometry_msgs::Point> vertices;
    float x, y;
    geometry_msgs::Point p;

    std::cout << "How many vertices? - ";
    std::cin >> nvert;
    for(int i=0;i<nvert;i++) {
        std::cout << "Vert[" << i+1 << "]: \n  x: ";
        std::cin >> x;
        std::cout << "  y: ";
        std::cin >> y;
        p.x = x;
        p.y = y;
        p.z = 0;
        vertices.push_back(p);
    }
    std::cout << std::endl;

    geometry_msgs::PointStamped testp;
    testp.point.z = 0;
    testp.header.frame_id = "map";
    special_zone my_zone("map", vertices);

    while(ros::ok()) {
        std::cout << "Test point:\n  x: ";
        std::cin >> x;
        std::cout << "  y: ";
        std::cin >> y;
        testp.point.x = x;
        testp.point.y = y;
        std::cout << "Is in?: " << my_zone.isIn(testp) << std::endl;
        ros::spinOnce();
    }

    return 0;
}