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
#ifndef HANDY_TOOLS_PID_CONTROLLER_H
#define HANDY_TOOLS_PID_CONTROLLER_H

#include <string>

namespace grvc { namespace utils {

// TODO: Add dynamic tuning tools
class PidController {
public:
    PidController(const std::string& _name, double _k_p, double _k_i, double _k_d) {
        name_ = _name;
        k_p_ = _k_p;
        k_i_ = _k_i;
        k_d_ = _k_d;
        // printf("Created PidController %s with gains: [%f, %f, %f]\n", name_.c_str(), k_p_, k_i_, k_d_);
    }

    double control_signal(double _error, double _dt) {
        // TODO: anti-windup and other pid sofistications :)
        error_sum_ += _error * _dt;
        double error_diff = _error - previous_error_;
        previous_error_ = _error;

        double output = k_p_ * _error + k_i_ * error_sum_ + k_d_ * error_diff / _dt;
        return output;
    }

protected:
    std::string name_;
    double k_p_;
    double k_i_;
    double k_d_;
    double error_sum_ = 0;
    double previous_error_ = 0;
};

}} // namespace grvc::utils

#endif  // HANDY_TOOLS_PID_CONTROLLER_H
