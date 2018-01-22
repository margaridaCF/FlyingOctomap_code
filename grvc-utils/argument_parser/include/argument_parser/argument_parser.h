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
#ifndef ARGUMENT_PARSER_ARGUMENT_PARSER_H
#define ARGUMENT_PARSER_ARGUMENT_PARSER_H

#include <string>
#include <sstream>
#include <iostream>
#include <map>

namespace grvc { namespace utils {

/// Util for argc-arv simplified argument parsing
class ArgumentParser {
public:
    /// Constructor performs parsing
    /// \param _argc number of command line arguments
    /// \param _argv array of command line arguments
    ArgumentParser(int _argc, char** _argv);

    /// Print parsed arguments
    void printArguments();

    /// Get a parsed argument, template
    /// \param _label identifer of requested argument, e.g: "-foo"
    /// \param _defaultValue in case -foo=value is not provided
    template<typename T_>
    T_ getArgument(const std::string& _label, const T_& _defaultValue) const;

    /// Set value to argument, add to (or modify) inner map, not argc_/argv_
    /// \param _label identifer of argument, e.g: "-foo"
    /// \param _value as a string
    void setArgument(const std::string& _label, const std::string& _value);

    /// Get argc as passed into constructor
    int& argc() { return argc_; }

    /// Get argv as passed into constructor
    char** argv() { return argv_; }

private:
    /// All provided arguments stored by label
    std::map<std::string, std::string> labelToValueMap_;
    /// Cached argc, argv
    int argc_;
    char** argv_;
};

inline ArgumentParser::ArgumentParser (int _argc, char** _argv) {
    // Considering that, in order to be parsable by this class:
    // 1. Label must start with '-'
    // 2. There can't be spaces between label, '=' and value
    // 3. There's no possible type checking, as arguments are always char*!
    for (int i = 0; i < _argc; ++i) {
        std::string arg = _argv[i];
        auto valuepos = arg.find("=");
        if(arg[0] == '-' && valuepos != std::string::npos) {
            labelToValueMap_[arg.substr(1, valuepos-1)] = arg.substr(valuepos+1);
            //std::cout << arg.substr(1, valuepos-1) << "=" << arg.substr(valuepos+1) << std::endl;
        }
    }
    // Keep argc and argv
    argc_ = _argc;
    argv_ = _argv;  // Trust argv scope is main scope!
}

inline void ArgumentParser::printArguments() {
    for (auto it = labelToValueMap_.begin(); it != labelToValueMap_.end(); ++it) {
        std::cout << "[" << it->first << "] = [" << it->second << "]" << std::endl;
    }
}

inline void ArgumentParser::setArgument(const std::string& _label, const std::string& _value) {
    labelToValueMap_[_label] = _value;
}

template<class T_>
T_ ArgumentParser::getArgument(const std::string& _label, const T_& _defaultValue) const {
    if(labelToValueMap_.count(_label) > 0) {
        std::stringstream ss(labelToValueMap_.at(_label));
        T_ value;  // Requires default constructor
        ss >> value;
        return value;
    } else {
        return _defaultValue;
    }
}

}} // namespace grvc::utils

#endif // ARGUMENT_PARSER_ARGUMENT_PARSER_H
