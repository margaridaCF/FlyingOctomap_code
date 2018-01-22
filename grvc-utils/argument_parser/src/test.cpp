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
#include <cassert>
#include <argument_parser/argument_parser.h>

using namespace grvc::utils;

// Custom class
struct MyStruct {
    MyStruct() {}  // Required by ArgumentParser::getArgument
    MyStruct(int i): flag(i) {}
    int flag = 0;
};

// Must define operator >> to parse class from arguments
inline std::istream& operator >> (std::istream& _is, MyStruct& _s) {
    _s.flag = 1;
    return _is;
}

int main(int, char**) {

    int argcTest = 7;
    char* argvTest[] = {
        (char*)"./test",
        (char*)"-foo_s=cucu",
        (char*)"-foo_i=42",
        (char*)"-foo_d=3.14",
        (char*)"-foo_s=tras",  // Should override previous value "cucu"
        (char*)"just-some-noise-foo_s=bad",  // Not a parsable argument
        (char*)"-mystruct=whatever"  // Custom class
    };

    ArgumentParser options(argcTest, argvTest);

    // Provided argc and argv are cached for further use
    std::cout << "Print cached arguments:" << std::endl;
    for (int i = 0; i < options.argc(); i++) {
        printf("argv[%d] = %s\n", i, options.argv()[i]);
    }

    std::cout << "Test argument parsing:" << std::endl;
    // Return type might be implicit in default value...
    std::string s = options.getArgument("foo_s", std::string("default"));
    std::cout << "Arg [foo_s] = [" << s << "]" << std::endl;
    assert(s == "tras");

    // ...or template-explicit
    s = options.getArgument<std::string>("foo_s", "default");
    assert(s == "tras");

    int i = options.getArgument("foo_i", 33);
    std::cout << "Arg [foo_i] = [" << i << "]" << std::endl;
    assert(i == 42);

    double d = options.getArgument("foo_d", 2.72);
    std::cout << "Arg [foo_d] = [" << d << "]" << std::endl;
    assert(d == 3.14);

    std::string t = options.getArgument("not-provided", std::string("default-value"));
    std::cout << "Arg [not-provided] = [" << t << "]" << std::endl;
    assert(t == "default-value");

    MyStruct m = options.getArgument("mystruct", MyStruct(2));
    std::cout << "Arg [mystruct].flag = " << m.flag << std::endl;
    assert(m.flag==1);

    std::cout << "Test to add an argument:" << std::endl;
    options.setArgument("add-an-argument", "done!");
    s = options.getArgument<std::string>("add-an-argument", ":(");
    std::cout << "Arg [add-an-argument] = [" << s << "]" << std::endl;
    assert(s == "done!");

    std::cout << "Tests passed!"  << std::endl;

    std::cout << "Print finally parsed arguments:" << std::endl;
    options.printArguments();
    return 0;
}
