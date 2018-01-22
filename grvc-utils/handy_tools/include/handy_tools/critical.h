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
#ifndef HANDY_TOOLS_CRITICAL_H
#define HANDY_TOOLS_CRITICAL_H

#include <mutex>

namespace grvc { namespace utils {

	/// Util to create a minimal critical section for setting/getting
  /// shared data in multiple threads
  template <class T_>
  class Critical {
  public:
    /// Simplest getter
    /// \return a copy of last set data
    T_ get();

    /// Simplest setter
    /// \param _in data to copy and keep
    void set(const T_& _in);

    /// Set data only if current value equals _compared value
    /// \param _in data to set if condition is satified
    /// \param _compared value to check equality with
    /// \return true only if value has changed in current call
    bool setIfDataEquals(const T_& _in, const T_& _compared);

    /// Ask if you don't want to get the same data twice
    /// \return true only if data has changed since last get() call
    bool hasNewData() { return new_data_; }

  protected:
    T_ data_;
    bool new_data_ = false;
    std::mutex mutex_;
  };

  template <class T_>
  inline T_ Critical<T_>::get() {
    mutex_.lock();
    T_ out = data_;  // T_ is copy-constructible
    new_data_ = false;
    mutex_.unlock();
    return out;
  }

  template <class T_>
  inline void Critical<T_>::set(const T_& _in) {
    mutex_.lock();
    data_ = _in;
    new_data_ = true;
    mutex_.unlock();
  }

  template <class T_>
  inline bool Critical<T_>::setIfDataEquals(const T_& _in, const T_& _compared) {
    bool ret = false;
    mutex_.lock();
    if(data_ == _compared){
      data_ = _in;
      new_data_ = true;
      ret = true;
    }
    mutex_.unlock();
    return ret;
  }

}} // namespace grvc::utils

#endif // HANDY_TOOLS_CRITICAL_H
