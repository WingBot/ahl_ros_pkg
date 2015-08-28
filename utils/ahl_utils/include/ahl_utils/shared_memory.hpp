/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Daichi Yoshikawa
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Daichi Yoshikawa nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Daichi Yoshikawa
 *
 *********************************************************************/

#ifndef __AHL_UTILS_SHARED_MEMORY_HPP
#define __AHL_UTILS_SHARED_MEMORY_HPP

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>

using namespace std;
using namespace boost::interprocess;

namespace ahl_utils
{

  template<typename T>
  class SharedMemory
  {
  public:
    typedef boost::shared_ptr< SharedMemory<T> > Ptr;

    SharedMemory(const std::string& name, bool remove = false)
      : name_(name)
    {
      size_ = sizeof(T);

      if(remove)
      {
        shared_memory_object::remove(name.c_str());
      }

      try
      {
        shm_    = SharedMemoryObjectPtr(new shared_memory_object(open_or_create, name_.c_str(), read_write));
        shm_->truncate(size_);
      }
      catch(interprocess_exception&)
      {
        shared_memory_object::remove(name.c_str());
        shm_    = SharedMemoryObjectPtr(new shared_memory_object(open_or_create, name_.c_str(), read_write));
        shm_->truncate(size_);
      }

      region_ = MappedRegionPtr(new mapped_region(*shm_, read_write));
      std::string mutex_name = name + "_mutex";
      mutex_ = NamedMutexPtr(new named_mutex(open_or_create, mutex_name.c_str()));
      data_ = static_cast<T*>(region_->get_address());
    }

    ~SharedMemory()
    {
    }

    void write(const T& val)
    {
      scoped_lock<named_mutex> lock(*mutex_);
      *data_ = val;
    }

    void read(T& val)
    {
      scoped_lock<named_mutex> lock(*mutex_);
      val = *data_;
    }

    const std::string& getName() const
    {
      return name_;
    }

    unsigned int getSize()
    {
      return size_;
    }

  private:
    typedef boost::shared_ptr<shared_memory_object> SharedMemoryObjectPtr;
    typedef boost::shared_ptr<mapped_region> MappedRegionPtr;
    typedef boost::shared_ptr<named_mutex> NamedMutexPtr;

    std::string name_;
    unsigned int size_;
    SharedMemoryObjectPtr shm_;
    MappedRegionPtr region_;
    NamedMutexPtr mutex_;
    T* data_;
  };
}

#endif /* __AHL_UTILS_SHARED_MEMORY_HPP */
