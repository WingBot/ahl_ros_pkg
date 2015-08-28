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
