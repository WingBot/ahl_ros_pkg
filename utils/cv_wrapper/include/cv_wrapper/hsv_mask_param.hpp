#ifndef __CV_WRAPPER_HSV_MASK_PARAM_HPP
#define __CV_WRAPPER_HSV_MASK_PARAM_HPP

#include <boost/shared_ptr.hpp>

namespace cv_wrapper
{

  class HSVMaskParam
  {
  public:
    HSVMaskParam(int h_min, int h_max, int s_min, int s_max, int v_min, int v_max)
      : h_min_(h_min), h_max_(h_max), s_min_(s_min), s_max_(s_max), v_min_(v_min), v_max_(v_max) {}

    const int getHMin() const
    {
      return h_min_;
    }

    const int getHMax() const
    {
      return h_max_;
    }

    const int getSMin() const
    {
      return s_min_;
    }

    const int getSMax() const
    {
      return s_max_;
    }

    const int getVMin() const
    {
      return v_min_;
    }

    const int getVMax() const
    {
      return v_max_;
    }

  private:
    int h_min_;
    int h_max_;
    int s_min_;
    int s_max_;
    int v_min_;
    int v_max_;
  };

  typedef boost::shared_ptr<HSVMaskParam> HSVMaskParamPtr;
}

#endif /* __CV_WRAPPER_HSV_MASK_PARAM_HPP */
