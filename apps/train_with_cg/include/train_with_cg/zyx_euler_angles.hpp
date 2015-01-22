/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Daichi Yoshikawa
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

#ifndef __TRAIN_WITH_CG_ZYX_EULER_ANGLES_HPP
#define __TRAIN_WITH_CG_ZYX_EULER_ANGLES_HPP

#include <boost/shared_ptr.hpp>
#include "train_with_cg/orientation.hpp"

namespace train
{

  class ZYXEulerAngles : public Orientation
  {
  public:
    ZYXEulerAngles(const std::vector<double>& max, const std::vector<double>& min, double step);
    virtual bool isQuaternion();
    virtual bool isSameAs(const OrientationPtr& orientation, double threshold);
    virtual bool update();
    virtual bool isLast();
    virtual void set(const Eigen::MatrixXd& orientation);
    virtual void set(double euler_x, double euler_y, double euler_z);
    virtual const Eigen::MatrixXd& getOrientation() const;

  private:
    Eigen::MatrixXd euler_;

    std::vector<double> max_;
    std::vector<double> min_;
    double step_;
    std::vector<unsigned int> step_size_;
    std::vector<unsigned int> step_idx_;
  };

  typedef boost::shared_ptr<ZYXEulerAngles> ZYXEulerAnglesPtr;
}

#endif /* __TRAIN_WITH_CG_ZYX_EULER_ANGLES_HPP */
