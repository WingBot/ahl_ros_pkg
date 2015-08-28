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

#ifndef __AHL_DIGITAL_FILTER_PSEUDO_DIFFERENTIATOR_HPP
#define __AHL_DIGITAL_FILTER_PSEUDO_DIFFERENTIATOR_HPP

#include <Eigen/Dense>
#include "ahl_digital_filter/differentiator.hpp"
#include "ahl_digital_filter/pseudo_differentiator.hpp"

namespace ahl_filter
{

  class PseudoDifferentiator : public Differentiator
  {
  public:
    PseudoDifferentiator(double period, double cutoff_freq);

    virtual void init(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);
    virtual void apply(const Eigen::VectorXd& q);
    virtual void copyDerivativeValueTo(Eigen::VectorXd& dq)
    {
      dq = dq_;
    }

  private:
    Eigen::VectorXd q_;
    Eigen::VectorXd dq_;
    Eigen::VectorXd pre_q_;
    Eigen::VectorXd pre_dq_;
    double period_;
    double T_;

    double coeff1_;
    double coeff2_;
  };

}

#endif /* __AHL_DIGITAL_FILTER_PSEUDO_DIFFERENTIATOR_HPP */
