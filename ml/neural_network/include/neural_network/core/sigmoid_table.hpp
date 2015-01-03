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

#ifndef __NEURAL_NETWORK_CORE_SIGMOID_TABLE_HPP
#define __NEURAL_NETWORK_CORE_SIGMOID_TABLE_HPP

#include <cmath>
#include <vector>
#include "neural_network/core/activation.hpp"

namespace nn
{

  class SigmoidTable : public Activation
  {
  public:
    SigmoidTable(double gain = 1.0, unsigned long resolution = 100000);

    const double getOutput(double input) const
    {
      if(input < INPUT_MIN)
        return 0.0;
      if(input > INPUT_MAX)
        return 1.0;

      return table_[(input - INPUT_MIN) / step_ - 1];
    }

    const double getDerivative(double input) const
    {
      return gain_ * (1.0 - input) * input;
    }

  private:
    void createTable(unsigned long resolution);
    double sigmoid(double input);

    static const double INPUT_MIN = -10.0;
    static const double INPUT_MAX = 10.0;
    double step_;
    double gain_;
    std::vector<double> table_;
  };

}

#endif /* __NEURAL_NETWORK_CORE_SIGMOID_TABLE_HPP */
