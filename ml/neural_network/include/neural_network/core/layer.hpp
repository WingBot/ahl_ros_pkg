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

#ifndef __NEURAL_NETWORK_CORE_LAYER_HPP
#define __NEURAL_NETWORK_CORE_LAYER_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace nn
{

  class Layer;
  typedef boost::shared_ptr<Layer> LayerPtr;

  class Layer
  {
  public:
    Layer(unsigned int neuron_num);

    void setNext(const LayerPtr& next);
    void setPre(const LayerPtr& pre);

    const unsigned int getNeuronSize() const
    {
      return neuron_.rows();
    }

    const Eigen::MatrixXd& getNeuron() const
    {
      return neuron_;
    }

    Eigen::MatrixXd& getNeuronRef()
    {
      return neuron_;
    }

    const Eigen::MatrixXd& getW() const
    {
      return w_;
    }

    Eigen::MatrixXd& getWRef()
    {
      return w_;
    }

    const LayerPtr& getNext() const
    {
      return next_;
    }

    const LayerPtr& getPre() const
    {
      return pre_;
    }

  private:
    Eigen::MatrixXd neuron_;
    Eigen::MatrixXd w_;

    LayerPtr next_;
    LayerPtr pre_;
  };

}

#endif /* __NEURAL_NETWORK_CORE_LAYER_HPP */
