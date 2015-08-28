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

#include <fstream>
#include "ahl_utils/io_utils.hpp"
#include "ahl_utils/yaml_utils.hpp"
#include "ahl_utils/yaml_loader.hpp"
#include "neural_network/scaler.hpp"
#include "neural_network/exceptions.hpp"

using namespace nn;

Scaler::Scaler(double max, double min)
  : max_(max),
    min_(min),
    range_(max_ - min_)
{
}

void Scaler::init(const std::string& yaml)
{
  std::ifstream ifs(yaml.c_str());
  if(ifs.fail())
  {
    std::stringstream msg;
    msg << "Could not open \"" << yaml << "\".";

    throw nn::Exception("Scaler::init", msg.str());
  }

  ahl_utils::YAMLLoaderPtr yaml_loader = ahl_utils::YAMLLoaderPtr(new ahl_utils::YAMLLoader(yaml));

  yaml_loader->loadValue("neuron_max", max_);
  yaml_loader->loadValue("neuron_min", min_);
  range_ = max_ - min_;

  yaml_loader->loadVector("max_in", max_in_);
  yaml_loader->loadVector("min_in", min_in_);
  range_in_ = max_in_ - min_in_;

  yaml_loader->loadVector("max_out", max_out_);
  yaml_loader->loadVector("min_out", min_out_);
  range_out_ = max_out_ - min_out_;
}

void Scaler::init(const TrainingDataPtr& data)
{
  if(data->getInput().size() != data->getOutput().size() ||
     data->getInput().size() == 0 ||
     data->getOutput().size() == 0)
  {
    std::stringstream msg;
    msg << "Training data size is not appropriate." << std::endl
        << "        input data size  : " << data->getInput().size() << std::endl
        << "        output data size : " << data->getOutput().size();

    throw nn::Exception("Scaler::init", msg.str());
  }

  this->calcMaxMin(data->getInput(), max_in_, min_in_);
  this->calcMaxMin(data->getOutput(), max_out_, min_out_);

  range_in_  = max_in_ - min_in_;
  range_out_ = max_out_ - min_out_;
}

void Scaler::init(std::vector<double>& max_in, std::vector<double>& min_in,
                  std::vector<double>& max_out, std::vector<double>& min_out)
{
  if(max_in.size() != min_in.size())
  {
    std::stringstream msg;
    msg << "max_in size is different from min_in size." << std::endl
        << "        max_in size : " << max_in.size() << std::endl
        << "        min_in_size : " << min_in.size();

    throw nn::Exception("Scaler::init", msg.str());
  }

  if(max_out.size() != min_out.size())
  {
    std::stringstream msg;
    msg << "max_out size is different from min_out size." << std::endl
        << "        max_out size : " << max_out.size() << std::endl
        << "        min_out size : " << min_out.size();

    throw nn::Exception("Scaler::init", msg.str());
  }

  max_in_ = Eigen::MatrixXd::Zero(max_in.size(), 1);
  min_in_ = Eigen::MatrixXd::Zero(min_in.size(), 1);

  for(unsigned int i = 0; i < max_in_.rows(); ++i)
  {
    max_in_.coeffRef(i) = max_in[i];
    min_in_.coeffRef(i) = min_in[i];
  }

  range_in_ = max_in_ - min_in_;

  max_out_ = Eigen::MatrixXd::Zero(max_out.size(), 1);
  min_out_ = Eigen::MatrixXd::Zero(min_out.size(), 1);

  for(unsigned int i = 0; i < max_out_.rows(); ++i)
  {
    max_out_.coeffRef(i) = max_out[i];
    min_out_.coeffRef(i) = min_out[i];
  }

  range_out_ = max_out_ - min_out_;
}

void Scaler::init(Eigen::MatrixXd& max_in, Eigen::MatrixXd& min_in,
                  Eigen::MatrixXd& max_out, Eigen::MatrixXd& min_out)
{
  if(max_in.rows() != min_in.rows())
  {
    std::stringstream msg;
    msg << "max_in rows is different from min_in rows." << std::endl
        << "        max_in rows : " << max_in.rows() << std::endl
        << "        min_in rows : " << min_in.rows();

    throw nn::Exception("Scaler::init", msg.str());
  }

  if(max_out.rows() != min_out.rows())
  {
    std::stringstream msg;
    msg << "max_out rows is different from min_in rows." << std::endl
        << "        max_out rows : " << max_out.rows() << std::endl
        << "        min_out rows : " << min_out.rows();

    throw nn::Exception("Scaler::init", msg.str());
  }

  max_in_   = max_in;
  min_in_   = min_in;
  range_in_ = max_in_ = min_in_;

  max_out_   = max_out;
  min_out_   = min_out;
  range_out_ = max_out_ - min_out_;
}

void Scaler::normalize(TrainingDataPtr& data)
{
  if(data.get() == NULL)
  {
    throw nn::Exception("Scaler::normalize", "Training data pointer is null.");
  }

  if(data->getInput().size() != data->getOutput().size() ||
     data->getInput().size() == 0 ||
     data->getOutput().size() == 0)
  {
    std::stringstream msg;
    msg << "Training data size is not appropriate." << std::endl
        << "        input data size  : " << data->getInput().size() << std::endl
        << "        output data size : " << data->getOutput().size();

    throw nn::Exception("Scaler::normalize", msg.str());
  }

  unsigned int data_num = data->getInput().size();

  for(unsigned int i = 0; i < data_num; ++i)
  {
    Eigen::MatrixXd tmp_input = data->getInput()[i];
    this->normalize(tmp_input, min_in_, range_in_);
    data->setInput(i, tmp_input);

    Eigen::MatrixXd tmp_output = data->getOutput()[i];
    this->normalize(tmp_output, min_out_, range_out_);
    data->setOutput(i, tmp_output);
  }
}

void Scaler::normalizeInput(std::vector<double>& input)
{
  Eigen::MatrixXd tmp_input(input.size(), 1);
  for(unsigned int i = 0; i < input.size(); ++i)
  {
    tmp_input.coeffRef(i) = input[i];
  }

  this->normalize(tmp_input, min_in_, range_in_);

  for(unsigned int i = 0; i < input.size(); ++i)
  {
    input[i] - tmp_input.coeff(i);
  }
}

void Scaler::normalizeInput(Eigen::MatrixXd& input)
{
  this->normalize(input, min_in_, range_in_);
}

void Scaler::denormalizeOutput(std::vector<double>& output)
{
  Eigen::MatrixXd tmp_output(output.size(), 1);
  for(unsigned int i = 0; i < output.size(); ++i)
  {
    tmp_output.coeffRef(i) = output[i];
  }

  this->denormalize(tmp_output, min_out_, range_out_);

  for(unsigned int i = 0; i < output.size(); ++i)
  {
    output[i] = tmp_output.coeff(i);
  }
}

void Scaler::denormalizeOutput(Eigen::MatrixXd& output)
{
  this->denormalize(output, min_out_, range_out_);
}

void Scaler::save(const std::string& yaml)
{
  if(yaml == "")
  {
    throw nn::Exception("Scaler::save", "File name is empty.");
  }

  if(max_in_.size() == 0 || min_in_.size() == 0 || max_out_.size() == 0 || min_out_.size() == 0)
  {
    std::stringstream msg;
    msg << "max, min vector's sizes are not appropriate." << std::endl
        << "        max_in_ size    : " << max_in_.size() << std::endl
        << "        min_in_ size    : " << min_in_.size() << std::endl
        << "        max_out_ size   : " << max_out_.size() << std::endl
        << "        min_out_ size   : " << min_out_.size();

    throw nn::Exception("Scaler::save", msg.str());
  }

  std::ofstream ofs(yaml.c_str());
  if(ofs.fail())
  {
    std::stringstream msg;
    msg << "Could not open \"" << yaml << "\"";
    throw nn::Exception("Scaler::save", msg.str());
  }

  ofs << "# This file is generated by nn::Scaler and includes max and min values of raw feature data." << std::endl;

  ofs << "neuron_max: " << max_ << std::endl
      << "neuron_min: " << min_ << std::endl
      << std::endl;

  ofs << ahl_utils::YAMLUtils::getVectorStr("max_in", max_in_);
  ofs << ahl_utils::YAMLUtils::getVectorStr("min_in", min_in_);

  ofs << std::endl;

  ofs << ahl_utils::YAMLUtils::getVectorStr("max_out", max_out_);
  ofs << ahl_utils::YAMLUtils::getVectorStr("min_out", min_out_);
}

void Scaler::print()
{
  std::cout << "Scaler::print" << std::endl
            << "neuron max   : " << max_ << std::endl
            << "neuron min   : " << min_ << std::endl
            << "neuron range : " << range_ << std::endl;

  std::cout << "max in :" << std::endl;
  ahl_utils::IOUtils::print(max_in_);

  std::cout << "min in :" << std::endl;
  ahl_utils::IOUtils::print(min_in_);

  std::cout << "range in :" << std::endl;
  ahl_utils::IOUtils::print(range_in_);

  std::cout << "max out :" << std::endl;
  ahl_utils::IOUtils::print(max_out_);

  std::cout << "min out :" << std::endl;
  ahl_utils::IOUtils::print(min_out_);

  std::cout << "range out :" << std::endl;
  ahl_utils::IOUtils::print(range_out_);
}

void Scaler::calcMaxMin(const std::vector<Eigen::MatrixXd>& src, Eigen::MatrixXd& max, Eigen::MatrixXd& min)
{
  if(src.size() == 0)
  {
    throw nn::Exception("Scaler::calcMaxMin", "Vector size of src is zero.");
  }
  if(src[0].rows() == 0)
  {
    throw nn::Exception("Scaler::calcMaxMin", "src[0].rows() is zero.");
  }

  max = src[0];
  min = src[0];

  for(unsigned int i = 0; i < src.size(); ++i)
  {
    for(unsigned int j = 0; j < src[i].rows(); ++j)
    {
      if(max.coeff(j) < src[i].coeff(j))
        max.coeffRef(j) = src[i].coeff(j);
      if(min.coeff(j) > src[i].coeff(j))
        min.coeffRef(j) = src[i].coeff(j);
    }
  }
}

void Scaler::normalize(Eigen::MatrixXd& src, const Eigen::MatrixXd& min, const Eigen::MatrixXd& range)
{
  src = src - min;
  for(unsigned int i = 0; i < src.rows(); ++i)
  {
    src.coeffRef(i) /= range.coeff(i);
  }
  src = src * range_ + Eigen::MatrixXd::Constant(src.rows(), src.cols(), min_);
}

void Scaler::denormalize(Eigen::MatrixXd& src, const Eigen::MatrixXd& min, const Eigen::MatrixXd& range)
{
  src = (src - Eigen::MatrixXd::Constant(src.rows(), src.cols(), min_)) / range_;
  for(unsigned int i = 0; i < src.rows(); ++i)
  {
    src.coeffRef(i) *= range.coeff(i);
  }
  src = src + min;
}


