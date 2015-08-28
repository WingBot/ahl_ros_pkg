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
#include <algorithm>
#include "ahl_utils/io_utils.hpp"
#include "ahl_utils/exception.hpp"
#include "neural_network/training_data.hpp"
#include "neural_network/exceptions.hpp"

using namespace nn;

TrainingData::TrainingData()
  : input_(0), output_(0)
{
}

void TrainingData::add(const std::vector<double>& input, const std::vector<double>& output)
{
  if(input_.size() > 0)
  {
    if(input_[0].rows() != input.size())
    {
      std::stringstream msg;
      msg << "Failed to add input data." << std::endl
          << "Added input data size is different from pre-existed input data size." << std::endl
          << "        Added input data size : " << input.size() << std::endl
          << "        Pre-existed input data size : " << input_[0].rows();

      throw nn::Exception("TrainingData::add", msg.str());
    }
  }

  if(output_.size() > 0)
  {
    if(output_[0].rows() != output.size())
    {
      std::stringstream msg;
      msg << "Failed to add output data." << std::endl
          << "Added output data size is different from pre-existed output data size." << std::endl
          << "        Added output data size : " << output.size() << std::endl
          << "        Pre-existed output data size : " << output_[0].rows();

      throw nn::Exception("TrainingData::add", msg.str());
    }
  }

  Eigen::MatrixXd tmp_input  = Eigen::MatrixXd::Zero(input.size() + 1, 1);
  Eigen::MatrixXd tmp_output = Eigen::MatrixXd::Zero(output.size(), 1);

  for(unsigned int i = 0; i < input.size(); ++i)
  {
    tmp_input.coeffRef(i, 0) = input[i];
  }
  tmp_input.coeffRef(input.size(), 0) = 1.0;

  for(unsigned int i = 0; i < output.size(); ++i)
  {
    tmp_output.coeffRef(i, 0) = output[i];
  }

  input_.push_back(tmp_input);
  output_.push_back(tmp_output);
}

void TrainingData::add(const Eigen::MatrixXd& input, const Eigen::MatrixXd& output)
{
  if(input_.size() > 0)
  {
    if(input_[0].rows() != input.rows())
    {
      std::stringstream msg;
      msg << "Failed to add input data." << std::endl
          << "Added input data size is different from pre-existed input data size." << std::endl
          << "        Added input data size : " << input.rows() << std::endl
          << "        Pre-existed input data size : " << input_[0].rows();

      throw nn::Exception("TrainingData::add", msg.str());
    }
  }

  if(output_.size() > 0)
  {
    if(output_[0].rows() != output.rows())
    {
      std::stringstream msg;
      msg << "Failed to add output data." << std::endl
          << "Added output data size is different from pre-existed output data size." << std::endl
          << "        Added output data size : " << output.rows() << std::endl
          << "        Pre-existed output data size : " << output_[0].rows();

      throw nn::Exception("TrainingData::add", msg.str());
    }
  }

  Eigen::MatrixXd tmp_input  = Eigen::MatrixXd::Ones(input.rows() + 1, 1);
  tmp_input.block(0, 0, input.rows(), 1) = input;

  input_.push_back(input);
  output_.push_back(output);
}

void TrainingData::init(const std::string& name_in, const std::string& name_out)
{
  std::ifstream ifs_in;
  std::ifstream ifs_out;

  try
  {
    ifs_in.open(name_in.c_str());
    if(ifs_in.fail())
    {
      throw std::string(name_in);
    }

    ifs_out.open(name_out.c_str());
    if(ifs_out.fail())
    {
      throw std::string(name_out);
    }
  }
  catch(std::string& str)
  {
    std::stringstream msg;
    msg << "Could not open \"" << str << "\"." << std::endl;

    throw nn::Exception("TrainingData::init", msg.str());
  }

  try
  {
    if(ahl_utils::IOUtils::getValues(ifs_in, input_) == false)
    {
      throw std::string(name_in);
    }
    if(ahl_utils::IOUtils::getValues(ifs_out, output_) == false)
    {
      throw std::string(name_out);
    }
  }
  catch(std::string& str)
  {
    std::stringstream msg;
    msg << "Failed to load values from \"" << str << "\".";

    throw nn::Exception("TrainingData::init", msg.str());
  }

  if(input_.size() != output_.size())
  {
    std::stringstream msg;
    msg << "Size of input data is different from size of output data." << std::endl
        << "        input data size : "  << input_.size() << std::endl
        << "        output data size : " << output_.size();

    throw nn::Exception("TrainingData::init", msg.str());
  }
}

void TrainingData::shuffle()
{
  std::vector<unsigned int> shuffled_idx(input_.size());
  for(unsigned int i = 0; i < input_.size(); ++i)
  {
    shuffled_idx[i] = i;
  }

  std::random_shuffle(shuffled_idx.begin(), shuffled_idx.end());

  std::vector<Eigen::MatrixXd> tmp_input = input_;
  std::vector<Eigen::MatrixXd> tmp_output = output_;

  for(unsigned int i = 0; i < input_.size(); ++i)
  {
    input_[i]  = tmp_input[shuffled_idx[i]];
    output_[i] = tmp_output[shuffled_idx[i]];
  }
}

void TrainingData::separate(std::vector<TrainingDataPtr>& dst, unsigned int sep_num)
{
  unsigned int data_num = input_.size();
  dst.clear();

  if(data_num < sep_num)
  {
    std::stringstream msg;
    msg << "Failed to separate training data." << std::endl
        << "The number of training data is too small." << std::endl
        << "        sep_num  : " << sep_num << std::endl
        << "        data num : " << data_num;

    throw nn::Exception("TrainingData::separate", msg.str());
  }

  std::vector<TrainingDataPtr> sep_data;
  for(unsigned int i = 0; i < sep_num; ++i)
  {
    sep_data.push_back(TrainingDataPtr(new TrainingData()));
  }

  unsigned int idx = 0;
  for(unsigned int i = 0; i < this->getSize(); ++i)
  {
    sep_data[idx]->add(input_[i], output_[i]);
    ++idx;
    if(idx == sep_data.size())
      idx = 0;
  }

  for(unsigned int i = 0; i < sep_num; ++i)
  {
    dst.push_back(sep_data[i]);
  }
}

void TrainingData::separate(std::vector< std::vector<TrainingDataPtr> >& dst, unsigned int thread_num, unsigned int batch_size)
{
  std::vector<TrainingDataPtr> sep_data;
  this->separate(sep_data, thread_num);

  unsigned int max_data_size = 0;

  for(unsigned int i = 0; i < sep_data.size(); ++i)
  {
    if(max_data_size < sep_data[i]->getSize())
    {
      max_data_size = sep_data[i]->getSize();
    }
  }

  unsigned int thread_batch_size = batch_size / thread_num;
  unsigned int thread_batch_num  = static_cast<unsigned int>(std::ceil(1.0 * max_data_size / thread_batch_size));

  dst.resize(thread_num);
  for(unsigned int i = 0; i < thread_num; ++i)
  {
    std::vector<TrainingDataPtr> tmp(thread_batch_num);

    for(unsigned int j = 0; j < tmp.size(); ++j)
    {
      tmp[j] = TrainingDataPtr(new TrainingData());
    }

    for(unsigned int j = 0; j < thread_batch_num; ++j)
    {
      for(unsigned int k = 0; k < thread_batch_size; ++k)
      {
        unsigned int idx = j * thread_batch_size + k;

        if(idx < sep_data[i]->getSize())
        {
          tmp[j]->add(sep_data[i]->getInput()[idx], sep_data[i]->getOutput()[idx]);
        }
        else
        {
          break;
        }
      }

      dst[i].push_back(tmp[j]);
    }
  }
}

void TrainingData::setInput(unsigned int idx, std::vector<double>& input)
{
  if(idx >= input_.size())
  {
    std::stringstream msg;
    msg << "Failed to set training data. idx should be smaller than " << input_.size() << "." << std::endl
        << "        idx : " << idx;

    throw nn::Exception("TrainingData::setInput", msg.str());
  }

  if(input.size() != input_[0].rows())
  {
    std::stringstream msg;
    msg << "Specified data's row number is different from pre existed data's row number." << std::endl
        << "        data row num         : " << input.size() << std::endl
        << "        existed data row num : " << input_[0].rows();

    throw nn::Exception("TrainingData::setInput", msg.str());
  }

  Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(input.size(), 1);

  for(unsigned int i = 0; i < input.size(); ++i)
  {
    tmp.coeffRef(i) = input[i];
  }

  input_[idx] = tmp;
}

void TrainingData::setInput(unsigned int idx, Eigen::MatrixXd& input)
{
  if(idx >= input_.size())
  {
    std::stringstream msg;
    msg << "Failed to set training data. idx should be smaller than " << input_.size() << "." << std::endl
        << "        idx : " << idx;
    throw nn::Exception("TrainingData::setInput", msg.str());
  }

  if(input.rows() != input_[0].rows())
  {
    std::stringstream msg;
    msg << "Specified data's row number is different from pre existed data's row number." << std::endl
        << "        data row num         : " << input.rows() << std::endl
        << "        existed data row num : " << input_[0].rows();

    throw nn::Exception("TrainingData::setInput", msg.str());
  }

  input_[idx] = input;
}

void TrainingData::setOutput(unsigned int idx, std::vector<double>& output)
{
  if(idx >= output_.size())
  {
    std::stringstream msg;
    msg << "Failed to set training data. idx should be smaller than " << output_.size() << "." << std::endl
        << "        idx : " << idx;

    throw nn::Exception("TrainingData::setOutput", msg.str());
  }

  if(output.size() != output_[0].rows())
  {
    std::stringstream msg;
    msg << "Specified data's row number is different from pre existed data's row number." << std::endl
        << "        data row num         : " << output.size() << std::endl
        << "        existed data row num : " << output_[0].rows();

    throw nn::Exception("TrainingData::setOutput", msg.str());
  }

  Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(output.size(), 1);

  for(unsigned int i = 0; i < output.size(); ++i)
  {
    tmp.coeffRef(i) = output[i];
  }

  output_[idx] = tmp;
}

void TrainingData::setOutput(unsigned int idx, Eigen::MatrixXd& output)
{
  if(idx >= output_.size())
  {
    std::stringstream msg;
    msg << "Failed to set training data. idx should be smaller than " << output_.size() << "." << std::endl
        << "        idx : " << idx;
    throw nn::Exception("TrainingData::setOutput", msg.str());
  }

  if(output.rows() != output_[0].rows())
  {
    std::stringstream msg;
    msg << "Specified data's row number is different from pre existed data's row number." << std::endl
        << "        data row num         : " << output.rows() << std::endl
        << "        existed data row num : " << output_[0].rows();

    throw nn::Exception("TrainingData::setOutput", msg.str());
  }

  output_[idx] = output;
}

void TrainingData::print()
{
  std::cout << "TrainingData::print" << std::endl;

  for(unsigned int i = 0; i < input_.size(); ++i)
  {
    std::cout << i << "th data :" << std::endl
              << "input = [ ";

    for(unsigned int j = 0; j < input_[i].rows(); ++j)
    {
      std::cout << input_[i].coeff(j);
      if(j < input_[i].rows() - 1)
      {
        std::cout << " ,";
      }
    }

    std::cout << " ]" << std::endl
              << "output = [";

    for(unsigned int j = 0; j < output_[i].rows(); ++j)
    {
      std::cout << output_[i].coeff(j);
      if(j < output_[i].rows() - 1)
      {
        std::cout << " ,";
      }

      std::cout << " ]" << std::endl;
    }
  }
}
