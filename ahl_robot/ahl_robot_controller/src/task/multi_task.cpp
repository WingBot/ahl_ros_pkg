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

#include "ahl_robot_controller/exception.hpp"
#include "ahl_robot_controller/task/multi_task.hpp"

using namespace ahl_ctrl;

void MultiTask::addTask(const TaskPtr& task, int priority)
{
  if(multi_task_.find(priority) != multi_task_.end())
  {
    if(task->haveNullSpace())
    {
      for(unsigned int i = 0; i < multi_task_[priority].size(); ++i)
      {
        if(multi_task_[priority][i]->haveNullSpace())
        {
          std::stringstream msg;
          msg << "Two tasks with the same priority have null spaces." << std::endl
              << "  priority : " << priority;
          throw ahl_ctrl::Exception("MultiTask::addTask", msg.str());
        }
      }
    }
  }

  multi_task_[priority].push_back(task);
}

void MultiTask::clear()
{
  multi_task_.clear();
}

void MultiTask::updateModel()
{
  std::map<int, std::vector<TaskPtr> >::iterator it;
  for(it = multi_task_.begin(); it != multi_task_.end(); ++it)
  {
    for(unsigned int i = 0; i < it->second.size(); ++i)
    {
      it->second[i]->updateModel();
    }
  }
}

void MultiTask::computeGeneralizedForce(int dof, Eigen::VectorXd& tau)
{
  tau = Eigen::VectorXd::Zero(dof);

  std::map<int, std::vector<TaskPtr> >::iterator it = multi_task_.begin();
  for(it = multi_task_.begin(); it != multi_task_.end(); ++it)
  {
    N_ = Eigen::MatrixXd::Identity(dof, dof);
    Eigen::VectorXd tau_sum = Eigen::VectorXd::Zero(dof);

    for(unsigned int i = 0; i < it->second.size(); ++i)
    {
      Eigen::VectorXd tau_task;
      it->second[i]->computeGeneralizedForce(tau_task);
      tau_sum += tau_task;

      if(it->second[i]->haveNullSpace())
      {
        N_ = it->second[i]->getNullSpace();
      }
    }

    tau = N_ * tau;
    tau += tau_sum;
  }
}

