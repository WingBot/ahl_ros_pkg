#include <algorithm>
#include "ahl_robot/exception.hpp"
#include "ahl_robot/parser/mnp_generator.hpp"
#include "ahl_robot/utils/math.hpp"

using namespace ahl_robot;

MnpGenerator::MnpGenerator(const RobotPtr& robot)
  : robot_(robot)
{
}

void MnpGenerator::generate()
{
  this->checkValidityOfName();
  this->checkConnection();

  // remove unnecessary joints
  std::map<std::string, std::string>::iterator it;
  for(it = robot_->mnp_to_ee.begin(); it != robot_->mnp_to_ee.end(); ++it)
  {
    ManipulatorPtr mnp = ManipulatorPtr(new Manipulator());
    mnp->name = it->first;
    this->generate(it->second, mnp);
    robot_->mnp[it->first] = mnp;
  }

  // add branch link to manipulator
  std::map<std::string, ManipulatorPtr>::iterator mnp_it;
  for(mnp_it = robot_->mnp.begin(); mnp_it != robot_->mnp.end(); ++mnp_it)
  {
    this->setLinkAttachedToManipulator(mnp_it->second);
  }

  // add branch link to base link
  TfLinkPtr tf_base_link = robot_->tf_link[robot_->base_name];
  this->setLinkAttachedToBase();

  // rename link names
  this->rename();

  // show results
  for(mnp_it = robot_->mnp.begin(); mnp_it != robot_->mnp.end(); ++mnp_it)
  {
    ManipulatorPtr mnp = mnp_it->second;

    std::cout << "**********************" << std::endl;
    std::cout << "mnp : " << mnp->name << std::endl;
    std::cout << "**********************" << std::endl;

    std::vector<LinkPtr>::iterator it;
    for(it = mnp->link.begin(); it != mnp->link.end(); ++it)
    {
      LinkPtr link = *it;
      std::cout << "link : " << link->name << std::endl
                << "m : " << link->m << std::endl
                << "I : " << std::endl << link->I << std::endl
                << "com : " << std::endl << link->com << std::endl
                << "axis : " << std::endl << link->axis << std::endl
                << "q_min : " << link->q_min << std::endl
                << "q_max : " << link->q_max << std::endl
                << "dq_max : " << link->dq_max << std::endl
                << "tau_max : " << link->tau_max << std::endl
                << std::endl;
    }
  }
}

void MnpGenerator::checkValidityOfName()
{
  TfLinkMap tf_link = robot_->tf_link;

  if(tf_link.find(robot_->base_name) == tf_link.end())
  {
    std::stringstream msg;
    msg << robot_->name << " doesn't have base link : " << robot_->base_name;
    throw ahl_robot::Exception("ahl_robot::MnpGenerator::checkValidityOfName", msg.str());
  }

  std::map<std::string, std::string>::iterator it;

  for(it = robot_->mnp_to_ee.begin(); it != robot_->mnp_to_ee.end(); ++it)
  {
    if(tf_link.find(it->second) == tf_link.end())
    {
      std::stringstream msg;
      msg << robot_->name << " doesn't have end effector : " << it->second;
      throw ahl_robot::Exception("ahl_robot::MnpGenerator::checkValidityOfName", msg.str());
    }
  }
}

void MnpGenerator::checkConnection()
{
  std::map<std::string, std::string>::iterator it;

  for(it = robot_->mnp_to_ee.begin(); it != robot_->mnp_to_ee.end(); ++it)
  {
    TfLinkPtr tf_link = robot_->tf_link[it->second];
    if(tf_link->name == robot_->base_name)
    {
      std::stringstream msg;
      msg << it->first << " has the same base link name as end effector.";
      throw ahl_robot::Exception("ahl_robot::MnpGenerator::checkConnection", msg.str());
    }

    bool found_movable_joint = false;

    while(true)
    {
      mnp_link_set_.insert(tf_link->name);

      TfJointPtr tf_joint = tf_link->joint;
      if(!tf_joint)
      {
        std::stringstream msg;
        msg << tf_link->name << " doesn't have joint.";
        throw ahl_robot::Exception("ahl_robot::MnpGenerator::checkConnection", msg.str());
      }

      if(tf_joint->type == Link::REVOLUTE || tf_joint->type == Link::PRISMATIC)
      {
        found_movable_joint = true;
      }

      tf_link = tf_joint->parent_link;
      if(!tf_link)
      {
        std::stringstream msg;
        msg << tf_joint->name << " doesn't have parent link.";
        throw ahl_robot::Exception("ahl_robot::MnpGenerator::checkConnection", msg.str());
      }
      else if(tf_link->name == robot_->base_name)
      {
        mnp_link_set_.insert(tf_link->name);
        break;
      }
    }

    if(!found_movable_joint)
    {
      std::stringstream msg;
      msg << it->first << " doesn't have movable joint between "
          << robot_->base_name << " and " << it->second << ".";
        throw ahl_robot::Exception("ahl_robot::MnpGenerator::checkConnection", msg.str());
    }
  }
}

void MnpGenerator::generate(const std::string& ee_name, ManipulatorPtr& mnp)
{
  TfLinkPtr tf_link = robot_->tf_link[ee_name];
  if(tf_link->name == robot_->base_name)
    return;

  std::vector<LinkPtr> tmp_link;
  while(true)
  {
    TfJointPtr tf_joint = tf_link->joint;
    LinkPtr link = LinkPtr(new Link());

    tf_link->copyTo(link);
    tf_joint->copyTo(link);
    tmp_link.push_back(link);

    tf_link = tf_joint->parent_link;
    if(tf_link->name == robot_->base_name)
    {
      LinkPtr base_link = LinkPtr(new Link());

      tf_link->copyTo(base_link);
      tmp_link.push_back(base_link);

      break;
    }
  }

  ManipulatorPtr tmp_mnp = ManipulatorPtr(new Manipulator());
  for(unsigned int i = 0; i < tmp_link.size(); ++i)
  {
    tmp_mnp->link.push_back(tmp_link[i]);
  }

  std::reverse(tmp_mnp->link.begin(), tmp_mnp->link.end());
  this->modifyManipulator(tmp_mnp->link, mnp);
}

void MnpGenerator::modifyManipulator(const std::vector<LinkPtr>& link, ManipulatorPtr& mnp)
{
  std::map<std::string, Eigen::Matrix4d> T_abs;
  LinkPtr parent_link = link.front();

  for(unsigned int i = 0; i < link.size(); ++i)
  {
    T_abs[link[i]->name] = link[i]->T;
  }

  for(unsigned int i = 0; i < link.size(); ++i)
  {
    if(link[i]->name == robot_->base_name)
    {
      mnp->link.push_back(link[i]);
      continue;
    }

    if(link[i]->type == Link::FIXED)
    {
      connection_[link[i]->name] = parent_link->name;

      if(parent_link->name == robot_->base_name)
      {
        // Modify mass
        double m_p = parent_link->m;
        parent_link->m += link[i]->m;

        // Modify inertia
        // Inertia is converted to that in joint frame
        parent_link->I += link[i]->I;

        // Modify COM
        Eigen::Vector3d parent_com_b = parent_link->com.block(0, 3, 3, 1);
        Eigen::Vector3d child_com_c = link[i]->com.block(0, 3, 3, 1);

        Eigen::Matrix3d Rbc = link[i]->T.block(0, 0, 3, 3);
        Eigen::Vector3d child_com_b = Rbc * child_com_c;

        double m_c = link[i]->m;
        double m = m_p + m_c;
        if(m > 0.0)
        {
          parent_link->com.block(0, 3, 3, 1) = (m_p / m) * parent_com_b + (m_c / m) * child_com_b;
        }
      }
      else
      {
        // Modify mass
        double m_p = parent_link->m;
        parent_link->m += link[i]->m;

        // Modify inertia
        // Inertia is converted to that in joint frame
        Eigen::Matrix3d Rbp = T_abs[parent_link->name].block(0, 0, 3, 3);
        Eigen::Matrix3d Rbc = T_abs[link[i]->name].block(0, 0, 3, 3);
        Eigen::Matrix3d Rpc = Rbp.transpose() * Rbc;
        Eigen::Matrix3d Ip = Rpc * link[i]->I * Rpc.transpose();
        parent_link->I += Ip;

        // Modify COM
        Eigen::Vector3d parent_com_p = parent_link->com.block(0, 3, 3, 1);
        Eigen::Vector3d child_com_c = link[i]->com.block(0, 3, 3, 1);
        Eigen::Vector3d child_com_p = Rpc * child_com_c;

        double m_c = link[i]->m;
        double m = m_p + m_c;
        if(m > 0.0)
        {
          parent_link->com.block(0, 3, 3, 1) = (m_p / m) * parent_com_p + (m_c / m) * child_com_p;
        }
      }
    }
    else if(link[i]->type == Link::REVOLUTE ||
            link[i]->type == Link::PRISMATIC)
    {
      if(parent_link->name == robot_->base_name)
      {
        // Do nothing.
      }
      else
      {
        // Modify link representation of I, T_org, T, com, axis
        Eigen::Matrix4d Tbp = T_abs[parent_link->name];
        Eigen::Matrix4d Tbc = T_abs[link[i]->name];
        Eigen::Matrix4d Tpb;
        math::calculateInverseTransformationMatrix(Tbp, Tpb);
        Eigen::Matrix4d Tpc = Tpb * Tbc;

        Eigen::Matrix4d Tcb;
        math::calculateInverseTransformationMatrix(Tbc, Tcb);
        Eigen::Matrix3d Rcb = Tcb.block(0, 0, 3, 3);
        Eigen::Matrix3d I_b = link[i]->I;
        Eigen::Matrix4d com_b = link[i]->com;
        Eigen::Vector3d axis_b = link[i]->axis;

        link[i]->T = link[i]->T_org = Tpc;
        link[i]->I = Rcb * I_b * Rcb.transpose();
        link[i]->com = Tcb * com_b;
        link[i]->axis = Rcb * axis_b;
      }

      mnp->link.push_back(link[i]);
      parent_link = link[i];
    }
    else
    {
      std::stringstream msg;
      msg << "Invalid joint type." << std::endl
          << "  type : " << link[i]->type;
      throw ahl_robot::Exception("ahl_robot::MnpGenerator::removeUnnecessaryJoint", msg.str());
    }
  }
}

void MnpGenerator::setLinkAttachedToManipulator(ManipulatorPtr& mnp)
{
  std::map<std::string, std::string> branch_map;// first : name of the branch link, second : the branch is attached to which link ?

  std::set<std::string>::iterator set_it;
  for(set_it = mnp_link_set_.begin(); set_it != mnp_link_set_.end(); ++set_it)
  {
    std::string name = *set_it;
    TfLinkPtr tf_link = robot_->tf_link[name];

    this->storeBranch(mnp, tf_link->name, tf_link, branch_map);
  }

  std::map<std::string, std::string>::iterator branch_it;
  for(branch_it = branch_map.begin(); branch_it != branch_map.end(); ++branch_it)
  {
    std::string branch = branch_it->first;
    std::string root = branch_it->second;

    if(root == robot_->base_name)
      continue;

    if(robot_->tf_link.find(branch) == robot_->tf_link.end())
    {
      std::stringstream msg;
      msg << robot_->name << " doesn't have " << branch << " as TfLink.";
      throw ahl_robot::Exception("ahl_robot::MnpGenerator::setLinkAttachedToManipulator", msg.str());
    }
    TfLinkPtr tf_branch_link = robot_->tf_link[branch];

    if(robot_->tf_link.find(root) == robot_->tf_link.end())
    {
      std::stringstream msg;
      msg << robot_->name << " doesn't have " << root << " as TfLink.";
      throw ahl_robot::Exception("ahl_robot::MnpGenerator::setLinkAttachedToManipulator", msg.str());
    }
    TfLinkPtr tf_root_link = robot_->tf_link[root];

    std::vector<LinkPtr>::iterator it;
    LinkPtr root_link;
    for(it = mnp->link.begin(); it != mnp->link.end(); ++it)
    {
      if((*it)->name == root)
      {
        root_link = (*it);
      }
    }

    if(!root_link)
    {
      std::stringstream msg;
      msg << robot_->name << " doesn't have " << root << ".";
      throw ahl_robot::Exception("ahl_robot::MnpGenerator::setLinkAttachedToManipulator", msg.str());
    }

    Eigen::Matrix4d Tbase_r = tf_root_link->T;
    Eigen::Matrix4d Tbase_b = tf_branch_link->T;
    Eigen::Matrix4d Tr_base;
    math::calculateInverseTransformationMatrix(Tbase_r, Tr_base);
    Eigen::Matrix4d Trb = Tr_base * Tbase_b;
    Eigen::Matrix3d Rrb = Trb.block(0, 0, 3, 3);
    Eigen::Matrix4d Tb_base;
    math::calculateInverseTransformationMatrix(Tbase_b, Tb_base);
    Eigen::Matrix3d Rb_base = Tb_base.block(0, 0, 3, 3);

    double m_r = root_link->m;
    root_link->m += tf_branch_link->m;
    root_link->I += Rrb * Rb_base * tf_branch_link->I * Rb_base.transpose() * Rrb.transpose();

    double m_b = tf_branch_link->m;
    double m = m_r + m_b;
    Eigen::Vector3d com_root_r = root_link->com.block(0, 3, 3, 1);
    Eigen::Vector3d com_branch_base = tf_branch_link->com.block(0, 3, 3, 1);
    Eigen::Vector3d com_branch_r = Rrb * Rb_base * com_branch_base;

    if(m > 0.0)
    {
      root_link->com.block(0, 3, 3, 1) = (m_r / m) * com_root_r + (m_b / m) * com_branch_r;
    }
  }
}

void MnpGenerator::storeBranch(const ManipulatorPtr& mnp, const std::string& root, const TfLinkPtr& tf_link, std::map<std::string, std::string>& branch_map)
{
  if(tf_link->child_joint.size() <= 1)
    return;

  TfJointMap::iterator it;

  for(it = tf_link->child_joint.begin(); it != tf_link->child_joint.end(); ++it)
  {
    TfJointPtr joint = it->second;

    if(!joint->link)
    {
      continue;
    }

    TfLinkPtr branch = it->second->link;

    bool found = false;
    std::vector<LinkPtr>::iterator link_it;
    for(link_it = mnp->link.begin(); link_it != mnp->link.end(); ++link_it)
    {
      if(branch->name == (*link_it)->name)
      {
        found = true;
      }
    }

    if(found)
    {
      continue;
    }

    std::string new_root = root;
    if(connection_.find(root) != connection_.end())
    {
      new_root = connection_[root];
    }

    branch_map[branch->name] = new_root;
    this->storeBranch(mnp, new_root, branch, branch_map);
  }
}

void MnpGenerator::setLinkAttachedToBase()
{
  std::map<std::string, TfLinkPtr>::iterator it;
  for(it = robot_->tf_link.begin(); it != robot_->tf_link.end(); ++it)
  {
    if(mnp_link_set_.find(it->second->name) == mnp_link_set_.end())
    {
      branch_base_.insert(it->second->name);
    }
  }

  LinkPtr base_link = robot_->mnp.begin()->second->link.front();

  std::set<std::string>::iterator branch_it;
  for(branch_it = branch_base_.begin(); branch_it != branch_base_.end(); ++branch_it)
  {
    std::string name = *branch_it;

    if(robot_->tf_link.find(name) == robot_->tf_link.end())
    {
      std::stringstream msg;
      msg << robot_->name << " doesn't have " << name << "." << std::endl;
      throw ahl_robot::Exception("ahl_robot::MnpGenerator::setLinkAttachedToBase", msg.str());
    }

    TfLinkPtr tf_link = robot_->tf_link[name];
    double m_b = base_link->m;
    base_link->m += tf_link->m;
    base_link->I += tf_link->I;

    Eigen::Vector3d com_b = base_link->com.block(0, 3, 3, 1);
    Eigen::Vector3d com_l = tf_link->com.block(0, 3, 3, 1);

    double m_l = tf_link->m;
    double m = m_b + m_l;
    if(m > 0.0)
    {
      base_link->com.block(0, 3, 3, 1) = (m_b / m) * com_b + (m_l / m) * com_l;
    }
  }
}

void MnpGenerator::rename()
{
  std::map<std::string, ManipulatorPtr>::iterator mnp_it;
  for(mnp_it = robot_->mnp.begin(); mnp_it != robot_->mnp.end(); ++mnp_it)
  {
    ManipulatorPtr mnp = mnp_it->second;
    std::vector<LinkPtr>::iterator it;

    for(it = mnp->link.begin(); it != mnp->link.end(); ++it)
    {
      std::string name = robot_->name + "/" + (*it)->name;
      std::cout << name << std::endl;
      (*it)->name = name;
    }
  }
}
