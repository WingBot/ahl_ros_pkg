#include <ros/ros.h>
#include "ahl_robot_controller/exceptions.hpp"
#include "ahl_robot_controller/robot/xml_parser.hpp"

using namespace ahl_robot;

void XMLParser::load(const std::string& file_name, const std::string& robot_name, RobotPtr& robot)
{
  file_name_  = file_name;
  robot_name_ = robot_name;
  robot_ = robot;

  if(doc_.LoadFile(file_name_) == false)
  {
    std::stringstream msg;
    msg << "Could not open \"" << file_name_ << "\".";

    throw ahl_robot::Exception("ahl_robot::XMLParser::load", msg.str());
  }

  root_ = doc_.FirstChildElement();
  if(!root_)
  {
    std::stringstream msg;
    msg << robot_name << " doesn't have the first child element.";

    throw ahl_robot::Exception("ahl_robot::XMLParser::load", msg.str());
  }

  last_row_.resize(1, 4);
  last_row_.coeffRef(0, 0) = 0.0;
  last_row_.coeffRef(0, 1) = 0.0;
  last_row_.coeffRef(0, 2) = 0.0;
  last_row_.coeffRef(0, 3) = 1.0;

  this->loadRobot();
  this->loadLinks();
  this->loadJoints();
  this->addJoints();
  this->addLinks();
}

void XMLParser::loadRobot()
{
  bool found = false;

  for(TiXmlElement* elem = root_->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Attribute("name") == robot_name_)
    {
      found = true;
      robot_elem_ = elem;
    }
  }

  if(found == false)
  {
    std::stringstream msg;
    msg << file_name_ << " doesn't include tag of \"" << robot_name_ << "\"." << std::endl;

    throw ahl_robot::Exception("ahl_robot::XMLParser::loadRobot", msg.str());
  }
}

void XMLParser::loadLinks()
{
  for(TiXmlElement* elem = robot_elem_->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("pose"))
    {
      this->loadBaseFrame(elem);
    }
    else if(elem->Value() == std::string("link"))
    {
      this->loadLink(elem);
    }
    else if(elem->Value() == std::string("static"))
    {
      robot_->fix();
    }
  }
}

void XMLParser::loadJoints()
{
  for(TiXmlElement* elem = robot_elem_->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("joint"))
    {
      this->loadJoint(elem);
    }
  }
}

void XMLParser::loadBaseFrame(TiXmlElement* robot_elem)
{
  this->convertTextToMatrix4d(robot_elem->GetText(), base_frame_);
}

void XMLParser::loadLink(TiXmlElement* link_elem)
{
  for(TiXmlElement* elem = link_elem->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    std::string link_name = link_elem->Attribute("name");
    if(link_name.size() == 0)
    {
      std::stringstream msg;
      msg << elem->Value() << " doesn't have name.";

      throw ahl_robot::Exception("ahl_robot::XMLParser::loadLink", msg.str());
    }

    if(elem->Value() == std::string("pose"))
    {
      this->convertTextToMatrix4d(elem->GetText(), this->link(link_name)->org);
    }
    else if(elem->Value() == std::string("inertial"))
    {
      this->loadInertial(link_name, elem);
    }
  }
}

void XMLParser::loadInertial(const std::string& link_name, TiXmlElement* inertial_elem)
{
  for(TiXmlElement* elem = inertial_elem->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("pose"))
    {
      this->convertTextToMatrix4d(elem->GetText(), this->link(link_name)->com);
    }
    else if(elem->Value() == std::string("inertia"))
    {
      this->loadInertia(link_name, elem);
    }
    else if(elem->Value() == std::string("mass"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), this->link(link_name)->M);
    }
  }
}

void XMLParser::loadInertia(const std::string& link_name, TiXmlElement* inertia_elem)
{
  for(TiXmlElement* elem = inertia_elem->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("ixx"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), this->link(link_name)->I.coeffRef(0, 0));
    }
    else if(elem->Value() == std::string("iyy"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), this->link(link_name)->I.coeffRef(1, 1));
    }
    else if(elem->Value() == std::string("izz"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), this->link(link_name)->I.coeffRef(2, 2));
    }
    else if(elem->Value() == std::string("ixy"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), this->link(link_name)->I.coeffRef(0, 1));
      this->link(link_name)->I.coeffRef(1, 0) = this->link(link_name)->I.coeff(0, 1);
    }
    else if(elem->Value() == std::string("iyz"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), this->link(link_name)->I.coeffRef(1, 2));
      this->link(link_name)->I.coeffRef(2, 1) = this->link(link_name)->I.coeff(1, 2);
    }
    else if(elem->Value() == std::string("ixz"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), this->link(link_name)->I.coeffRef(0, 2));
      this->link(link_name)->I.coeffRef(2, 0) = this->link(link_name)->I.coeff(0, 2);
    }
  }
}

void XMLParser::loadJoint(TiXmlElement* joint_elem)
{
  std::string name = joint_elem->Attribute("name");
  std::string type = joint_elem->Attribute("type");

  if(type == "revolute")
  {
    this->joint(name)->is_revolute = true;
  }
  else if(type == "prismatic")
  {
    this->joint(name)->is_revolute = false;
  }
  else
  {
    std::stringstream msg;
    msg << "The joint \"" << name << "\"'s type is invalid." << std::endl
        << "  type : " << type;

    throw ahl_robot::Exception("ahl_robot::XMLParser::loadJoint", msg.str());
  }

  for(TiXmlElement* elem = joint_elem->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("pose"))
    {
      this->convertTextToMatrix4d(elem->GetText(), this->joint(name)->org);
    }
    else if(elem->Value() == std::string("parent"))
    {
      this->setParentLink(name, elem->GetText());
    }
    else if(elem->Value() == std::string("child"))
    {
      this->setChildLink(name, elem->GetText());
    }
    else if(elem->Value() == std::string("axis"))
    {
      this->loadAxisParams(name, elem);
    }
  }
}

void XMLParser::setParentLink(const std::string& joint_name, const std::string& parent_name)
{
  if(parent_name.size() == 0)
  {
    throw ahl_robot::Exception("ahl_robot::XMLParer::setParentLink", "Parent name is empty.");
  }

  this->joint(joint_name)->parent_link = this->link(parent_name);
}

void XMLParser::setChildLink(const std::string& joint_name, const std::string& child_name)
{
  if(child_name.size() == 0)
  {
    throw ahl_robot::Exception("ahl_robot::XMLParser::setChildLink", "Child name is empty.");
  }

  this->joint(joint_name)->link = this->link(child_name);
}

void XMLParser::loadAxisParams(const std::string& joint_name, TiXmlElement* axis_elem)
{
  for(TiXmlElement* elem = axis_elem->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("limit"))
    {
      this->loadLimits(joint_name, elem);
    }
    else if(elem->Value() == std::string("use_parent_model_frame"))
    {
      std_utils::StrUtils::convertToBoolean(elem->GetText(), use_parent_model_frame_[joint_name]);
    }
  }

  for(TiXmlElement* elem = axis_elem->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("xyz"))
    {
      this->loadAxis(joint_name, elem);
    }
  }
}

void XMLParser::loadLimits(const std::string& joint_name, TiXmlElement* limit_elem)
{
  for(TiXmlElement* elem = limit_elem->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("lower"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), this->joint(joint_name)->q_min);
    }
    else if(elem->Value() == std::string("upper"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), this->joint(joint_name)->q_max);
    }
    else if(elem->Value() == std::string("effort"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), this->joint(joint_name)->tau_max);
    }
    else if(elem->Value() == std::string("velocity"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), this->joint(joint_name)->dq_max);
    }
  }
}

void XMLParser::loadAxis(const std::string& joint_name, TiXmlElement* xyz_elem)
{
  std::vector<double> xyz;
  std_utils::StrUtils::convertToVector(xyz_elem->GetText(), xyz, std::string(" "));

  if(xyz.size() != 3)
  {
    std::stringstream msg;
    msg << "Text of \"xyz\" of the joint \"" << joint_name << "\" is invalid." << std::endl
        << "xyz's size : " << xyz.size();

    throw ahl_robot::Exception("ahl_robot::XMLParser::loadAxis", msg.str());
  }

  for(unsigned int i = 0; i < xyz.size(); ++i)
  {
    this->joint(joint_name)->axis.coeffRef(i) = xyz[i];
  }
}

void XMLParser::convertTextToMatrix4d(const std::string& text, Eigen::Matrix4d& mat)
{
  if(text.size() == 0)
  {
    throw ahl_robot::Exception("ahl_robot::XMLParser::convertTextToMatrix4d", "text size is zero.");
  }

  std::vector<double> state;

  if(!std_utils::StrUtils::convertToVector(text, state, std::string(" ")))
  {
    std::stringstream msg;
    msg << "\"" << text << "\" is invalid description.";

    throw ahl_robot::Exception("ahl_robot::XMLParser::convertTextToMatrix4d", msg.str());
  }

  std::vector<double> rpy(3);
  for(unsigned int i = 0; i < 3; ++i)
  {
    mat.coeffRef(i, 3) = state[i];
    rpy[i] = state[i + 3];
  }

  Eigen::Matrix3d rot_mat;
  this->convertRPYToMatrix3d(rpy, rot_mat);

  mat.block(0, 0, 3, 3) = rot_mat;
  mat.block(3, 0, 1, 4) = last_row_;
}

void XMLParser::convertRPYToMatrix3d(const std::vector<double>& rpy, Eigen::Matrix3d& mat)
{
  double alpha = rpy[2];
  double beta  = rpy[1];
  double gamma = rpy[0];

  mat.coeffRef(0, 0) = cos(alpha) * cos(beta);
  mat.coeffRef(0, 1) = cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma);
  mat.coeffRef(0, 2) = cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma);
  mat.coeffRef(1, 0) = sin(alpha) * cos(beta);
  mat.coeffRef(1, 1) = sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma);
  mat.coeffRef(1, 2) = sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma);
  mat.coeffRef(2, 0) = -sin(beta);
  mat.coeffRef(2, 1) = cos(beta) * sin(gamma);
  mat.coeffRef(2, 2) = cos(beta) * cos(gamma);
}

LinkPtr& XMLParser::link(const std::string& name)
{
  if(!links_[name])
  {
    links_[name] = LinkPtr(new Link());
    links_[name]->name = name;
  }

  return links_[name];
}

JointPtr& XMLParser::joint(const std::string& name)
{
  if(!joints_[name])
  {
    joints_[name] = JointPtr(new Joint());
    joints_[name]->name = name;
  }

  return joints_[name];
}

void XMLParser::printResults()
{
  std::map<std::string, LinkPtr>::iterator links_it;
  for(links_it = links_.begin(); links_it != links_.end(); ++links_it)
  {
    links_it->second->print();
  }

  std::map<std::string, JointPtr>::iterator joints_it;
  for(joints_it = joints_.begin(); joints_it != joints_.end(); ++joints_it)
  {
    joints_it->second->print();
  }
}

void XMLParser::addJoints()
{
  std::map<std::string, JointPtr>::iterator joints_it;
  for(joints_it = joints_.begin(); joints_it != joints_.end(); ++joints_it)
  {
    robot_->addJoint(joints_it->first, joints_it->second);
  }
}

void XMLParser::addLinks()
{
  std::map<std::string, LinkPtr>::iterator links_it;
  for(links_it = links_.begin(); links_it != links_.end(); ++links_it)
  {
    robot_->addLink(links_it->first, links_it->second);
  }
}
