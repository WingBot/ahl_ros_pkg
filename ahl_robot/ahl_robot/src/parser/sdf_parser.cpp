#include <std_utils/str_utils.hpp>
#include "ahl_robot/exception.hpp"
#include "ahl_robot/parser/sdf_parser.hpp"
#include "ahl_robot/utils/math.hpp"

using namespace ahl_robot;

SDFParser::SDFParser()
  : path_("")
{
}

void SDFParser::load(const std::string& path, RobotPtr& robot)
{
  path_ = path;
  robot_ = robot;

  if(doc_.LoadFile(path_) == false)
  {
    std::stringstream msg;
    msg << "Could not open \"" << path_ << "\".";
    throw ahl_robot::Exception("ahl_robot::SDFParser::load", msg.str());
  }

  root_ = doc_.FirstChildElement();
  if(!root_)
  {
    std::stringstream msg;
    msg << path_ << " doesn't have the first child element.";
    throw ahl_robot::Exception("ahl_robot::SDFParser::load", msg.str());
  }
  std::cout << path_ << " was opened successfully." << std::endl;

  this->loadRobot();
  this->loadPose();
  this->loadLink();
  this->loadJoint();
  this->modifyTransformationMatrix();

  robot_->tf_joint = joint_;
  robot_->tf_link  = link_;

  mnp_generator_ = MnpGeneratorPtr(new MnpGenerator(robot_));
  mnp_generator_->generate();
}

void SDFParser::loadRobot()
{
  bool found = false;

  for(TiXmlElement* elem = root_->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Attribute("name") == robot_->name)
    {
      found = true;
      robot_elem_ = elem;
    }
  }

  if(found == false)
  {
    std::stringstream msg;
    msg << path_ << " doesn't include tag of \"" << robot_->name << "\"." << std::endl;
    throw ahl_robot::Exception("ahl_robot::SDFParser::loadRobot", msg.str());
  }

  std::cout << robot_->name << " was found." << std::endl;
}

void SDFParser::loadPose()
{
  for(TiXmlElement* elem = robot_elem_->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("pose"))
    {
      this->loadPose(elem);
      std::cout << "Base pose was found." << std::endl
                << T_pose_ << std::endl;
      return;
    }
  }

  std::cout << "Base pose was not found. Default value is used." << std::endl;
}

void SDFParser::loadLink()
{
  bool found = false;

  for(TiXmlElement* elem = robot_elem_->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("link"))
    {
      this->loadLink(elem);
      found = true;
    }
  }

  if(!found)
  {
    std::stringstream msg;
    msg << path_ << " doesn't have any links.";
    throw ahl_robot::Exception("ahl_robot::SDFParser::loadLink", msg.str());
  }
}

void SDFParser::loadJoint()
{
  for(TiXmlElement* elem = robot_elem_->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("joint"))
    {
      this->loadJoint(elem);
    }
  }
}

void SDFParser::loadPose(TiXmlElement* pose_elem)
{
  this->textToMatrix4d(pose_elem->GetText(), T_pose_);
}

void SDFParser::loadLink(TiXmlElement* link_elem)
{
  double m = 0.0;
  Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  Eigen::Vector3d xyz_com = Eigen::Vector3d::Zero();
  Eigen::Vector3d rpy_com = Eigen::Vector3d::Zero();

  std::string name = link_elem->Attribute("name");
  if(name.size() == 0)
  {
    std::stringstream msg;
    msg << link_elem->Value() << "doesn't have name.";
    throw ahl_robot::Exception("ahl_robot::SDFParser::loadLink", msg.str());
  }

  for(TiXmlElement* elem = link_elem->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("pose"))
    {
      this->loadLinkPose(elem, xyz, rpy);
    }
    else if(elem->Value() == std::string("inertial"))
    {
      this->loadLinkInertia(elem, m, I, xyz_com, rpy_com);
    }
  }

  TfLinkPtr link = TfLinkPtr(new TfLink());
  link->name = name;
  link->I = I;
  link->m = m;

  link->T = Eigen::Matrix4d::Identity();
  link->T.block(0, 3, 3, 1) = xyz;
  Eigen::Matrix3d R;
  math::rpyToRotationMatrix(rpy, R);
  link->T.block(0, 0, 3, 3) = R;

  link->com = Eigen::Matrix4d::Identity();
  link->com.block(0, 3, 3, 1) = xyz_com;
  math::rpyToRotationMatrix(rpy_com, R);
  link->com.block(0, 0, 3, 3) = R;

  link_[link->name] = link;

  std::cout << "Loaded link : " << link->name << std::endl;
}

void SDFParser::loadLinkPose(TiXmlElement* pose_elem, Eigen::Vector3d& xyz, Eigen::Vector3d& rpy)
{
  this->textToVectors(pose_elem->GetText(), xyz, rpy);
}

void SDFParser::loadLinkInertia(
  TiXmlElement* inertia_elem, double& m, Eigen::Matrix3d& I,
  Eigen::Vector3d& xyz_com, Eigen::Vector3d& rpy_com)
{
  for(TiXmlElement* elem = inertia_elem->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("pose"))
    {
      this->textToVectors(elem->GetText(), xyz_com, rpy_com);
    }
    else if(elem->Value() == std::string("inertia"))
    {
      this->loadInertia(elem, I);
    }
    else if(elem->Value() == std::string("mass"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), m);
    }
  }
}

void SDFParser::loadInertia(TiXmlElement* inertia_elem, Eigen::Matrix3d& I)
{
  for(TiXmlElement* elem = inertia_elem->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("ixx"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), I.coeffRef(0, 0));
    }
    else if(elem->Value() == std::string("iyy"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), I.coeffRef(1, 1));
    }
    else if(elem->Value() == std::string("izz"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), I.coeffRef(2, 2));
    }
    else if(elem->Value() == std::string("ixy"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), I.coeffRef(0, 1));
      I.coeffRef(1, 0) = I.coeff(0, 1);
    }
    else if(elem->Value() == std::string("iyz"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), I.coeffRef(1, 2));
      I.coeffRef(2, 1) = I.coeff(1, 2);
    }
    else if(elem->Value() == std::string("ixz"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), I.coeffRef(0, 2));
      I.coeffRef(2, 0) = I.coeff(0, 2);
    }
  }
}

void SDFParser::loadJoint(TiXmlElement* joint_elem)
{
  std::string name = joint_elem->Attribute("name");
  if(name.size() == 0)
  {
    std::stringstream msg;
    msg << joint_elem->Value() << " doesn't have a name.";
    throw ahl_robot::Exception("ahl_robot::SDFParser::loadJoint", msg.str());
  }

  Link::JointType type = Link::FIXED;

  std::string type_str = joint_elem->Attribute("type");
  if(type_str == std::string("revolute"))
  {
    type = Link::REVOLUTE;
  }
  else if(type_str == "prismatic")
  {
    type = Link::PRISMATIC;
  }
  else if(type_str == "screw")
  {
    type = Link::PRISMATIC;
    std::cout << "Warning : joint type \"screw\" is taken as \"prismatic\"." << std::endl;
  }
  else
  {
    std::stringstream msg;
    msg << joint_elem->Value() << " has an invalid joint type : " << type_str;
    throw ahl_robot::Exception("ahl_robot::SDFParser::loadJoint", msg.str());
  }

  Eigen::Vector3d xyz;
  Eigen::Vector3d rpy;
  Eigen::Vector3d axis;

  double q_min;
  double q_max;
  double dq_max;
  double tau_max;

  std::string link_name;
  std::string parent_link_name;

  for(TiXmlElement* elem = joint_elem->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("parent"))
    {
      parent_link_name = elem->GetText();
    }
    else if(elem->Value() == std::string("child"))
    {
      link_name = elem->GetText();
    }
    else if(elem->Value() == std::string("pose"))
    {
      this->textToVectors(elem->GetText(), xyz, rpy);
    }
    else if(elem->Value() == std::string("axis"))
    {
      this->loadAxis(elem, axis, q_min, q_max, dq_max, tau_max);
    }
  }

  if((q_min == 0.0 && q_max == 0.0) || tau_max == 0.0)
  {
    type = Link::FIXED;
  }

  TfJointPtr joint = TfJointPtr(new TfJoint());
  joint->name = name;
  joint->type = type;

  joint->T = Eigen::Matrix4d::Identity();
  joint->T.block(0, 3, 3, 1) = xyz;
  Eigen::Matrix3d R;
  math::rpyToRotationMatrix(rpy, R);
  joint->T.block(0, 0, 3, 3) = R;
  joint->axis = axis;

  joint->q_min = q_min;
  joint->q_max = q_max;
  joint->dq_max = dq_max;
  joint->tau_max = tau_max;

  if(link_.find(link_name) == link_.end())
  {
    std::stringstream msg;
    msg << "Could not find " << link_name << "." <<std::endl;
    throw ahl_robot::Exception("ahl_robot::SDFParser::loadJoint", msg.str());
  }
  else if(link_.find(parent_link_name) == link_.end())
  {
    std::stringstream msg;
    msg << "Could not find " << parent_link_name << "." <<std::endl;
    throw ahl_robot::Exception("ahl_robot::SDFParser::loadJoint", msg.str());
  }

  // connect joint to child link
  joint->link = link_[link_name];
  link_[link_name]->joint = joint;

  // connect joint to parent link
  joint->parent_link = link_[parent_link_name];
  link_[parent_link_name]->child_joint[joint->name] = joint;

  joint_[joint->name] = joint;

  std::cout << "Loaded joint : " << joint->name << std::endl
            << "  parent : " << joint->parent_link->name << std::endl
            << "  child  : " << joint->link->name << std::endl;
}

void SDFParser::loadAxis(
  TiXmlElement* axis_elem, Eigen::Vector3d& axis,
  double& q_min, double& q_max, double& dq_max, double& tau_max)
{
  for(TiXmlElement* elem = axis_elem->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("limit"))
    {
      this->loadLimit(elem, q_min, q_max, dq_max, tau_max);
    }
    else if(elem->Value() == std::string("xyz"))
    {
      this->loadXYZ(elem, axis);
    }
    else if(elem->Value() == std::string("use_paret_model_frame"))
    {
      if(elem->GetText() == std::string("false"))
      {
        throw ahl_robot::Exception("ahl_robot::SDFParser::loadAxis", "use_parent_model_frame == false is not supported.");
      }
    }
  }
}

void SDFParser::loadLimit(
  TiXmlElement* limit_elem,
  double& q_min, double& q_max, double& dq_max, double& tau_max)
{
  for(TiXmlElement* elem = limit_elem->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("lower"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), q_min);
    }
    else if(elem->Value() == std::string("upper"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), q_max);
    }
    else if(elem->Value() == std::string("effort"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), tau_max);
    }
    else if(elem->Value() == std::string("velocity"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), dq_max);
    }
  }
}

void SDFParser::loadXYZ(TiXmlElement* xyz_elem, Eigen::Vector3d& axis)
{
  std::vector<double> xyz;
  std_utils::StrUtils::convertToVector(xyz_elem->GetText(), xyz, std::string(" "));

  if(xyz.size() != 3)
  {
    std::stringstream msg;
    msg << xyz_elem->Value() << " has invalid description.";
    throw ahl_robot::Exception("ahl_robot::SDFParser::loadXYZ", msg.str());
  }

  for(unsigned int i = 0; i < xyz.size(); ++i)
  {
    axis.coeffRef(i) = xyz[i];
  }
}

void SDFParser::textToMatrix4d(const std::string& text, Eigen::Matrix4d& mat)
{
  std::vector<double> state;

  bool ret = std_utils::StrUtils::convertToVector(text, state, std::string(" "));

  if(ret == false || state.size() != 6)
  {
    std::stringstream msg;
    msg << "\"" << text << "\" is invalid description.";
    throw ahl_robot::Exception("ahl_robot::SDFParser::textToMatrix4d", msg.str());
  }

  std::vector<double> rpy(3);
  for(unsigned int i = 0; i < 3; ++i)
  {
    mat.coeffRef(i, 3) = state[i];
    rpy[i] = state[i + 3];
  }

  Eigen::Matrix3d rot_mat;
  math::rpyToRotationMatrix(rpy, rot_mat);

  mat.block(0, 0, 3, 3) = rot_mat;
  mat.block(3, 0, 1, 4) = Eigen::MatrixXd::Zero(1, 4);
  mat.coeffRef(3, 3) = 1.0;
}

void SDFParser::textToVectors(const std::string& text,Eigen::Vector3d& xyz, Eigen::Vector3d& rpy)
{
  std::vector<double> vals;

  bool ret = std_utils::StrUtils::convertToVector(text, vals, std::string(" "));
  if(ret == false || vals.size() != 6) // vals should have xyz, rpy
  {
    std::stringstream msg;
    msg << "\"" << text << "\" is invalid description.";
    throw ahl_robot::Exception("ahl_robot::SDFParser::textToVectors", msg.str());
  }

  for(unsigned int i = 0; i < xyz.rows(); ++i)
  {
    xyz.coeffRef(i) = vals[i];
    rpy.coeffRef(i) = vals[i + 3];
  }
}

void SDFParser::modifyTransformationMatrix()
{
  // Frame which TfLink has is represented in world fame.
  // Frames attached to link other than base link are converted to frames in base frame.

  if(link_.find(robot_->base_name) == link_.end())
  {
    std::stringstream msg;
    msg << robot_->name << " doesn't have " << robot_->base_name << "." << std::endl;
    throw ahl_robot::Exception("ahl_robot::SDFParser::modifyTransformationMatrix", msg.str());
  }
  TfLinkPtr tf_base_link = link_[robot_->base_name];

  TfLinkMap::iterator link_it;

  for(link_it = link_.begin(); link_it != link_.end(); ++link_it)
  {
    // Frame attached to link is not converted. It's in world frame.
    if(link_it->first == robot_->base_name)
      continue;

    TfLinkPtr link = link_it->second;

    // world to base
    Eigen::Matrix4d Twb = tf_base_link->T;
    // base link to world
    Eigen::Matrix4d Tbw;
    math::calculateInverseTransformationMatrix(Twb, Tbw);
    // base link to link (link->T is equal to Twl)
    Eigen::Matrix4d Tbl = Tbw * link->T;

    Eigen::Matrix3d Rbl = Tbl.block(0, 0, 3, 3);
    // center of mass is represented in link frame
    Eigen::Matrix4d Tlcom = link->com;
    Eigen::Matrix3d Rlcom = Tlcom.block(0, 0, 3, 3);

    // Convert from frame attached to link in world frame to that in base frame
    link->T = Tbl;
    // Convert from I in com frame to that in base frame
    link->I = Rbl * Rlcom * link->I * Rlcom.transpose() * Rbl.transpose();
    // Convert from com in link frame to that in base frame
    link->com = Tbl * Tlcom;
  }

  // Frame which TfJoint has is represented in link fame.
  // These frames are converted to those in base frame.
  TfJointMap::iterator joint_it;
  std::map<std::string, Eigen::Matrix4d> T_joint;

  for(joint_it = joint_.begin(); joint_it != joint_.end(); ++joint_it)
  {
    T_joint[joint_it->first] = joint_it->second->T;
  }

  for(joint_it = joint_.begin(); joint_it != joint_.end(); ++joint_it)
  {
    TfJointPtr joint = joint_it->second;

    TfLinkPtr link = joint->link;
    if(!link)
      continue;

    // base link to link
    Eigen::Matrix4d Tbl = link->T;
    // link to joint
    Eigen::Matrix4d Tlj = T_joint[joint->name];
    // parent link to joint
    Eigen::Matrix4d Tbj = Tbl * Tlj;

    // Convert from frame attached to joint in world frame to that in base frame
    joint->T = Tbj;

    // Axis is represented in joint frame
    Eigen::Vector3d axis_j = joint->axis;
    // Convert from axis in joint frame to that in base frame
    Eigen::Matrix3d Rbj = Tbj.block(0, 0, 3, 3);
    joint->axis = Rbj * axis_j;
  }
}
/*
void SDFParser::modifyTransformationMatrix()
{
  TfLinkMap::iterator link_it;
  std::map<std::string, Eigen::Matrix4d> T_link;

  for(link_it = link_.begin(); link_it != link_.end(); ++link_it)
  {
    T_link[link_it->first] = T_pose_ * link_it->second->T;
  }

  for(link_it = link_.begin(); link_it != link_.end(); ++link_it)
  {
    TfLinkPtr link = link_it->second;

    TfJointPtr joint = link->joint;
    if(!joint)
      continue;

    TfLinkPtr parent_link = joint->parent_link;
    if(!parent_link)
      continue;

    // world to parent link
    Eigen::Matrix4d Twp = T_link[parent_link->name];
    // parent link to world
    Eigen::Matrix4d Tpw;
    math::calculateInverseTransformationMatrix(Twp, Tpw);
    // parent link to link
    Eigen::Matrix4d Tpl = Tpw * T_link[link->name];

    link->T = link->T_org = Tpl;
  }

  TfJointMap::iterator joint_it;
  std::map<std::string, Eigen::Matrix4d> T_joint;

  for(joint_it = joint_.begin(); joint_it != joint_.end(); ++joint_it)
  {
    T_joint[joint_it->first] = T_pose_ * joint_it->second->T;
  }

  for(joint_it = joint_.begin(); joint_it != joint_.end(); ++joint_it)
  {
    TfJointPtr joint = joint_it->second;

    TfLinkPtr link = joint->link;
    if(!link)
      continue;

    // parent link to link
    Eigen::Matrix4d Tpl = link->T;
    // link to joint
    Eigen::Matrix4d Tlj = T_joint[joint->name];
    // parent link to joint
    Eigen::Matrix4d Tpj = Tpl * Tlj;

    joint->T = joint->T_org = Tpj;
  }
}
*/
