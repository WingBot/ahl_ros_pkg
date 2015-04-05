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

  this->loadRobot();
  this->loadLinks();
  this->loadJoints();


/*
  this->setupLinks();
  this->setupJoints();
  this->addJoints();
  this->addLinks();
*/
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
  std::string name("");
  double M(0.0);
  Eigen::Matrix3d I   = Eigen::Matrix3d::Zero();
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  Eigen::Vector3d pos_com = Eigen::Vector3d::Zero();
  Eigen::Vector3d rpy_com = Eigen::Vector3d::Zero();

  for(TiXmlElement* elem = link_elem->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    name = link_elem->Attribute("name");
    if(name.size() == 0)
    {
      std::stringstream msg;
      msg << elem->Value() << " doesn't have name.";

      throw ahl_robot::Exception("ahl_robot::XMLParser::loadLink", msg.str());
    }

    if(elem->Value() == std::string("pose"))
    {
      this->loadLinkPose(elem, pos, rpy);
    }
    else if(elem->Value() == std::string("inertial"))
    {
      this->loadInertial(elem, M, I, pos_com, rpy_com);
    }
  }

  LinkPtr link = LinkPtr(
                   new Link(
                     name, I, M,
                     pos.coeff(0), pos.coeff(1), pos.coeff(2),
                     rpy.coeff(0), rpy.coeff(1), rpy.coeff(2),
                     pos_com.coeff(0), pos_com.coeff(1), pos_com.coeff(2),
                     rpy_com.coeff(0), rpy_com.coeff(1), rpy_com.coeff(2)
                   )
                 );

  robot_->addLink(link);


/*
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
      this->convertTextToMatrix4d(elem->GetText(), this->link(link_name)->Tw_org_);
    }
    else if(elem->Value() == std::string("inertial"))
    {
      this->loadInertial(link_name, elem);
    }
  }
*/
}

void XMLParser::loadLinkPose(TiXmlElement* pose_elem,
                             Eigen::Vector3d& pos, Eigen::Vector3d& rpy)
{
  this->convertTextToVectors(pose_elem->GetText(), pos, rpy);
}

void XMLParser::loadInertial(TiXmlElement* inertial_elem, double& M, Eigen::Matrix3d& I,
                             Eigen::Vector3d& pos_com, Eigen::Vector3d& rpy_com)
{
  for(TiXmlElement* elem = inertial_elem->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("pose"))
    {
      this->convertTextToVectors(elem->GetText(), pos_com, rpy_com);
    }
    else if(elem->Value() == std::string("inertia"))
    {
      this->loadInertia(elem, I);
    }
    else if(elem->Value() == std::string("mass"))
    {
      std_utils::StrUtils::convertToNum(elem->GetText(), M);
    }
  }
}


void XMLParser::loadInertia(TiXmlElement* inertia_elem, Eigen::Matrix3d& I)
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

void XMLParser::loadJoint(TiXmlElement* joint_elem)
{
  std::string name = joint_elem->Attribute("name");
  std::string type = joint_elem->Attribute("type");
  bool is_revolute;

  Eigen::Vector3d pos;
  Eigen::Vector3d rpy;

  Eigen::Vector3d axis;

  double q_min;
  double q_max;
  double dq_max;
  double tau_max;

  std::string link_name;
  std::string parent_link_name;

  if(name.size() == 0)
  {
    std::stringstream msg;
    msg << joint_elem->Value() << " doesn't include name.";

    throw ahl_robot::Exception("XMLParser::loadJoint", msg.str());
  }

  if(type == "revolute")
  {
    is_revolute = true;
  }
  else if(type == "prismatic")
  {
    is_revolute = false;
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
      this->convertTextToVectors(elem->GetText(), pos, rpy);
    }
    else if(elem->Value() == std::string("parent"))
    {
      parent_link_name = elem->GetText();
    }
    else if(elem->Value() == std::string("child"))
    {
      link_name = elem->GetText();
    }
    else if(elem->Value() == std::string("axis"))
    {
      this->loadAxisParams(elem, axis, q_min, q_max, dq_max, tau_max);
    }
  }

  JointPtr joint = JointPtr(
                     new Joint(
                       name, is_revolute,
                       pos.coeff(0), pos.coeff(1), pos.coeff(2),
                       rpy.coeff(0), rpy.coeff(1), rpy.coeff(2),
                       axis,
                       q_min, q_max, dq_max, tau_max
                     )
                   );

  robot_->addJoint(joint);
  robot_->connectJointWithLink(name, link_name);
  robot_->connectJointWithParentLink(name, parent_link_name);
}

void XMLParser::loadAxisParams(TiXmlElement* axis_elem, Eigen::Vector3d& axis,
                               double& q_min, double& q_max, double& dq_max, double& tau_max)
{
  for(TiXmlElement* elem = axis_elem->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("limit"))
    {
      this->loadLimits(elem, q_min, q_max, dq_max, tau_max);
    }
    else if(elem->Value() == std::string("use_parent_model_frame"))
    {
      // TODO
    }
  }

  for(TiXmlElement* elem = axis_elem->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
  {
    if(elem->Value() == std::string("xyz"))
    {
      this->loadAxis(elem, axis);
    }
  }
}

void XMLParser::loadLimits(TiXmlElement* limit_elem,
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

void XMLParser::loadAxis(TiXmlElement* xyz_elem, Eigen::Vector3d& axis)
{
  std::vector<double> xyz;
  std_utils::StrUtils::convertToVector(xyz_elem->GetText(), xyz, std::string(" "));

  if(xyz.size() != 3)
  {
    std::stringstream msg;
    msg << "Text of \"xyz\" is invalid." << std::endl
        << "xyz's size : " << xyz.size();

    throw ahl_robot::Exception("ahl_robot::XMLParser::loadAxis", msg.str());
  }

  for(unsigned int i = 0; i < xyz.size(); ++i)
  {
    axis.coeffRef(i) = xyz[i];
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
  mat.block(3, 0, 1, 4) = utils::getLastRowOfTransformationMatrix();
}

void XMLParser::convertTextToVectors(const std::string& text, Eigen::Vector3d& pos, Eigen::Vector3d& rpy)
{
  std::vector<double> values;

  if(!std_utils::StrUtils::convertToVector(text, values, std::string(" ")))
  {
    std::stringstream msg;
    msg << "\"" << text << "\" is invalid description.";

    throw ahl_robot::Exception("ahl_robot::XMLParser::convertTextToVectors", msg.str());
  }

  if(values.size() != 6)
  {
    std::stringstream msg;
    msg << "The number of parameters are wrong." << std::endl
        << "  values.size : " << values.size();

    throw ahl_robot::Exception("ahl_robot::XMLParser::convertTextToVectors", msg.str());
  }

  for(unsigned int i = 0; i < 3; ++i)
  {
    pos.coeffRef(i) = values[i];
    rpy.coeffRef(i) = values[i + 3];
  }
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

/*
void XMLParser::setupLinks()
{
  Links::iterator it;
  LinkPtr root_link;

  for(it = links_.begin(); it != links_.end(); ++it)
  {
    if(it->second->joint_.get() == NULL)
    {
      if(!root_link)
      {
        root_link = it->second;
      }
      else
      {
        std::stringstream msg;
        msg << it->first << " and " << root_link->name_ << " could be root link.";

        throw ahl_robot::Exception("ahl_robot::XMLParser::setupLinks", msg.str());
      }
    }
    else if(it->second->joint_->parent_link_.get() != NULL)
    {
      LinkPtr parent = it->second->joint_->parent_link_;

      // Transformation matrix from parent to world
      Eigen::Matrix4d Tpw_org;
      Eigen::Matrix4d Tpw_com;

      utils::calculateInverseTransformationMatrix(parent->Tw_org_, Tpw_org);
      utils::calculateInverseTransformationMatrix(parent->Tw_com_, Tpw_com);

      it->second->Tr_org_ = Tpw_org * it->second->Tw_org_;
      it->second->Tr_com_org_ = Tpw_com * it->second->Tw_com_org_;
    }
  }

  for(it = links_.begin(); it != links_.end(); ++it)
  {
    it->second->Tw_ = it->second->Tw_org_;
    it->second->Tr_ = it->second->Tr_org_;
    it->second->Tw_com_ = it->second->Tw_com_org_;
    it->second->Tr_com_ = it->second->Tr_com_org_;
  }

  robot_->setRootLink(root_link);
}

void XMLParser::setupJoints()
{
  LinkPtr root_link = robot_->getRootLink();

  Joints::iterator it;
  for(it = root_link->child_joints_.begin(); it != root_link->child_joints_.end(); ++it)
  {
    this->setupJoints(it->second);

    it->second->Tw_ = it->second->Tw_org_;
    it->second->Tr_ = it->second->Tr_org_;
  }
}

void XMLParser::setupJoints(const JointPtr& joint)
{
  if(!joint->parent_link_)
    return;

  // Transformation matrix from parent link to world
  Eigen::Matrix4d Tpw;
  // Transformation matrix from world to parent link
  Eigen::Matrix4d Twp = joint->parent_link_->Tw_org_;

  utils::calculateInverseTransformationMatrix(Twp, Tpw);
  joint->Tr_org_ = Tpw * joint->Tw_org_;

  LinkPtr link = joint->link_;

  if(!link)
    return;

  Joints::iterator it;
  for(it = link->child_joints_.begin(); it != link->child_joints_.end(); ++it)
  {
    this->setupJoints(it->second);
  }

}
*/
