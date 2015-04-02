#ifndef __AHL_ROBOT_CONTROLLER_XML_PARSER_HPP
#define __AHL_ROBOT_CONTROLLER_XML_PARSER_HPP

#include <string>
#include <map>
#include <Eigen/Dense>
#include <tinyxml.h>
#include <std_utils/str_utils.hpp>
#include "ahl_robot_controller/robot/link.hpp"
#include "ahl_robot_controller/robot/joint.hpp"
#include "ahl_robot_controller/robot/parser.hpp"

namespace ahl_robot
{

  class XMLParser : public Parser
  {
  public:
    void load(const std::string& file_name, const std::string& robot_name, RobotPtr& robot);

  private:
    void loadRobot();
    void loadLinks();
    void loadJoints();

    void loadBaseFrame(TiXmlElement* robot_elem);
    void loadLink(TiXmlElement* link_elem);
    void loadInertial(const std::string& link_name, TiXmlElement* inertial_elem);
    void loadInertia(const std::string& link_name, TiXmlElement* inertia_elem);

    void loadJoint(TiXmlElement* joint_elem);
    void setParentLink(const std::string& joint_name, const std::string& parent_name);
    void setChildLink(const std::string& joint_name, const std::string& parent_name);
    void loadAxisParams(const std::string& joint_name, TiXmlElement* axis_elem);
    void loadLimits(const std::string& joint_name, TiXmlElement* limit_elem);
    void loadAxis(const std::string& joint_name, TiXmlElement* xyz_elem);

    void convertTextToMatrix4d(const std::string& text, Eigen::Matrix4d& mat);
    void convertRPYToMatrix3d(const std::vector<double>& rpy, Eigen::Matrix3d& mat);

    void addJoints();
    void addLinks();

    LinkPtr& link(const std::string& name);
    JointPtr& joint(const std::string& name);

    void printResults();

    std::string file_name_;
    std::string robot_name_;
    RobotPtr robot_;

    TiXmlDocument doc_;
    TiXmlElement* root_;
    TiXmlElement* robot_elem_;

    Eigen::Matrix4d base_frame_; // homogeneous transformation

    // Link
    std::map<std::string, LinkPtr> links_;

    // Joint
    std::map<std::string, JointPtr> joints_;
    std::map<std::string, bool> use_parent_model_frame_;

    Eigen::MatrixXd last_row_;
  };

}

#endif /* __AHL_ROBOT_CONTROLLER_XML_PARSER_HPP */
