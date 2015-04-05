#ifndef __AHL_ROBOT_CONTROLLER_XML_PARSER_HPP
#define __AHL_ROBOT_CONTROLLER_XML_PARSER_HPP

#include <string>
#include <map>
#include <Eigen/Dense>
#include <tinyxml.h>
#include <std_utils/str_utils.hpp>
#include "ahl_robot_controller/utils/math.hpp"
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
    void loadLinkPose(TiXmlElement* pose_elem,
                      Eigen::Vector3d& pos, Eigen::Vector3d& rpy);
    void loadInertial(TiXmlElement* inertial_elem, double& M, Eigen::Matrix3d& I,
                      Eigen::Vector3d& pos_com, Eigen::Vector3d& rpy_com);
    void loadInertia(TiXmlElement* inertia_elem, Eigen::Matrix3d& I);

    void loadJoint(TiXmlElement* joint_elem);
    void loadAxisParams(TiXmlElement* axis_elem, Eigen::Vector3d& axis,
                        double& q_min, double& q_max, double& dq_max, double& tau_max);
    void loadLimits(TiXmlElement* limit_elem,
                    double& q_min, double& q_max, double& dq_max, double& tau_max);
    void loadAxis(TiXmlElement* xyz_elem, Eigen::Vector3d& axis);

    void convertTextToMatrix4d(const std::string& text, Eigen::Matrix4d& mat);
    void convertTextToVectors(const std::string& text, Eigen::Vector3d& pos, Eigen::Vector3d& rpy);
    void convertRPYToMatrix3d(const std::vector<double>& rpy, Eigen::Matrix3d& mat);

    //void setupLinks();
    //void setupJoints();
    //void setupJoints(const JointPtr& joint);

    std::string file_name_;
    std::string robot_name_;
    RobotPtr robot_;

    TiXmlDocument doc_;
    TiXmlElement* root_;
    TiXmlElement* robot_elem_;

    Eigen::Matrix4d base_frame_; // homogeneous transformation

    // Link
    //std::map<std::string, LinkPtr> links_;

    // Joint
    //std::map<std::string, JointPtr> joints_;
    std::map<std::string, bool> use_parent_model_frame_;
  };

}

#endif /* __AHL_ROBOT_CONTROLLER_XML_PARSER_HPP */
