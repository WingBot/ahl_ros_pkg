#ifndef __AHL_ROBOT_SDF_PARSER_HPP
#define __AHL_ROBOT_SDF_PARSER_HPP

#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>
#include <tinyxml.h>
#include "ahl_robot/parser/parser.hpp"
#include "ahl_robot/parser/mnp_generator.hpp"
#include "ahl_robot/tf/tf_link.hpp"
#include "ahl_robot/tf/tf_joint.hpp"

namespace ahl_robot
{

  class SDFParser : public Parser
  {
  public:
    SDFParser();
    virtual void load(const std::string& path, RobotPtr& robot);

  private:
    void loadRobot();
    void loadPose();
    void loadLink();
    void loadJoint();

    void loadPose(TiXmlElement* pose_elem);
    void loadLink(TiXmlElement* link_elem);
    void loadLinkPose(TiXmlElement* pose_elem, Eigen::Vector3d& xyz, Eigen::Vector3d& rpy);
    void loadLinkInertia(
      TiXmlElement* inertia_elem, double& m, Eigen::Matrix3d& I,
      Eigen::Vector3d& xyz_com, Eigen::Vector3d& rpy_com);
    void loadInertia(TiXmlElement* inertia_elem, Eigen::Matrix3d& I);
    void loadJoint(TiXmlElement* joint_elem);
    void loadAxis(
      TiXmlElement* axis_elem, Eigen::Vector3d& axis,
      double& q_min, double& q_max, double& dq_max, double& tau_max);
    void loadLimit(
      TiXmlElement* limit_elem,
      double& q_min, double& q_max, double& dq_max, double& tau_max);
    void loadXYZ(TiXmlElement* xyz_elem, Eigen::Vector3d& axis);

    void textToMatrix4d(const std::string& text, Eigen::Matrix4d& mat);
    void textToVectors(const std::string& text, Eigen::Vector3d& xyz, Eigen::Vector3d& rpy);

    void modifyTransformationMatrix();

    std::string path_;
     RobotPtr robot_;

    TiXmlDocument doc_;
    TiXmlElement* root_;
    TiXmlElement* robot_elem_;

    Eigen::Matrix4d T_pose_;

    TfLinkMap link_;
    TfJointMap joint_;

    MnpGeneratorPtr mnp_generator_;
  };

}

#endif /* __AHL_ROBOT_SDF_PARSER_HPP */
