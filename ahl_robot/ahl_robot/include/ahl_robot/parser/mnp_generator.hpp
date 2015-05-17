#ifndef __AHL_ROBOT_MNP_GENERATOR_HPP
#define __AHL_ROBOT_MNP_GENERATOR_HPP

#include <set>
#include "ahl_robot/robot/robot.hpp"

namespace ahl_robot
{

  class MnpGenerator
  {
  public:
    explicit MnpGenerator(const RobotPtr& robot);
    void generate();

  private:
    void checkValidityOfName();
    void checkConnection();
    void generate(const std::string& ee_name, ManipulatorPtr& mnp);
    void modifyManipulator(const std::vector<LinkPtr>& link, ManipulatorPtr& mnp);
    void setLinkAttachedToManipulator(ManipulatorPtr& mnp);
    void storeBranch(const ManipulatorPtr& mnp, const std::string& root, const TfLinkPtr& tf_link, std::map<std::string, std::string>& branch_map);
    void setLinkAttachedToBase();

    void rename();

    RobotPtr robot_;
    std::map<std::string, std::string> connection_; // first : name of the link, second : connected to which link ?
    std::set<std::string> mnp_link_set_;
    std::set<std::string> branch_base_;
  };

  typedef boost::shared_ptr<MnpGenerator> MnpGeneratorPtr;
}

#endif /* __AHL_ROBOT_MNP_GENERATOR_HPP */
