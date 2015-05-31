#ifndef __AHL_ROBOT_TRANSFORMATION_HPP
#define __AHL_ROBOT_TRANSFORMATION_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace ahl_robot
{

  class Transformation
  {
  public:
    Transformation();
    virtual ~Transformation() {}
    virtual const Eigen::Matrix4d& T(double q) = 0;
    virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T) = 0;
    virtual Eigen::Vector3d& axis()
    {
      return axis_;
    }
  protected:
    Eigen::Matrix4d T_;
    Eigen::Vector3d axis_;
  };
  typedef boost::shared_ptr<Transformation> TransformationPtr;

  class Fixed : public Transformation
  {
  public:
    virtual const Eigen::Matrix4d& T(double q)
    {
      return T_;
    }
    virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T)
    {
    }
  };

  class RevoluteX : public Transformation
  {
  public:
    RevoluteX();
    virtual const Eigen::Matrix4d& T(double q);
    virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T);
  private:
    Eigen::Matrix3d R_;
  };

  class RevoluteY : public Transformation
  {
  public:
    RevoluteY();
    virtual const Eigen::Matrix4d& T(double q);
    virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T);
  private:
    Eigen::Matrix3d R_;
  };

  class RevoluteZ : public Transformation
  {
  public:
    RevoluteZ();
    virtual const Eigen::Matrix4d& T(double q);
    virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T);
  private:
    Eigen::Matrix3d R_;
  };

  class PrismaticX : public Transformation
  {
  public:
    PrismaticX();
    virtual const Eigen::Matrix4d& T(double q);
    virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T);
  };

  class PrismaticY : public Transformation
  {
  public:
    PrismaticY();
    virtual const Eigen::Matrix4d& T(double q);
    virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T);
  };

  class PrismaticZ : public Transformation
  {
  public:
    PrismaticZ();
    virtual const Eigen::Matrix4d& T(double q);
    virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T);
  };

}

#endif /* __AHL_ROBOT_TRANSFORMATION_HPP */
