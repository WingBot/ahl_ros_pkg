#include <vector>
#include <omp.h>
#include <ros/ros.h>
#include <gl_wrapper/gl_wrapper.hpp>
#include "ahl_rrt/Edge3D.h"

using namespace gl_wrapper;

class Scene
{
public:
  Scene()
  {
    ros::NodeHandle nh;
    sub_ = nh.subscribe("ahl_rrt/edge", 10000, &Scene::callback, this);
  }

  void draw()
  {
    glClear(GL_COLOR_BUFFER_BIT);

    glLineWidth(2);

    for(unsigned int i = 0; i < x_parent_.size(); ++i)
    {
      double scale = 0.025;

      glBegin(GL_LINE_LOOP);
      glVertex3d(scale * x_parent_[i], scale * y_parent_[i], scale * z_parent_[i]);
      glVertex3d(scale * x_child_[i], scale * y_child_[i], scale * z_child_[i]);
      glEnd();
    }
  }

private:
  void callback(const ahl_rrt::Edge3D::ConstPtr& msg)
  {
    x_parent_.push_back(msg->x_parent);
    y_parent_.push_back(msg->y_parent);
    z_parent_.push_back(msg->z_parent);
    x_child_.push_back(msg->x_child);
    y_child_.push_back(msg->y_child);
    z_child_.push_back(msg->z_child);
  }

  ros::Subscriber sub_;
  std::vector<double> x_parent_;
  std::vector<double> y_parent_;
  std::vector<double> z_parent_;
  std::vector<double> x_child_;
  std::vector<double> y_child_;
  std::vector<double> z_child_;
};

void display()
{
  Render::start();
  Render::LIGHT->on();

  static Scene scene;
  scene.draw();

  Render::end();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualization");
  ros::NodeHandle nh;

  omp_set_num_threads(2);
  int thread_id;
  #pragma omp parallel private(thread_id)
  {
    thread_id = omp_get_thread_num();
    if(thread_id == 1)
    {
      ros::spin();
    }
    else
    {
      RenderPtr render;
      render = RenderPtr(new Render(argc, argv));
      render->start(display);
    }
  }

  return 0;
}
