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

    for(int i = 0; i < 3; ++i)
    {
      amb_[i] = 0.8f;
      dif_[i] = 0.8f;
      spe_[i] = 0.8f;
    }
    amb_[3] = 1.0f;
    dif_[3] = 1.0f;
    spe_[3] = 1.0f;

    shi_ = 128.0f;
  }

  void draw()
  {
    glClear(GL_COLOR_BUFFER_BIT);

    glLineWidth(2);

    const double norm_square_max = 50.0 * 50.0;
    const double bias = 0.1;
    const double ratio_max = 0.9;

    for(unsigned int i = 0; i < x_parent_.size(); ++i)
    {
      double scale = 0.025;
      Render::LIGHT->off();

      double norm_square = x_parent_[i] * x_parent_[i]
                         + y_parent_[i] * y_parent_[i]
                         + z_parent_[i] * z_parent_[i];

      double ratio = norm_square / norm_square_max;
      if(ratio > ratio_max)
        ratio = ratio_max;
      double r, g, b;
      this->HSVToRGB(ratio - bias, 1.0, 1.0, r, g, b);

      glColor3d(r, g, b);

      glLoadIdentity();
      glBegin(GL_LINE_LOOP);
      glVertex3d(scale * x_parent_[i], scale * y_parent_[i], scale * z_parent_[i]);

      norm_square = x_child_[i] * x_child_[i]
                    + y_child_[i] * y_child_[i]
                    + z_child_[i] * z_child_[i];

      ratio = norm_square / norm_square_max;
      if(ratio > ratio_max)
        ratio = ratio_max;

      this->HSVToRGB(ratio - bias, 1.0, 1.0, r, g, b);
      glColor3d(r, g, b);
      glVertex3d(scale * x_child_[i], scale * y_child_[i], scale * z_child_[i]);
      glEnd();

      Render::LIGHT->on();
      glLoadIdentity();
      glTranslated(scale * x_child_[i], scale * y_child_[i], scale * z_child_[i]);

      dif_[0] = r;
      dif_[1] = g;
      dif_[2] = b;

      spe_[0] = r;
      spe_[1] = g;
      spe_[2] = b;

      Material material(amb_, dif_, spe_, shi_);
      material.apply();

      glutSolidSphere(0.008, 8, 8);
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

  void HSVToRGB(double h, double s, double v, double& r, double& g, double& b)
  {
    h = h - std::floor(h);
    double f = h * 6.0;
    int i = static_cast<int>(f);
    double fr = f - i;
    double m = v * (1.0 - s);
    double n = v * (1.0 - s * fr);
    double p = v * (1.0 - s * (1.0 - fr));

    switch(i)
    {
    case 0:
      r = v;
      g = p;
      b = m;
      break;
    case 1:
      r = n;
      g = v;
      b = m;
      break;
    case 2:
      r = m;
      g = v;
      b = p;
      break;
    case 3:
      r = m;
      g = n;
      b = v;
      break;
    case 4:
      r = p;
      g = m;
      b = v;
      break;
    default:
      r = v;
      g = m;
      b = n;
      break;
    }
  }

  ros::Subscriber sub_;
  std::vector<double> x_parent_;
  std::vector<double> y_parent_;
  std::vector<double> z_parent_;
  std::vector<double> x_child_;
  std::vector<double> y_child_;
  std::vector<double> z_child_;

  GLfloat amb_[4];
  GLfloat dif_[4];
  GLfloat spe_[4];
  GLfloat shi_;
};

void display()
{
  Render::start();

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
