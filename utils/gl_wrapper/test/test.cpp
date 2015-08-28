#include <ros/ros.h>
#include <gl_wrapper/gl_wrapper.hpp>
#include <gl_wrapper/object/x_hand.hpp>
#include <gl_wrapper/exception/exceptions.hpp>

using namespace gl_wrapper;

void display()
{
  try
  {
    Render::start();
    Render::LIGHT->on();

    static double t = 0.0;
    t += 0.03333;

    GLfloat amb[4] = {0.25f, 0.20f, 0.07f, 1.f};
    GLfloat dif[4] = {0.75f, 0.61f, 0.23f, 1.f};
    GLfloat spe[4] = {0.63f, 0.56f, 0.37f, 1.f};
    GLfloat shine = 51.2f;

    glTranslated(0.0, 1.0, 0.0);
    glRotated(90.0, 1.0, 0.0, 0.0);
    Material::setMaterial(amb, dif, spe, shine);
    glutSolidTeapot(0.1);

    std::string path = "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/utils/gl_wrapper/blender/";
    static XObjectPtr lwr1 = XObjectPtr(new XObject(path + "lwr07.x"));
    static XObjectPtr lwr2 = XObjectPtr(new XObject(path + "lwr07.x"));
    static XObjectPtr lwr3 = XObjectPtr(new XObject(path + "lwr07.x"));
    static SimpleObjectPtr grid = SimpleObjectPtr(new Grid(20, 20, 0.5));

    lwr1->setEulerZYX(std::string("Link01"), M_PI * std::sin(2.0 * M_PI * 0.1 * t), 0.0, 0.0);
    lwr1->setEulerZYX(std::string("Link02"), 0.0, 0.35 * M_PI * std::sin(2.0 * M_PI * 0.3 * t), 0.0);
    lwr1->setEulerZYX(std::string("Link03"), 0.5 * M_PI * std::sin(2.0 * M_PI * 0.2 * t), 0.0, 0.0);
    lwr1->setEulerZYX(std::string("Link04"), 0.0, 0.5 * M_PI * std::sin(2.0 * M_PI * 0.2 * t), 0.0);
    lwr1->setEulerZYX(std::string("Link05"), 0.4 * M_PI * std::sin(2.0 * M_PI * 0.3 * t), 0.0, 0.0);
    lwr1->setEulerZYX(std::string("Link06"), 0.0, 0.5 * M_PI * std::sin(2.0 * M_PI * 0.3 * t), 0.0);
    lwr1->setEulerZYX(std::string("EndEffector"), M_PI * std::sin(2.0 * M_PI * 0.6 * t), 0.0, 0.0);

    lwr2->setEulerZYX(std::string("Link01"), M_PI * std::sin(2.0 * M_PI * 0.1 * t), 0.0, 0.0);
    lwr2->setEulerZYX(std::string("Link02"), 0.0, 0.35 * M_PI * std::sin(2.0 * M_PI * 0.3 * t), 0.0);
    lwr2->setEulerZYX(std::string("Link03"), 0.5 * M_PI * std::sin(2.0 * M_PI * 0.2 * t), 0.0, 0.0);
    lwr2->setEulerZYX(std::string("Link04"), 0.0, 0.5 * M_PI * std::sin(2.0 * M_PI * 0.2 * t), 0.0);
    lwr2->setEulerZYX(std::string("Link05"), 0.4 * M_PI * std::sin(2.0 * M_PI * 0.3 * t), 0.0, 0.0);
    lwr2->setEulerZYX(std::string("Link06"), 0.0, 0.5 * M_PI * std::sin(2.0 * M_PI * 0.3 * t), 0.0);
    lwr2->setEulerZYX(std::string("EndEffector"), M_PI * std::sin(2.0 * M_PI * 0.6 * t), 0.0, 0.0);


    lwr3->setEulerZYX(std::string("Link01"), M_PI * std::sin(2.0 * M_PI * 0.1 * t), 0.0, 0.0);
    lwr3->setEulerZYX(std::string("Link02"), 0.0, 0.35 * M_PI * std::sin(2.0 * M_PI * 0.3 * t), 0.0);
    lwr3->setEulerZYX(std::string("Link03"), 0.5 * M_PI * std::sin(2.0 * M_PI * 0.2 * t), 0.0, 0.0);
    lwr3->setEulerZYX(std::string("Link04"), 0.0, 0.5 * M_PI * std::sin(2.0 * M_PI * 0.2 * t), 0.0);
    lwr3->setEulerZYX(std::string("Link05"), 0.4 * M_PI * std::sin(2.0 * M_PI * 0.3 * t), 0.0, 0.0);
    lwr3->setEulerZYX(std::string("Link06"), 0.0, 0.5 * M_PI * std::sin(2.0 * M_PI * 0.3 * t), 0.0);
    lwr3->setEulerZYX(std::string("EndEffector"), M_PI * std::sin(2.0 * M_PI * 0.6 * t), 0.0, 0.0);

    glLoadIdentity();
    lwr1->display();

    glLoadIdentity();
    glTranslated(0.0, -0.6, 0.0);
    lwr2->display();

    glLoadIdentity();
    glTranslated(0.0, -1.2, 0.0);
    lwr3->display();

    glLoadIdentity();
    grid->setColor(0, 0, 255);
    grid->setPosition(0.0, 0.0, 0.0);
    grid->display();
/*
    gl_wrapper::RightHandPtr right_hand;
    right_hand = RightHandPtr(new RightHand("/home/daichi/work/catkin_ws/src/gl_wrapper/righthand.cfg"));
    right_hand->display();
*/
    Render::end();
  }
  catch(Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
  }
  catch(FatalException& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
  }
  catch(std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
  }
  catch(...)
  {
    ROS_ERROR_STREAM("display : Unknown exception was thrown.");
    exit(1);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gl_wrapper_test");
  ros::NodeHandle nh;

  try
  {
    RenderPtr render;
    render = RenderPtr(new Render(argc, argv));
    render->start(display);
  }
  catch(FatalException& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
  }
  catch(Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
  }
  catch(...)
  {
    ROS_ERROR_STREAM("Unknown exception was thrown.");
    exit(1);
  }

  return 0;
}
