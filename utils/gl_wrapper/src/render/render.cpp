/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Daichi Yoshikawa
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Daichi Yoshikawa nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Daichi Yoshikawa
 *
 *********************************************************************/

#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <gl_wrapper/render/render.hpp>
#include <ros/ros.h>

using namespace gl_wrapper;

ParamPtr   Render::PARAM;
CameraPtr  Render::CAMERA;
LightPtr   Render::LIGHT;
DisplayPtr Render::DISPLAY;
MousePtr   Render::MOUSE;

Render::Render(int& argc, char** argv)
{
  glutInit(&argc, argv);

  Render::PARAM = ParamPtr(new Param);
  Render::DISPLAY = DisplayPtr(
    new Display(
      Render::PARAM->window_name, Render::PARAM->window_h, Render::PARAM->window_w, Render::PARAM->color));
  Render::CAMERA = CameraPtr(
    new Camera(
      true, Render::PARAM->fovy, Render::PARAM->z_near, Render::PARAM->z_far,
      Render::PARAM->camera_pos, Render::PARAM->camera_center, Render::PARAM->camera_up,
      Render::PARAM->zoom_rate, Render::PARAM->translate_rate, Render::PARAM->rotate_rate));
  Render::LIGHT = LightPtr(
    new Light(
      Render::PARAM->light_pos[0], Render::PARAM->ambient[0], Render::PARAM->diffuse[0], Render::PARAM->specular[0]));
  Render::MOUSE = MousePtr(new Mouse());
}

void Render::start(void (*display)())
{
  try
  {
    glutTimerFunc     (Render::PARAM->fps, gl_wrapper::timer, 0);
    glutReshapeFunc   (gl_wrapper::reshape);
    glutKeyboardFunc  (gl_wrapper::keyboard);
    glutSpecialFunc   (gl_wrapper::specialKey);
    glutMouseFunc     (gl_wrapper::mouseClick);
    glutMotionFunc    (gl_wrapper::mouseDrag);
    glutDisplayFunc   (display);
    glutMainLoop();
  }
  catch(std::bad_alloc& e)
  {
    std::cerr << e.what() << std::endl;
    exit(1);
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    exit(1);
  }
  catch(...)
  {
    std::cerr << "gl_wrapper::Render::start : Unknown exception was thrown." << std::endl;
    exit(1);
  }
}

void Render::start()
{
  boost::mutex::scoped_lock(Render::PARAM->mutex);

  Render::CAMERA->look(Render::PARAM->window_w, Render::PARAM->window_h);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
}

void Render::end()
{
  glPopMatrix();
  glutSwapBuffers();
}

namespace gl_wrapper
{
  void reshape(int w, int h)
  {
    try
    {
      boost::mutex::scoped_lock(Render::PARAM->mutex);
      Render::CAMERA->look(w, h);

      Render::PARAM->window_w = w;
      Render::PARAM->window_h = h;
    }
    catch(gl_wrapper::Exception& e)
    {
      std::cerr << e.what() << std::endl;
    }
    catch(gl_wrapper::FatalException& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(std::bad_alloc& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(...)
    {
      std::cerr << "gl_wrapper::reshape : Unknown exception was thrown." << std::endl;
      exit(1);
    }
  }

  void timer(int value)
  {
    try
    {
      glutPostRedisplay();
      boost::mutex::scoped_lock(Render::PARAM->mutex);
      glutTimerFunc(Render::PARAM->fps, gl_wrapper::timer, 0);
    }
    catch(std::bad_alloc& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(...)
    {
      std::cerr << "gl_wrapper::timer : Unknown exception was thrown." << std::endl;
      exit(1);
    }
  }

  void keyboard(unsigned char key, int x, int y)
  {
    try
    {
      boost::mutex::scoped_lock(Render::PARAM->mutex);
      const char ESC_KEY = 27;

      switch(key)
      {
      case ESC_KEY:
        exit(0);
        break;
      case 'i':
        Render::CAMERA->reset();
        break;
      default:
        break;
      };
    }
    catch(gl_wrapper::Exception& e)
    {
      std::cerr << e.what() << std::endl;
    }
    catch(gl_wrapper::FatalException& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(std::bad_alloc& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(...)
    {
      std::cerr << "gl_wrapper::keyboard : Unknown exception was thrown." << std::endl;
      exit(1);
    }
  }

  void specialKey(int key, int x, int y)
  {
    try
    {
      boost::mutex::scoped_lock(Render::PARAM->mutex);
      switch(key)
      {
      case GLUT_KEY_UP:
        Render::CAMERA->zoomin();
        break;
      case GLUT_KEY_DOWN:
        Render::CAMERA->zoomout();
        break;
      default:
        break;
      }
    }
    catch(gl_wrapper::Exception& e)
    {
      std::cerr << e.what() << std::endl;
    }
    catch(gl_wrapper::FatalException& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(std::bad_alloc& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(...)
    {
      std::cerr << "gl_wrapper::specialKey : Unknown exception was thrown." << std::endl;
      exit(1);
    }
  }

  void mouseClick(int button, int state, int x, int y)
  {
    try
    {
      boost::mutex::scoped_lock(Render::PARAM->mutex);
      Render::MOUSE->click(button, state, x, y);
    }
    catch(gl_wrapper::Exception& e)
    {
      std::cerr << e.what() << std::endl;
    }
    catch(gl_wrapper::FatalException& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(std::bad_alloc& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(...)
    {
      std::cerr << "gl_wrapper::mouseClick : Unknown exception was thrown." << std::endl;
      exit(1);
    }
  }

  void mouseDrag(int x, int y)
  {
    try
    {
      boost::mutex::scoped_lock(Render::PARAM->mutex);
      Render::MOUSE->drag(x, y);

      if(Render::MOUSE->getButton() == GLUT_LEFT_BUTTON)
      {
        Render::CAMERA->rotate(Render::MOUSE->getdX(), Render::MOUSE->getdY());
      }
      else if(Render::MOUSE->getButton() == GLUT_RIGHT_BUTTON)
      {
        Render::CAMERA->translate(Render::MOUSE->getdX(), Render::MOUSE->getdY());
      }
    }
    catch(gl_wrapper::Exception& e)
    {
      std::cerr << e.what() << std::endl;
    }
    catch(gl_wrapper::FatalException& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(std::bad_alloc& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(...)
    {
      std::cerr << "gl_wrapper::mouseDrag : Unknown exception was thrown." << std::endl;
      exit(1);
    }
  }

  void mousePassiveMotion(int x, int y)
  {
    try
    {
      boost::mutex::scoped_lock(Render::PARAM->mutex);
    }
    catch(gl_wrapper::Exception& e)
    {
      std::cerr << e.what() << std::endl;
    }
    catch(gl_wrapper::FatalException& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(std::bad_alloc& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(...)
    {
      std::cerr << "gl_wrapper::mousePassiveMotion : Unknown exception was thrown." << std::endl;
      exit(1);
    }
  }

  void mouseWheel(int wheel_id, int direction, int x, int y)
  {
    try
    {
      boost::mutex::scoped_lock(Render::PARAM->mutex);
      Render::MOUSE->scroll(wheel_id, direction, x, y);
    }
    catch(gl_wrapper::Exception& e)
    {
      std::cerr << e.what() << std::endl;
    }
    catch(gl_wrapper::FatalException& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(std::bad_alloc& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      exit(1);
    }
    catch(...)
    {
      std::cerr << "gl_wrapper::mouseWheel : Unknown exception was thrown." << std::endl;
      exit(1);
    }
  }
}
