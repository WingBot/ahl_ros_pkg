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

#ifndef __GL_WRAPPER_RENDER_RENDER_HPP
#define __GL_WRAPPER_RENDER_RENDER_HPP

#include <boost/shared_ptr.hpp>
#include <gl_wrapper/render/param.hpp>
#include <gl_wrapper/render/display.hpp>
#include <gl_wrapper/render/camera.hpp>
#include <gl_wrapper/render/light.hpp>
#include <gl_wrapper/interface/mouse.hpp>

namespace gl_wrapper
{
  class Render
  {
  public:
    static ParamPtr PARAM;
    static CameraPtr CAMERA;
    static LightPtr LIGHT;
    static DisplayPtr DISPLAY;
    static MousePtr MOUSE;

    Render(int& argc, char** argv);
    void start(void (*display)());
    static void start();
    static void end();

  private:
    Render() {}
    Render(const Render& render) {}
    Render& operator=(const Render& render) {}

    //DisplayPtr display_;

    //ParamPtr param_;
    //WindowPtr window_;

    //ViewPtr view_;
  };

  void reshape(int w, int h);
  void timer(int value);
  void keyboard(unsigned char key, int x, int y);
  void specialKey(int key, int x, int y);
  void specialKeyUp(int key, int x, int y);
  void mouseClick(int button, int state, int x, int y);
  void mouseDrag(int x, int y);
  void mousePassiveMotion(int x, int y);
  void mouseWheel(int wheel_id, int direction, int x, int y);

  typedef boost::shared_ptr<Render> RenderPtr;
}

#endif /* __GL_WRAPPER_RENDER_RENDER_HPP */
 
