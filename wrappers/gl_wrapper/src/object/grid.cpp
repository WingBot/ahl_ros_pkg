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

#include <gl_wrapper/object/grid.hpp>

using namespace gl_wrapper;

Grid::Grid(unsigned int grid_num_x, unsigned int grid_num_y, double resolution)
  : grid_num_x_(grid_num_x), grid_num_y_(grid_num_y), resolution_(resolution)
{
}

void Grid::displayImpl()
{
  double start_x = 0.0;
  double start_y = 0.0;

  if(grid_num_x_ % 2 == 0)
  {
    start_x = -(grid_num_x_ / 2.0 * resolution_ - resolution_ * 0.5);
  }
  else
  {
    start_x = -(grid_num_x_ / 2.0 * resolution_);
  }

  if(grid_num_y_ % 2 == 0)
  {
    start_y = -(grid_num_y_ / 2.0 * resolution_ - resolution_ * 0.5);
  }
  else
  {
    start_y = -(grid_num_y_ / 2.0 * resolution_);
  }

  double length_x = resolution_ * (grid_num_x_ - 1);
  double length_y = resolution_ * (grid_num_y_ - 1);

  double x = start_x;
  double y = start_y;

  glDisable(GL_LIGHTING);
  for(unsigned int i = 0; i < grid_num_x_; ++i)
  {
    glBegin(GL_LINES);
    glVertex2d(x, y);
    glVertex2d(x, y + length_y);
    glEnd();

    x += resolution_;
  }

  x = start_x;
  for(unsigned int i = 0; i < grid_num_y_; ++i)
  {
    glBegin(GL_LINES);
    glVertex2d(x, y);
    glVertex2d(x + length_x, y);
    glEnd();

    y += resolution_;
  }
  glEnable(GL_LIGHTING);
}
