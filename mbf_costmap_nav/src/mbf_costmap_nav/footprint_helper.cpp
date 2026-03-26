/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: TKruse
 *********************************************************************/

#include "mbf_costmap_nav/footprint_helper.h"

namespace mbf_costmap_nav
{

void FootprintHelper::getLineCells(int x0, int x1, int y0, int y1, std::vector<Cell>& pts) {
  //Bresenham Ray-Tracing
  int deltax = abs(x1 - x0);        // The difference between the x's
  int deltay = abs(y1 - y0);        // The difference between the y's
  int x = x0;                       // Start x off at the first pixel
  int y = y0;                       // Start y off at the first pixel

  int xinc1, xinc2, yinc1, yinc2;
  int den, num, numadd, numpixels;

  Cell pt;

  if (x1 >= x0)                 // The x-values are increasing
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          // The x-values are decreasing
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (y1 >= y0)                 // The y-values are increasing
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          // The y-values are decreasing
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay)         // There is at least one x-value for every y-value
  {
    xinc1 = 0;                  // Don't change the x when numerator >= denominator
    yinc2 = 0;                  // Don't change the y for every iteration
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         // There are more x-values than y-values
  }
  else                          // There is at least one y-value for every x-value
  {
    xinc2 = 0;                  // Don't change the x for every iteration
    yinc1 = 0;                  // Don't change the y when numerator >= denominator
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         // There are more y-values than x-values
  }

  for (int curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    pt.x = x;      //Draw the current pixel
    pt.y = y;
    pts.push_back(pt);

    num += numadd;              // Increase the numerator by the top of the fraction
    if (num >= den)             // Check if numerator >= denominator
    {
      num -= den;               // Calculate the new numerator value
      x += xinc1;               // Change the x as appropriate
      y += yinc1;               // Change the y as appropriate
    }
    x += xinc2;                 // Change the x as appropriate
    y += yinc2;                 // Change the y as appropriate
  }
}

void FootprintHelper::supercover(int x0, int x1, int y0, int y1, std::vector<Cell>& pts)
{
  {
    auto add_point = [&pts](int y, int x) {
      Cell pt;
      pt.x = x;
      pt.y = y;
      pts.push_back(pt);
    };

    int i;               // loop counter
    int ystep, xstep;    // the step on y and x axis
    int error;           // the error accumulated during the increment
    int errorprev;       // *vision the previous value of the error variable
    int y = y0, x = x0;  // the line points
    int ddy, ddx;        // compulsory variables: the double values of dy and dx
    int dx = x1 - x0;
    int dy = y1 - y0;
    add_point(y0, x0);  // first point

    // NB the last point can't be here, because of its previous point (which has to be verified)
    if (dy < 0)
    {
      ystep = -1;
      dy = -dy;
    }
    else
    {
      ystep = 1;
    }
    if (dx < 0)
    {
      xstep = -1;
      dx = -dx;
    }
    else
    {
      xstep = 1;
    }

    ddy = 2 * dy;  // work with double values for full precision
    ddx = 2 * dx;
    if (ddx >= ddy)
    {  // first octant (0 <= slope <= 1)
      // compulsory initialization (even for errorprev, needed when dx==dy)
      errorprev = error = dx;  // start in the middle of the square
      for (i = 0; i < dx; i++)
      {  // do not use the first point (already done)
        x += xstep;
        error += ddy;
        if (error > ddx)
        {  // increment y if AFTER the middle ( > )
          y += ystep;
          error -= ddx;
          // three cases (octant == right->right-top for directions below):
          if (error + errorprev < ddx)  // bottom square also
            add_point(y - ystep, x);
          else if (error + errorprev > ddx)  // left square also
            add_point(y, x - xstep);
          else
          {  // corner: bottom and left squares also
            add_point(y - ystep, x);
            add_point(y, x - xstep);
          }
        }
        add_point(y, x);
        errorprev = error;
      }
    }
    else
    {  // the same as above
      errorprev = error = dy;
      for (i = 0; i < dy; i++)
      {
        y += ystep;
        error += ddx;
        if (error > ddy)
        {
          x += xstep;
          error -= ddy;
          if (error + errorprev < ddy)
            add_point(y, x - xstep);
          else if (error + errorprev > ddy)
            add_point(y - ystep, x);
          else
          {
            add_point(y, x - xstep);
            add_point(y - ystep, x);
          }
        }
        add_point(y, x);
        errorprev = error;
      }
    }
  }
}

void FootprintHelper::getFillCells(std::vector<Cell>& footprint){
  //quick bubble sort to sort pts by x
  Cell swap, pt;
  unsigned int i = 0;
  while (i < footprint.size() - 1) {
    if (footprint[i].x > footprint[i + 1].x) {
      swap = footprint[i];
      footprint[i] = footprint[i + 1];
      footprint[i + 1] = swap;
      if(i > 0) {
        --i;
      }
    } else {
      ++i;
    }
  }

  i = 0;
  Cell min_pt;
  Cell max_pt;
  unsigned int min_x = footprint[0].x;
  unsigned int max_x = footprint[footprint.size() -1].x;
  //walk through each column and mark cells inside the footprint
  for (unsigned int x = min_x; x <= max_x; ++x) {
    if (i >= footprint.size() - 1) {
      break;
    }
    if (footprint[i].y < footprint[i + 1].y) {
      min_pt = footprint[i];
      max_pt = footprint[i + 1];
    } else {
      min_pt = footprint[i + 1];
      max_pt = footprint[i];
    }

    i += 2;
    while (i < footprint.size() && footprint[i].x == x) {
      if(footprint[i].y < min_pt.y) {
        min_pt = footprint[i];
      } else if(footprint[i].y > max_pt.y) {
        max_pt = footprint[i];
      }
      ++i;
    }

    //loop though cells in the column
    for (unsigned int y = min_pt.y; y < max_pt.y; ++y) {
      pt.x = x;
      pt.y = y;
      footprint.push_back(pt);
    }
  }
}

/**
 * @brief Helper function which clears and returns the input.
 */
static const std::vector<Cell>& clearAndReturn(std::vector<Cell>& _cells)
{
  _cells.clear();
  return _cells;
}

/**
 * get the cells of a footprint at a given position
 */
std::vector<Cell> FootprintHelper::getFootprintCells(double x, double y, double theta,
                                                     const std::vector<geometry_msgs::Point>& footprint_spec,
                                                     const costmap_2d::Costmap2D& costmap, bool fill)
{
  std::vector<Cell> footprint_cells;

  //if we have no footprint... do nothing
  if (footprint_spec.size() <= 1) {
    unsigned int mx, my;
    if (costmap.worldToMap(x, y, mx, my)) {
      Cell center;
      center.x = mx;
      center.y = my;
      footprint_cells.push_back(center);
    }
    return footprint_cells;
  }

  //pre-compute cos and sin values
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  double new_x, new_y;
  unsigned int x0, y0, x1, y1;
  unsigned int last_index = footprint_spec.size() - 1;

  for (unsigned int i = 0; i < last_index; ++i) {
    //find the cell coordinates of the first segment point
    new_x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    new_y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    if(!costmap.worldToMap(new_x, new_y, x0, y0)) {
      return clearAndReturn(footprint_cells);
    }

    //find the cell coordinates of the second segment point
    new_x = x + (footprint_spec[i + 1].x * cos_th - footprint_spec[i + 1].y * sin_th);
    new_y = y + (footprint_spec[i + 1].x * sin_th + footprint_spec[i + 1].y * cos_th);
    if (!costmap.worldToMap(new_x, new_y, x1, y1)) {
      return clearAndReturn(footprint_cells);
    }

    supercover(x0, x1, y0, y1, footprint_cells);
  }

  //we need to close the loop, so we also have to raytrace from the last pt to first pt
  new_x = x + (footprint_spec[last_index].x * cos_th - footprint_spec[last_index].y * sin_th);
  new_y = y + (footprint_spec[last_index].x * sin_th + footprint_spec[last_index].y * cos_th);
  if (!costmap.worldToMap(new_x, new_y, x0, y0)) {
    return clearAndReturn(footprint_cells);
  }
  new_x = x + (footprint_spec[0].x * cos_th - footprint_spec[0].y * sin_th);
  new_y = y + (footprint_spec[0].x * sin_th + footprint_spec[0].y * cos_th);
  if(!costmap.worldToMap(new_x, new_y, x1, y1)) {
    return clearAndReturn(footprint_cells);
  }

  supercover(x0, x1, y0, y1, footprint_cells);

  if(fill) {
    getFillCells(footprint_cells);
  }

  return footprint_cells;
}

} /* namespace mbf_costmap_nav */
