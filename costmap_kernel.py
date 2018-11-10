"""
  Software License Agreement (BSD License)

  Copyright (c) 2018, laujinseoi
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above
     copyright notice, this list of conditions and the following
     disclaimer in the documentation and/or other materials provided
     with the distribution.
   * Neither the name of Willow Garage, Inc. nor the names of its
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

 Author: laujinseoi
"""

import cv2
import numpy as np

NO_INFORMATION = 255
LETHAL_OBSTACLE = 254
INSCRIBED_INFLATED_OBSTACLE = 253
FREE_SPACE = 0

RESOLUTION = 0.05
INSCRIBED_RADIUS = 0.5
COST_SCALING_FACTOR = 10
CELL_INFLATION_RADIUS = 40


# compute cost value
def computeCost(dist):
    cost = 0
    if dist == 0:
        cost = LETHAL_OBSTACLE
    elif (dist * RESOLUTION) <= INSCRIBED_RADIUS:
        cost = INSCRIBED_INFLATED_OBSTACLE
    else:
        euclidean_distance = dist * RESOLUTION
        factor = np.exp(-1.0 * COST_SCALING_FACTOR * (euclidean_distance - INSCRIBED_RADIUS))
        cost = int((INSCRIBED_INFLATED_OBSTACLE - 1) * factor)
    return cost


cell_inflation_radius = CELL_INFLATION_RADIUS
cached_cost = np.empty([cell_inflation_radius + 2, cell_inflation_radius + 2], dtype=int)
cached_distances = np.empty([cell_inflation_radius + 2, cell_inflation_radius+2], dtype=int)


# initial distance matrix
for i in xrange(cell_inflation_radius + 2):
    for j in xrange(cell_inflation_radius + 2):
        cached_distances[i,j] = np.hypot(i, j)


# initial a blank image
blank_image = np.zeros((cell_inflation_radius + 2, cell_inflation_radius+2, 3), np.uint8)


# create a inflation kernel
for i in xrange(cell_inflation_radius + 2):
    for j in xrange(cell_inflation_radius + 2):
        cached_cost[i, j] = computeCost(cached_distances[i,j])
        if cached_cost[i, j] == LETHAL_OBSTACLE:
            color = (LETHAL_OBSTACLE, 0, 0)
        elif cached_cost[i, j] == INSCRIBED_INFLATED_OBSTACLE:
            color = (0, INSCRIBED_INFLATED_OBSTACLE, 0)
        else:
            color = (255- cached_cost[i,j], 255-cached_cost[i,j], 255-cached_cost[i,j])
        blank_image[i, j] = color

# let's see what happen
cv2.namedWindow("Costmap")
cv2.imshow("Costmap", blank_image)
cv2.waitKey()