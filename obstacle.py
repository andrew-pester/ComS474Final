import math
from geometry import is_inside_circle
import numpy as np


class Obstacle:
    def contain(self, s):
        return False
        
class CubeObstacle(Obstacle):
    """A class representing a circular obstacle"""

    def __init__(self, xlim, ylim, zlim):
        self.xmin = xlim[0]
        self.xmax = xlim[1]
        self.ymin = ylim[0]
        self.ymax = ylim[1]
        self.zmin = zlim[0]
        self.zmax = zlim[1]


    def get_boundaries(self):
        """Return the list of coordinates (x,y) of the boundary of the obstacle"""
        return [
            (
                self.xmin,
                self.ymin,
                self.zmin,
            ),
            (
                self.xmax,
                self.ymin,
                self.zmin,
            ),
            (
                self.xmax,
                self.ymax,
                self.zmin,
            ),
            (
                self.xmin,
                self.ymax,
                self.zmin,
            ),
            (
                self.xmin,
                self.ymin,
                self.zmin,
            ),


            (
                self.xmin,
                self.ymin,
                self.zmax,
            ),
            (
                self.xmax,
                self.ymin,
                self.zmax,
            ),
            (
                self.xmax,
                self.ymin,
                self.zmin,
            ),
            (
                self.xmax,
                self.ymin,
                self.zmax,
            ),
            (
                self.xmax,
                self.ymax,
                self.zmax,
            ),
            (
                self.xmax,
                self.ymax,
                self.zmin,
            ),
            (
                self.xmax,
                self.ymax,
                self.zmax,
            ),
            (
                self.xmin,
                self.ymax,
                self.zmax,
            ),
            (
                self.xmin,
                self.ymax,
                self.zmin,
            ),
            (
                self.xmin,
                self.ymax,
                self.zmax,
            ),
            (
                self.xmin,
                self.ymin,
                self.zmax,
            ),
        ]

    def contain(self, s):
        """Return whether a point s is inside this obstacle"""
        return s[0] >= self.xmin and s[0] <= self.xmax and s[1] >= self.ymin and s[1] <= self.ymax and s[2] >= self.zmin and s[2] <= self.zmax

class CylinderObstacle(Obstacle):
    """A class representing a Cylinder obstacle"""

    def __init__(self, center, radius, theta_lim, height):
        self.center = center
        self.radius = radius
        self.height = height
        self.theta_min = theta_lim[0]
        self.theta_max = theta_lim[1]


    def get_boundaries(self):
        """Return the list of coordinates (x,y,z) of the boundary of the obstacle"""
        num_theta = 100
        theta_inc = (self.theta_max - self.theta_min) / num_theta
        theta_range = [self.theta_min + theta_inc * i for i in range(num_theta + 1)]
        ret = []
        count = 0
        for theta in theta_range:
            if count%10 == 0:
                ret.append((self.radius * math.cos(theta) + self.center[0], self.radius * math.sin(theta) + self.center[1], self.center[2] - self.height/2, ))
            ret.append((self.radius * math.cos(theta) + self.center[0], self.radius * math.sin(theta) + self.center[1], self.center[2] + self.height/2, ))
            count = count + 1
            
        for theta in theta_range:
            ret.append((self.radius * math.cos(theta) + self.center[0], self.radius * math.sin(theta) + self.center[1], self.center[2]-self.height/2, ))

        return ret

    def contain(self, s):
        """Return whether a point s is inside this obstacle"""
        return is_inside_circle(self.center, self.radius, s) and s[2] <= self.center[2] + self.height/2 and s[2] >= self.center[2] - self.height/2

        


class WorldBoundary2D(Obstacle):
    """A class representing the world"""

    def __init__(self, xlim, ylim):
        self.xmin = xlim[0]
        self.xmax = xlim[1]
        self.ymin = ylim[0]
        self.ymax = ylim[1]

    def contain(self, s):
        """Return True iff the given point is not within the boundary (i.e., the point is
        "in collision" with an obstacle.).
        """
        return (
            s[0] < self.xmin or s[0] > self.xmax or s[1] < self.ymin or s[1] > self.ymax
        )
class WorldBoundary3D(Obstacle):
    def __init__(self, xlim, ylim, zlim):
        self.xmin = xlim[0]
        self.xmax = xlim[1]
        self.ymin = ylim[0]
        self.ymax = ylim[1]
        self.zmin = zlim[0]
        self.zmax = zlim[1]

    def contain(self, s):
        """Return True iff the given point is not within the boundary (i.e., the point is
        "in collision" with an obstacle.).
        """
        return (
            s[0] < self.xmin or s[0] > self.xmax or s[1] < self.ymin or s[1] > self.ymax or s[2] < self.zmin or s[2] > self.zmax
        )



def construct_circular_obstacles(dt):
    r = 1 - dt  # the radius of the circle
    c = [(0, -4, 1), (0, 4, 0)]  # the center of each circle
    t = [(0, 2*math.pi), (0, 2*math.pi)]  # range of theta of each circle
    obstacles = []
    cube =  [(0, 1.5), (1.5, 2), (-2, 0)]
    cube2 =  [(0, 1.5), (-2, 1), (-2, 0)]
    obstacles.append(CubeObstacle(cube[0],cube[1], cube[2]))
    obstacles.append(CubeObstacle(cube2[0],cube2[1], cube2[2]))
    # obstacles.append(CylinderObstacle(c, r, t, 2))
    for i in range(len(c)):
        obstacles.append(CylinderObstacle(c[i], r, t[i], 2))
    return obstacles


