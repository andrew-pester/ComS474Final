import json
import os
import sys, argparse
import matplotlib.pyplot as plt
from planning import (
    ObstacleCollisionChecker3D,
    rrt,
    StraightEdgeCreator,
    EuclideanDistanceComputator,
    EmptyCollisionChecker,
    ObstacleCollisionChecker,
)
from obstacle import WorldBoundary3D, construct_circular_obstacles, WorldBoundary2D
from draw_cspace import draw
import numpy as np



def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="Run forward search")
    parser.add_argument(
        "desc",
        metavar="problem_description_path",
        type=str,
        help="path to the problem description file containing the obstacles, the width and length of each link, and the distance between two points of attachment",
    )

    args = parser.parse_args(sys.argv[1:])

    print("Problem description: ", args.desc)

    return args

    
def parse_desc(desc):
    """Parse problem description json file to get the problem description"""
    with open(desc) as desc:
        data = json.load(desc)

    qI = tuple(data["qI"])
    qG = tuple(data["qG"])
    return (qI, qG)    


def main_rrt(
    cspace, qI, qG, edge_creator, distance_computator, collision_checker, obs_boundaries
):
    """Task 1 (Exploring the C-space using RRT) and Task 2 (Solve the planning problem using RRT)"""
    fig = plt.figure()
    ax = plt.axes(projection ='3d')

    # Task 1a: Neglect obstacles and goal

    # Task 1b: Include obstacles, neglect goal
    title3 = "RRT planning"
    (G3, root3, goal3) = rrt(
        cspace=cspace,
        qI=qI,
        qG=qG,
        edge_creator=edge_creator,
        distance_computator=distance_computator,
        collision_checker=collision_checker,
    )
    path = []
    if goal3 is not None:
        path = G3.get_path(root3, goal3)
    draw(ax, cspace, obs_boundaries, qI, qG, G3, path, title3)
    plt.ylabel('Y')
    plt.xlabel('X')
    plt.show(block = True)


if __name__ == "__main__":
    cspace = [(-5, 5), (-5, 5), (-5, 5)]
    obstacles = construct_circular_obstacles(0.02)
    obs_boundaries = [obstacle.get_boundaries() for obstacle in obstacles]

    # We don't really need to explicitly need to check the world boundary
    # because the world is convex and we're connecting points by a straight line.
    # So as long as the two points are within the world, the line between them
    # are also within the world.
    # I'm just including this for completeness.
    world_boundary = WorldBoundary3D(cspace[0], cspace[1], cspace[2])
    obstacles.append(world_boundary)

    edge_creator = StraightEdgeCreator(0.1)
    collision_checker = ObstacleCollisionChecker3D(obstacles)
    distance_computator = EuclideanDistanceComputator()

    args = parse_args()
    (qI, qG) = parse_desc(args.desc)
    main_rrt(
            cspace,
            qI,
            qG,
            edge_creator,
            distance_computator,
            collision_checker,
            obs_boundaries,
        )