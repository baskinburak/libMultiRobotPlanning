# distutils: language = c++
from libcpp cimport bool
from libcpp.vector cimport vector
from libcpp.pair cimport pair

cdef extern from "grid_planning.hpp":
    pair[bool, vector[vector[pair[int, int]]]] cbs_plan(
        const vector[vector[bool]] &obstacles,
        const vector[pair[int, int]] &init_pos,
        const vector[pair[int, int]] &goals) except +

cdef extern from "grid_planning.hpp":
    pair[bool, vector[vector[pair[int, int]]]] ecbs_plan(
        const vector[vector[bool]] &obstacles,
        const vector[pair[int, int]] &init_pos,
        const vector[pair[int, int]] &goals,
        float optimality_factor) except +


def ecbs_path(world, init_pos, goals, optimality_factor):
    """Finds a path invoking C++ implementation

    Uses recursive CBS to explore a 4 connected grid

    world - matrix specifying obstacles, 1 for obstacle, 0 for free
    init_pos  - [[x, y], ...] specifying start position for each robot
    goals     - [[x, y], ...] specifying goal position for each robot

    returns:
    [[[x1, y1], ...], [[x2, y2], ...], ...] path in the joint
    configuration space

    raises:
    NoSolutionError if problem has no solution
    OutOfTimeError if the planner ran out of time
    """

    import resource
    resource.setrlimit(resource.RLIMIT_AS, (2**33,2**33)) # 8Gb

    # convert to boolean.  For some reason coercion doesn't seem to
    # work properly
    cdef vector[vector[bool]] obs
    cdef vector[bool] temp
    for row in world:
        temp = vector[bool]()
        for i in row:
            temp.push_back(i == 1)
        obs.push_back(temp)
    return ecbs_plan(obs, init_pos, goals, optimality_factor)


def cbs_path(world, init_pos, goals):
    """Finds a path invoking C++ implementation

    Uses recursive CBS to explore a 4 connected grid

    world - matrix specifying obstacles, 1 for obstacle, 0 for free
    init_pos  - [[x, y], ...] specifying start position for each robot
    goals     - [[x, y], ...] specifying goal position for each robot

    returns:
    [[[x1, y1], ...], [[x2, y2], ...], ...] path in the joint
    configuration space

    raises:
    NoSolutionError if problem has no solution
    OutOfTimeError if the planner ran out of time
    """

    import resource
    resource.setrlimit(resource.RLIMIT_AS, (2**33,2**33)) # 8Gb

    # convert to boolean.  For some reason coercion doesn't seem to
    # work properly
    cdef vector[vector[bool]] obs
    cdef vector[bool] temp
    for row in world:
        temp = vector[bool]()
        for i in row:
            temp.push_back(i == 1)
        obs.push_back(temp)
    return cbs_plan(obs, init_pos, goals)
