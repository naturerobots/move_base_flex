# Generic set of parameters required by any MBF-based navigation framework
# To use:
#
#  from mbf_abstract_nav import add_mbf_abstract_nav_params
#  gen = ParameterGenerator()
#  add_mbf_abstract_nav_params(gen)
#  ...
# WARN: parameters added here must be copied on the specific MBF implementation reconfigure callback, e.g. in:
#       https://github.com/magazino/move_base_flex/blob/master/mbf_costmap_nav/src/mbf_costmap_nav/costmap_navigation_server.cpp#L130
# Also, you need to touch https://github.com/magazino/move_base_flex/blob/master/mbf_costmap_nav/cfg/MoveBaseFlex.cfg
# when recompiling to ensure configuration code is regenerated with the new parameters

# need this only for dataype declarations
from dynamic_reconfigure.parameter_generator_catkin import str_t, double_t, int_t, bool_t


def add_mbf_abstract_nav_params(gen):
    # gen.add("planner_frequency", double_t, 0,
    #         "The rate in Hz at which to run the planning loop", 0, 0, 100)
    # gen.add("planner_patience", double_t, 0,
    #         "How long the planner will wait in seconds in an attempt to find a valid plan before giving up", 5.0, 0, 100)
    # gen.add("planner_max_retries", int_t, 0,
    #         "How many times we will recall the planner in an attempt to find a valid plan before giving up", -1, -1, 1000)

    # gen.add("controller_frequency", double_t, 0,
    #         "The rate in Hz at which to run the control loop and send velocity commands to the base", 20, 0, 100)
    # gen.add("controller_patience", double_t, 0,
    #         "How long the controller will wait in seconds without receiving a valid control before giving up", 5.0, 0, 100)
    # gen.add("controller_max_retries", int_t, 0,
    #         "How many times we will recall the controller in an attempt to find a valid command before giving up", -1, -1, 1000)

    gen.add("recovery_enabled", bool_t, 0,
            "Whether or not to enable the move_base_flex recovery behaviors to attempt to clear out space", True)
    # gen.add("recovery_patience", double_t, 0,
    #         "How much time we allow recovery behaviors to complete before canceling (or stopping if cancel fails) them", 15.0, 0, 100)

    # gen.add("oscillation_timeout", double_t, 0,
    #         "How long in seconds to allow for oscillation before executing recovery behaviors", 0.0, 0, 60)
    # gen.add("oscillation_distance", double_t, 0,
    #         "How far in meters the robot must move to be considered not to be oscillating", 0.5, 0, 10)
