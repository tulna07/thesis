import math
from enum import Enum

class RobotType(Enum):
    circle = 0
    rectangle = 1

class Picking_strategy(Enum):
    global_first = 0
    local_first = 1

class Ranking_type(Enum):
    Distance_Angle = 0
    RRTstar = 1


class Robot_base:
    def __init__(self, vision_range=20, robot_type=RobotType.circle, robot_radius=0.2):
        self.max_speed = 100.0                              # [m/s]
        self.min_speed = -100.0                             # [m/s]
        self.max_yaw_rate = 40.0 * math.pi / 180.0          # [rad/s]
        self.max_accel = 0.2                                # [m/ss]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0    # [rad/ss]
        self.v_resolution = 0.01                            # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0    # [rad/s]
        self.dt = 0.1                                       # [s] Time tick for motion prediction
        self.predict_time = 0.1                             # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001                  # constant to prevent robot stucked
        self.robot_type = robot_type
        self.vision_range = vision_range                    # the range of input vision
        
        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.radius = robot_radius                          # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.width = 0.1                              # [m] for collision check
        self.length = 0.1                             # [m] for collision check
    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value