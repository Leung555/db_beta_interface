# Make sure to have the add-on "ZMQ remote API"
# running in CoppeliaSim
#
# All CoppeliaSim commands will run in blocking mode (block
# until a reply from CoppeliaSim is received). For a non-
# blocking example, see simpleTest-nonBlocking.py
#
# if there is an error on dependencies
# python3 -m pip install coppeliasim-zmqremoteapi-client

import math
import numpy as np

class CPG:
    """
    The CPG is based on SO2 CPG with MI modulation for diff frequency
    """
    def __init__(self, MI=0):
        # CPGs
        self.MI = MI
        self.w11, self.w22 = 1.4, 1.4
        self.w12 =  0.18 + MI
        self.w21 = -0.18 - MI
        self.o1 = 0.01
        self.o2 = 0.01
    
    def step(self):
        self.o1 = math.tanh(self.w11*self.o1 + self.w12*self.o2)
        self.o2 = math.tanh(self.w22*self.o2 + self.w21*self.o1)
        
    def get_CPG_output(self):
        return [self.o1, self.o2]
    
class MotorMapper:
    """
    A class responsible for mapping gait patterns to motor commands.
    Converts CPG outputs into joint position commands for each leg.
    """
    def __init__(self, num_legs, num_motors_per_leg):
        """
        Initializes the MotorMapper.

        Args:
            num_legs (int): Number of legs in the robot.
            num_motors_per_leg (int): Number of motors per leg.
        """
        self.num_legs = num_legs
        self.num_motors_per_leg = num_motors_per_leg
        self.weight_V_leg = [0] * num_motors_per_leg

    def set_weight_leg(self, weights = [0, 0, 0]):
        self.weight_V_leg = np.array(weights) # 0, 1, 1

    def map_to_motors(self, cpg_outputs, gait_pattern, joints_bias):
        """
        Maps CPG outputs and gait pattern to motor joint positions.

        Args:
            cpg_outputs (np.array): Array of CPG outputs for each leg.
            gait_pattern (list): List specifying which leg to lift (1 for lift, 0 for grounded).

        Returns:
            dict: Dictionary with motor indices and corresponding joint positions.
        """
        joints_bias = np.array(joints_bias)
        motor_commands = [0] * self.num_legs * self.num_motors_per_leg
        for leg_id in range(self.num_legs):
            motor_index = leg_id * self.num_motors_per_leg
            # print(gait_pattern[leg_id])

            # If the gait pattern specifies lifting this leg, adjust the CPG output
            if gait_pattern[leg_id] == False:
                command = self.weight_V_leg + joints_bias  # Full amplitude for lifted leg
            else:
                command = joints_bias  # Set to zero (or adjust for grounded position)
            # print(command)

            # Store the command in the dictionary
            motor_commands[motor_index:motor_index+self.num_motors_per_leg] = command
        # print(motor_commands)
        return motor_commands