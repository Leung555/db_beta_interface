# Make sure to have the add-on "ZMQ remote API"
# running in CoppeliaSim
#
# All CoppeliaSim commands will run in blocking mode (block
# until a reply from CoppeliaSim is received). For a non-
# blocking example, see simpleTest-nonBlocking.py
#
# if there is an error on dependencies
# python3 -m pip install coppeliasim-zmqremoteapi-client

import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
from shapely import plotting

class GaitPlanner:
    def __init__(self, angles, leg_radius=1.0, max_legs_lifted=1, lifting_period=20):
        self.timesteps = 0
        self.gait_plan = []
        self.lifted_legs_timer = {} # Store lifted legs with their remaining time
        self.leg_stance_state = {}
        self.leg_stance_state_commands = {}
        self.this_cycle_lifted = []
        self.leg_radius = leg_radius
        self.max_legs_lifted = max_legs_lifted
        self.lifting_period = lifting_period
        self.fast_lifting_period = 4
        self.stance_transition_time = 40
        # timesteps = 0
        for i in range(len(angles)):
            self.lifted_legs_timer[i] = 0
            self.leg_stance_state[i] = True
            self.leg_stance_state_commands[i] = True

    def calculate_support_polygon(self, angles, leg_radius=1):
        """
        Calculate the support polygon given leg angles and the COG of the polygon.
        Args:
            angles (list): Array of angles (0-360 degrees) for each leg.
            leg_radius (float): Distance of each leg's ground contact point from the center of the robot.

        Returns:
            polygon (Polygon): Support polygon formed by the leg ground contact points.
            cog (tuple): Center of gravity (centroid of the support polygon).
        """
        # Convert angles to Cartesian coordinates
        coords = [(leg_radius * np.cos(np.radians(angle)), 
                leg_radius * np.sin(np.radians(angle))) for angle in angles]
        polygon = Polygon(coords)
    
        # Calculate the centroid of the polygon
        cog = Point([0,0])  # Centroid of the polygon
        return polygon, cog

    # Function to calculate stability index and whether COG is outside the support polygon
    def calculate_stability(self,
                            polygon, 
                            cog, 
                            test_leg_index, 
                            lifted_legs_index, 
                            visualize=False):
        # print('polygon: ', polygon)
        # Get the list of coordinates from the polygon's exterior
        lifted_leg = list(lifted_legs_index)
        lifted_leg.append(test_leg_index)
        coords = list(polygon.exterior.coords)

        # Remove the point at the specified index
        del coords[-1]
        for index in sorted(lifted_leg, reverse=True):
            del coords[index]
        
        if len(coords) == 2:
            for i in range(len(coords)):
                coords.insert(-1, coords[-1])
        elif len(coords) == 1:
            for i in range(3):
                coords.insert(-1, coords[-1])
        
        # Create a new polygon with the updated list of coordinates
        new_polygon = Polygon(coords)    
        # print('n_polygon: ', new_polygon)
        # new_cog = new_polygon.centroid
        # cog_point = Point(new_cog.x, new_cog.y)

        if visualize:
            plt.figure(figsize=(2,2))
            plt.clf()  # Clear the current figure
            x,y = new_polygon.exterior.xy
            plotting.plot_polygon(new_polygon, alpha=0.2)
            plt.scatter(cog.x, cog.y, c='orange', s=100, marker='*')
            plt.xlim([-1.1, 1.1])
            plt.ylim([-1.1, 1.1])
            plt.show(block=False)  # Set block=False to avoid pausing the program

        # Calculate distance of COG to the nearest edge
        distance_to_edge = new_polygon.exterior.distance(cog)

        # Check if COG is outside the polygon
        is_outside = not new_polygon.contains(cog)

        # Return negative distance if COG is outside
        if is_outside:
            distance_to_edge = -distance_to_edge


        return distance_to_edge, is_outside
    
    def lift_leg(self, leg_index, lifting_period):
        self.lifted_legs_timer[leg_index] = lifting_period  # Start a timer for lifting this leg
        self.leg_stance_state[leg_index] = False
        self.leg_stance_state_commands[leg_index] = False
        self.this_cycle_lifted.append(leg_index)
        if len(self.this_cycle_lifted) == len(self.leg_stance_state):
            self.this_cycle_lifted = []


    def gait_planner(self, angles, stability_threshold = 0.2):
        """
        Plan the gait for the hexapod robot.
        
        Args:
            angles (list): Array of angles of each leg (0-360 degrees).
            leg_radius (float): Distance of each leg's ground contact point from the robot center.
            max_legs_lifted (int): Maximum number of legs that can be lifted at the same time.
            max_timesteps (int): Maximum timesteps for the gait planner.
            lifting_period (int): The duration each leg stays lifted before returning to stance.
        
        Returns:
            gait_plan (list): Sequence of leg lifting actions.
            lifted_legs_timer (dict): Remaining lifted legs with their timers.
        """
        polygon, cog = self.calculate_support_polygon(angles)

        # Get the index of legs which are in stance phase
        # leg_stance_state[0] = False
        index_stance_leg = [key for key, value in self.leg_stance_state.items() if value]
        index_lifted_leg = [key for key, value in self.leg_stance_state.items() if not value]
        
        # Update timers for lifted legs
        for leg in list(self.lifted_legs_timer.keys()):
            self.lifted_legs_timer[leg] -= 1
            timer = self.lifted_legs_timer[leg]
            if timer <= 0:
                # Leg has completed its lifting period, put it back to stance
                self.lifted_legs_timer[leg] -= 1  # Mark it as ready for stance
                self.leg_stance_state[leg] = True
                self.leg_stance_state_commands[leg] = True
            elif timer < self.stance_transition_time:
                self.leg_stance_state[leg] = True
                self.leg_stance_state_commands[leg] = True
        
        # Stability prediction for each leg
        stability_scores = []
        for i in range(len(angles)):
            if i not in index_lifted_leg and i not in self.this_cycle_lifted:
                stability_index, is_cog_outside = self.calculate_stability(polygon, cog, i, index_lifted_leg, visualize=False)
                stability_scores.append((i, stability_index, is_cog_outside))
                # print('i: ', i)
        
        # Sort stability scores in descending order (higher stability first)
        stability_scores.sort(reverse=True, key=lambda x: x[1])
        # print('stability_scores: ', stability_scores)

        # Check if the number of lifted legs is below the limit
        if len(index_lifted_leg) < self.max_legs_lifted:
            for leg_index, stability_index, is_cog_outside in stability_scores:
                if stability_index > stability_threshold:
                    self.lift_leg(leg_index, self.lifting_period)
                    break  # Only lift one leg at a time
                elif stability_index > 0.0:
                    self.lift_leg(leg_index, self.fast_lifting_period)
                    break  # Only lift one leg at a time
                else:
                    self.lift_leg(leg_index, self.fast_lifting_period-4)
                    break  # Only lift one leg at a time


        elif len(index_lifted_leg) == self.max_legs_lifted:
        # Wait for all legs to be lifted before proceeding
            print('Max leg lifting')
                        
        return self.leg_stance_state_commands, index_lifted_leg