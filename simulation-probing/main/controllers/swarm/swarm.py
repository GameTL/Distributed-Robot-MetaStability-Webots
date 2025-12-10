import cv2
import ast
import json
import asyncio
import numpy as np
import builtins
import inspect
import yaml
import os
import time
from rich.pretty import pprint
from controller import Robot, Camera, Motor, Display, Supervisor
from swarmtools import FormationMaster
from swarmtools import ObjectDetector
from swarmtools import Communicator
from swarmtools import Driver

PRIORITY_LIST = ["TurtleBot3Burger_1", "TurtleBot3Burger_2", "TurtleBot3Burger_3"]

cylinder_position = {"x": 0.75, "y": -0.25, "theta": 0.0}

ROBOT = Robot()
ROBOT_NAME = ROBOT.getName()
# Override built-in print globally to show calling function name
# This will help identify which function is printing None
_original_print = builtins.print

def print(*args, **kwargs):
    """Override print() to show calling function name"""
    try:
        frame = inspect.currentframe().f_back
        func_name = frame.f_code.co_name
        class_name = None
        prefix = f"({ROBOT_NAME})"
        # prefix = f""
        if 'self' in frame.f_locals:
            class_name = frame.f_locals['self'].__class__.__name__
            prefix += f"[{class_name}.{func_name}]"
        else:
            prefix += f"[{func_name}]"
        
        # Filter out None values to avoid printing them
        filtered_args = [arg for arg in args if arg is not None]
        if filtered_args or args:  # Only print if there are non-None args or if explicitly passed
            _original_print(f"{prefix}:", *filtered_args if filtered_args else args, **kwargs)
    except:
        # Fallback to original print if inspection fails
        _original_print(*args, **kwargs)

# Override in builtins so it affects all modules
builtins.print = print


""" 
NOTES:
Uncomment line 252 in driver.py to enable live plotting of the robot's path + waypoints, disable live plotting will allow for FASTER SIMULATION
"""

import csv

def read_test_counter():
    """Read the current test run counter without incrementing."""
    counter_file = "test_counter.yaml"
    
    # Read current counter value
    if os.path.exists(counter_file):
        with open(counter_file, 'r') as f:
            data = yaml.safe_load(f)
            current_count = data.get('test_runs', 0)
    else:
        current_count = 0
    
    print(f"[TEST_COUNTER] Test run #{current_count}")
    return current_count

class DataCollector:
    def __init__(self, robot_name, test_run_number, simulation_name):
        self.transitions = []  # List of transition records
        self.robot_name = robot_name
        self.test_run_number = test_run_number
        self.simulation_name = simulation_name
        
        # Create directory structure: /Users/game/Github/Swarm-RAL/runs/{sim_name}/{test_counter}/
        self.runs_dir = f"/Users/game/Github/Swarm-RAL/runs/{simulation_name}/{test_run_number}"
        os.makedirs(self.runs_dir, exist_ok=True)
        
        # CSV file path
        self.csv_file = f"{self.runs_dir}/{robot_name}.csv"
        
        # Initialize CSV file with headers
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['robot_name', 'test_counter', 'transition_from', 'transition_to', 'simulation_time', 'send_delay', 'note'])
    
    def record_transition(self, transition_from: str, transition_to: str, simulation_time: float, send_delay: float = 0.0, note: str = ""):
        """Record a state transition with simulation time, send_delay, and optional note."""
        self.transitions.append({
            'robot_name': self.robot_name,
            'test_counter': self.test_run_number,
            'transition_from': transition_from,
            'transition_to': transition_to,
            'simulation_time': simulation_time,
            'send_delay': send_delay,
            'note': note
        })
        
        # Append to CSV immediately for real-time logging
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                self.robot_name,
                self.test_run_number,
                transition_from,
                transition_to,
                f"{simulation_time:.3f}",
                f"{send_delay:.3f}",
                note
            ])
    
    def save_data(self):
        """Save any final data (CSV is already written incrementally)."""
        print(f"[DataCollector] Saved {len(self.transitions)} transitions to {self.csv_file}")

class SwarmMember:
    def __init__(self, test_run_number, mode=0, verbose=False):
        # Instantiate the robot & big objects
        
        
        self.robot = ROBOT
        
        # Get simulation name from world file
        world_path = self.robot.getWorldPath()
        # Extract world name without path and extension (e.g., "/path/to/world.wbt" -> "world")
        simulation_name = os.path.splitext(os.path.basename(world_path))[0] if world_path else "unknown_world"
        print(f"[INIT] Running simulation: {simulation_name}, Test run: {test_run_number}")
        
        # For Data Collection
        self.data_collector = DataCollector(ROBOT_NAME, test_run_number, simulation_name)
        self.timestep = int(self.robot.getBasicTimeStep())
        self.verbose = verbose
        self.object_detector = ObjectDetector(self.robot)
        # send_delay introduces simulated communication lag (seconds)
        # self.communicator = Communicator(self.robot, send_delay=10.0)
        self.communicator = Communicator(self.robot)
        self.driver = Driver(self.robot)
        self.tick = 0
        # Computer vision
        self.detected_flag = False

        # Retrieve robot parameters
        self.robot_name = ROBOT_NAME
        self.mode = mode
        self.full_ack_set = {x for x in PRIORITY_LIST if x != self.robot_name}
        self.ack_set= set()
        self.priority_queue = PRIORITY_LIST
        self.communicator.robot_entries[self.robot_name] = (
            self.driver.robot_position["x"],
            self.driver.robot_position["y"],
            self.driver.robot_position["theta"],
            self.driver.robot_position["x"],
            self.driver.robot_position["y"],
            self.driver.robot_position["theta"],
        )
        self.buf_message = None

        # Detection parameters
        self.object_coordinates = ()
        self.task_master = ""
        self.path = None
        self.previous_state = None
        self.state = "random_walk"
        self.reassign_flag = False
        self.state_note = ""  # For recording contextual information
        self.yielded_to_robot = None  # Track which robot we've already yielded to via res_ack
        
        # Record initial state
        self.data_collector.record_transition(
            transition_from="INIT",
            transition_to=self.state,
            simulation_time=self.robot.getTime(),
            send_delay=self.communicator.send_delay,
            note="Initial state"
        )
    
    def change_state(self, new_state, note=""):
        """Change state and record the transition with simulation time."""
        self.previous_state = self.state
        self.state = new_state
        simulation_time = self.robot.getTime()
        
        # Record the transition
        self.data_collector.record_transition(
            transition_from=self.previous_state if self.previous_state else "INIT",
            transition_to=self.state,
            simulation_time=simulation_time,
            send_delay=self.communicator.send_delay,
            note=note
        )
        
        print(f"State change: {self.previous_state} -> {self.state} @ {simulation_time:.3f}s | delay={self.communicator.send_delay:.3f}s | {note}")

    def print_position(self):
        print(f"[helper] Robot X position: {self.driver.robot_position['x']:6.3f}    Robot Y position: {self.driver.robot_position['y']:6.3f}    Robot Theta position: {self.driver.robot_position['theta']:6.3f}")

    def path_finding(self):
        print(f"[path_finding] calculating...")
        paths_json = self.formation_object()

        self.communicator.broadcast_message("[path_following]", robot_to=-1, content= paths_json)

        if self.path == None:
            self.path = self.communicator.path
        if self.verbose:
            print(f"[{self.state}]{self.robot_name}: {self.path}")  # big print
        
        paths = ast.literal_eval(paths_json)
        if self.robot_name in paths.keys():
            self.path = paths.get(self.robot_name, "")
        self.communicator.path = self.path # Sync with communicator
    
    def formation_object(self):
        if self.detected_flag:
            coords = self.communicator.robot_entries.copy()
            for robot_name in coords:
                coords[robot_name] = list(
                    map(lambda x: round(x, 2), coords[robot_name])
                )
            # print(f"[path_finding]({self.robot_name}): current coords={coords}")

            #! Obstacles are in a list format e.g. [(-1, -1.4), (0.6, 0.3), (0.1, 1.67)]; should be input from map so leaving it empty for now
            self.formationer = FormationMaster(
                robot=self.robot,
                current_coords=coords,
                object_coords=(cylinder_position["x"], cylinder_position["y"]),
                obstacles=[],
                debug_level=0,
            )
            self.formationer.calculate_target_coords()
            self.formationer.plan_paths()

            paths = json.dumps(self.formationer.paths)
            self.path = json.loads(paths)[self.robot_name]

            return paths
        else:
            return None
    
    #  ------------------------------------------------------------------------
    #  Handle States
    #  ------------------------------------------------------------------------
    #1 ------------------------------------------------------------------------
    def handle_random_walk_state(self):
        # Check Object Detection
        self.buf_message = self.communicator.listen_to_message()
        # Updating Postion
        self.communicator.send_position(
            robot_position={
                "x": self.driver.robot_position["x"],
                "y": self.driver.robot_position["y"],
                "theta": self.driver.robot_position["theta"],
            }
        )
        
        # Object detect asking to Host
        if self.object_detector.detect() and not self.detected_flag:
            print(f"[object_detected_req_ack] found cylinder @ {cylinder_position}")
            # Start consensus and broadcast detection
            self.communicator.object_detected(cylinder_position)
            self.driver.stop()
            # self.communicator.broadcast_message("[object_detected_req_ack]", robot_to=-1, content=None)
            self.change_state("waiting_replies", note="Detected object, requesting acknowledgments")
            return
        
        # if not detected anything then read the status.title if a Host is asking of ack
        if self.buf_message.title == "object_detected_req_ack": # another robot detects an object
            print(f"[object_detected_req_ack] received request from {self.buf_message}")
            self.communicator.broadcast_message("[object_detected_res_ack]", robot_to=self.buf_message.robot_from, content=None)
            self.yielded_to_robot = self.buf_message.robot_from
            self.change_state("waiting_plans", note=f"Received detection request from {self.buf_message.robot_from}, sent acknowledgment")
            return

        # Driving Options
        self.driver.move_along_polynomial() # option for driving 1

        # # Updating Postion
        # self.communicator.send_position(
        #     robot_position={
        #         "x": self.driver.robot_position["x"],
        #         "y": self.driver.robot_position["y"],
        #         "theta": self.driver.robot_position["theta"],
        #     }
        # )
    
    #2 ------------------------------------------------------------------------
    def handle_waiting_plans_state(self):
        self.buf_message = self.communicator.listen_to_message()

        if self.buf_message.title == "path_following":
            self.change_state("path_following", note=f"Received path from {self.buf_message.robot_from}")
        # if not detect anything then read the status.title
        elif self.buf_message.title == "object_detected_req_ack": # another robot detects an object
            self.change_state("consensus", note=f"Conflict: received detection request from {self.buf_message.robot_from}")

        # Listens to detected
        self.driver.stop()
    #3 ------------------------------------------------------------------------
    def handle_waiting_replies_state(self):
        self.buf_message = self.communicator.listen_to_message()
        if self.buf_message.title == "object_detected_res_ack":
            self.ack_set.add(self.buf_message.robot_from)
            if self.ack_set == self.full_ack_set:
                print(f"[object_detected_res_ack] all acks received: {self.ack_set}")
                self.detected_flag = True
                self.change_state("path_finding", note=f"Received all acknowledgments from: {', '.join(sorted(self.ack_set))}")
                
                return
        if self.buf_message.title == "object_detected_req_ack": # another robot detects an object
            self.change_state("consensus", note=f"Conflict: received detection request from {self.buf_message.robot_from}")
        
    #4 ------------------------------------------------------------------------
    def handle_consensus_state(self):
        #! Don't read new messages until consensus is reached
        index_of_this_robot = PRIORITY_LIST.index(self.robot_name)
        index_of_req_robot = PRIORITY_LIST.index(self.buf_message.robot_from)
        
        if self.yielded_to_robot is not None: # I'm not a taskmaster
            index_of_yielded_robot = PRIORITY_LIST.index(self.yielded_to_robot)
            # Only yield to the new robot if it has higher priority than the one we already yielded to
            if index_of_req_robot < index_of_yielded_robot:
                # New requester has higher priority than our current promise - switch allegiance
                old_yielded = self.yielded_to_robot
                self.communicator.broadcast_message("[object_detected_res_ack]", robot_to=self.buf_message.robot_from, content=None)
                self.yielded_to_robot = self.buf_message.robot_from
                self.change_state("waiting_plans", note=f"Switching allegiance to higher priority {self.buf_message.robot_from} (was yielded to {old_yielded})")
            else:
                # Already yielded to a higher or equal priority robot, stay loyal
                self.change_state("waiting_plans", note=f"Ignoring {self.buf_message.robot_from}, already yielded to higher priority {self.yielded_to_robot}")

        else: # I am taskmaster, don't yield to no one.
            if index_of_this_robot < index_of_req_robot: # this robot is higher priority
                # back to waiting for replies - we're the taskmaster now
                self.yielded_to_robot = None  # Clear any previous yield since we're taking over
                self.change_state("waiting_replies", note=f"Higher priority than {self.buf_message.robot_from}, continuing as taskmaster")
            else: # respond with ack to the higher priority robot
                # Check if we've already yielded to another robot
                if self.yielded_to_robot is not None:
                    index_of_yielded_robot = PRIORITY_LIST.index(self.yielded_to_robot)
                    # Only yield to the new robot if it has higher priority than the one we already yielded to
                    if index_of_req_robot < index_of_yielded_robot:
                        # New requester has higher priority than our current promise - switch allegiance
                        old_yielded = self.yielded_to_robot
                        self.communicator.broadcast_message("[object_detected_res_ack]", robot_to=self.buf_message.robot_from, content=None)
                        self.yielded_to_robot = self.buf_message.robot_from
                        self.change_state("waiting_plans", note=f"Switching allegiance to higher priority {self.buf_message.robot_from} (was yielded to {old_yielded})")
                    else:
                        # Already yielded to a higher or equal priority robot, stay loyal
                        self.change_state("waiting_plans", note=f"Ignoring {self.buf_message.robot_from}, already yielded to higher priority {self.yielded_to_robot}")
                else:
                    # First time yielding - send ack and remember who we yielded to
                    self.communicator.broadcast_message("[object_detected_res_ack]", robot_to=self.buf_message.robot_from, content=None)
                    self.yielded_to_robot = self.buf_message.robot_from
                    self.change_state("waiting_plans", note=f"Lower priority than {self.buf_message.robot_from}, yielding and sending acknowledgment")
    
    #5 ------------------------------------------------------------------------
    def handle_path_finding_state(self):
        # Used only by the TaskMaster
        self.path_finding()
        self.change_state("path_following", note="Completed path planning, broadcasting paths to all robots")
        # self.state = "waiting_plans"
    
    #6 ------------------------------------------------------------------------
    def handle_path_following_state(self):
        if not self.communicator.path:
            print(f"[path_following] Missing path; returning to waiting_plans")
            self.change_state("waiting_plans", note="Missing path, waiting for path update")
            return

        self.path = self.communicator.path
        list_waypoint = list(self.path.values())
        
        # Sampling
        # self.driver.sorted_waypoints = list(self.path.values())[:30]
        self.driver.sorted_waypoints = list()
        self.driver.sorted_waypoints.append(list_waypoint[-1])
        print(f"[path_printing_reduced]({self.robot_name}) {self.driver.sorted_waypoints}")
        if self.path != "":
            print(self.driver.sorted_waypoints)
            print(f'{self.driver.sorted_waypoints=}')
            print(f'{self.driver.robot_position=}')
            self.driver.pid_path_follow()
            # quit()
            self.change_state("arrived", note=f"Completed path following to target position")
    #7 ------------------------------------------------------------------------
    def handle_arrived_state(self):
        self.driver.stop()
        self.ack_set.clear()
        self.yielded_to_robot = None  # Reset for next detection cycle
        # quit()
    #  ------------------------------------------------------------------------
    #  Main Loop
    #  ------------------------------------------------------------------------
    def main_loop(self): # main loop
        while self.robot.step(self.timestep) != -1:

            # if self.state == "consensus" or self.communicator.awaiting_paths:
            #     self.communicator.send_coordinates(
            #         robot_position={
            #             "x": self.driver.robot_position["x"],
            #             "y": self.driver.robot_position["y"],
            #             "theta": self.driver.robot_position["theta"],
            #         }
            #     )
            # Check for incoming messages

            # print(self.robot.getName(),f'{status.title=}')
            if self.state == "random_walk": #1
                self.handle_random_walk_state()

            elif self.state == "waiting_plans": #2
                self.handle_waiting_plans_state()

            elif self.state == "waiting_replies": #3
                self.handle_waiting_replies_state()

            elif self.state == "consensus": #4
                self.handle_consensus_state()

            elif self.state == "path_finding": #5
                self.handle_path_finding_state()

            elif self.state == "path_following": #6
                # self.handle_path_following_state()
                time.sleep(1)
                self.change_state("arrived", note="Completed path following to target position")

            elif self.state == "arrived": #7
                self.handle_arrived_state()

            self.communicator.robot_entries[self.robot_name] = (
                self.driver.robot_position["x"],
                self.driver.robot_position["y"],
                self.driver.robot_position["theta"],
            )
            self.communicator.flush_pending() #* Actually send the pending msgs
        


def main():
    import threading
    import traceback
    class bcolors:
        RED_FAIL       = '\033[91m'
        GRAY_OK        = '\033[90m'
        GREEN_OK       = '\033[92m'
        YELLOW_WARNING = '\033[93m'
        BLUE_OK        = '\033[94m'
        MAGENTA_OK     = '\033[95m'
        CYAN_OK        = '\033[96m'
        ENDC           = '\033[0m'
        BOLD           = '\033[1m'
        ITALIC         = '\033[3m'
        UNDERLINE      = '\033[4m'
    
    # Read test run counter (don't increment)
    test_run_number = read_test_counter()
        
    member = SwarmMember(test_run_number)
    localisation_service = threading.Thread(target=member.driver.run_odometry_service)

    if member.driver.check_gps_valid_and_init():
        localisation_service.start()
        try:
            member.main_loop()
        except:
            tb_str = traceback.format_exc()
            print(bcolors.RED_FAIL + tb_str + bcolors.ENDC)
            member.driver.alive = False
            member.data_collector.save_data()
            quit()
        member.driver.alive = False
        member.data_collector.save_data()


main()
