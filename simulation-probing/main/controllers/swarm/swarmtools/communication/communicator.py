""" 
In the state response, 
2 replies then  """

from controller import Robot, Camera, Motor, Display, Supervisor
import json
from rich.pretty import pprint
import time
import ast

from ..navigation.formation import FormationMaster
# Translator is available but optional for now
# from .translator import Translator

EMITTER_DEVICE_NAME = "emitter"
RECEIVER_DEVICE_NAME = "receiver"
MESSAGE_INTERVAL = 2000 # ms
PRIORITY_LIST = ["TurtleBot3Burger_1", "TurtleBot3Burger_2", "TurtleBot3Burger_3"]

class Message:
    def __init__(self, title: str= "", robot_from: str = "", robot_to: str = "", message_id: int = 0, content = None):
        self.title = title
        self.robot_from = robot_from
        self.robot_to = robot_to
        self.message_id = message_id
        self.content = content
class Communicator:
    def __init__(self, robot: Robot, mode=0, verbose=False, send_delay: float = 5):
        self.verbose = verbose
        # setting up
        self.robot : Robot = robot

        self.timestep = 64
        self.name = self.robot.getName()
        self.mode = mode
        self.robot_entries = {}
        
        self.emitter = self.robot.getDevice(EMITTER_DEVICE_NAME) # sending info using webots
        self.receiver = self.robot.getDevice(RECEIVER_DEVICE_NAME) # receiving info using webots
        self.receiver.enable(self.timestep)

        self.message_interval = MESSAGE_INTERVAL
        self.time_tracker = 0
        self.priority_queue = PRIORITY_LIST.copy()
        self.taskmaster_claims = []
        self.awaiting_paths = False
        self.send_delay = send_delay

        self.coords_dict = {}
        self.obstacle_coords = []
        self.object_coordinates = {}
        self.task_master = ""
        self.path = None
        self.message_id = 0
        self.msg_from = None
        # self.send_delay = 0.0
        if self.name[-1] == "1":
            self.send_delay = 0.1
        elif self.name[-1] == "2":
            self.send_delay = 0.1
        elif self.name[-1] == "3":
            self.send_delay = 0.1

        self._pending_messages = []  # messages waiting for delayed send
        self.current_request : Message = Message()
        self.previous_request : Message = Message()

    def listen_to_message(self) -> None | Message:
        """ 
        listen for ['[probe]', '[object_detected]', '[task]', '[task_conflict]', '[task_successful]']
        return: 
        """
        # Receive messages from other robots and print
        while self.receiver.getQueueLength() > 0:  
            # print(list(self.robot_entries.keys()))
            received_message = self.receiver.getString()
            if self.verbose: self.print_received_message(received_message)
            title, robot_from, robot_to, message_id, content = json.loads(received_message)
            if title != "[probe]":
                print(f'{received_message=}')
            self.receiver.nextPacket()
            
            # print(f"{self.robot.getName()} received a message from {robot_from}; message ID: {message_id}")
            if title == "[path_receiving]":
                return Message(title[1:-1:], robot_from, robot_to, message_id, content)

            elif title == "[probe]":
                self.robot_entries[robot_from] = content

            elif title == "[object_detected_req_ack]":
                self.previous_request = self.current_request
            
                self.current_request = Message(title[1:-1:], robot_from, robot_to, message_id, content)
                
                return Message(title[1:-1:], robot_from, robot_to, message_id, content)

            elif title == "[object_detected_res_ack]":
                return Message(title[1:-1:], robot_from, robot_to, message_id, content)

            elif title == "[coordinates]":
                # Collect coordinates for path planning
                self.coords_dict[robot_from] = content
                if self.name not in self.coords_dict and self.name in self.robot_entries:
                    self.coords_dict[self.name] = self.robot_entries[self.name]
                if self.awaiting_paths and len(self.coords_dict) >= len(self.priority_queue) and self.object_coordinates:
                    self.path_planning()
                    self.awaiting_paths = False

            elif title in ("[path_following]", "[path]"):
                paths = json.loads(content) if isinstance(content, str) else content
                if self.name in paths.keys():
                    self.path = paths.get(self.name, "")
                    return Message("path_following", robot_from, robot_to, message_id, content)
                else:
                    self.mode = 2
                    return Message("waiting_plans", robot_from, robot_to, message_id, content)

            elif title == "[task_successful]":
                if self.task_master == self.name:
                    return "path_finding"
                else:
                    return Message("waiting_plans", robot_from, robot_to, message_id, content)
                    
            else:
                if self.verbose:
                    print("x")
            
        return Message()
    
    def broadcast_message(self, title: str, robot_to, content):
        # Send the message
        message = json.dumps([title, self.name, robot_to, self.message_id, content])
        if self.send_delay > 0:
            send_at = self.robot.getTime() + self.send_delay
            self._pending_messages.append((send_at, message))
            if self.verbose:
                print(f"(broadcast_message delayed {self.send_delay}s) {message}")
        else:
            if self.verbose:
                print(f"(broadcast_message) {message}")
            self.emitter.send(message)
        self.message_id += 1

    def flush_pending(self):
        """Send any queued messages whose delay has elapsed."""
        if not self._pending_messages:
            return
        now = self.robot.getTime()
        ready = []
        still_pending = []
        for send_at, msg in self._pending_messages:
            if send_at <= now:
                ready.append(msg)
            else:
                still_pending.append((send_at, msg))
        self._pending_messages = still_pending
        for msg in ready:
            if self.verbose:
                print(f"(broadcast_message flush) {msg}")
            self.emitter.send(msg)

    def send_position(self, robot_position):
        # Broadcast the message
        self.broadcast_message("[probe]", robot_to=-1, content=(robot_position["x"], robot_position["y"], robot_position["theta"]))
        # Reset the timer
        self.time_tracker = 0

    def send_coordinates(self, robot_position):
        """Broadcast coordinates for consensus/path planning."""
        coords = [
            robot_position.get("x", 0.0),
            robot_position.get("y", 0.0),
            robot_position.get("theta", 0.0),
        ]
        self.coords_dict[self.name] = coords
        self.broadcast_message("[coordinates]", robot_to=-1, content=coords)

    def object_detected(self, coords):
        """Start consensus when this robot detects an object."""
        self.object_coordinates = coords
        # self.consensus(self.name) #* Mehul method
        self.broadcast_message("[object_detected_req_ack]", robot_to=-1, content=coords)

    def consensus(self, new_claimant):
        """Pick a task master using the priority queue with simple voting."""
        self.taskmaster_claims.append(new_claimant)
        unique_claims = set(self.taskmaster_claims)

        if len(unique_claims) == 1:
            taskmaster = list(unique_claims)[0]
        else:
            taskmaster = None
            for robot in self.priority_queue:
                if robot in unique_claims:
                    taskmaster = robot
                    break
            if taskmaster is None:
                taskmaster = list(unique_claims)[0]

        self.task_master = taskmaster

        if self.task_master in self.priority_queue:
            self.priority_queue.remove(self.task_master)
            self.priority_queue.append(self.task_master)

        self.awaiting_paths = self.task_master == self.name
        self.taskmaster_claims.clear()

        if self.verbose:
            print(f"[consensus] task_master={self.task_master}")

    def path_planning(self):
        """Compute paths as the task master and broadcast them."""
        if not self.object_coordinates or not self.coords_dict:
            return

        object_xy = (
            self.object_coordinates.get("x", 0.0),
            self.object_coordinates.get("y", 0.0),
        ) if isinstance(self.object_coordinates, dict) else tuple(self.object_coordinates)

        formationer = FormationMaster(
            robot=self.robot,
            current_coords=self.coords_dict,
            object_coords=object_xy,
            obstacles=self.obstacle_coords,
            debug_level=0,
        )
        formationer.calculate_target_coords()
        formationer.plan_paths()

        paths = formationer.paths
        payload = json.dumps(paths)
        self.broadcast_message("[path]", robot_to=-1, content=payload)
        if self.name in paths:
            self.path = paths.get(self.name, "")

    def print_received_message(self, msg):
        print(f"[helper] {msg}")
        