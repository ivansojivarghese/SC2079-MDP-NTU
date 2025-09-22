import sys
import ast
import time
from typing import List
import socket
import constants
import pickle
from misc.positioning import Position
from misc.direction import Direction
from grid.grid import Grid
from robot.robot import Robot
from simulation import Simulation
from connection_to_rpi.rpi_client import RPiClient
from grid.obstacle import Obstacle
from commands.go_straight_command import StraightCommand
from commands.scan_obstacle_command import ScanCommand
from pygame_app import AlgoMinimal
import threading
import ast
import re


class Main:
    def __init__(self):
        self.client = None
        self.commands = None
        self.count = 0

    def parse_obstacle_data(self, data) -> List[Obstacle]:
        obs = []
        print("data: ", data)
        # Mapping compass directions to angles
        direction_map = {'N': Direction.TOP, 'S': Direction.BOTTOM, 'E': Direction.RIGHT, 'W': Direction.LEFT}
        

        for obstacle_params in data:
            if len(obstacle_params) < 4:
                print(f"WARNING: Skipping invalid obstacle data: {obstacle_params}")
                continue
            
            # Convert direction from 'N'/'S'/'E'/'W' to angles
            dir = direction_map.get(obstacle_params[2], 90)  # Default to 90 (North) if invalid

            obs.append(Obstacle(Position(obstacle_params[0],
                                        obstacle_params[1],
                                        dir),  # Pass angle instead of string
                                obstacle_params[3]))
        
        print(f"Parsed obstacle data: {obs}")
        return obs

    def run_simulator(self):
        world1 = [
            [0, 100, Direction.RIGHT],
            [80, 10, Direction.LEFT],
            [10, 180, Direction.BOTTOM],
            [70, 90, Direction.TOP],
            [120, 90, Direction.BOTTOM],
            [180, 0, Direction.TOP],
            [180, 110, Direction.LEFT],
            [90, 170, Direction.BOTTOM]
        ]
        obstacles = []
        i = 0
        for x, y, direction in world1:
            position: Position = Position(x, y, direction)
            obstacle: Obstacle = Obstacle(position, i)
            i += 1
            obstacles.append(obstacle)
        grid = Grid(obstacles)
        bot = Robot(grid)
        sim = Simulation()
        sim.runSimulation(bot)
        # """
        # Fill in obstacle positions with respect to lower bottom left corner.
        # (x-coordinate, y-coordinate, Direction)
        # obstacles = [[15, 75, 0, 0]]
        # obs = parse_obstacle_data(obstacles)
        # """
        # obs = self.parse_obstacle_data([])
        # app = AlgoSimulator(obs)
        # app.init()
        # app.execute()

    def run_minimal(self, also_run_simulator):
        if self.client is None:
            print(f"Attempting to connect to {constants.RPI_HOST}:{constants.RPI_PORT}")
            self.client = RPiClient(constants.RPI_HOST, constants.RPI_PORT)

            #  Wait to connect to RPi.
            while True:
                try:
                    self.client.connect()
                    break
                except OSError:
                    pass
                except KeyboardInterrupt:
                    self.client.close()
                    sys.exit(1)
            print("Connected to RPi!\n")

        # Wait for message from RPi
        print("Waiting to receive data from RPi...")
        d = self.client.receive_message()  # Receive binary data

        # Deserialize using pickle instead of UTF-8 decoding
        try:
            d = pickle.loads(d)  # Unpickle instead of decoding UTF-8
            print("Decoded data from RPi:", d)

        except pickle.UnpicklingError:
            print("Error: Received data is not valid Pickle format.")
            return  # Exit function if data is invalid

        to_return = []

        if isinstance(d, str) and "[" in d and "]" in d:
            try:
                # Replace unquoted letters (N, S, E, W) with quoted versions
                d = re.sub(r'(?<=\[|\s|,)(N|S|E|W)(?=,|\s|\])', r'"\1"', d)

                # Convert string representation of list into an actual list
                to_return = ast.literal_eval(d)
                print("Parsed list from ALGO message:", to_return)
            except Exception as e:
                print(f"Error parsing ALGO list: {e}")
                return


        # If data is a string (e.g., "ALGO#OBSTACLE,4,130,70,NORTH"), process it differently
        elif isinstance(d, str) and d.startswith("ALGO#"):
            d = d[5:]  # Remove "ALGO#"
            
            if "[" in d and "]" in d:
                try:
                    to_return = eval(d)  # Convert string representation of list to actual list
                    print("Parsed list from ALGO message:", to_return)
                except Exception as e:
                    print(f"Error parsing ALGO list: {e}")
                    return
            else:
                d = d.split(',')
                for x in range(len(d) - 1):  # Avoid last empty split
                    d_split = d[x].split(',')
                    temp = []
                    for y in range(len(d_split)):
                        if y <= 1:
                            temp.append(int(d_split[y]) * 10)
                        elif y == 2:
                            if d_split[y] == 'N':
                                temp.append(90)
                            elif d_split[y] == 'S':
                                temp.append(-90)
                            elif d_split[y] == 'E':
                                temp.append(0)
                            else:
                                temp.append(180)
                        else:
                            temp.append(int(d_split[y]))
                    to_return.append(temp)
            
            print("Processed ALGO# data:", to_return)
            self.decision(self.client, to_return, also_run_simulator)

        # If it's a simple string command, process it directly
        elif isinstance(d, str):
            print("Received command from RPi:", d)
            self.decision(self.client, d, also_run_simulator)

        else:
            print("Received unknown data format:", d)
        
        self.decision(self.client, d, also_run_simulator)


    def decision(self, client, data, also_run_simulator):
        print("DEBUG: Received data =", repr(data))  # Debugging
        data = ast.literal_eval(data)
        print(len(data) > 1)
        print(isinstance(data,list))
        if isinstance(data, list) and len(data) > 1:
            # Extract Robot Data
            robot_info = data[0]
            if len(robot_info) == 3:
                robot_x, robot_y, robot_direction = robot_info
                print(f"Robot Position: X={robot_x}, Y={robot_y}, Dir={robot_direction}")
            else:
                print("ERROR: Invalid robot data format:", robot_info)
                return  # Exit if robot data is incorrect

            # Extract Obstacles Data
            obstacle_data = data[1:]  # Remaining elements in the list
            obstacles = []

            for obs in obstacle_data:
                if len(obs) == 4:  # Ensure valid obstacle format
                    obs_x, obs_y, obs_dir, obs_index = obs
                    obstacles.append((obs_x, obs_y, obs_dir, obs_index))
                else:
                    print(f"WARNING: Invalid obstacle format: {obs}")

            print(f"Obstacles received: {obstacles}")  # Debug output

            # Process obstacles if there are any
            if obstacles:
                self.process_obstacles(obstacles, also_run_simulator)
            else:
                print("WARNING: No obstacles detected!")

    def process_obstacles(self, obstacles,also_run_simulator):
        print("Processing obstacles for pathfinding...")
        
        obs_list = self.parse_obstacle_data(obstacles)
        if obs_list:
            app = AlgoMinimal(obs_list)
            app.init()

            if also_run_simulator:
                app.simulate()
            else:
                app.execute()

            obs_priority = app.robot.hamiltonian.get_simple_hamiltonian()
            print("Sending list of commands to RPi...")
            self.commands = app.robot.convert_all_commands()
            
            if self.commands:
                print(f"Commands to be sent: {self.commands}")
                self.client.send_message(self.commands)
            else:
                print("ERROR: No commands generated!")



    def run_rpi(self):
        # Ensure the client is connected before starting threads
        try:
            while True:
                self.run_minimal(False)
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\nClosing connection...")
            sys.exit(1)
            
        while True:
            time.sleep(5)




def initialize():
    algo = Main()
    algo.run_rpi()


def sim():
    algo = Main()
    algo.run_simulator()


def receive_messages(server):
    while True:
        try:
            msg = server.recv(1024)
            if msg:
                print("Received:", pickle.loads(msg))
        except socket.timeout:
            continue  # Avoid blocking forever
        except Exception as e:
            print("Error receiving:", e)
            break  # Exit if connection closes

def testflood():
    print('Start test')
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    server.connect(("192.168.35.10", 4060))
    print("Connected")

    # Start receiving thread
    threading.Thread(target=receive_messages, args=(server,), daemon=True).start()

    try:
        while True:
            #server.sendall(pickle.dumps("12345"))
            print("Sent")
            time.sleep(2)
    except KeyboardInterrupt:
        print("\nClosing connection...")
        server.shutdown(socket.SHUT_WR)
        server.close()


def test():
    print('start test')
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.connect(("192.168.35.10", 4060))
    print("connected")
    server.send(pickle.dumps("12345"))
    # msg = server.recv(1024)
    server.shutdown(socket.SHUT_WR)
    server.close()


if __name__ == '__main__':
    """
    Simulator 
    Rpi Connection
    Test
    """
    if True:
        initialize()
    else:
        sim()

