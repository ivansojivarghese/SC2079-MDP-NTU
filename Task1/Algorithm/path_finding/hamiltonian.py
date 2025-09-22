import itertools
import math
import constants as constants

from typing import Tuple
from collections import deque
from commands.scan_obstacle_command import ScanCommand
from commands.go_straight_command import StraightCommand
from commands.turn_command import TurnCommand
from misc.direction import Direction
from grid.grid import Grid
from grid.obstacle import Obstacle
from path_finding.modified_a_star import ModifiedAStar


class Hamiltonian:
    def __init__(self, robot, grid: Grid):
        self.robot = robot
        self.grid = grid

        # Compute the simple Hamiltonian path for all obstacles
        self.simple_hamiltonian = tuple()

        # Create all the commands required to finish the course.
        self.commands = deque()

    def get_simple_hamiltonian(self):
        return self.simple_hamiltonian

    def compute_simple_hamiltonian_path(self) -> Tuple[Obstacle]:
        """
        Get the Hamiltonian Path to all points with the best possible effort.
        This is a simple calculation where we assume that we travel directly to the next obstacle.
        """
        # Generate all possible permutations for the image obstacles
        perms = list(itertools.permutations(self.grid.obstacles))

        # Get the path that has the least distance travelled.
        def calc_distance(path):
            def weight_factor(source_pos, dest_pos, is_first) -> int:
                # Right Grid to Left Grid, Top of Grid to Bottom of Grid unlikely
                # if same direction (robot and targeted position has same direction)
                if source_pos.direction.value - dest_pos.direction.value == 0:
                    weight = 1 if is_first else 5
                # if opposite direction
                elif abs(source_pos.direction.value - dest_pos.direction.value) == 180:
                    weight = 3
                # if turn right or left
                else:
                    weight = 1.5 if check_pos(source_pos, dest_pos) else 7

                return weight

            def check_pos(robot_pos, target_pos):
                # left turn
                if robot_pos.direction.value - target_pos.direction.value == -90:
                    if robot_pos.direction == Direction.TOP:
                        return True if target_pos.x < robot_pos.x else False
                    if robot_pos.direction == Direction.BOTTOM:
                        return True if target_pos.x > robot_pos.x else False
                    if robot_pos.direction == Direction.LEFT:
                        return True if target_pos.y < robot_pos.y else False
                    if robot_pos.direction == Direction.RIGHT:
                        return True if target_pos.y > robot_pos.y else False
                # right turn
                else:
                    if robot_pos.direction == Direction.TOP:
                        return True if target_pos.x > robot_pos.x else False
                    if robot_pos.direction == Direction.BOTTOM:
                        return True if target_pos.x < robot_pos.x else False
                    if robot_pos.direction == Direction.LEFT:
                        return True if target_pos.y > robot_pos.y else False
                    if robot_pos.direction == Direction.RIGHT:
                        return True if target_pos.y < robot_pos.y else False

            def manhattan_distance(x1, y1, x2, y2):
                return abs(x1 - x2) + abs(y1 - y2)

            # Create all target points, including the start.
            targets = [self.robot.pos.xy()]
            # Try out all the different permutations
            for obstacle in path:
                targets.append(obstacle.target_position.xy())

            # print("Targets ", targets)
            # print("Path ", path)
            # print()

            dist = 0
            multiplier = 1
            for i in range(len(targets)-1):
                # Weight factor
                # multiplier = weight_factor(
                #     path[i].target_position.get_dir(), path[i+1].target_position.get_dir())
                # dist += multiplier * math.sqrt(((targets[i][0] - targets[i + 1][0]) ** 2) +
                #                                ((targets[i][1] - targets[i + 1][1]) ** 2))

                # Weight factor
                if i == 0:
                    # From start to first obstacle
                    # print(
                    #     f"Checking multiplier: {self.robot.pos.get_dir()}, {path[i].target_position.x}, {path[i].target_position.y}, {path[i].target_position.get_dir()}")
                    multiplier = weight_factor(
                        self.robot.pos, path[i].target_position, True)
                    # start, end = self.robot.pos.copy(), path[i].target_position
                    # tmp, cmds = ModifiedAStar(self.grid, self, start, end, 2).start_astar(False)
                else:
                    # From obstacle to another obstacle
                    multiplier = weight_factor(
                        path[i-1].target_position, path[i].target_position, False)
                    # start, end = path[i-1].target_position, path[i].target_position
                    # tmp, cmds = ModifiedAStar(self.grid, self, start, end, 2).start_astar(False)

                # for cmd in cmds:
                #     if isinstance(cmd, StraightCommand):
                #         dist += cmd.dist
                #     elif isinstance(cmd, TurnCommand):
                #         dist += 50

                # dist += abs(targets[i][0] - targets[i + 1][0]) + abs(targets[i][1] - targets[i + 1][1])

                dist += multiplier * (math.sqrt(((targets[i][0] - targets[i + 1][0])**2) +
                                                ((targets[i][1] - targets[i + 1][1])**2)))
                # temp = math.sqrt(((targets[i][0] - targets[i + 1][0])**2) +
                #                                 ((targets[i][1] - targets[i + 1][1])**2))
                # dist += (temp ** multiplier)

                # temp = manhattan_distance(
                #     targets[i][0], targets[i][1], targets[i + 1][0], targets[i + 1][1])
                # dist += temp ** multiplier
                # print(dist)
                # dist += multiplier * (1 * math.sqrt(((targets[i][0] - targets[i + 1][0])**2) +
                #                                     ((targets[i][1] - targets[i + 1][1])**2)) + 2 * manhattan_distance(
                #     targets[i][0], targets[i][1], targets[i +
                #                                           1][0], targets[i + 1][1]
                # ))
                # dist += multiplier * (manhattan_distance(
                #     targets[i][0], targets[i][1], targets[i +
                #                                           1][0], targets[i + 1][1]
                # ))

            # print("Path = ", targets, "\nTotal weighted Euclidean distance = ", dist)
            # print(f"Dist for this permutation is {dist}")
            return dist

        print("Calculating Distance for all possible permutation\n")
        # Change to max to show paths change
        # simple now holds the permutation that gives the lowest distance
        simple = min(perms, key=calc_distance)
        print("\nFound a simple hamiltonian path: ")
        # print(simple)

        # print out every obstacle in order of visitation
        for ob in simple:
            print(f"\t{ob}")
        print()
        print("Found Shortest Hamiltonian Path")

        # calc_distance(simple)
        # returns order of visitation of obstacles (the lowest cost)
        return simple

    def compress_paths(self):
        """
        Compress similar commands into one command.
        Compressing many straight line commands into one.
        Helps to reduce the number of commands.
        """
        print("Compressing commands... ", end="")
        index = 0
        new_commands = deque()

        while index < len(self.commands):
            command = self.commands[index]

            if isinstance(command, StraightCommand):
                new_length = 0
                while index < len(self.commands) and isinstance(self.commands[index], StraightCommand):
                    new_length += self.commands[index].dist
                    index += 1
                command = StraightCommand(new_length)
                new_commands.append(command)
            else:
                new_commands.append(command)
                index += 1

        self.commands = new_commands
        print("Done!")

    def plan_path(self):
        print("-" * 40)
        print("STARTING PATH COMPUTATION...")
        self.simple_hamiltonian = self.compute_simple_hamiltonian_path()
        print()

        curr = self.robot.pos.copy()
        for obstacle in self.simple_hamiltonian:
            # Use the default target (e.g. 20cm behind the obstacle's face)
            target = obstacle.get_robot_target_pos()  
            rerun = 0
            
            # Attempt path planning using the current target and effective safety
            res, cmd = ModifiedAStar(self.grid, self, curr, target, rerun).start_astar(True)
            
            # Retry while no path is found and we haven't hit 3 attempts.
            while res is None and rerun < 3:
                print(f"No path found from {curr} to {obstacle} with target {target}")
                print("Retrying with increased yolo...", end=" ")
                rerun += 1
                res, cmd = ModifiedAStar(self.grid, self, curr, target, rerun).start_astar(True)
            
            # If no path is found after 5 tries, change the target offset.
            if res is None:
                print(f"\nNo path found after {rerun} tries. Switching target offset for {obstacle}.")
                # Compute the alternate target position (e.g. 10cm in front of the target face)
                target = obstacle.get_robot_target_pos(offset=0)
                # You can either keep the same rerun counter or reset it.
                res, cmd = ModifiedAStar(self.grid, self, curr, target, rerun).start_astar(True)
                
                if res:
                    print(f"Alternate path found with target {target}")
                else:
                    print(f"Still no path found from {curr} to {obstacle} with alternate target.")

            if res is None:
                print(f"No path found from {curr} to {obstacle}")
            else:
                print("Path found!")
                curr = res
                self.commands.append(ScanCommand(constants.ROBOT_SCAN_TIME, obstacle.index))

        self.compress_paths()

        print()
        print("-" * 60)
        for command in self.commands:
            print(f'{command}')
        print("-" * 60)
        print()
