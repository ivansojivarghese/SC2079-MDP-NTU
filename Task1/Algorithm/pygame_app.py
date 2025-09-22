from typing import List
from abc import ABC, abstractmethod

import pygame

import constants
import misc.timer as timer
from misc.direction import Direction
from grid.grid import Grid
from grid.obstacle import Obstacle
from robot.robot import Robot
from misc.positioning import Position
from simulation import Simulation


class AlgoApp(ABC):
    def __init__(self, obstacles: List[Obstacle]):
        self.grid = Grid(obstacles)
        self.robot = Robot(self.grid)
        self.direction = Direction.TOP
        # self.simulation = Simulation()
        self.obstacles = obstacles
        self.index = 0

    @abstractmethod
    def init(self):
        pass

    @abstractmethod
    def execute(self):
        pass




class AlgoMinimal(AlgoApp):
    """
    Minimal app to just calculate a path and then send the commands over.
    """

    def __init__(self, obstacles):
        # We run it as a server.
        super().__init__(obstacles)

    def init(self):
        pass

    def execute(self):
        # Calculate path
        print("Calculating path...")
        self.robot.hamiltonian.plan_path()
        print("Done!")

    def simulate(self):
        # Calculate path
        self.simulation.runSimulation(self.robot)
