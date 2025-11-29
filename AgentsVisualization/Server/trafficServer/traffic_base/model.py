"""
Module that contains the CityModel class.
This module is part of the traffic simulation model using the Mesa framework.

Diego Córdova Rodríguez
Lorena Estefanía Chewtat Torres
Aquiba Yudah Benarroch Bittán

2025-11-27
"""

from mesa import Model
from mesa.discrete_space import OrthogonalMooreGrid
from .agent import *
import json

class CityModel(Model):
    """
    Creates a model based on a city map.

    Args:
        N: Number of agents in the simulation
        seed: Random seed for the model
    """

    def __init__(self, N, seed=42, car_spawn_rate=5, vehicles_per_step=4, ambulance_per_step=1):

        super().__init__(seed=seed)

        self.car_spawn_rate = car_spawn_rate
        self.vehicles_per_step = vehicles_per_step
        self.ambulance_per_step = ambulance_per_step

        # Load the map dictionary. The dictionary maps the characters in the map file to the corresponding agent.
        dataDictionary = json.load(open("city_files/mapDictionary.json"))

        self.num_agents = N
        self.traffic_lights = []
        self.roads = []
        self.cars = []
        self.obstacles = []
        self.hospitals = []
        self.destinations = []
        self.sidewalks = []
        self.ambulances = []

        # Counters to keep track of spawned vehicles
        self.car_spawned_count = 0
        self.ambulance_spawned_count = 0

        # Load the map file. The map file is a text file where each character represents an agent.
        with open("city_files/2022_base.txt") as baseFile:
            lines = baseFile.readlines()
            lines = [line.strip() for line in lines if line.strip()]
            self.width = len(lines[0])
            self.height = len(lines)

            self.grid = OrthogonalMooreGrid(
                [self.width, self.height], capacity=100, torus=False
            )

            # Goes through each character in the map file and creates the corresponding agent.
            for r, row in enumerate(lines):
                for c, col in enumerate(row):

                    cell = self.grid[(c, self.height - r - 1)]

                    if col in ["v", "^", ">", "<"]:
                        agent = Road(self, cell, dataDictionary[col])
                        self.roads.append(agent)

                    elif col in ["S", "s"]:
                        # Count surrounding road directions
                        horizontal_right = row.count(">")
                        horizontal_left = row.count("<")
                        
                        # Count vertical directions in the current column
                        vertical_down = sum(1 for i, line in enumerate(lines) if c < len(line) and line[c] == "v")
                        vertical_up = sum(1 for i, line in enumerate(lines) if c < len(line) and line[c] == "^")
                        
                        if horizontal_right + horizontal_left > vertical_down + vertical_up:
                            # Avenida horizontal
                            if horizontal_right >= horizontal_left:
                                direction = "Right"
                            else:
                                direction = "Left"
                        else:
                            # Avenida vertical
                            if vertical_down >= vertical_up:
                                direction = "Down"
                            else:
                                direction = "Up"
                        
                        # Place both Road and Traffic Light agents
                        road_agent = Road(self, cell, direction)
                        agent = Traffic_Light(
                            self,
                            cell,
                            False if col == "S" else True,
                            int(dataDictionary[col]),
                        )
                        self.traffic_lights.append(agent)
                        self.roads.append(road_agent)

                    elif col == "#":
                        sidewalk_agent = SideWalk(self, cell)
                        agent = Obstacle(self, cell)
                        self.obstacles.append(agent)
                        self.sidewalks.append(sidewalk_agent)

                    elif col == "D":
                        sidewalk_agent = SideWalk(self, cell)
                        agent = Destination(self, cell)
                        self.destinations.append(agent)
                        self.sidewalks.append(sidewalk_agent)
                    
                    elif col == "H":
                        sidewalk_agent = SideWalk(self, cell)
                        agent = Hospital(self, cell)
                        self.hospitals.append(agent)
                        self.sidewalks.append(sidewalk_agent)
                    
                    elif col == "C":
                        # First create the Road agent below the car
                        # We check for the direction, the car is represented by 'C' and there is no direction info.
                        road_agent = Road(self, cell, dataDictionary[">"]) 
                        agent = CarAgent(self, cell)
                        self.cars.append(agent)
                        self.roads.append(road_agent)

                    elif col == "A":
                        # First create the Road agent below the ambulance
                        # We use a default direction because in the map, the ambulance is represented by 'A' and there is no direction info.
                        road_agent = Road(self, cell, dataDictionary["v"]) 
                        agent = Ambulance(self, cell)
                        self.ambulances.append(agent)
                        self.roads.append(road_agent)
        
        # Get corners of the grid
        self.corners = [
            self.grid[0, 0], # bottom left
            self.grid[0, self.height - 1], # top left
            self.grid[self.width - 1, 0], # bottom right
            self.grid[self.width - 1, self.height - 1], # top right
        ]

        self.spawn_vehicles()
        self.running = True
    
    def spawn_vehicles(self):
        """Spawn vehicles at the corners of the grid based on the spawn rates."""

        # Check if the vehicles can be spawn based on the step
        if self.steps % self.car_spawn_rate != 0:
            return

        cars_spawned = 0
        ambulances_spawned = 0
        corners_to_use = self.corners.copy()

        # Spawn vehicles at each corner
        for _ in range(len(corners_to_use)):

            # Get random corner
            corner = self.random.choice(corners_to_use)
            corners_to_use.remove(corner)

            # Spawn ambulances
            if ambulances_spawned < self.ambulance_per_step:
                ambulance = Ambulance(self, corner)
                self.ambulances.append(ambulance)
                ambulances_spawned += 1
            else:
                # Spawn cars
                car = CarAgent(self, corner)
                self.cars.append(car)
                cars_spawned += 1

    def step(self):
        """Advance the model by one step."""
        self.spawn_vehicles()
        self.agents.shuffle_do("step")
