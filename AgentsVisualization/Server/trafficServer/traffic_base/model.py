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
from mesa.datacollection import DataCollector
from .agent import *
import json

class CityModel(Model):
    """
    Creates a model based on a city map.

    Args:
        N: Number of agents in the simulation
        seed: Random seed for the model
    """

    def __init__(
        self, N, seed=42,
        vehicle_spawn_rate=15,
        vehicles_per_step=4,
        ambulance_per_step=1,
        emergency_chance=0.5
    ):

        super().__init__(seed=seed)
        self.vehicle_spawn_rate = vehicle_spawn_rate
        self.vehicles_per_step = vehicles_per_step
        self.ambulance_per_step = ambulance_per_step
        self.emergency_chance = emergency_chance
        
        # Counters for data collection
        self.total_crashes = 0
        self.cars_reached_destination = 0
        self.ambulances_reached_hospital = 0
        self.cars_spawned_this_step = 0
        self.ambulances_spawned_this_step = 0
        
        # Historical counters (total spawned, including those that reached destination)
        self.total_cars_historical = 0
        self.total_ambulances_historical = 0
        self.total_reached_ambulances_historical = 0
        self.total_reached_cars_historical = 0
        
        # Setup data collection
        model_reporters = {
            "Time (Steps)": lambda m: m.steps,
            "Total Cars": lambda m: len(m.agents_by_type.get(CarAgent, [])),
            "Total Ambulances": lambda m: len(m.agents_by_type.get(Ambulance, [])),
            "Emergency Ambulances": lambda m: len([agent for agent in m.agents_by_type.get(Ambulance, []) if agent.has_emergency]),
            "Total Crashes": lambda m: m.total_crashes,
            "Cars Reached Destination": lambda m: m.cars_reached_destination,
            "Ambulances Reached Hospital": lambda m: m.ambulances_reached_hospital,
            "Cars Spawned": lambda m: m.cars_spawned_this_step,
            "Ambulances Spawned": lambda m: m.ambulances_spawned_this_step,
            "Total Cars Historical": lambda m: m.total_cars_historical,
            "Total Ambulances Historical": lambda m: m.total_ambulances_historical,
            "Total Reached Ambulances Historical": lambda m: m.total_reached_ambulances_historical,
            "Total Reached Cars Historical": lambda m: m.total_reached_cars_historical,
        }
        self.datacollector = DataCollector(model_reporters)

        # Load the map dictionary. The dictionary maps the characters in the map file to the corresponding agent.
        dataDictionary = json.load(open("city_files/mapDictionary.json"))

        # Initialize agent lists
        # Used for access to agents in the visualization
        self.num_agents = N
        self.traffic_lights = []
        self.roads = []
        self.cars = []
        self.obstacles = []
        self.hospitals = []
        self.destinations = []
        self.sidewalks = []
        self.ambulances = []

        # Store coordinates to make the model more efficient
        # This is used by A* to quickly check road directions and valid cells
        # Getting agents directly from a set is much faster than iterating through all agents in a cell
        self.road_directions = {}
        self.road_cells = set()
        self.destination_cells = set()
        self.hospital_cells = set()
        self.obstacle_cells = set()
        self.intersection_cells = set()

        # Counters to keep track of spawned vehicles
        self.car_spawned_count = 0
        self.ambulance_spawned_count = 0

        # Load the map file. The map file is a text file where each character represents an agent.
        with open("city_files/2025_base.txt") as baseFile:
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
                    cell_coord = (c, self.height - r - 1)

                    if col in ["v", "^", ">", "<"]:
                        agent = Road(self, cell, dataDictionary[col])
                        self.roads.append(agent)
                        
                        self.road_cells.add(cell_coord)
                        self.road_directions[cell_coord] = agent.direction

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

                        self.road_cells.add(cell_coord)
                        self.road_directions[cell_coord] = road_agent.direction

                    elif col == "#":
                        sidewalk_agent = SideWalk(self, cell)
                        agent = Obstacle(self, cell)
                        self.obstacles.append(agent)
                        self.sidewalks.append(sidewalk_agent)

                        self.obstacle_cells.add(cell_coord)

                    elif col == "D":
                        sidewalk_agent = SideWalk(self, cell)
                        agent = Destination(self, cell)
                        self.destinations.append(agent)
                        self.sidewalks.append(sidewalk_agent)

                        self.destination_cells.add(cell_coord)
                    
                    elif col == "H":
                        sidewalk_agent = SideWalk(self, cell)
                        agent = Hospital(self, cell)
                        self.hospitals.append(agent)
                        self.sidewalks.append(sidewalk_agent)

                        self.hospital_cells.add(cell_coord)
                    
                    elif col == "C":
                        # First create the Road agent below the car
                        # We check for the direction, the car is represented 
                        # by 'C' and there is no direction info.
                        road_agent = Road(self, cell, dataDictionary[">"]) 
                        agent = CarAgent(self, cell)
                        self.cars.append(agent)
                        self.roads.append(road_agent)

                        self.road_cells.add(cell_coord)
                        self.road_directions[cell_coord] = road_agent.direction

                    elif col == "A":
                        # First create the Road agent below the ambulance
                        # We use a default direction because in the map, the ambulance 
                        # is represented by 'A' and there is no direction info.
                        road_agent = Road(self, cell, dataDictionary["v"]) 
                        agent = Ambulance(self, cell)
                        self.ambulances.append(agent)
                        self.roads.append(road_agent)

                        self.road_cells.add(cell_coord)
                        self.road_directions[cell_coord] = road_agent.direction
        
        # Get corners of the grid
        self.corners = [
            self.grid[0, 0], # bottom left
            self.grid[0, self.height - 1], # top left
            self.grid[self.width - 1, 0], # bottom right
            self.grid[self.width - 1, self.height - 1], # top right
        ]

        # Detect intersections after all roads are created
        self.detectIntersections()
        
        self.spawnVehicles()
        self.running = True
    
    def detectIntersections(self):
        """
        Detect intersections in the city.
        An intersection is a road cell where 3 or more roads meet.
        """
        for coord in self.road_cells:
            x, y = coord
            
            # Count neighboring road cells
            neighbors = 0
            neighbor_coords = [
                (x, y + 1),  # Up
                (x, y - 1),  # Down
                (x - 1, y),  # Left
                (x + 1, y)   # Right
            ]
            
            for nx, ny in neighbor_coords:
                # Check if neighbor is within bounds and is a road
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    neighbor_coord = (nx, ny)
                    if neighbor_coord in self.road_cells:
                        neighbors += 1
            
            # If a road cell has 3 or more neighbors, it's an intersection
            if neighbors >= 3:
                self.intersection_cells.add(coord)
    
    def spawnVehicles(self):
        """Spawn vehicles at the corners of the grid based on the spawn rates."""

        # Check if the vehicles can be spawn based on the step
        if self.steps % self.vehicle_spawn_rate != 0:
            return

        cars_spawned = 0
        ambulances_spawned = 0
        corners_full = 0
        corners_to_use = self.corners.copy()

        # Spawn vehicles at each corner
        for _ in range(self.vehicles_per_step):

            # Get random corner
            corner = self.random.choice(corners_to_use)
            corners_to_use.remove(corner)

            # Check if that corner is full
            has_vehicle = any(
                isinstance(agent, (CarAgent, Ambulance))
                for agent in corner.agents
            )
            if has_vehicle:
                corners_full += 1
                if corners_full >= len(self.corners):
                    self.running
                    return
                continue

            # Spawn ambulances
            if ambulances_spawned < self.ambulance_per_step:
                ambulance = Ambulance(self, corner)
                self.ambulances.append(ambulance)
                self.ambulances_spawned_this_step += 1
                self.total_ambulances_historical += 1
                ambulances_spawned += 1
            else:
                # Spawn cars
                car = CarAgent(self, corner)
                self.cars.append(car)
                self.cars_spawned_this_step += 1
                self.total_cars_historical += 1
                cars_spawned += 1

    def step(self):
        """Advance the model by one step."""

        # If the model is not running, do nothing
        if not self.running:
            return
        
        # Reset counters
        self.cars_reached_destination = 0
        self.ambulances_reached_hospital = 0
        self.cars_spawned_this_step = 0
        self.ambulances_spawned_this_step = 0
        
        # Do step
        self.agents.shuffle_do("step")
        self.spawnVehicles()

        # Collect data
        self.datacollector.collect(self)
