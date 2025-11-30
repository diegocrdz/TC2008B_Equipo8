"""
Module that contains the agents for the traffic simulation model using the Mesa framework.

Diego Córdova Rodríguez
Lorena Estefanía Chewtat Torres
Aquiba Yudah Benarroch Bittán

2025-11-29
"""

from mesa.discrete_space import CellAgent, FixedAgent
import random
import heapq # For priority queue in A* algorithm

class VehicleAgent(CellAgent):
    """
    Base class for vehicle agents.
    """
    def __init__(self, model, cell):
        """
        Creates a new vehicle agent.
        Args:
            model: Model reference for the agent
        """
        super().__init__(model)
        self.cell = cell
        self.state = "idle"
        self.destination = None
        self.path = []
        self.path_index = 0

        # Initialize lastDirection from the road in the current cell
        self.lastDirection = None
        road_agent = next(
            (agent for agent in cell.agents if isinstance(agent, Road)),
            None
        )
        if road_agent:
            self.lastDirection = road_agent.direction
        
    def move(self, next_cell):
        """Moves the agent to the next cell."""
        if next_cell is not None:
            self.cell = next_cell
    
    def a_star(self, start, goal):
        """
        A* pathfinding algorithm adapted for the grid in the model

        This algorithm was adapted from the one made in the advanced algorithm class
        with Lizbeth Peralta. Made by Diego Cordova, Aquiba Benarroch and Lorena Chewtat.
        """

        # Calculate Manhattan distance
        # We have to estimate heuristic using this distance
        # since it is not given from the model
        # Ref: https://www.geeksforgeeks.org/dsa/a-search-algorithm/
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
        def getValidNeighbors(current_coord):
            """Get valid neighboring respecting road directions."""
            valid_neighbors = []
            
            # Get the current cell
            current_cell = self.model.grid[current_coord]
            
            # Get the current road direction
            current_road = None
            for agent in current_cell.agents:
                if isinstance(agent, Road):
                    current_road = agent
                    break
            
            if current_road is None:
                return []  # No road, no neighbors
            
            direction = current_road.direction
            x, y = current_coord
            
            # Get neighbors (including diagonals)
            neighbors = current_cell.get_neighborhood(radius=2, include_center=False)
            
            # Opposite directions to avoid
            opposites = {
                "Up": "Down",
                "Down": "Up",
                "Left": "Right",
                "Right": "Left"
            }
            opposite = opposites.get(direction)
            
            # Check each neighbor
            for neighbor_cell in neighbors:
                nx, ny = neighbor_cell.coordinate
            
                # Check if it's destination or has a road (no obstacle)
                has_road = any(isinstance(agent, Road) for agent in neighbor_cell.agents)
                has_destination = any(isinstance(agent, Destination) for agent in neighbor_cell.agents)
                has_hospital = any(isinstance(agent, Hospital) for agent in neighbor_cell.agents)
                has_obstacle = any(isinstance(agent, Obstacle) for agent in neighbor_cell.agents)
                
                if has_obstacle or (not has_road and not has_destination and not has_hospital):
                    continue
                
                # Get the direction of movement from current to neighbor
                move_direction = None
                if nx == x and ny == y + 1:
                    move_direction = "Up"
                elif nx == x and ny == y - 1:
                    move_direction = "Down"
                elif nx == x - 1 and ny == y:
                    move_direction = "Left"
                elif nx == x + 1 and ny == y:
                    move_direction = "Right"
                
                # Allow any direction except the opposite (backwards)
                if move_direction and move_direction != opposite:
                    valid_neighbors.append((nx, ny))

            return valid_neighbors

        # Initialize variables
        stack = [] # Stack of nodes to explore
        c_list = {}  # g values
        visited = set()  # visited nodes
        fathers = {} # Father vector to reconstruct path

        # Initialize stack with start node
        # Heap already sorts by smallest f value
        heapq.heappush(stack, (0, start))
        c_list[start] = 0

        # While the stack is not empty
        # Explore neighbors with lowest f value
        while len(stack) > 0:
            # Get node with lowest f value
            # This returns f, coordinate
            # but we only need coordinate, so we use _
            _, current = heapq.heappop(stack)

            # If the node hasnt been visited, process it
            if current not in visited:

                # Mark as visited
                visited.add(current)

                # If we reached the goal, finish
                if (current == goal):
                    print("Goal reached!")
                    break

                # Get valid neighbors based on road directions
                valid_neighbors = getValidNeighbors(current)

                # Explore neighbors
                for neighbor in valid_neighbors:
                    actual_coordinate = c_list[current] + 1 # Cost between nodes is 1

                    if actual_coordinate < c_list.get(neighbor, float('inf')):
                        c_list[neighbor] = actual_coordinate
                        fathers[neighbor] = current
                        f_value = actual_coordinate + heuristic(neighbor, goal)

                        # Add to stack
                        heapq.heappush(stack, (f_value, neighbor))

        # Reconstruct path
        if goal in fathers:
            path = []
            current = goal
            while current != start:
                path.append(current)
                current = fathers[current]
            path.reverse()
            return path
        
        # If no path found, return empty list
        return []

class CarAgent(VehicleAgent):
    """
    Agent that moves randomly.
    """
    def __init__(self, model, cell):
        """
        Creates a new random agent.
        Args:
            model: Model reference for the agent
            cell: The initial position of the agent
        """
        super().__init__(model, cell)
        self.moved_for_ambulance = False  # Track if already moved to let ambulance pass
    
    def assignRandomDestination(self):
        """Assigns a random destination from available Destination agents."""
        # Get all Destination agents from the model
        destinations = [
            agent for agent in self.model.agents 
            if isinstance(agent, Destination)
        ]
        
        if destinations:
            return random.choice(destinations)
        
        return None
    
    def pathToDestination(self):
        """Initialize destination and calculate path using A*."""
        if self.destination is None:
            self.destination = self.assignRandomDestination()

            if self.destination:
                start = self.cell.coordinate
                goal = self.destination.cell.coordinate
                print(f"\n>>> Car at {start} assigned destination {goal}")
                self.path = self.a_star(start, goal)

                if self.path:
                    print(f">>> Path found with {len(self.path)} steps")
                else:
                    print(f">>> NO PATH from {start} to {goal}")

                self.path_index = 0

    def getNextReturnStep(self):
        """Get the next step using A* path, checking traffic lights and cars."""
        
        # Get the next cell from the A* path
        next_coord = self.path[0]
        next_cell = self.model.grid[next_coord]
        
        # Check if there's a car in the next cell
        has_car = any(isinstance(agent, CarAgent) for agent in next_cell.agents)
        
        if has_car:
            self.state = "waitingCar" #Change state to waiting for car
            return None
        
        # Check if there's a traffic light
        has_traffic_light = any(isinstance(agent, Traffic_Light) for agent in next_cell.agents)
        
        if has_traffic_light:
            # If there's a traffic light, get it and check its state
            traffic_light = next(
                (obj for obj in next_cell.agents if isinstance(obj, Traffic_Light)), None
            )
            
            if traffic_light and not traffic_light.is_green:
                # Traffic light is red, wait
                self.state = "waitingTL"
                return None
            else:
                # Traffic light is green, can move
                # The A* path has already validated this move is possible
                self.state = "moving"
                return next_cell
        
        # Path is clear, change state to moving
        self.state = "moving"
        return next_cell

    def getNextCell(self, cell = None):
        """Gets the next cell to move to based on current position."""
        # Get the Road agent in current cell
        if cell is None:
            cell = self.cell

        # Get the road agent in current cell to extract direction
        road_agent = next(
            (agent for agent in self.cell.agents if isinstance(agent, Road)), None
        )
        
        if road_agent is None:
            return None
            
        direction = road_agent.direction

        # Get next cell based on current direction
        if direction == "Up":
            next_coord = (self.cell.coordinate[0], self.cell.coordinate[1] + 1)
        elif direction == "Down":
            next_coord = (self.cell.coordinate[0], self.cell.coordinate[1] - 1)
        elif direction == "Left":
            next_coord = (self.cell.coordinate[0] - 1, self.cell.coordinate[1])
        elif direction == "Right":
            next_coord = (self.cell.coordinate[0] + 1, self.cell.coordinate[1])
        else:
            return None

        # Get the next cell from grid
        next_cell = self.model.grid[next_coord]
        
        # Verify next cell has a road and no obstacle
        has_road = any(isinstance(agent, Road) for agent in next_cell.agents)
        has_obstacle = any(isinstance(agent, Obstacle) for agent in next_cell.agents)
        
        if has_road and not has_obstacle:
            return next_cell
        
        return None

    def checkCars(self):
        """Chooses the next cell based on the presence of cars."""
        
        # Get next cell
        next_cell = self.getNextCell()

        if next_cell is None:
            return None

        # Get next cell based on current direction
        next_cell = self.getNextCell()
        has_car = any(isinstance(agent, CarAgent) for agent in next_cell.agents)
        
        if has_car:
            # Try to find an alternative path (diagonal or lateral cells)
            alternative_cell = self.findAlternativePath()
            if alternative_cell is not None:
                self.state = "moving"
                return alternative_cell
            
            # No alternative found, wait for car to move
            self.state = "waitingCar"
            return None
        
        # Check if there's a traffic light
        has_traffic_light = any(isinstance(agent, Traffic_Light) for agent in next_cell.agents)
        
        if has_traffic_light:
            # If there's a traffic light, check it
            return self.checkTL(next_cell)
        
        # Path is clear, change state to moving
        self.state = "moving"
        return next_cell
    
    def findAlternativePath(self):
        """Try to find an alternative cell (diagonal or lateral) to avoid blockage."""
        x, y = self.cell.coordinate
        
        # Get current direction
        direction = self.lastDirection
        if direction is None:
            for agent in self.cell.agents:
                if isinstance(agent, Road):
                    direction = agent.direction
                    break
        
        if direction is None:
            return None
        
        # Get possible alternative cells based on direction
        alternative_coords = []
        if direction == "Up":
            alternative_coords = [(x - 1, y), (x + 1, y), (x - 1, y + 1), (x + 1, y + 1)]
        elif direction == "Down":
            alternative_coords = [(x - 1, y), (x + 1, y), (x - 1, y - 1), (x + 1, y - 1)]
        elif direction == "Left":
            alternative_coords = [(x, y + 1), (x, y - 1), (x - 1, y + 1), (x - 1, y - 1)]
        elif direction == "Right":
            alternative_coords = [(x, y + 1), (x, y - 1), (x + 1, y + 1), (x + 1, y - 1)]
        
        # Check each alternative cell
        for nx, ny in alternative_coords:
            # Validate bounds
            if not (0 <= nx < self.model.width and 0 <= ny < self.model.height):
                continue
            
            alt_cell = self.model.grid[(nx, ny)]
            
            # Check if cell has road and no obstacle
            has_road = any(isinstance(agent, Road) for agent in alt_cell.agents)
            has_obstacle = any(isinstance(agent, Obstacle) for agent in alt_cell.agents)
            
            if not has_road or has_obstacle:
                continue
            
            # Check if cell has car or ambulance
            has_car = any(isinstance(agent, CarAgent) for agent in alt_cell.agents)
            has_ambulance = any(isinstance(agent, Ambulance) for agent in alt_cell.agents)
            
            if has_car or has_ambulance:
                continue
            
            # Check if there's a traffic light and if it's green
            traffic_light = next(
                (obj for obj in alt_cell.agents if isinstance(obj, Traffic_Light)), None
            )
            
            if traffic_light and not traffic_light.is_green:
                continue
            
            # Cell is valid
            return alt_cell
        
        return None
    
    def isBlockingAmbulance(self, ambulance):
        """
        Checks if this car is blocking the path of the given ambulance.
            
        """
        # First check if ambulance is in emergency state
        if not hasattr(ambulance, 'state') or ambulance.state != "emergency":
            return False
        
        # Get ambulance position and direction
        amb_x, amb_y = ambulance.cell.coordinate
        amb_direction = ambulance.lastDirection
        
        # If ambulance has no direction yet, get it from its current road
        if amb_direction is None:
            for agent in ambulance.cell.agents:
                if isinstance(agent, Road):
                    amb_direction = agent.direction
                    break
        
        if amb_direction is None:
            return False
        
        # Get car position
        car_x, car_y = self.cell.coordinate
        
        # Check if car is directly in front of ambulance based on ambulance's direction
        if amb_direction == "Up":
            # Car must be directly above ambulance (same x, higher y) 
            # We use the range of 4 cells ahead
            if car_x == amb_x and car_y > amb_y and car_y <= amb_y + 4:
                return True
        elif amb_direction == "Down":
            # Car must be directly below ambulance (same x, lower y)
            # We use the range of 4 cells ahead
            if car_x == amb_x and car_y < amb_y and car_y >= amb_y - 4:
                return True
        elif amb_direction == "Right":
            # Car must be directly to the right of ambulance (same y, higher x)
            # We use the range of 4 cells ahead
            if car_y == amb_y and car_x > amb_x and car_x <= amb_x + 4:
                return True
        elif amb_direction == "Left":
            # Car must be directly to the left of ambulance (same y, lower x)
            # We use the range of 4 cells ahead
            if car_y == amb_y and car_x < amb_x and car_x >= amb_x - 4:
                return True
        
        return False

    def checkAmbulance(self):
        """Chooses the next cell based on the state of ambulances."""
        # Continue statement from geeksforgeeks: https://www.geeksforgeeks.org/python/python-continue-statement/
        # This method was obtained from Mesa API Documentation
        # https://mesa.readthedocs.io/latest/apis/discrete_space.html#mesa.discrete_space.__init__.OrthogonalMooreGrid
        neighbor_cells = self.cell.get_neighborhood(
            radius=5,  # Increased radius to detect ambulances earlier
            include_center=False
        )
        ambulance_nearby = False
    
        for neighbor_cell in neighbor_cells:
            ambulance = next(
                (obj for obj in neighbor_cell.agents if isinstance(obj, Ambulance)), None
            )

            # If an ambulance has been detected in emergency, clear the path
            if ambulance and ambulance.state == "emergency":
                ambulance_nearby = True
                # Check if the ambulance already moved diagonally this step
                # This method was obtained from Geekforgeeks
                # https://www.geeksforgeeks.org/python/python-hasattr-method/
                if hasattr(ambulance, 'moved_diagonally') and ambulance.moved_diagonally:
                    # Ambulance has already moved on its own, stay in place to avoid crashing or blocking 
                    return None
                
                # Check if this car is really blocking the ambulance's path
                if not self.isBlockingAmbulance(ambulance):
                    # Not blocking, continue normally
                    continue
                
                # Otherwise, if we're blocking and haven't moved yet
                if not self.moved_for_ambulance:
                    # Get diagonal cells based on the actual direction of the car
                    diagonal_cells = self.get_diagonal_cells()
                    
                    if diagonal_cells:
                        # Move to a random diagonal cell that is valid
                        valid_neighbors = []
                        for cell in diagonal_cells:
                            has_road = any(isinstance(obj, Road) for obj in cell.agents)
                            if not has_road:
                                continue
                            has_obstacle = any(isinstance(obj, Obstacle) for obj in cell.agents)
                            has_car = any(isinstance(obj, CarAgent) for obj in cell.agents)
                            has_ambulance_obj = any(isinstance(obj, Ambulance) for obj in cell.agents)
                            traffic_light = next(
                                (obj for obj in cell.agents if isinstance(obj, Traffic_Light)), None
                            )
                            if has_obstacle or has_car or has_ambulance_obj:
                                continue
                            if traffic_light and not traffic_light.is_green:
                                continue
                            valid_neighbors.append(cell)
                        
                        if valid_neighbors:
                            new_cell = random.choice(valid_neighbors)
                            self.move(new_cell)
                            self.moved_for_ambulance = True
                            self.state = "idle"
                            # ya nos hicimos a un lado, no avanzamos este turno
                            return None

                    # If no diagonal cell found, just stay in place
                    self.moved_for_ambulance = True
                    break 

                else:
                    break

        
        # No ambulance nearby reset flag to false
        if not ambulance_nearby:
            self.moved_for_ambulance = False
        
        # No ambulance in emergency nearby continue
        return self.getNextCell()
    
    def checkTL(self, next_cell):
        """Chooses the next cell based on the state of the traffic light."""
        # Get the traffic light agent in the next cell
        traffic_light = next(
            (agent for agent in next_cell.agents if isinstance(agent, Traffic_Light)), None
        )

        if traffic_light and not traffic_light.is_green:
            # Traffic light is red, wait
            self.state = "waitingTL"
            return None
        else:
            # Traffic light is green, can move
            self.state = "moving"
            return next_cell
    
    def checkDestination(self):
        """Checks if the car has reached its destination."""
        for agent in self.cell.agents:
            if isinstance(agent, Destination):
                self.remove()  # Remove car from the model
        
    def get_diagonal_cells(self):
        """Get diagonal neighboring cells based on the car's last movement."""
        # Continue statement from geeksforgeeks: https://www.geeksforgeeks.org/python/python-continue-statement/

        # Get valid neighbors
        valid_neighbors = self.cell.neighborhood.select(
            lambda cell: not any(isinstance(obj, Obstacle) for obj in cell.agents)
        )

        # Get neighbors without cars or ambulances
        no_obstacles = valid_neighbors.select(
            lambda cell: not any(
                isinstance(obj, CarAgent)
                or isinstance(obj, Ambulance)
                or isinstance(obj, Traffic_Light)
                for obj in cell.agents
            )
        )

        # Filter only diagonal cells
        diagonal_cells = [
            cell for cell in no_obstacles
            if 
            abs(cell.coordinate[0] - self.cell.coordinate[0]) == 1
            and abs(cell.coordinate[1] - self.cell.coordinate[1]) == 1
        ]

        return diagonal_cells
    
    def move(self, next_cell):
        """Moves the agent to the next cell."""
        if next_cell is not None:
            self.cell = next_cell
    
    def crash(self):
        """If a car crashes with another car or ambulance, we remove them."""
        actual_cell = self.cell
        agents_to_remove = []
        
        for agent in actual_cell.agents:
            # If the agent in the cell is another car or ambulance
            if isinstance(agent, CarAgent) and agent != self:
                agents_to_remove.append(agent)
            elif isinstance(agent, Ambulance):
                agents_to_remove.append(agent)
        
        # If the car crashed with other cars or ambulances, add this car too
        if agents_to_remove:
            agents_to_remove.append(self)
        
        # Remove all agents in the list
        for agent in agents_to_remove:
            self.model.grid.remove_agent(agent)
    
    def step(self):
        """
        Executes one step of the car's behavior.
        
        Possible states:
        - idle: Initial state, ready to move
        - waitingCar: Waiting for the car ahead to move
        - waitingTL: Waiting at the traffic light
        - moving: Moving to the next cell
        """

        # Check if we reached destination
        self.checkDestination()

        # Initialize destination and path if not already done
        if self.destination is None:
            self.pathToDestination()
        
        # If no path exists, cannot move
        if not self.path:
            self.remove()  # Remove car from the model
            return
        
        # First check if we are inside a traffic light cell
        current_traffic_light = next(
            (obj for obj in self.cell.agents if isinstance(obj, Traffic_Light)), None
        )
        if current_traffic_light and not current_traffic_light.is_green:
            self.state = "waitingTL"
            return

        # Check current state and decide next action
        if self.state == "idle":
            # Check for ambulance first
            next_cell = self.checkAmbulance()
            if next_cell is None:
                return

            # Check what's ahead
            next_cell = self.getNextReturnStep()

            # If state is moving, move
            if self.state == "moving" and next_cell:
                self.path.pop(0)  # Remove the first step as we are moving there
                self.move(next_cell)
                self.state = "idle"
        
        elif self.state == "waitingCar":
            # Check for ambulance first
            next_cell = self.checkAmbulance()
            if next_cell is None:
                return
            # Check again if car moved
            next_cell = self.getNextReturnStep()
            
            if self.state == "moving" and next_cell:
                self.path.pop(0)
                self.move(next_cell)
                self.state = "idle"
        
        elif self.state == "waitingTL":
            # Check for ambulance first
            next_cell = self.checkAmbulance()
            if next_cell is None:
                return
            
            # Get next cell again
            next_cell = self.getNextReturnStep()

            if self.state == "moving" and next_cell:
                self.path.pop(0)
                self.move(next_cell)
                self.state = "idle"

class Ambulance(VehicleAgent):
    """
    Ambulance that moves with priority over other vehicles.
    If in emergency state, traffic lights
    """
    def __init__(self, model, cell):
        """
        Creates a new random agent.
        Args:
            model: Model reference for the agent
            cell: The initial position of the agent
        """
        super().__init__(model, cell)
        self.has_emergency = False
        self.state = "idle"
        self.moved_for_ambulance = False  # Track if already moved to let ambulance pass
        self.moved_diagonally = False  # Track if ambulance moved diagonally this step

        # Siren light attributes
        self.light = False # Wether the siren light is on or off
        self.light_state = "red"  # Initial light state
        self.light_timer = 0  # Timer for light switching
        self.light_interval = 2  # Steps between light switches

        # Determine initial state
        random_value = random.random()
        if random_value < self.model.emergency_chance:
            self._has_emergency = True
            self.state = "emergency"
            self.light = True  # Turn siren light on
        else:
            self._has_emergency = False
            self.state = "idle"
        
        # Initialize lastDirection from the road in the current cell
        self.lastDirection = None
        for agent in cell.agents:
            if isinstance(agent, Road):
                self.lastDirection = agent.direction
                break
        
        self.moved_diagonally = False  # Track if ambulance moved diagonally this step
        print(f"ID: {self.unique_id} Ambulance state: {self.state}")
    
    @property
    def has_emergency(self):
        """Whether the ambulance has an emergency."""
        return self._has_emergency
    
    @has_emergency.setter
    def has_emergency(self, value: bool) -> None:
        """Set emergency status and update state accordingly."""
        self._has_emergency = value
        self.state = "emergency" if value else "idle"
    
    def assignRandomHospital(self):
        """Assigns a random hospital from available Hospital agents."""
        # Get all Hospital agents from the model
        hospitals = [
            agent for agent in self.model.agents 
            if isinstance(agent, Hospital)
        ]
        
        if hospitals:
            return random.choice(hospitals)
        
        return None
    
    def pathToHospital(self):
        """Initialize hospital destination and calculate path using A*."""
        if self.destination is None:
            self.destination = self.assignRandomHospital()

            if self.destination:
                start = self.cell.coordinate
                goal = self.destination.cell.coordinate
                print(f"\n>>> Ambulance at {start} assigned hospital {goal}")
                self.path = self.a_star(start, goal)

                if self.path:
                    print(f">>> Path found with {len(self.path)} steps")
                else:
                    print(f">>> NO PATH from {start} to {goal}")

                self.path_index = 0
    
    def checkDestination(self):
        """Checks if the car has reached its destination."""
        for agent in self.cell.agents:
            if isinstance(agent, Hospital):
                self.light = False  # Turn off siren light
                self.remove()  # Remove car from the model

    def checkAmbulance(self):
        """Chooses the next cell based on the state of ambulances."""
        # Continue statement from geeksforgeeks: https://www.geeksforgeeks.org/python/python-continue-statement/
        # This method was obtained from Mesa API Documentation
        # https://mesa.readthedocs.io/latest/apis/discrete_space.html#mesa.discrete_space.__init__.OrthogonalMooreGrid
        neighbor_cells = self.cell.get_neighborhood(
            radius=3,  # Increased radius to detect ambulances earlier
            include_center=False
        )
        ambulance_nearby = False
    
        for neighbor_cell in neighbor_cells:
            ambulance = next(
                (obj for obj in neighbor_cell.agents if isinstance(obj, Ambulance)), None
            )

            # If an ambulance has been detected in emergency, clear the path
            if ambulance and ambulance.state == "emergency":
                ambulance_nearby = True
                # Check if the ambulance already moved diagonally this step
                # This method was obtained from Geekforgeeks
                # https://www.geeksforgeeks.org/python/python-hasattr-method/
                if hasattr(ambulance, 'moved_diagonally') and ambulance.moved_diagonally:
                    # Ambulance has already moved on its own, stay in place to avoid crashing or blocking 
                    return None
                
                # Check if this car is really blocking the ambulance's path
                if not self.isBlockingAmbulance(ambulance):
                    # Not blocking, continue normally
                    continue
                
                # Otherwise, if we're blocking and haven't moved yet
                if not self.moved_for_ambulance:
                    # Get diagonal cells based on the actual direction of the car
                    diagonal_cells = self.get_diagonal_cells()
                    
                    if diagonal_cells:
                        # Move to a random diagonal cell that is valid
                        valid_neighbors = []
                        for cell in diagonal_cells:
                            # Must have road
                            has_road = any(isinstance(obj, Road) for obj in cell.agents)
                            if not has_road:
                                # Since we want to append the valid cells only, if there is no road it's invalid, so we skip it
                                continue
                            
                            # Cannot have obstacles, cars, or ambulances
                            has_obstacle = any(isinstance(obj, Obstacle) for obj in cell.agents)
                            has_car = any(isinstance(obj, CarAgent) for obj in cell.agents)
                            has_ambulance_obj = any(isinstance(obj, Ambulance) for obj in cell.agents)
                            
                            if has_obstacle or has_car or has_ambulance_obj:
                                # Since we want to append the valid cells only, if there is any of these it's invalid, so we skip it
                                continue
                            
                            # Check traffic light
                            traffic_light = next(
                                (obj for obj in cell.agents if isinstance(obj, Traffic_Light)), None
                            )
                            
                            if traffic_light and not traffic_light.is_green:
                                # Since we want to append the valid cells only, if the light is red it's invalid, so we skip it
                                continue
                            
                            # Cell is valid
                            valid_neighbors.append(cell)
                        
                        if valid_neighbors:
                            new_cell = random.choice(valid_neighbors)
                            self.move(new_cell)
                            self.moved_for_ambulance = True
                            self.state = "idle"
                            return None
                    # If no valid diagonal cells, stay in place to let ambulance pass
                    return None
                else:
                    # Already moved for ambulance, stay until it passes
                    return None
        
        # No ambulance nearby reset flag to false
        if not ambulance_nearby:
            self.moved_for_ambulance = False
        
        # No ambulance in emergency nearby continue
        return self.getNextCell()

    def isBlockingAmbulance(self, ambulance):
        """
        Checks if this car is blocking the path of the given ambulance.
            
        """
        # First check if ambulance is in emergency state
        if not hasattr(ambulance, 'state') or ambulance.state != "emergency":
            return False
        
        # Get ambulance position and direction
        amb_x, amb_y = ambulance.cell.coordinate
        amb_direction = ambulance.lastDirection
        
        # If ambulance has no direction yet, get it from its current road
        if amb_direction is None:
            for agent in ambulance.cell.agents:
                if isinstance(agent, Road):
                    amb_direction = agent.direction
                    break
        
        if amb_direction is None:
            return False
        
        # Get car position
        car_x, car_y = self.cell.coordinate
        
        # Check if car is directly in front of ambulance based on ambulance's direction
        if amb_direction == "Up":
            # Car must be directly above ambulance (same x, higher y) 
            # We use the range of 4 cells ahead
            if car_x == amb_x and car_y > amb_y and car_y <= amb_y + 4:
                return True
        elif amb_direction == "Down":
            # Car must be directly below ambulance (same x, lower y)
            # We use the range of 4 cells ahead
            if car_x == amb_x and car_y < amb_y and car_y >= amb_y - 4:
                return True
        elif amb_direction == "Right":
            # Car must be directly to the right of ambulance (same y, higher x)
            # We use the range of 4 cells ahead
            if car_y == amb_y and car_x > amb_x and car_x <= amb_x + 4:
                return True
        elif amb_direction == "Left":
            # Car must be directly to the left of ambulance (same y, lower x)
            # We use the range of 4 cells ahead
            if car_y == amb_y and car_x < amb_x and car_x >= amb_x - 4:
                return True
        
        return False
    
    def getNextCell(self, cell = None):
        """Gets the next cell to move to based on current position."""
        # Get the Road agent in current cell
        if cell is None:
            cell = self.cell

        # Get the road agent in current cell to extract direction
        road_agent = next(
            (agent for agent in self.cell.agents if isinstance(agent, Road)), None
        )
        
        if road_agent is None:
            return None
            
        direction = road_agent.direction

        # Get next cell based on current direction
        if direction == "Up":
            next_coord = (self.cell.coordinate[0], self.cell.coordinate[1] + 1)
        elif direction == "Down":
            next_coord = (self.cell.coordinate[0], self.cell.coordinate[1] - 1)
        elif direction == "Left":
            next_coord = (self.cell.coordinate[0] - 1, self.cell.coordinate[1])
        elif direction == "Right":
            next_coord = (self.cell.coordinate[0] + 1, self.cell.coordinate[1])
        else:
            return None

        # Get the next cell from grid
        next_cell = self.model.grid[next_coord]
        
        # Verify next cell has a road and no obstacle
        has_road = any(isinstance(agent, Road) for agent in next_cell.agents)
        has_obstacle = any(isinstance(agent, Obstacle) for agent in next_cell.agents)
        
        if has_road and not has_obstacle:
            return next_cell
        
        return None

    def get_diagonal_cells(self):
        """Get diagonal neighboring cells based on the car's last movement."""
        # Continue statement from geeksforgeeks: https://www.geeksforgeeks.org/python/python-continue-statement/

        x, y = self.cell.coordinate
        diagonal_coords = []
        
        # Get direction
        direction = self.lastDirection
        if direction is None:
            # If we don't have a direction yet, get it from the current road
            for agent in self.cell.agents:
                if isinstance(agent, Road):
                    direction = agent.direction
                    break

        if direction == "Up":
            diagonal_coords = [(x - 1, y + 1), (x + 1, y + 1)] # Up left and right
        elif direction == "Down":
            diagonal_coords = [(x - 1, y - 1), (x + 1, y - 1)] # Down left and right
        elif direction == "Right":
            diagonal_coords = [(x + 1, y + 1), (x + 1, y - 1)] # Right up and down
        elif direction == "Left":
            diagonal_coords = [(x - 1, y - 1), (x - 1, y + 1)] # Left up and down

        # Get grid dimensions
        grid_width = self.model.grid.width
        grid_height = self.model.grid.height

        diagonal_cells = []
        for coord in diagonal_coords:
            coord_x, coord_y = coord
            # Check if coordinate is within grid bounds
            if coord_x < 0 or coord_y < 0 or coord_x >= grid_width or coord_y >= grid_height:
                # Coordinate is out of bounds we skip it
                continue
            
            # Coordinate is valid get the cell
            cell = self.model.grid[coord]
            diagonal_cells.append(cell)

        return diagonal_cells
    
    def inEmergency(self):
        """If the ambulance is in emergency, and there's a car or another ambulance in emergency waiting directly in front, the ambulance moves diagonally to pass."""
        # Get the cell directly in front based on direction
        next_cell = self.getNextCell()
        
        if next_cell is None:
            return False
        
        # Check if there's a car or ambulance waiting directly in front
        car_waiting_in_front = False
        for agent in next_cell.agents:
            # Check for cars waiting (any state means they're in the way)
            if isinstance(agent, CarAgent):
                car_waiting_in_front = True
                break
            # Check for other ambulances in emergency
            if isinstance(agent, Ambulance) and agent != self:
                if hasattr(agent, 'state') and agent.state == "emergency":
                    car_waiting_in_front = True
                    break
        
        # Only move diagonally if there's an obstacle directly in front
        if car_waiting_in_front:
            diagonal_cells = self.get_diagonal_cells()
            
            if diagonal_cells:
                # Filter valid diagonal cells
                valid_neighbors = []
                for cell in diagonal_cells:
                    # Must have road
                    has_road = any(isinstance(obj, Road) for obj in cell.agents)
                    if not has_road:
                        continue
                    
                    # Check for any vehicle or obstacle in the cell
                    has_obstacle = any(isinstance(obj, Obstacle) for obj in cell.agents)
                    has_car = any(isinstance(obj, CarAgent) for obj in cell.agents)
                    has_ambulance_obj = any(isinstance(obj, Ambulance) for obj in cell.agents)
                    
                    # Only accept if cell is completely empty of vehicles/obstacles
                    if has_obstacle or has_car or has_ambulance_obj:
                        continue
                    
                    # Cell is valid
                    valid_neighbors.append(cell)
                
                # Move to a random valid diagonal cell if available
                if valid_neighbors:
                    new_cell = random.choice(valid_neighbors)
                    self.move(new_cell)
                    return True
        
        return False

    def checkCars(self):
        """Chooses the next cell based on the presence of cars.
        
        For ambulances in emergency state, cars don't block the path.
        For ambulances in idle state, follow normal traffic rules.
        """
        
        # Get next cell
        next_cell = self.getNextCell()

        if next_cell is None:
            return None

        # If in emergency state, only check for other emergency ambulances
        if self.state == "emergency":
            has_emergency_ambulance = any(
                isinstance(agent, Ambulance) and agent.state == "emergency" 
                for agent in next_cell.agents
            )
            if has_emergency_ambulance:
                self.state = "waitingCar"
                return None
            # Cars don't block emergency ambulances
            self.state = "moving"
            return next_cell
        
        # Normal (idle) ambulance behavior
        has_car = any(isinstance(agent, CarAgent) for agent in next_cell.agents)
        has_ambulance = any(isinstance(agent, Ambulance) for agent in next_cell.agents)
        
        if has_car or has_ambulance:
            self.state = "waitingCar"
            return None
        
        # Check if there's a traffic light
        has_traffic_light = any(isinstance(agent, Traffic_Light) for agent in next_cell.agents)
        
        if has_traffic_light:
            # If there's a traffic light, check it
            return self.checkTL(next_cell)
        
        # Path is clear, change state to moving
        self.state = "moving"
        return next_cell
    
    def checkTL(self, next_cell):
        """Chooses the next cell based on the state of the traffic light."""

        current = self.cell
        # Chek the actual cell traffic light first
        current_traffic_light = next(
            (obj for obj in self.cell.agents if isinstance(obj, Traffic_Light)), None
        )

        if current_traffic_light and not current_traffic_light.is_green:
            self.state = "waitingTL"
            return None
                
        # Check if the traffic light is green
        traffic_light = next(
            (obj for obj in next_cell.agents if isinstance(obj, Traffic_Light)), None
        )

        if traffic_light and not traffic_light.is_green:
            # Traffic light is red, wait
            self.state = "waitingTL"
            return None
        else:
            # Traffic light is green, can move
            self.state = "moving"
            return next_cell
    
    def move(self, next_cell):
        """Moves the agent to the next cell."""
        self.cell = next_cell
    
    def crash(self):
        """If an ambulance crashes with a car or another ambulance, we remove them."""
        actual_cell = self.cell
        agents_to_remove = []
        
        for agent in actual_cell.agents:
            # If the agent in the cell is a car or another ambulance
            if isinstance(agent, CarAgent):
                agents_to_remove.append(agent)
            elif isinstance(agent, Ambulance) and agent != self:
                agents_to_remove.append(agent)
        
        # If there are collisions with cars or ambulances, add this ambulance too
        if agents_to_remove:
            agents_to_remove.append(self)
        
        # Remove all agents in the list
        for agent in agents_to_remove:
            self.model.grid.remove_agent(agent)
    
    def getNextCellFromPath(self):
        """Get the next step using A* path, checking traffic lights and cars."""
        
        # Get the next cell from the A* path
        if not self.path:
            return None
            
        next_coord = self.path[0]
        next_cell = self.model.grid[next_coord]
        
        # Check if there's a car in the next cell
        has_car = any(isinstance(agent, CarAgent) for agent in next_cell.agents)
        
        if has_car:
            self.state = "waitingCar"  # Change state to waiting for car
            return None
        
        # Check if there's a traffic light
        has_traffic_light = any(isinstance(agent, Traffic_Light) for agent in next_cell.agents)
        
        if has_traffic_light:
            # If there's a traffic light, get it and check its state
            traffic_light = next(
                (obj for obj in next_cell.agents if isinstance(obj, Traffic_Light)), None
            )
            
            if traffic_light and not traffic_light.is_green:
                # Traffic light is red, wait
                self.state = "waitingTL"
                return None
            else:
                # Traffic light is green, can move
                self.state = "moving"
                return next_cell
        
        # Path is clear, change state to moving
        self.state = "moving"
        return next_cell

    def updateSirenLight(self):
        """Updates the siren light state."""
        self.light_timer += 1

        if self.light_timer >= self.light_interval:
            # Switch light state
            if self.light_state == "red":
                self.light_state = "blue"
            else:
                self.light_state = "red"
            self.light_timer = 0  # Reset timer

    def step(self):
        """
        Executes one step of the ambulance's behavior using A* pathfinding.
        
        Possible states:
        - idle: Initial state, ready to move
        - waitingCar: Waiting for the car ahead to move
        - waitingTL: Waiting at the traffic light
        - moving: Moving to the next cell
        - emergency: Moving regardless of traffic rules
        """

        # Reset the flag at each step
        self.moved_diagonally = False

        # Check if we reached destination
        self.checkDestination()

        # Initialize destination and path if not already done
        if self.destination is None:
            self.pathToHospital()
        
        # If no path exists, cannot move
        if not self.path:
            self.remove()  # Remove ambulance from the model
            return

        # If the ambulance is on emergency, move without checking traffic lights
        if self.state == "emergency":

            self.updateSirenLight()

            # Get next cell from A* path, ignoring traffic lights
            if self.path:
                next_coord = self.path[0]
                next_cell = self.model.grid[next_coord]
                
                # Check for obstacles, cars, and other ambulances
                has_obstacle = any(isinstance(agent, Obstacle) for agent in next_cell.agents)
                has_car = any(isinstance(agent, CarAgent) for agent in next_cell.agents)
                has_any_ambulance = any(
                    isinstance(agent, Ambulance) and agent != self
                    for agent in next_cell.agents
                )
                
                # Can only move if path is completely clear of vehicles and obstacles
                if not has_obstacle and not has_car and not has_any_ambulance:
                    self.path.pop(0)
                    self.move(next_cell)
                elif has_car or has_any_ambulance:
                    # If blocked by car or ambulance, try to move diagonally
                    if self.inEmergency():
                        self.moved_diagonally = True
                    else:
                        # If can't move diagonally, wait in place
                        self.state = "waitingCar"
            return

        # Check current state and decide next action
        if self.state == "idle":
            # Check for ambulance first
            next_cell = self.checkAmbulance()
            if next_cell is None:
                return

            # Check what's ahead using the A* path
            next_cell = self.getNextCellFromPath()

            # If state is moving, move
            if self.state == "moving" and next_cell:
                self.path.pop(0)  # Remove the first step as we are moving there
                self.move(next_cell)
                self.state = "idle"
        
        elif self.state == "waitingCar":
            # Check for ambulance first
            next_cell = self.checkAmbulance()
            if next_cell is None:
                return
            # Check again if car moved
            next_cell = self.getNextCellFromPath()
            
            if self.state == "moving" and next_cell:
                self.path.pop(0)
                self.move(next_cell)
                self.state = "idle"
        
        elif self.state == "waitingTL":
            # Check for ambulance first
            next_cell = self.checkAmbulance()
            if next_cell is None:
                return
            
            # Get next cell again
            next_cell = self.getNextCellFromPath()

            if self.state == "moving" and next_cell:
                self.path.pop(0)
                self.move(next_cell)
                self.state = "idle"

class Traffic_Light(FixedAgent):
    """
    Traffic light. Where the traffic lights are in the grid.
    """
    def __init__(self, model, cell, state = False, timeToChange = 10):
        """
        Creates a new Traffic light.
        Args:
            model: Model reference for the agent
            cell: The initial position of the agent
            state: Whether the traffic light is green or red
            timeToChange: After how many step should the traffic light change color 
        """
        super().__init__(model)
        self.cell = cell
        self.state = state
        self.timeToChange = timeToChange

    def step(self):
        """ 
        To change the state (green or red) of the traffic light in case you consider the time to change of each traffic light.
        """
        if self.model.steps % self.timeToChange == 0:
            self.state = not self.state

    @property
    def is_green(self):
        """Whether the traffic light is green."""
        return self.state
    
    @is_green.setter
    def is_green(self, value: bool) -> None:
        """Set traffic light state."""
        self.state = value

class Destination(FixedAgent):
    """
    Car destination agent. Where each car should go.
    """
    def __init__(self, model, cell):
        """
        Creates a new car destination agent
        Args:
            model: Model reference for the agent
            cell: The initial position of the agent
        """
        super().__init__(model)
        self.cell = cell

class Hospital(FixedAgent):
    """
    Hospital agent. Represents hospital buildings.
    """
    def __init__(self, model, cell):
        """
        Creates a new hospital.
        
        Args:
            model: Model reference for the agent
            cell: The initial position of the agent
        """
        super().__init__(model)
        self.cell = cell

class Destination(FixedAgent):
    """
    Destination agent. Represents destination buildings.
    """
    def __init__(self, model, cell):
        """
        Creates a new destination.
        
        Args:
            model: Model reference for the agent
            cell: The initial position of the agent
        """
        super().__init__(model)
        self.cell = cell

class Obstacle(FixedAgent):
    """
    Obstacle agent. Represents normal buildings that block the path.
    """
    def __init__(self, model, cell):
        """
        Creates a new obstacle.
        
        Args:
            model: Model reference for the agent
            cell: The initial position of the agent
        """
        super().__init__(model)
        self.cell = cell

class Road(FixedAgent):
    """
    Road agent. Determines where the cars can move, and in which direction.
    """
    def __init__(self, model, cell, direction= "Left"):
        """
        Creates a new road.
        Args:
            model: Model reference for the agent
            cell: The initial position of the agent
        """
        super().__init__(model)
        self.cell = cell
        self.direction = direction

class SideWalk(FixedAgent):
    """
    Sidewalk agent. Represents sidewalk areas.
    """
    def __init__(self, model, cell):
        """
        Creates a new sidewalk.
        
        Args:
            model: Model reference for the agent
            cell: The initial position of the agent
        """
        super().__init__(model)
        self.cell = cell