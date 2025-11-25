from mesa.discrete_space import CellAgent, FixedAgent
import heapq
import random

class CarAgent(CellAgent):
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
        super().__init__(model)
        self.cell = cell
        self.state = "idle"  # Initial state
        self.lastDirection = None  # To keep track of last movement direction
        self.destination = None  # Will be assigned on first step
        self.path = []  # List of coordinates to follow
        
    def assignRandomDestination(self):
        """Assigns a random destination from available Destination agents."""
        # Get all Destination agents from the model
        destinations = [agent for agent in self.model.agents if isinstance(agent, Destination)]
        
        if destinations:
            return random.choice(destinations)
        return None
    
    def pathToDestination(self):
        """Initialize destination and calculate path using A*."""
        if self.destination is None:
            self.destination = self.assignRandomDestination()
            if self.destination:
                self.path = self.a_star(self.cell.coordinate, self.destination.cell.coordinate)
                self.path_index = 0

    def getNextReturnStep(self):
        """Get the next step to return to the station using A* algorithm."""
        if not self.path:
            self.pathToDestination() # Calculate path if not already done

        if self.path:
            next_coord = self.path.pop(0) # Get next coordinate in path and remove it from the list
            next_cell = self.model.grid[next_coord]
            self.state = "moving"  #change state to moving when path exists
            return next_cell
        else:
            self.state = "idle"  #change state to idle if no path to destination
            return None

    def step(self):
        """
        Executes one step of the car's behavior.
        
        Possible states:
        - idle: Initial state, ready to move 
        - waitingCar: Waiting for the car ahead to move
        - waitingTL: Waiting at the traffic light
        - moving: Moving to the next cell
        """
        
        # Safety check: if cell is None, skip this step
        if self.cell is None:
            return

        # Check current state and decide next action
        if self.state == "idle":
            # Check what's ahead
            next_cell = self.checkCars()

            # If state is now "moving", execute the move
            if self.state == "moving" and next_cell:
                next_cell = self.getNextReturnStep()
                self.move(next_cell)
                self.state = "idle"
        
        elif self.state == "waitingCar":
            # Check again if car moved
            next_cell = self.checkCars()
            if self.state == "moving" and next_cell:
                self.move(next_cell)
                self.state = "idle"
        
        elif self.state == "waitingTL":
            # Check if light is green
            next_cell = self.checkCars()
            # If Traffic light is green, we can move
            if self.state == "moving" and next_cell:
                self.move(next_cell)
                self.state = "idle"
    
    def getNextCell(self):
        """Gets the next cell to move to based on current position."""
        
        # Get the Road agent in current cell
        current_road = None
        traffic_light_found = False
        # Set direction to None initially
        direction = None

        for agent in self.cell.agents:
            if isinstance(agent, Traffic_Light):
                traffic_light_found = True
                # If there's a traffic light, use the last known direction
                direction = self.lastDirection
                break
        
        if not traffic_light_found:
            for agent in self.cell.agents:
                if isinstance(agent, Road):
                    current_road = agent
                    # Get direction from the road
                    direction = current_road.direction
                    self.lastDirection = direction
                    break
        # If no road found, cannot move
        if direction is None:
            return None 

        # Get next cell based on road direction
        x, y = self.cell.coordinate
        
        if direction == "Up":
            next_coord = (x, y + 1)
        elif direction == "Down":
            next_coord = (x, y - 1)
        elif direction == "Left":
            next_coord = (x - 1, y)
        elif direction == "Right":
            next_coord = (x + 1, y)
        else:
            return None  # Invalid direction

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

        # Check if there's a car in the next cell
        has_car = any(isinstance(agent, CarAgent) for agent in next_cell.agents)
        
        if has_car:
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
        
    def a_star(self, start, goal):
        """
        A* pathfinding algorithm adapted for the grid in the model

        This algorithm was adapted from the one made in the advanced algorithm class
        with Lizbeth Peralta. Made by Diego Cordova, Aquiba Benarroch and me.
        """

        # Calculate Manhattan distance
        # We have to estimate heuristic using this distance
        # since it is not given from the model
        # Ref: https://www.geeksforgeeks.org/dsa/a-search-algorithm/
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        def getNeighborsDirections(current_coord):
            current_cell = self.model.grid[current_coord]
            valid_neighbors = []

            current_road  = None
            for agent in current_cell.agents:
                if isinstance(agent, Road):
                    current_road = agent
                    break

            if current_road is None:
                return []
            
            direction = current_road.direction
            x, y = current_coord

            next_coords = []
            if direction == "Up":
                next_coords.append((x, y + 1))
            elif direction == "Down":
                next_coords.append((x, y - 1))
            elif direction == "Left":
                next_coords.append((x - 1, y))
            elif direction == "Right":
                next_coords.append((x + 1, y))

            for coord in next_coords:
                if coord not in self.model.grid:
                    continue

                neighbor_cell = self.model.grid[coord]


                # Check if neighbor cell is valid (not an obstacle)
                has_road = any(isinstance(agent, Road) for agent in neighbor_cell.agents)
                has_obstacle = any(isinstance(agent, Obstacle) for agent in neighbor_cell.agents)

                if has_road and not has_obstacle:
                    valid_neighbors.append(coord)

            return valid_neighbors

            
        # Initialize variables
        grid = self.model.grid
        stack = [] # Stack of nodes to explore
        c_list = {}  # g values
        visited = set()  # visited nodes

        # Father vector to reconstruct path
        fathers = {}

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
                    break

                # Get valid neighbors based on road directions
                valid_neighbors = getNeighborsDirections(current)

                # Explore neighbors
                # Get valid neighbors (not obstacles)       
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
    
    def move(self, next_cell):
        """Moves the agent to the next cell."""
        if next_cell is not None:
            self.cell = next_cell

class Ambulance(CellAgent):
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
        super().__init__(model)
        self.cell = cell
        self.state = "idle"  # Initial state
        self.lastDirection = None  # To keep track of last movement direction

    def step(self):
        """
        Executes one step of the car's behavior.
        
        Possible states:
        - idle: Initial state, ready to move
        - waitingCar: Waiting for the car ahead to move
        - waitingTL: Waiting at the traffic light
        - moving: Moving to the next cell
        - Emergency: Moving regardless of traffic rules
        """

        # Check current state and decide next action
        if self.state == "idle":
            # Check what's ahead
            next_cell = self.checkCars()

            # If state is moving, move
            if self.state == "moving" and next_cell:
                self.move(next_cell)
                self.state = "idle"
        
        elif self.state == "waitingCar":
            # Check again if car moved
            next_cell = self.checkCars()
            if self.state == "moving" and next_cell:
                self.move(next_cell)
                self.state = "idle"
        
        elif self.state == "waitingTL":
            # Check if light is green
            next_cell = self.checkCars()
            # If Traffic light is green, we can move
            if self.state == "moving" and next_cell:
                self.move(next_cell)
                self.state = "idle"

    def getNextCell(self):
        """Gets the next cell to move to based on current position."""
        
        # Get the Road agent in current cell
        current_road = None
        traffic_light_found = False
        # Set direction to None initially
        direction = None

        for agent in self.cell.agents:
            if isinstance(agent, Traffic_Light):
                traffic_light_found = True
                # If there's a traffic light, use the last known direction
                direction = self.lastDirection
                break
        
        if not traffic_light_found:
            for agent in self.cell.agents:
                if isinstance(agent, Road):
                    current_road = agent
                    # Get direction from the road
                    direction = current_road.direction
                    self.lastDirection = direction
                    break
        # If no road found, cannot move
        if direction is None:
            return None 

        # Get next cell based on road direction
        x, y = self.cell.coordinate
        
        if direction == "Up":
            next_coord = (x, y + 1)
        elif direction == "Down":
            next_coord = (x, y - 1)
        elif direction == "Left":
            next_coord = (x - 1, y)
        elif direction == "Right":
            next_coord = (x + 1, y)
        else:
            return None  # Invalid direction

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

        # Check if there's a car in the next cell
        has_car = any(isinstance(agent, CarAgent) for agent in next_cell.agents)
        
        if has_car:
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
    Destination agent. Where each car should go.
    """
    def __init__(self, model, cell):
        """
        Creates a new destination agent
        Args:
            model: Model reference for the agent
            cell: The initial position of the agent
        """
        super().__init__(model)
        self.cell = cell

class Obstacle(FixedAgent):
    """
    Obstacle agent. Just to add obstacles to the grid.
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
