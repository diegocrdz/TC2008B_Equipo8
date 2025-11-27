"""
Module that contains the agents for the traffic simulation model using the Mesa framework.

Diego Córdova Rodríguez
Lorena Estefanía Chewtat Torres
Aquiba Yudah Benarroch Bittán

2025-11-27
"""

from mesa.discrete_space import CellAgent, FixedAgent
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
        self.moved_for_ambulance = False  # Track if already moved to let ambulance pass

    def step(self):
        """
        Executes one step of the car's behavior.
        
        Possible states:
        - idle: Initial state, ready to move
        - waitingCar: Waiting for the car ahead to move
        - waitingTL: Waiting at the traffic light
        - moving: Moving to the next cell
        """

        # First check if we are inside a traffic light cell
        current_traffic_light = next(
            (obj for obj in self.cell.agents if isinstance(obj, Traffic_Light)), None
        )
        if current_traffic_light and not current_traffic_light.is_green:
            self.state = "waitingTL"
            return
        # Check current state and decide next action
        if self.state == "idle":
            # Check what's ahead
            next_cell = self.checkAmbulance()
            if next_cell is None:
                return
            next_cell = self.checkCars()

            # If state is moving, move
            if self.state == "moving" and next_cell:
                self.move(next_cell)
                self.state = "idle"
        
        elif self.state == "waitingCar":
            # Chek for ambulance first
            next_cell = self.checkAmbulance()
            if next_cell is None:
                return
            # Check again if car moved
            next_cell = self.checkCars()
            if self.state == "moving" and next_cell:
                self.move(next_cell)
                self.state = "idle"
        
        elif self.state == "waitingTL":
            # Check for ambulance first
            next_cell = self.checkAmbulance()
            if next_cell is None:
                return
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
    
    def checkAmbulance(self):
        """Chooses the next cell based on the state of ambulances."""
        # Continue statement from geeksforgeeks: https://www.geeksforgeeks.org/python/python-continue-statement/
        # This method was obtained from Mesa API Documentation
        # https://mesa.readthedocs.io/latest/apis/discrete_space.html#mesa.discrete_space.__init__.OrthogonalMooreGrid
        neighbor_cells = self.cell.get_neighborhood(
            radius=4,
            include_center=False

        )
        ambulance_nearby = False
    
        for neighbor_cell in neighbor_cells:
            ambulance = next(
                (obj for obj in neighbor_cell.agents if isinstance(obj, Ambulance)), None
            )
        
            if ambulance and ambulance.state == "emergency":
                ambulance_nearby = True
                # Check if the ambulance already moved diagonally this step
                # This method was obtained from Geekforgeeks
                # https://www.geeksforgeeks.org/python/python-hasattr-method/
                if hasattr(ambulance, 'moved_diagonally') and ambulance.moved_diagonally:
                    # Ambulance has already moved on its own, stay in place to avoid crashing or blocking 
                    return None
                # Otherwise, if there's an ambulance in emergency state nearby and we haven't moved yet
                if not self.moved_for_ambulance:
                    # Get diagonal cells based on the actual direction of the car
                    diagonal_cells = self.get_diagonal_cells()
                    
                    if diagonal_cells:
                        # Filter valid cells (with road, without obstacles/cars/ambulances)
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
                            self.state = "idle"
                            self.moved_for_ambulance = True
                            return None  # Already moved, stay in place
                    # If no valid diagonal cells do nothing
                    return None
                else:
                    # Already moved for ambulance, stay until it passes
                    return None
        
        # No ambulance nearby reset flag to false
        if not ambulance_nearby:
            self.moved_for_ambulance = False
        
        # No ambulance in emergency nearby continue moving normally
        return self.getNextCell()   
        
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
        self._has_emergency = random.choice([True, False])  # Random emergency state
        self.state = "emergency" if self._has_emergency else "idle"  # Initial state
        self.lastDirection = None  # To keep track of last movement direction
        self.moved_for_ambulance = False  # Track if already moved to let ambulance pass
        self.moved_diagonally = False  # Track if ambulance moved diagonally this step
        print(f"ID: {self.unique_id} Ambulance state: {self.state}")

    def step(self):
        """
        Executes one step of the car's behavior.
        
        Possible states:
        - idle: Initial state, ready to move
        - waitingCar: Waiting for the car ahead to move
        - waitingTL: Waiting at the traffic light
        - moving: Moving to the next cell
        - emergency: Moving regardless of traffic rules
        """

        # REset the flag at each step
        self.moved_diagonally = False

        # If the ambulance is on emergency, move without checking traffic lights, only cars, obstacles, or ambulances (not in emergency state)
        if self.state == "emergency":
            # If there is a car in front, the ambulance checks for his state and tries to move diagonally
            if self.inEmergency():
                self.moved_diagonally = True
                return
            
            # If inEmergency returned False it means that there was no car in front, so we can move normally
            next_cell = self.getNextCell()
            if next_cell:
                self.move(next_cell)
            return

        # If the ambulance is not on emergency, first check if we are inside a traffic light cell
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
            next_cell = self.checkCars()

            # If state is moving, move
            if self.state == "moving" and next_cell:
                self.move(next_cell)
                self.state = "idle"
        
        elif self.state == "waitingCar":
            # Chek for ambulance first
            next_cell = self.checkAmbulance()
            if next_cell is None:
                return
            # Check again if car moved
            next_cell = self.checkCars()
            if self.state == "moving" and next_cell:
                self.move(next_cell)
                self.state = "idle"
        
        elif self.state == "waitingTL":
            # Check for ambulance first
            next_cell = self.checkAmbulance()
            if next_cell is None:
                return
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

    def checkAmbulance(self):
        """Chooses the next cell based on the state of ambulances."""
        # Continue statement from geeksforgeeks: https://www.geeksforgeeks.org/python/python-continue-statement/
        # This method was obtained from Mesa API Documentation
        # https://mesa.readthedocs.io/latest/apis/discrete_space.html#mesa.discrete_space.__init__.OrthogonalMooreGrid
        neighbor_cells = self.cell.get_neighborhood(
            radius=4,
            include_center=False

        )
        ambulance_nearby = False
    
        for neighbor_cell in neighbor_cells:
            ambulance = next(
                (obj for obj in neighbor_cell.agents if isinstance(obj, Ambulance)), None
            )
        
            if ambulance and ambulance.state == "emergency":
                ambulance_nearby = True
                # Check if the ambulance already moved diagonally this step
                # This method was obtained from Geekforgeeks
                # https://www.geeksforgeeks.org/python/python-hasattr-method/
                if hasattr(ambulance, 'moved_diagonally') and ambulance.moved_diagonally:
                    # Ambulance moved on its own, don't move to avoid blocking it
                    return None
                # If there's an ambulance in emergency state nearby and we haven't moved yet
                if not self.moved_for_ambulance:
                    # Get diagonal cells based on the actual direction of the car
                    diagonal_cells = self.get_diagonal_cells()
                    
                    if diagonal_cells:
                        # Filter valid cells (with road, without obstacles/cars/ambulances)
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
                            
                            if traffic_light:
                                # If traffic light is red we can't move there we skip it
                                if not traffic_light.is_green:
                                    # Since we want to append the valid cells only, if the light is red it's invalid, so we skip it
                                    continue
                                # If green we can use this cell
                            
                            # Cell is valid
                            valid_neighbors.append(cell)
                        
                        if valid_neighbors:
                            new_cell = random.choice(valid_neighbors)
                            self.move(new_cell)
                            self.state = "idle"
                            self.moved_for_ambulance = True
                            return None  # Already moved, stay in place
                    # If no valid diagonal cells do nothing
                    return None
                else:
                    # Already moved for ambulance, stay until it passes
                    return None
        
        # No ambulance nearby reset flag to false
        if not ambulance_nearby:
            self.moved_for_ambulance = False
        
        # No ambulance in emergency nearby continue moving normally
        return self.getNextCell() 

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
        """If the ambulance is in emergency, and there's a car waiting directly in front, the ambulance moves diagonally to pass."""
        # Get the cell directly in front based on direction
        next_cell = self.getNextCell()
        
        if next_cell is None:
            return False
        
        # Check if there's a car or ambulance waiting directly in front
        car_waiting_in_front = False
        for agent in next_cell.agents:
            if isinstance(agent, CarAgent) or isinstance(agent, Ambulance):
                # Check if car or ambulance are waiting
                # This method was obtained from Geekforgeeks
                # https://www.geeksforgeeks.org/python/python-hasattr-method/
                if hasattr(agent, 'state') and agent.state in ["waitingCar", "waitingTL"]:
                    car_waiting_in_front = True
                    break
        
        # Only move diagonally if there's a car or ambulance waiting directly in front
        if car_waiting_in_front:
            diagonal_cells = self.get_diagonal_cells()
            
            if diagonal_cells:
                # Filter valid diagonal cells
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
                    
                    # Cell is valid
                    valid_neighbors.append(cell)
                
                # Move to a random valid diagonal cell if available
                if valid_neighbors:
                    new_cell = random.choice(valid_neighbors)
                    self.move(new_cell)
                    return True
        
        return False

    def checkCars(self):
        """Chooses the next cell based on the presence of cars."""
        
        # Get next cell
        next_cell = self.getNextCell()

        if next_cell is None:
            return None

        # Check if there's a car in the next cell
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

    @property
    def has_emergency(self):
        """Whether the ambulance has an emergency."""
        return self._has_emergency
    
    @has_emergency.setter
    def has_emergency(self, value: bool) -> None:
        """Set emergency status and update state accordingly."""
        self._has_emergency = value
        self.state = "emergency" if value else "idle"
 

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