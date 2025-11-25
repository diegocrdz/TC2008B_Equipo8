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
        
        neighbors = self.cell.neighborhood
        ambulance_nearby = False
    
        for neighbor_cell in neighbors:
            ambulance = next(
                (obj for obj in neighbor_cell.agents if isinstance(obj, Ambulance)), None
            )
        
            if ambulance and ambulance.state == "emergency":
                ambulance_nearby = True
                # If there's an ambulance in emergency state nearby and we haven't we need to let him pass
                if not self.moved_for_ambulance:
                    valid_neighbors = self.cell.neighborhood.select(
                        lambda cell: any(isinstance(obj, Road) for obj in cell.agents) and 
                        not any(isinstance(obj, Obstacle) 
                        or isinstance(obj, Ambulance) 
                        or isinstance (obj, Traffic_Light) 
                        or isinstance(obj, CarAgent) for obj in cell.agents)
                    ) 
                    if valid_neighbors:
                        new_cell = random.choice(list(valid_neighbors))
                        self.move(new_cell)
                        self.state = "idle"
                        self.moved_for_ambulance = True
                        return None  # Already moved
                else:
                    # Already moved for ambulance
                    return None
        
        # No ambulance nearby, reset flag to false
        if not ambulance_nearby:
            self.moved_for_ambulance = False
        
        # No ambulance in emergency nearby continue
        return self.getNextCell()   
        

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
        print(f"Ambulance state: {self.state}")

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

        # If the ambulance is on emergency, move without checking traffic lights
        # Checks for obstacles and cars only
        if self.state == "emergency":
            next_cell = self.getNextCell()
            if next_cell:
                # Check if there's a car or ambulance blocking
                has_car = any(isinstance(agent, CarAgent) for agent in next_cell.agents)
                has_ambulance = any(isinstance(agent, Ambulance) for agent in next_cell.agents)
                
                if not has_car and not has_ambulance:
                    self.move(next_cell)
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
        
        neighbors = self.cell.neighborhood
        ambulance_nearby = False
    
        for neighbor_cell in neighbors:
            ambulance = next(
                (obj for obj in neighbor_cell.agents if isinstance(obj, Ambulance)), None
            )
        
            if ambulance and ambulance.state == "emergency":
                ambulance_nearby = True
                # If there's an ambulance in emergency state nearby and we haven't we need to let him pass
                if not self.moved_for_ambulance:
                    valid_neighbors = self.cell.neighborhood.select(
                        lambda cell: any(isinstance(obj, Road) for obj in cell.agents) and 
                        not any(isinstance(obj, Obstacle) 
                        or isinstance(obj, Ambulance) 
                        or isinstance (obj, Traffic_Light) 
                        or isinstance(obj, CarAgent) for obj in cell.agents)
                    ) 
                    if valid_neighbors:
                        new_cell = random.choice(list(valid_neighbors))
                        self.move(new_cell)
                        self.state = "idle"
                        self.moved_for_ambulance = True
                        return None  # Already moved
                else:
                    # Already moved for ambulance
                    return None
        
        # No ambulance nearby, reset flag to false
        if not ambulance_nearby:
            self.moved_for_ambulance = False
        
        # No ambulance in emergency nearby continue
        return self.getNextCell() 

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
