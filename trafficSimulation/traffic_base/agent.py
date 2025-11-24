from mesa.discrete_space import CellAgent, FixedAgent

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

    def step(self):
        """
        Executes one step of the car's behavior.
        
        Possible states:
        - idle: Initial state, ready to move
        - waitingCar: Waiting for the car ahead to move
        - waitingTL: Waiting at the traffic light
        """

        # Try to get the next valid cell
        next_cell = self.getNextCell()
        
        if next_cell is None:
            # Can't move anywhere
            self.state = "idle"
            return
        
        # Check if there's a car blocking
        has_car = any(isinstance(agent, CarAgent) for agent in next_cell.agents)
        if has_car:
            self.state = "waitingCar"
            return
        
        # Check if there's a traffic light
        has_traffic_light = any(isinstance(agent, Traffic_Light) for agent in next_cell.agents)
        if has_traffic_light:
            # Get the traffic light
            traffic_light = next((agent for agent in next_cell.agents if isinstance(agent, Traffic_Light)), None)
            if traffic_light and not traffic_light.is_green:
                # Red light, wait
                self.state = "waitingTL"
                return
        
        # Path is clear, move to the next cell
        self.move(next_cell)
        self.state = "idle"

    def getNextCell(self):
        """Gets the next cell to move to based on current position."""
        
        # Get the Road agent in current cell
        current_road = None
        for agent in self.cell.agents:
            if isinstance(agent, Road): 
                current_road = agent
                break
        
        if not current_road:
            return None  # No road found, cannot move

        # Get next cell based on road direction
        x, y = self.cell.coordinate
        direction = current_road.direction
        
        if direction == "Up":
            next_coord = (x, y + 1)
        elif direction == "Down":
            next_coord = (x, y - 1)
        elif direction == "Left":
            next_coord = (x - 1, y)
        elif direction == "Right":
            next_coord = (x + 1, y)
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
        # Select valid neighboring cells (without obstacles)

        #First gets next cell
        next_cell = self.getNextCell()

        if next_cell is None:
            return None

        # Check if there's a car in the next cell
        has_car = any(isinstance(agent, CarAgent) for agent in next_cell.agents)
        
        if has_car:
            return None
        else:
            # Check if there's a traffic light
            has_traffic_light = any(isinstance(agent, Traffic_Light) for agent in next_cell.agents)
            
            if has_traffic_light:
                return self.checkTL(next_cell)
            else: 
                return next_cell
    
    def checkTL(self, next_cell):
        """Chooses the next cell based on the state of the traffic light."""
        # Check if the traffic light is green
        TL_cell = next(
            (obj for obj in next_cell.agents if isinstance(obj, Traffic_Light)), None
        )
        
        if TL_cell and not TL_cell.is_green:
            # Traffic light is red, wait
            return None
        else:
            # Traffic light is green or no traffic light, can move
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
