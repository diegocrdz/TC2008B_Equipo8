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

        # Tolerance parameters for waiting
        self.tolerance_timer = 0  # Timer for tolerance to waiting
        self.max_tolerance = 5  # Max tolerance before recalculating path
        self.path_recalculated = False  # Flag to indicate if path was recalculated

        # Lane noise scale for changing lane costs in A*
        # This adds variability to lane selection
        self.lane_noice_scale = 0.3

        # Initialize lastDirection from the road in the current cell
        self.lastDirection = None
        road_agent = next(
            (agent for agent in cell.agents if isinstance(agent, Road)),
            None
        )
        if road_agent:
            self.lastDirection = road_agent.direction
        
    def move(self, next_cell):
        """Moves the agent to the next cell and updates lastDirection."""
        if next_cell is not None:
            self.cell = next_cell

            # Update lastDirection based on road in new cell
            road_agent = next(
                (agent for agent in next_cell.agents if isinstance(agent, Road)),
                None
            )
            if road_agent:
                self.lastDirection = road_agent.direction

    def checkDestination(self):
        """
        Checks if the vehicle has reached its destination.
        Handles both Destination (for cars) and Hospital (for ambulances).
        """
        for agent in self.cell.agents:
            # Check for Destination (cars)
            if isinstance(agent, Destination):
                self.model.cars_reached_destination += 1
                self.remove()
                return
            # Check for Hospital (ambulances)
            if isinstance(agent, Hospital):
                self.model.ambulances_reached_hospital += 1
                # Turn off siren light if ambulance has it
                if hasattr(self, 'light'):
                    self.light = False
                self.remove()
                return

    def crash(self):
        """If a vehicle crashes with another car or ambulance, we remove them."""
        actual_cell = self.cell
        agents_to_remove = []
        
        for agent in actual_cell.agents:
            # Detect collisions with other vehicles
            if isinstance(agent, CarAgent) and agent != self:
                agents_to_remove.append(agent)
            elif isinstance(agent, Ambulance) and agent != self:
                agents_to_remove.append(agent)
        
        # If there are collisions, remove all involved vehicles
        if agents_to_remove:
            agents_to_remove.append(self)
            self.model.total_crashes += 1
        
        # Remove all agents in the list
        for agent in agents_to_remove:
            self.model.grid.remove_agent(agent)

    def assignRandomDestination(self, destination_type='destination'):
        """Assigns a random destination from available Destination agents."""
        # Get all Destination agents from the model
        destinations = []

        if destination_type == 'hospital':
            destinations = [
                agent for agent in self.model.agents 
                if isinstance(agent, Hospital)
            ]
        else:
            destinations = [
                agent for agent in self.model.agents 
                if isinstance(agent, Destination)
            ]
        
        if destinations:
            return random.choice(destinations)
        
        return None

    def calculatePath(self, start, goal, find_closest=True):
        """
        Determines the path from start to goal.

        Args:
        - start: Starting coordinate
        - goal: Goal coordinate
        - find_closest: If True, finds the closest node to goal instead of exact goal.
        Returns:
        - List of coordinates representing the path.
        """

        path = []

        # Use A* to calculate path to goal
        if not path:
            path = self.a_star(start, goal, find_closest=find_closest)

        return path

    def a_star(self, start, goal, find_closest=True):
        """
        A* pathfinding algorithm adapted for the grid in the model

        This algorithm was adapted from the one made in the advanced algorithm class
        with Lizbeth Peralta. Made by Diego Cordova, Aquiba Benarroch and Lorena Chewtat.
        
        Args:
            start: Starting coordinate
            goal: Goal coordinate
            find_closest: If True, finds the closest node to goal instead of exact goal.
            Useful for recalculation to distribute traffic across alternative routes.
        """

        # Calculate Manhattan distance
        # We have to estimate heuristic using this distance
        # since it is not given from the model
        # Ref: https://www.geeksforgeeks.org/dsa/a-search-algorithm/
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
        def getValidNeighbors(current_coord):
            """Get valid neighboring respecting road directions."""
            # Arbitrary high cost for invalid neighbors
            NEIGHBOR_C = 5
            INTERSECTION_C = 2
            valid_neighbors = []
            neighbor_extra_cost = {}

            # If the current coordinate is not a road, return empty
            if (
                current_coord not in self.model.road_cells
                and current_coord not in self.model.destination_cells
                and current_coord not in self.model.hospital_cells
            ):
                return [], 0

            direction = self.model.road_directions.get(current_coord)
            if direction is None:
                return [], 0
            
            x, y = current_coord
            
            # Get neighbors
            neighbor_coords = [
                (x, y + 1),  # Up
                (x, y - 1),  # Down
                (x - 1, y),  # Left
                (x + 1, y)   # Right
            ]
            
            # Opposite directions to avoid
            opposites = {
                "Up": "Down",
                "Down": "Up",
                "Left": "Right",
                "Right": "Left"
            }
            opposite = opposites.get(direction)
            
            # Check each neighbor
            for nx, ny in neighbor_coords:
                extra_cost = 0

                # Check bounds
                if not (0 <= nx < self.model.grid.width and 0 <= ny < self.model.grid.height):
                    continue

                neighbor_coord = (nx, ny)

                # Check obstacles
                if neighbor_coord in self.model.obstacle_cells:
                    continue

                # Check any vehicles
                has_vehicle = any(
                    agent for agent in self.model.grid[neighbor_coord].agents
                    if isinstance(agent, (CarAgent, Ambulance))
                )
                if has_vehicle:
                    extra_cost += NEIGHBOR_C
                
                # Check for intersections
                if neighbor_coord in self.model.intersection_cells:
                    extra_cost += INTERSECTION_C

                # Check if the road is traversable
                if not (
                    neighbor_coord in self.model.road_cells
                    or neighbor_coord in self.model.destination_cells
                    or neighbor_coord in self.model.hospital_cells
                ):
                    continue
                
                # Get the direction of movement from current to neighbor
                move_dir = None
                if nx == x and ny == y + 1:
                    move_dir = "Up"
                elif nx == x and ny == y - 1:
                    move_dir = "Down"
                elif nx == x - 1 and ny == y:
                    move_dir = "Left"
                elif nx == x + 1 and ny == y:
                    move_dir = "Right"
                else:
                    # If not a direct neighbor, skip
                    continue
                
                # Allow any direction except the opposite (backwards)
                if move_dir == opposite:
                    continue
                
                # If all checks passed, add to valid neighbors
                valid_neighbors.append(neighbor_coord)
                neighbor_extra_cost[neighbor_coord] = extra_cost

            return valid_neighbors, neighbor_extra_cost

        # Initialize variables
        stack = [] # Stack of nodes to explore
        c_list = {}  # g values
        visited = set()  # visited nodes
        fathers = {} # Father vector to reconstruct path
        closest_node = start  # Track closest node to goal
        closest_distance = heuristic(start, goal)

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

                # Track closest node to goal (for find_closest mode)
                distance_to_goal = heuristic(current, goal)
                if distance_to_goal < closest_distance:
                    closest_distance = distance_to_goal
                    closest_node = current

                # If we reached the goal, finish
                if (current == goal):
                    #print("Goal reached!")
                    break

                # Get valid neighbors based on road directions
                valid_neighbors, neighbor_extra_cost = getValidNeighbors(current)

                # Explore neighbors
                for neighbor in valid_neighbors:
                    base_cost = 1

                    # Add small noise to cost to diversify paths
                    lane_noise = self.random.uniform(0, self.lane_noice_scale)
                    extra_cost = neighbor_extra_cost.get(neighbor, 0)

                    # Get step cost
                    step_cost = base_cost + lane_noise + extra_cost
                    c_new = c_list[current] + step_cost

                    if c_new < c_list.get(neighbor, float('inf')):
                        c_list[neighbor] = c_new
                        fathers[neighbor] = current
                        f_value = c_new + heuristic(neighbor, goal)

                        # Add to stack
                        heapq.heappush(stack, (f_value, neighbor))

        # Reconstruct path
        path = []
        # Determine each point to track from
        # If goal not reached, return empty path
        if goal not in fathers and goal != start:
            current = closest_node
        else:
            current = goal
        
        # If start is goal, return empty path
        if current == start:
            return []  # No path needed
        
        # Reconstruct path backwards
        while current != start:
            path.append(current)
            current = fathers.get(current)
            if current is None:
                return []  # No path found
        path.reverse()
        return path
    
    def pathToDestination(self, agent_type = 'car'):
        """Initialize destination and calculate path using A*."""
        if self.destination is None:
            if (agent_type == 'ambulance'):
                self.destination = self.assignRandomDestination('hospital')
            else:
                self.destination = self.assignRandomDestination('destination')

            if self.destination:
                start = self.cell.coordinate
                goal = self.destination.cell.coordinate
                # Recalculate path to destination
                self.path = self.calculatePath(start, goal, find_closest=True)
                self.path_index = 0
    
    def reduceWaitingTimer(self, agent_type='car'):
        """Reduces the waiting tolerance timer if greater than zero."""
        if self.tolerance_timer >= 0:
            self.tolerance_timer += 1
        if self.tolerance_timer >= self.max_tolerance:
            #Recalculate A* path if timer expired - find closest node to destination
            self.tolerance_timer = 0
            self.path = []

            if self.destination:
                start = self.cell.coordinate
                goal = self.destination.cell.coordinate
                # Use find_closest=True to find route to closest accessible 
                # node instead of exact destination
                # This helps distribute traffic across alternative routes
                self.path = self.calculatePath(start, goal, find_closest=True)
                self.path_index = 0

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
        
        # Parameters for lane changing
        self.lane_change_cooldown = 4 # Cooldown steps between lane changes
        self.lane_change_steps_since = 999 # Steps since last lane change, starts high to allow immediate change
        self.preferred_lane_side = self.random.choice(["left", "right"]) # Assign random lane preference

    def getNextReturnStep(self):
        """Get the next step using A* path, checking traffic lights and cars."""

        # If no path, return None
        if not self.path:
            return None
        
        # Get the next cell from the A* path
        next_coord = self.path[0]
        next_cell = self.model.grid[next_coord]
        
        # Check if there is a car or obstacle in the next cell
        has_obstacle = any(isinstance(agent, Obstacle) for agent in next_cell.agents)
        has_car = any(isinstance(agent, CarAgent) for agent in next_cell.agents)
        has_ambulance = any(isinstance(agent, Ambulance) for agent in next_cell.agents)
        
        if has_car or has_obstacle or has_ambulance:
            self.state = "waitingCar"
            return None
        
        # Check if there's a traffic light
        traffic_light = next(
            (agent for agent in next_cell.agents if isinstance(agent, Traffic_Light)),
            None
        )
        
        if traffic_light and not traffic_light.is_green:
            # Traffic light is red, wait
            self.state = "waitingTL"
            return None
        
        # Path is clear, change state to moving
        self.state = "moving"
        return next_cell

    def getNextReturnStepEmergency(self):
        """Get the next step using A* path, ignoring traffic lights (for emergency vehicles)."""

        # If no path, return None
        if not self.path:
            return None
        
        # Get the next cell from the A* path
        next_coord = self.path[0]
        next_cell = self.model.grid[next_coord]
        
        # Check if there is a car or obstacle in the next cell
        has_obstacle = any(isinstance(agent, Obstacle) for agent in next_cell.agents)
        has_car = any(isinstance(agent, CarAgent) for agent in next_cell.agents)
        has_ambulance = any(isinstance(agent, Ambulance) for agent in next_cell.agents)
        
        if has_car or has_obstacle or has_ambulance:
            self.state = "waitingCar"
            return None
        
        # In emergency mode, ignore traffic lights and move
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
    
    def checkObstacles(self):
        """Chooses the next cell based on the presence of obstacles."""
        next_cell = self.getNextCell()

        if next_cell is None:
            return None
        
        # Check for any obstacle
        has_obstacle = any(isinstance(agent, Obstacle) for agent in next_cell.agents)
        has_car = any(isinstance(agent, CarAgent) for agent in next_cell.agents)
        has_ambulance = any(isinstance(agent, Ambulance) for agent in next_cell.agents)
        has_red_light = any(isinstance(agent, Traffic_Light) and not agent.is_green for agent in next_cell.agents)

        if has_obstacle or has_car or has_ambulance or has_red_light:
            # Try to find an alternative path (diagonal or lateral cells)
            alternative_cell = self.findAlternativePath()
            if alternative_cell is not None:
                self.state = "moving"
                return alternative_cell
            
            # No alternative found, wait
            self.state = "waitingCar"
            return None

    
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
    
    def isBlockingAmbulance(self, ambulance, max_distance=2):
        """
        Checks if this car is blocking the path of the given ambulance.
            
        """
        # First check if ambulance is in emergency state
        if not hasattr(ambulance, 'state') or ambulance.state != "emergency":
            return False
        
        # Get ambulance position
        amb_x, amb_y = ambulance.cell.coordinate
        
        # Get ambulance direction
        amb_direction = ambulance.lastDirection

        # If ambulance has no lastDirection, try to get from road agent
        if amb_direction is None:
            for agent in ambulance.cell.agents:
                if isinstance(agent, Road):
                    amb_direction = agent.direction
                    break
        
        if amb_direction is None:
            return False
        
        # Get car position
        car_x, car_y = self.cell.coordinate
        
        # Check if car is in front of ambulance based on ambulance's direction
        if amb_direction == "Up":
            # Car must be above ambulance (same x, higher y) 
            return (car_x == amb_x) and (amb_y < car_y <= amb_y + max_distance)
        elif amb_direction == "Down":
            # Car must be below ambulance (same x, lower y)
            return (car_x == amb_x) and (amb_y > car_y >= amb_y - max_distance)
        elif amb_direction == "Right":
            # Car must be to the right of ambulance (same y, higher x)
            return (car_y == amb_y) and (amb_x < car_x <= amb_x + max_distance)
        elif amb_direction == "Left":
            # Car must be to the left of ambulance (same y, lower x)
            return (car_y == amb_y) and (amb_x > car_x >= amb_x - max_distance)
        
        return False

    def checkAmbulance(self):
        """
        Chooses the next cell based on the state of ambulances.

        - If there is an ambulance in emergency nearby and this car is blocking its path,
          the car will try to move diagonally out of the way if possible.
        - If the car has already moved for the ambulance this step, it will not move again
        - If no ambulance in emergency is nearby, the car continues normally.
        """

        # Get self coordinate
        x, y = self.cell.coordinate

        # Get direction of the current road
        road_dir = self.model.road_directions.get((x, y))
        if road_dir is None:
            return self.cell

        # Check nearby ambulances within a radius
        # This method was obtained from Mesa API Documentation
        # https://mesa.readthedocs.io/latest/apis/discrete_space.html#mesa.discrete_space.__init__.OrthogonalMooreGrid
        neighbor_cells = self.cell.get_neighborhood(
            radius=2,
            include_center=False
        )

        ambulance_nearby = False

        # For each neighboring cell, check for ambulances
        for neighbor_cell in neighbor_cells:
            ambulance = next(
                (obj for obj in neighbor_cell.agents if isinstance(obj, Ambulance)), None
            )

            # If an ambulance has been detected in emergency, clear the path
            if ambulance and ambulance.state == "emergency":
                ambulance_nearby = True
                
                # Check if this car is really blocking the ambulance's path
                if not self.isBlockingAmbulance(ambulance):
                    # Not blocking, continue normally
                    continue

                # If we already moved for ambulance, don't move again
                if self.moved_for_ambulance:
                    return None
                
                # Otherwise, try to move diagonally out of the way
                if not self.moved_for_ambulance:
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
                            has_ambulance = any(isinstance(obj, Ambulance) for obj in cell.agents)
                            traffic_light = next(
                                (obj for obj in cell.agents if isinstance(obj, Traffic_Light)), None
                            )

                            if has_obstacle or has_car or has_ambulance:
                                continue

                            if traffic_light and not traffic_light.is_green:
                                continue

                            valid_neighbors.append(cell)
                        
                        # If we have valid diagonal cells, move to one
                        if valid_neighbors:
                            new_cell = random.choice(valid_neighbors)
                            self.move(new_cell)
                            self.moved_for_ambulance = True
                            self.state = "idle"
                            # Return None to indicate we already moved this step
                            return None

                    # If no diagonal cell found, just stay in place
                    self.moved_for_ambulance = True
                    return None

        # No ambulance nearby reset flag to false
        if not ambulance_nearby:
            self.moved_for_ambulance = False
        
        # No ambulance in emergency nearby, continue
        return self.cell
    
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
    
    def get_diagonal_cells(self):
        """Get diagonal neighboring cells based on the car's last movement direction."""
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

        # Only get 2 diagonal cells in front based on direction
        if direction == "Up":
            diagonal_coords = [(x - 1, y + 1), (x + 1, y + 1)]  # Up left and right
        elif direction == "Down":
            diagonal_coords = [(x - 1, y - 1), (x + 1, y - 1)]  # Down left and right
        elif direction == "Right":
            diagonal_coords = [(x + 1, y + 1), (x + 1, y - 1)]  # Right up and down
        elif direction == "Left":
            diagonal_coords = [(x - 1, y - 1), (x - 1, y + 1)]  # Left up and down

        # Get grid dimensions
        grid_width = self.model.grid.width
        grid_height = self.model.grid.height

        diagonal_cells = []
        for coord in diagonal_coords:
            coord_x, coord_y = coord
            # Check if coordinate is within grid bounds
            if coord_x < 0 or coord_y < 0 or coord_x >= grid_width or coord_y >= grid_height:
                continue
            
            # Check if cell has road and no obstacles/vehicles
            cell = self.model.grid[coord]
            has_road = any(isinstance(agent, Road) for agent in cell.agents)
            has_obstacle = any(isinstance(agent, Obstacle) for agent in cell.agents)
            has_vehicle = any(
                isinstance(agent, (CarAgent, Ambulance))
                for agent in cell.agents
            )
            
            if has_road and not has_obstacle and not has_vehicle:
                diagonal_cells.append(cell)

        return diagonal_cells

    def tryLaneChange(self):
        """
        Attempts to change lanes if there is a vehicle directly in front.
        Only allows diagonal movements forward (ahead + lateral), not pure lateral moves.
        Returns the new cell if lane change is successful, otherwise None.
        """

        # Optional cooldown to prevent lane changes every step
        if hasattr(self, "lane_change_cooldown") and hasattr(self, "lane_change_steps_since"):
            if self.lane_change_steps_since < self.lane_change_cooldown:
                return None

        x, y = self.cell.coordinate

        # Get current road direction
        road_dir = self.model.road_directions.get((x, y))
        if road_dir is None:
            return None

        # Directional vectors
        dir_map = {
            "Up":    (0, 1),
            "Down":  (0, -1),
            "Left":  (-1, 0),
            "Right": (1, 0),
        }
        dx_f, dy_f = dir_map[road_dir]

        # Get the coordinates of the cell directly ahead
        ahead_x = x + dx_f
        ahead_y = y + dy_f

        # Check Bounds
        if not (0 <= ahead_x < self.model.grid.width and 0 <= ahead_y < self.model.grid.height):
            return None

        next_cell = self.model.grid[(ahead_x, ahead_y)]

        # Check if there is a vehicle ahead
        has_vehicle_ahead = any(isinstance(a, (CarAgent, Ambulance)) for a in next_cell.agents)
        if not has_vehicle_ahead:
            return None
        
        # The car needs to have the same direction as us to consider lane changing
        road_ahead = next(
            (agent for agent in next_cell.agents if isinstance(agent, Road)),
            None
        )
        if road_ahead is None or road_ahead.direction != road_dir:
            return None

        # Get lateral direction vectors
        lateral_left  = (-dy_f, dx_f)
        lateral_right = (dy_f, -dx_f)

        # Decide order based on lane preference if available
        if hasattr(self, "preferred_lane_side") and self.preferred_lane_side == "left":
            lateral_vectors = [lateral_left, lateral_right]
        else:
            lateral_vectors = [lateral_right, lateral_left]

        # Try to move diagonally forward (ahead + lateral)
        for dx_lat, dy_lat in lateral_vectors:
            # Get diagonal cell coordinates (ahead + lateral)
            diag_x = x + dx_f + dx_lat
            diag_y = y + dy_f + dy_lat

            # Check bounds
            if not (0 <= diag_x < self.model.grid.width and 0 <= diag_y < self.model.grid.height):
                continue
            
            diag_coord = (diag_x, diag_y)

            # If diagonal cell is not a road, skip
            if diag_coord not in self.model.road_cells:
                continue

            diag_cell = self.model.grid[diag_coord]

            # If diagonal cell has any vehicle, skip
            if any(isinstance(a, (CarAgent, Ambulance, Obstacle)) for a in diag_cell.agents):
                continue

            # If we reach here, this diagonal cell is valid
            return diag_cell

        # No lane change possible
        return None
    
    def step(self):
        """
        Executes one step of the car's behavior.
        
        Possible states:
        - idle: Ready to move
        - moving: Currently moving to next cell
        - waitingCar: Waiting for another car to move
        - waitingTL: Waiting for traffic light to turn green
        """

        self.lane_change_steps_since += 1

        # Check if we reached destination
        self.checkDestination()

        # If no destination, calculate one with A*
        if self.destination is None:
            self.pathToDestination('car')

        # If there is no path, cannot move
        if not self.path:
            return

        # Check current state and decide next action
        if self.state == "idle":
            # Check for ambulance first
            next_cell = self.checkAmbulance()

            if next_cell is None:
                return

            # Check path
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

            # Try lane change
            lane_cell = self.tryLaneChange()
            if lane_cell is not None:
                self.move(lane_cell)
                
                # After lane change, recalculate path to destination
                if self.destination is not None:
                    start = self.cell.coordinate
                    goal = self.destination.cell.coordinate
                    self.path = self.calculatePath(start, goal, find_closest=True)
                    self.path_index = 0
                
                self.state = "idle"
                self.tolerance_timer = 0  # Reset tolerance timer after lane change
                self.path_recalculated = False
                return
            
            # Update waiting timer
            self.reduceWaitingTimer('car')

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

class Ambulance(CarAgent):
    """
    Ambulance agent that reuses CarAgent logic.
    When in emergency state, ignores traffic lights but follows same movement rules.
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
    
    @property
    def has_emergency(self):
        """Whether the ambulance has an emergency."""
        return self._has_emergency
    
    @has_emergency.setter
    def has_emergency(self, value: bool) -> None:
        """Set emergency status and update state accordingly."""
        self._has_emergency = value
        self.state = "emergency" if value else "idle"
    
    
    def inEmergency(self):
        """
        If the ambulance is in emergency, and there's a car or another ambulance in emergency 
        waiting directly in front, the ambulance moves diagonally to pass.
        Returns True if moved diagonally, False otherwise.
        """
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
    
    def getNextCellFromPath(self):
        """
        Get the next step using A* path, checking traffic 
        lights and vehicles
        """
        
        if not self.path:
            return None

        model = self.model
        next_coord = self.path[0]
        next_cell = model.grid[next_coord]

        # Check if there is a car or obstacle in the next cell
        has_obstacle = any(isinstance(agent, Obstacle) for agent in next_cell.agents)
        has_car = any(isinstance(agent, CarAgent) for agent in next_cell.agents)
        has_ambulance = any(
            isinstance(agent, Ambulance) and agent != self
            for agent in next_cell.agents
        )

        if has_obstacle or has_car or has_ambulance:
            self.state = "waitingCar"
            return None

        # Check if there's a traffic light
        traffic_light = next(
            (agent for agent in next_cell.agents if isinstance(agent, Traffic_Light)),
            None
        )
        if traffic_light and not traffic_light.is_green:
            self.state = "waitingTL"
            return None

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
        Executes one step of the ambulance's behavior.
        
        Reuses CarAgent logic but ignores traffic lights when in emergency state.
        """
        # Update lane change cooldown
        self.lane_change_steps_since += 1

        # Check if we reached destination
        self.checkDestination()

        # If no destination, calculate one
        if self.destination is None:
            self.pathToDestination('ambulance')

        # If there is no path, cannot move
        if not self.path:
            return

        # Update siren light if in emergency
        if self.state == "emergency":
            self.updateSirenLight()

        # Check current state and decide next action
        if self.state == "idle":
            # Check for ambulance first
            next_cell = self.checkAmbulance()

            if next_cell is None:
                return

            # Check path
            next_cell = self.getNextReturnStep()

            # If state is moving, move
            if self.state == "moving" and next_cell:
                self.path.pop(0)
                self.move(next_cell)
                self.state = "idle"
        
        elif self.state == "waitingCar":
            # Check for ambulance first
            next_cell = self.checkAmbulance()

            if next_cell is None:
                return

            # Try lane change
            lane_cell = self.tryLaneChange()
            if lane_cell is not None:
                self.move(lane_cell)
                
                # After lane change, recalculate path to destination
                if self.destination is not None:
                    start = self.cell.coordinate
                    goal = self.destination.cell.coordinate
                    self.path = self.calculatePath(start, goal, find_closest=True)
                    self.path_index = 0
                
                self.state = "idle"
                self.tolerance_timer = 0
                self.path_recalculated = False
                return
            
            # Update waiting timer
            self.reduceWaitingTimer('ambulance')

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

        elif self.state == "emergency":
            # In emergency, move without checking traffic lights
            next_cell = self.getNextReturnStepEmergency()
            
            if self.state == "moving" and next_cell:
                self.path.pop(0)
                self.move(next_cell)
                self.state = "emergency"
        
        elif self.state == "waitingTL":
            # Check again if traffic light has changed
            next_cell = self.getNextReturnStep()
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