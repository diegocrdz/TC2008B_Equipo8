"""
Python flask server for traffic simulation visualization.

Diego Córdova Rodríguez
Lorena Estefanía Chewtat Torres
Aquiba Yudah Benarroch Bittán

2025-11-27
"""

from flask import Flask, request, jsonify
from flask_cors import CORS, cross_origin
from traffic_base.model import CityModel
from traffic_base.agent import *

# Size of the board:
number_agents = 10
width = 28
height = 28
randomModel = None
currentStep = 0
previous_agent_ids = set()  # Track previous agent IDs to detect removals
previous_ambulance_ids = set()  # Track previous ambulance IDs to detect removals

# This application will be used to interact with Unity
app = Flask("Traffic example")
cors = CORS(app, origins=['http://localhost'])

# This route will be used to send the parameters of the simulation to the server.
# The servers expects a POST request with the parameters in a.json.
@app.route('/init', methods=['GET', 'POST'])
@cross_origin()
def initModel():
    global currentStep, randomModel, number_agents, width, height

    if request.method == 'POST':
        try:
            number_agents = int(request.json.get('NAgents'))
            width = int(request.json.get('width'))
            height = int(request.json.get('height'))
            currentStep = 0

        except Exception as e:
            print(e)
            return jsonify({"message": "Error initializing the model"}), 500

    print(f"Model parameters:{number_agents, width, height}")

    # Create the model using the parameters sent by the application
    randomModel = CityModel(number_agents)

    # Return a message to saying that the model was created successfully
    return jsonify({"message": f"Parameters recieved, model initiated.\nSize: {width}x{height}"})

# This route will be used to get the positions of the agents
@app.route('/getAgents', methods=['GET'])
@cross_origin()
def getAgents():
    global randomModel, previous_agent_ids

    if request.method == 'GET':
        # Get the positions of the agents and return them to WebGL in JSON.json.t.
        # Note that the positions are sent as a list of dictionaries, where each dictionary has the id and position of an agent.
        # The y coordinate is set to 1, since the agents are in a 3D world. The z coordinate corresponds to the row (y coordinate) of the grid in mesa.
        try:
            agentCells = randomModel.grid.all_cells.select(
                lambda cell: any(isinstance(obj, CarAgent) for obj in cell.agents)
            ).cells

            agents = [
                (cell.coordinate, agent)
                for cell in agentCells
                for agent in cell.agents
                if isinstance(agent, CarAgent)
            ]

            agentPositions = [
                {
                    "id": str(a.unique_id),
                    "x": coordinate[0],
                    "y": 1,
                    "z": coordinate[1],
                    "direction": next((road.direction for road in randomModel.grid[coordinate].agents if isinstance(road, Road)), None),
                }
                for (coordinate, a) in agents
            ]
            
            # Get current agent IDs
            current_agent_ids = {agent["id"] for agent in agentPositions}
            
            # Find removed agents
            removed_agent_ids = list(previous_agent_ids - current_agent_ids)
            
            # Update tracking
            previous_agent_ids = current_agent_ids

            return jsonify({'positions': agentPositions, 'removed': removed_agent_ids})
        except Exception as e:
            print(e)
            return jsonify({"message": "Error with the agent positions"}), 500

# This route will be used to get the positions of the ambulances
@app.route('/getAmbulances', methods=['GET'])
@cross_origin()
def getAmbulances():
    global randomModel, previous_ambulance_ids

    if request.method == 'GET':
        try:
            # Get the positions of the ambulances and return them to WebGL in JSON.json.t.
            ambulanceCells = randomModel.grid.all_cells.select(
                lambda cell: any(isinstance(obj, Ambulance) for obj in cell.agents)
            )

            ambulances = [
                (cell.coordinate, agent)
                for cell in ambulanceCells
                for agent in cell.agents
                if isinstance(agent, Ambulance)
            ]

            ambulancePositions = [
                {
                    "id": str(a.unique_id),
                    "x": coordinate[0],
                    "y": 1,
                    "z": coordinate[1],
                    "direction": next((road.direction for road in randomModel.grid[coordinate].agents if isinstance(road, Road)), None),
                }
                for (coordinate, a) in ambulances
            ]
            
            # Get current ambulance IDs
            current_ambulance_ids = {ambulance["id"] for ambulance in ambulancePositions}
            
            # Find removed ambulances
            removed_ambulance_ids = list(previous_ambulance_ids - current_ambulance_ids)
            
            # Update tracking
            previous_ambulance_ids = current_ambulance_ids

            return jsonify({'positions': ambulancePositions, 'removed': removed_ambulance_ids})
        except Exception as e:
            print(e)
            return jsonify({"message": "Error with ambulance positions"}), 500

# This route will be used to get the positions of the obstacles
@app.route('/getObstacles', methods=['GET'])
@cross_origin()
def getObstacles():
    global randomModel

    if request.method == 'GET':
        try:
            # Get the positions of the obstacles and return them to WebGL in JSON.json.t.
            # Same as before, the positions are sent as a list of dictionaries, where each dictionary has the id and position of an obstacle.

            obstacleCells = randomModel.grid.all_cells.select(
                lambda cell: any(isinstance(obj, Obstacle) for obj in cell.agents)
            )
            # print(f"CELLS: {agentCells}")

            agents = [
                (cell.coordinate, agent)
                for cell in obstacleCells
                for agent in cell.agents
                if isinstance(agent, Obstacle)
            ]
            # print(f"AGENTS: {agents}")

            obstaclePositions = [
                {
                    "id": str(a.unique_id),
                    "x": coordinate[0],
                    "y": 1,
                    "z": coordinate[1]
                }
                for (coordinate, a) in agents
            ]
            # print(f"OBSTACLE POSITIONS: {obstaclePositions}")

            return jsonify({'positions': obstaclePositions})
        except Exception as e:
            print(e)
            return jsonify({"message": "Error with obstacle positions"}), 500

# This route will be used to get the positions of the traffic lights
@app.route('/getTrafficLights', methods=['GET'])
@cross_origin()
def getTrafficLights():
    global randomModel

    if request.method == 'GET':
        try:
            # Get the positions of the traffic lights and return them to WebGL in JSON.json.t.
            # Same as before, the positions are sent as a list of dictionaries, where each dictionary has the id and position of an obstacle.
            TLCells = randomModel.traffic_lights

            # Get the coordinates of the traffic lights
            trafficLights = [
                (agent.cell.coordinate, agent)
                for agent in TLCells
            ]

            TLPositions = [
                {
                    "id": str(a.unique_id),
                    "x": coordinate[0],
                    "y": 1,
                    "z": coordinate[1],
                    "state": a.state,
                    "direction": next((road.direction for road in a.cell.agents if isinstance(road, Road)), None),
                }
                for (coordinate, a) in trafficLights
            ]

            return jsonify({'positions': TLPositions})
        except Exception as e:
            print(e)
            return jsonify({"message": "Error with obstacle positions"}), 500

# This route will be used to get the positions of the roads
@app.route('/getRoads', methods=['GET'])
@cross_origin()
def getRoads():
    global randomModel

    if request.method == 'GET':
        try:
            # Get the positions of the roads and return them to WebGL in JSON.json.t.
            roadCells = randomModel.grid.all_cells.select(
                lambda cell: any(isinstance(obj, Road) for obj in cell.agents)
            ).cells

            roads = [
                (cell.coordinate, agent)
                for cell in roadCells
                for agent in cell.agents
                if isinstance(agent, Road)
            ]

            roadPositions = [
                {
                    "id": str(a.unique_id),
                    "x": coordinate[0],
                    "y": 0.9,
                    "z": coordinate[1],
                    "direction": a.direction,
                }
                for (coordinate, a) in roads
            ]

            return jsonify({'positions': roadPositions})
        except Exception as e:
            print(e)
            return jsonify({"message": "Error with road positions"}), 500

# This route will be used to get the positions of the sidewalks
@app.route('/getSidewalks', methods=['GET'])
@cross_origin()
def getSidewalks():
    global randomModel

    if request.method == 'GET':
        try:
            sidewalkCells = randomModel.grid.all_cells.select(
                lambda cell: any(isinstance(obj, SideWalk) for obj in cell.agents)
            ).cells

            sidewalks = [
                (cell.coordinate, agent)
                for cell in sidewalkCells
                for agent in cell.agents
                if isinstance(agent, SideWalk)
            ]

            sidewalkPositions = [
                {
                    "id": str(a.unique_id),
                    "x": coordinate[0],
                    "y": 0.85,
                    "z": coordinate[1],
                }
                for (coordinate, a) in sidewalks
            ]

            return jsonify({'positions': sidewalkPositions})
        except Exception as e:
            print(e)
            return jsonify({"message": "Error with sidewalk positions"}), 500

# This route will be used to get the positions of the hospitals
@app.route('/getHospitals', methods=['GET'])
@cross_origin()
def getHospitals():
    global randomModel

    if request.method == 'GET':
        try:
            hospitalCells = randomModel.hospitals

            hospitals = [
                (agent.cell.coordinate, agent)
                for agent in hospitalCells
            ]

            hospitalPositions = [
                {
                    "id": str(a.unique_id),
                    "x": coordinate[0],
                    "y": 1,
                    "z": coordinate[1],
                }
                for (coordinate, a) in hospitals
            ]

            return jsonify({'positions': hospitalPositions})
        except Exception as e:
            print(e)
            return jsonify({"message": "Error with hospital positions"}), 500

# This route will be used to get the positions of the destinations
@app.route('/getDestinations', methods=['GET'])
@cross_origin()
def getDestinations():
    global randomModel

    if request.method == 'GET':
        try:
            destinationCells = randomModel.destinations

            destinations = [
                (agent.cell.coordinate, agent)
                for agent in destinationCells
            ]

            destinationPositions = [
                {
                    "id": str(a.unique_id),
                    "x": coordinate[0],
                    "y": 1,
                    "z": coordinate[1],
                }
                for (coordinate, a) in destinations
            ]

            return jsonify({'positions': destinationPositions})
        except Exception as e:
            print(e)
            return jsonify({"message": "Error with destination positions"}), 500

# This route will be used to update the model
@app.route('/update', methods=['GET'])
@cross_origin()
def updateModel():
    global currentStep, randomModel
    if request.method == 'GET':
        try:
        # Update the model and return a message to WebGL saying that the model was updated successfully
            randomModel.step()
            currentStep += 1
            return jsonify({'message': f'Model updated to step {currentStep}.', 'currentStep':currentStep})
        except Exception as e:
            print(e)
            return jsonify({"message": "Error during step."}), 500

if __name__=='__main__':
    # Run the flask server in port 8585
    app.run(host="localhost", port=8585, debug=True)
