/*
 * Functions to connect to an external API to get the coordinates of agents
 *
 * Diego Córdova Rodríguez
 * Lorena Estefanía Chewtat Torres
 * Aquiba Yudah Benarroch Bittán
 * 
 * 2025-11-29
 */

'use strict';

import { Object3D, TrafficLight, Road } from '../libs/object3d';
import { getRotation } from '../visualization/utils.js';

// Define the agent server URI
const agent_server_uri = "http://localhost:8585/";

// Initialize arrays to store agents and obstacles
const agents = [];
const obstacles = [];
const trafficLights = [];
const roads = [];
const hospitals = [];
const destinations = [];
const sidewalks = [];
const ambulances = [];

// Global scene reference for removing objects
let globalScene = null;

// Setup functions for adding new agents
import {
    setupNewCarAgent,
    setupNewAmbulanceAgent
} from '../visualization/random_agents.js';

// Define the data object
const initData = {
    NAgents: 20,
    width: 28,
    height: 28
};

/* FUNCTIONS FOR THE INTERACTION WITH THE MESA SERVER */

/*
 * Sets the global scene reference for removing objects.
 */
function setScene(scene) {
    globalScene = scene;
}

/*
 * Initializes the agents model by sending a POST request to the agent server.
 */
async function initAgentsModel() {
    try {
        // Send a POST request to the agent server to initialize the model
        let response = await fetch(agent_server_uri + "init", {
            method: 'POST',
            headers: { 'Content-Type':'application/json' },
            body: JSON.stringify(initData)
        });

        // Check if the response was successful
        if (response.ok) {
            // Parse the response as JSON and log the message
            let result = await response.json();
            console.log(result.message);
        }

    } catch (error) {
        // Log any errors that occur during the request
        console.log(error);
    }
}

/*
 * Retrieves the current positions of all agents from the agent server.
 */
async function getAgents() {
    try {
        // Send a GET request to the agent server to retrieve the agent positions
        let response = await fetch(agent_server_uri + "getAgents");

        // Check if the response was successful
        if (response.ok) {
            // Parse the response as JSON
            let result = await response.json();

            // Remove agents that no longer exist
            if (result.removed) {
                for (const removedId of result.removed) {
                    const index = agents.findIndex((agent) => agent.id == removedId);
                    if (index !== -1) {
                        agents.splice(index, 1);
                        // Also remove from scene if available
                        if (globalScene) {
                            globalScene.removeObject(removedId);
                        }
                    }
                }
            }

            // Check if the agents array is empty
            if (agents.length == 0) {
                // Create new agents and add them to the agents array
                for (const agent of result.positions) {
                    const newAgent = new Object3D(agent.id, [agent.x, agent.y, agent.z]);

                    // Store the initial position
                    newAgent['oldPosArray'] = newAgent.posArray;

                    // Store the direction
                    newAgent['direction'] = agent.direction;

                    // Initialize the rotation using initial position
                    const rotationY = getRotation(newAgent.posArray, newAgent.posArray);
                    newAgent['oldRotY'] = rotationY;
                    newAgent['targetRotY'] = rotationY;
                    agents.push(newAgent);
                }

            } else {
                // Update the positions of existing agents
                for (const agent of result.positions) {
                    const current_agent = agents.find((object3d) => object3d.id == agent.id);

                    // Check if the agent exists in the agents array
                    if(current_agent != undefined){
                        // Update the agent's position
                        current_agent.oldPosArray = current_agent.posArray;
                        current_agent.position = {x: agent.x, y: agent.y, z: agent.z};

                        // Update the direction and rotation
                        current_agent.oldRotY = current_agent.targetRotY;
                        current_agent.direction = agent.direction;
                        const rotationY = getRotation(current_agent.oldPosArray, current_agent.posArray);

                        // Only update target rotation if there was actual movement
                        if (rotationY !== null) {
                            current_agent.targetRotY = rotationY;
                        }
                    } else {
                        // If the agent does not exist, create it
                        const newAgent = new Object3D(agent.id, [agent.x, agent.y, agent.z]);

                        // Store the initial position
                        newAgent['oldPosArray'] = newAgent.posArray;

                        // Store the direction
                        newAgent['direction'] = agent.direction;

                        // Initialize the rotation
                        const rotationY = getRotation(newAgent.posArray, newAgent.posArray);
                        newAgent['oldRotY'] = rotationY;
                        newAgent['targetRotY'] = rotationY;
                        agents.push(newAgent);
                        
                        // Setup the new agent in the scene
                        setupNewCarAgent(newAgent);
                    }
                }
            }
        }

    } catch (error) {
        // Log any errors that occur during the request
        console.log(error);
    }
}

/*
 * Retrieves the current positions of all ambulances from the agent server.
 */
async function getAmbulances() {
    try {
        // Send a GET request to the agent server to retrieve the agent positions
        let response = await fetch(agent_server_uri + "getAmbulances");

        // Check if the response was successful
        if (response.ok) {
            // Parse the response as JSON
            let result = await response.json();

            // Remove ambulances that no longer exist
            if (result.removed) {
                for (const removedId of result.removed) {
                    const index = ambulances.findIndex((ambulance) => ambulance.id == removedId);
                    if (index !== -1) {
                        const ambulance = ambulances[index];
                        ambulance.light = false; // Turn off siren light
                        // Remove siren light from scene
                        if (ambulance.sirenLight && globalScene) {
                            globalScene.removeLight(ambulance.sirenLight);
                        }
                        ambulance.sirenLight = null;
                        // Remove ambulance from scene
                        if (globalScene) {
                            globalScene.removeObject(removedId);
                        }
                        // Remove from ambulances array
                        ambulances.splice(index, 1);
                    }
                }
            }

            // Check if the ambulances array is empty
            if (ambulances.length == 0) {
                // Create new ambulances and add them to the ambulances array
                for (const agent of result.positions) {
                    const newAgent = new Object3D(agent.id, [agent.x, agent.y, agent.z]);

                    // Store the initial position
                    newAgent['oldPosArray'] = newAgent.posArray;

                    // Store the direction
                    newAgent['direction'] = agent.direction;

                    // Store the rotation
                    const rotationY = getRotation(newAgent.posArray, newAgent.posArray);
                    newAgent['oldRotY'] = rotationY;
                    newAgent['targetRotY'] = rotationY;

                    // Store the siren light state
                    newAgent['light'] = agent.light;
                    newAgent['light_state'] = agent.light_state;

                    // Store emergency state
                    newAgent['has_emergency'] = agent.has_emergency;

                    // Add to ambulances array
                    ambulances.push(newAgent);
                }

            } else {
                // Update the positions of existing ambulances
                for (const agent of result.positions) {

                    const current_agent = ambulances.find((object3d) => object3d.id == agent.id);

                    // Check if the ambulance exists in the ambulances array
                    if(current_agent != undefined){
                        // Update the ambulance's position
                        current_agent.oldPosArray = current_agent.posArray;
                        current_agent.position = {x: agent.x, y: agent.y, z: agent.z};

                        // Update the direction and rotation
                        current_agent.oldRotY = current_agent.targetRotY;
                        current_agent.direction = agent.direction;
                        const rotationY = getRotation(current_agent.oldPosArray, current_agent.posArray);

                        // Only update target rotation if there was actual movement
                        if (rotationY !== null) {
                            current_agent.targetRotY = rotationY;
                        }

                        // Update siren light state
                        current_agent.light = agent.light;
                        current_agent.light_state = agent.light_state;

                        // Update emergency state
                        current_agent.has_emergency = agent.has_emergency;

                    } else {
                        // If the ambulance does not exist, create it
                        const newAgent = new Object3D(agent.id, [agent.x, agent.y, agent.z]);

                        // Store the initial position
                        newAgent['oldPosArray'] = newAgent.posArray;

                        // Store the direction
                        newAgent['direction'] = agent.direction;

                        // Store the rotation
                        const rotationY = getRotation(newAgent.posArray, newAgent.posArray);
                        newAgent['oldRotY'] = rotationY;
                        newAgent['targetRotY'] = rotationY;

                        // Store the siren light state
                        newAgent['light'] = agent.light;
                        newAgent['light_state'] = agent.light_state;

                        // Store emergency state
                        newAgent['has_emergency'] = agent.has_emergency;

                        // Add to ambulances array
                        ambulances.push(newAgent);

                        // Setup the new ambulance in the scene
                        setupNewAmbulanceAgent(newAgent);
                    }
                }
            }
        }

    } catch (error) {
        // Log any errors that occur during the request
        console.log(error);
    }
}

/*
 * Retrieves the current positions of all obstacles from the agent server.
 */
async function getObstacles() {
    try {
        // Send a GET request to the agent server to retrieve the obstacle positions
        let response = await fetch(agent_server_uri + "getObstacles");

        // Check if the response was successful
        if (response.ok) {
            // Parse the response as JSON
            let result = await response.json();

            // Create new obstacles and add them to the obstacles array
            for (const obstacle of result.positions) {
                const newObstacle = new Object3D(obstacle.id, [obstacle.x, obstacle.y, obstacle.z]);
                obstacles.push(newObstacle);
            }
        }

    } catch (error) {
        // Log any errors that occur during the request
        console.log(error);
    }
}

/*
 * Gets the current positions of all traffic lights from the agent server.
 */
async function getTrafficLights() {
    try {
        // Send a GET request to the agent server to retrieve the traffic light positions
        let response = await fetch(agent_server_uri + "getTrafficLights");

        // Check if the response was successful
        if (response.ok) {
            // Parse the response as JSON
            let result = await response.json();

            // Check if the lights array is empty
            if (trafficLights.length == 0) {
                // Create new agents and add them to the agents array
                for (const tl of result.positions) {
                    const newTL = new TrafficLight(tl.id, [tl.x, tl.y, tl.z]);

                    // Set state and direction
                    newTL.state = tl.state;
                    newTL.direction = tl.direction;

                    // Add to array
                    trafficLights.push(newTL);
                }

            } else {
                // Update the positions of existing agents
                for (const tl of result.positions) {
                    const current_tl = trafficLights.find((object3d) => object3d.id == tl.id);

                    // Check if the agent exists in the trafficLights array
                    if(current_tl != undefined){
                        // Update state and direction
                        current_tl.state = tl.state;
                        current_tl.direction = tl.direction;
                    }
                }
            }
        }

    } catch (error) {
        // Log any errors that occur during the request
        console.log(error);
    }
}

async function getRoads() {
    try {
        // Send a GET request to the agent server to retrieve the road positions
        let response = await fetch(agent_server_uri + "getRoads");

        // Check if the response was successful
        if (response.ok) {
            // Parse the response as JSON
            let result = await response.json();

            // Create new roads and add them to the roads array
            for (const road of result.positions) {
                const newRoad = new Road(road.id, [road.x, road.y, road.z]);
                newRoad.direction = road.direction;
                roads.push(newRoad);
            }
        }

    } catch (error) {
        // Log any errors that occur during the request
        console.log(error);
    }
}

/*
 * Gets the current positions of all sidewalks from the agent server.
 */
async function getSidewalks() {
    try {
        // Send a GET request to the agent server to retrieve the sidewalk positions
        let response = await fetch(agent_server_uri + "getSidewalks");

        // Check if the response was successful
        if (response.ok) {
            // Parse the response as JSON
            let result = await response.json();

            // Create new sidewalks and add them to the sidewalks array
            for (const sidewalk of result.positions) {
                const newSidewalk = new Object3D(sidewalk.id, [sidewalk.x, sidewalk.y, sidewalk.z]);
                sidewalks.push(newSidewalk);
            }
        }

    } catch (error) {
        // Log any errors that occur during the request
        console.log(error);
    }
}

/*
 * Gets the current positions of all hospitals from the agent server.
 */
async function getHospitals() {
    try {
        // Send a GET request to the agent server to retrieve the hospital positions
        let response = await fetch(agent_server_uri + "getHospitals");

        // Check if the response was successful
        if (response.ok) {
            // Parse the response as JSON
            let result = await response.json();

            // Create new hospitals and add them to the hospitals array
            for (const hospital of result.positions) {
                const newHospital = new Object3D(hospital.id, [hospital.x, hospital.y, hospital.z]);
                hospitals.push(newHospital);
            }
        }

    } catch (error) {
        // Log any errors that occur during the request
        console.log(error);
    }
}

/*
 * Gets the current positions of all destinations from the agent server.
 */
async function getDestinations() {
    try {
        // Send a GET request to the agent server to retrieve the destination positions
        let response = await fetch(agent_server_uri + "getDestinations");

        // Check if the response was successful
        if (response.ok) {
            // Parse the response as JSON
            let result = await response.json();

            // Create new destinations and add them to the destinations array
            for (const destination of result.positions) {
                const newDestination = new Object3D(destination.id, [destination.x, destination.y, destination.z]);
                destinations.push(newDestination);
            }
        }

    } catch (error) {
        // Log any errors that occur during the request
        console.log(error);
    }
}

/*
 * Updates the agent positions by sending a request to the agent server.
 */
async function update() {
    try {
        // Send a request to the agent server to update the agent positions
        let response = await fetch(agent_server_uri + "update");

        // Check if the response was successful
        if (response.ok) {
            // Retrieve the updated agent positions
            await getAgents();
            await getAmbulances();
            await getTrafficLights();
        }

    } catch (error) {
        // Log any errors that occur during the request
        console.log(error);
    }
}

export {
    initAgentsModel, update, setScene,
    agents, getAgents,
    ambulances, getAmbulances,
    obstacles, getObstacles,
    trafficLights, getTrafficLights,
    roads, getRoads,
    hospitals, getHospitals,
    destinations, getDestinations,
    sidewalks, getSidewalks,
};
