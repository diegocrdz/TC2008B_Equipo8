/*
 * Base program for a 3D scene that connects to an API to get the movement
 * of agents.
 * The scene shows colored cubes
 *
 * Aquiba Yudah Benarroch Bittan, A01783710 
 * Diego Córdova Rodríguez, A01781166 
 * Lorena Estefanía Chewtat Torres, A01785378
 * 
 * 2025-12-04
 */

'use strict';

// Libs
import * as twgl from 'twgl-base.js';
import GUI from 'lil-gui';
import { M4 } from '../libs/3d-lib';
import { Scene3D } from '../libs/scene3d';
import { Object3D } from '../libs/object3d';
import { Light3D } from '../libs/light3d';
import { Camera3D } from '../libs/camera3d';
import { cubeTextured } from '../libs/shapes';

// Functions and arrays for the communication with the API
import {
  initAgentsModel, update, setScene,
  agents, getAgents,
  ambulances, getAmbulances,
  obstacles, getObstacles,
  trafficLights, getTrafficLights,
  roads, getRoads,
  hospitals, getHospitals,
  destinations, getDestinations,
  sidewalks, getSidewalks,
} from '../libs/api_connection.js';

// Texture shaders
import vsTex from '../assets/shaders/vs_multi_lights_attenuation.glsl?raw';
import fsTex from '../assets/shaders/fs_multi_lights_attenuation.glsl?raw';

// Models, materials and textures
import {
  // Models and materials
  buildingModels, hospitalModel,
  trafficLightModel,
  carModels, ambulanceModel,
  destinationModel, roadStraightModel,

  // Textures
  initTextures,
  skyboxTexture, sidewalkTexture, simpleBuildingTexture,
  simpleBuildingTextureA, simpleBuildingTextureB,
  complexBuildingTexture, car1Texture,
  car2Texture, car3Texture, ambulanceTexture,
  greenTexture, redTexture,
} from './objects.js';

// Utils
import {
  assignModelToAgents,
  getRotationByDirection,
  getTrafficLightRotation,
  createBaseModelWithMtl,
  updateTrafficLights,
  updateAmbulanceLights,
  getLightsCloseToCamera,
} from './utils.js';

// Create the 3D scene
const scene = new Scene3D();

// Global variables
let textureProgramInfo = undefined;
let gl = undefined;
let duration = 400; // ms
let elapsed = 0;
let then = 0;
const NUM_LIGHTS = 27; // Max number of lights to consider in the shader

// Simulation parameters
let simulationParams = {
  vehicle_spawn_rate: 4,
  vehicles_per_step: 4,
  ambulance_per_step: 1,
  emergency_chance: 0.5,
};

// Store base models and textures globally for adding new agents
let carModelsArray = [];
let carTexturesArray = [];
let ambulanceBaseModel = null;
let baseCubeTexGlobal = null;

// Setup a new car agent with model and texture
export function setupNewCarAgent(agent) {
  if (carModelsArray.length === 0) return; // Models not ready yet
  
  const randomIndex = Math.floor(Math.random() * 3);
  const car = carModelsArray[randomIndex];
  
  assignModelToAgents([agent], car, {
    scale: { x: 0.1, y: 0.1, z: 0.1 },
    color: [1.0, 1.0, 1.0, 1.0],
    shininess: 4.0,
    texture: carTexturesArray[randomIndex],
    programType: 'texture',
  });
  
  scene.addObject(agent);
}

// Setup a new ambulance agent with model and texture
export function setupNewAmbulanceAgent(agent) {
  if (!ambulanceBaseModel) {
    // If model is not ready yet, still add to scene so it gets rendered
    scene.addObject(agent);
    return;
  }
  
  assignModelToAgents([agent], ambulanceBaseModel, {
    scale: { x: 0.1, y: 0.1, z: 0.1 },
    color: [1.0, 1.0, 1.0, 1.0],
    shininess: 4.0,
    texture: ambulanceTexture,
    programType: 'texture',
  });
  
  scene.addObject(agent);
}

// Export simulation parameters
export function getSimulationParams() {
  return simulationParams;
}

// Function to restart the simulation with new parameters
export async function restartSimulation() {  
  // Clear all agents and obstacles from the scene
  agents.length = 0;
  ambulances.length = 0;
  obstacles.length = 0;
  trafficLights.length = 0;
  roads.length = 0;
  hospitals.length = 0;
  destinations.length = 0;
  sidewalks.length = 0;
  
  // Remove all objects from the scene
  const objectsCopy = [...scene.objects];
  for (const obj of objectsCopy) {
    scene.removeObject(obj.id);
  }
  
  // Remove all lights except the global light (index 0)
  const lightsCopy = [...scene.lights];
  for (let i = 1; i < lightsCopy.length; i++) {
    scene.removeLight(lightsCopy[i]);
  }
  
  // Reinitialize the model with new parameters
  await initAgentsModel();
  
  // Get the agents and obstacles
  await getAgents();
  await getAmbulances();
  await getObstacles();
  await getTrafficLights();
  await getRoads();
  await getHospitals();
  await getDestinations();
  await getSidewalks();
  
  // Reset the position in the setup
  setupObjects(scene, gl);
  
  console.log('Simulation restarted');
}

// Main function is async to be able to make the requests
async function main() {
  // Setup the canvas area
  const canvas = document.querySelector('canvas');
  gl = canvas.getContext('webgl2');
  twgl.resizeCanvasToDisplaySize(gl.canvas);
  gl.viewport(0, 0, gl.canvas.width, gl.canvas.height);

  // Initialize textures with gl context
  initTextures(gl);

  // Prepare the program with the shaders
  textureProgramInfo = twgl.createProgramInfo(gl, [vsTex, fsTex]);

  // Initialize the agents model
  await initAgentsModel();

  // Get the agents and obstacles
  await getAgents();
  await getAmbulances();
  await getObstacles();
  await getTrafficLights();
  await getRoads();
  await getHospitals();
  await getDestinations();
  await getSidewalks();

  // Initialize the scene
  setupScene();
  
  // Set the global scene reference for API connection
  setScene(scene);

  // Position the objects in the scene
  setupObjects(scene, gl);

  // Prepare the user interface
  setupUI();

  // Fisrt call to the drawing loop
  drawScene();
}

function setupScene() {
  let camera = new Camera3D(
    0, // id
    10, // Distance to target
    4, // Azimut
    0.6, // Elevation
    [0, 0, 10], // Position
    [0, 0, 0] // Target
  );

  camera.panOffset = [0, 8, 0];
  scene.setCamera(camera);
  scene.camera.setupControls();

  // Create global light
  let light = new Light3D(
    [50, 4, 50],          // Position
    [0.4, 0.4, 1.0, 1.0], // Ambient
    [0.0, 0.0, 0.0, 1.0], // Diffuse
    [0.5, 0.5, 0.5, 1.0], // Specular
  );
  scene.addLight(light);
}

function setupObjects(scene, gl) {
  // Cube with texture
  // Used for skybox and sidewalks
  const baseCubeTex = new Object3D(1);
  baseCubeTex.arrays = cubeTextured(2);
  baseCubeTex.bufferInfo = twgl.createBufferInfoFromArrays(gl, baseCubeTex.arrays);
  baseCubeTex.vao = twgl.createVAOFromBufferInfo(gl, textureProgramInfo, baseCubeTex.bufferInfo);

  // Store globally for traffic lights
  baseCubeTexGlobal = baseCubeTex;

  // Road with texture
  const baseRoadTex = new Object3D(2);
  baseRoadTex.prepareVAO(gl, textureProgramInfo);

  // Skybox
  let skybox = new Object3D(3);
  skybox.arrays = baseCubeTex.arrays;
  skybox.bufferInfo = baseCubeTex.bufferInfo;
  skybox.vao = baseCubeTex.vao;
  skybox.scale = { x: -50, y: -50, z: -50 };
  skybox.programType = 'texture';
  skybox.texture = skyboxTexture;
  scene.addObject(skybox);

  // Cars
  const car1 = createBaseModelWithMtl(
    4, gl, textureProgramInfo, carModels[0].obj, carModels[0].mtl
  );
  const car2 = createBaseModelWithMtl(
    5, gl, textureProgramInfo, carModels[1].obj, carModels[1].mtl
  );
  const car3 = createBaseModelWithMtl(
    6, gl, textureProgramInfo, carModels[2].obj, carModels[2].mtl
  );

  carModelsArray = [car1, car2, car3];
  carTexturesArray = [car1Texture, car2Texture, car3Texture];

  for (const agent of agents) {
    // Get random model
    const randomIndex = Math.floor(Math.random() * 3);
    const car = carModelsArray[randomIndex];

    // Assign model data
    assignModelToAgents([agent], car, {
      scale: { x: 0.1, y: 0.1, z: 0.1 },
      color: [1.0, 1.0, 1.0, 1.0],
      shininess: 4.0,
      texture: carTexturesArray[randomIndex],
      programType: 'texture',
    });

    scene.addObject(agent);
  }

  // Ambulances
  ambulanceBaseModel = createBaseModelWithMtl(
    -4, gl, textureProgramInfo, 
    ambulanceModel.obj, ambulanceModel.mtl
  );
  
  for (const agent of ambulances) {
    assignModelToAgents([agent], ambulanceBaseModel, {
      scale: { x: 0.1, y: 0.1, z: 0.1 },
      color: [1.0, 1.0, 1.0, 1.0],
      shininess: 4.0,
      texture: ambulanceTexture,
      programType: 'texture',
    });

    scene.addObject(agent);
  }

  // Obstacles
  const baseObstacle0 = createBaseModelWithMtl(
    7, gl, textureProgramInfo, 
    buildingModels[0].obj, buildingModels[0].mtl
  );
  const baseObstacle1 = createBaseModelWithMtl(
    8, gl, textureProgramInfo, 
    buildingModels[1].obj, buildingModels[1].mtl
  );
  const baseObstacle2 = createBaseModelWithMtl(
    9, gl, textureProgramInfo, 
    buildingModels[2].obj, buildingModels[2].mtl
  );
  const baseObstacle3 = createBaseModelWithMtl(
    10, gl, textureProgramInfo, 
    buildingModels[3].obj, buildingModels[3].mtl
  );
  const baseObstacle4 = createBaseModelWithMtl(
    11, gl, textureProgramInfo, 
    buildingModels[4].obj, buildingModels[4].mtl
  );

  // Obstacle array
  const baseObstacles = [
    baseObstacle0, baseObstacle1, 
    baseObstacle2, baseObstacle3,
    baseObstacle4,
  ];

  // Building configuration by type
  const buildingConfigs = [
    { 
      scale: { x: 1, y: 1.2, z: 1 }, 
      shininess: 16.0, 
      texture: simpleBuildingTexture 
    },
    { 
      scale: { x: 1, y: 1.2, z: 1 }, 
      shininess: 16.0, 
      texture: simpleBuildingTextureB 
    },
    { 
      scale: { x: 0.35, y: 0.5, z: 0.35 }, 
      shininess: 16.0, 
      texture: complexBuildingTexture 
    },
    { 
      scale: { x: 0.35, y: 0.5, z: 0.35 }, 
      shininess: 16.0, 
      texture: complexBuildingTexture 
    },
    { 
      scale: { x: 1, y: 1.5, z: 1 }, 
      shininess: 16.0, 
      texture: simpleBuildingTextureA 
    },
  ];

  for (const agent of obstacles) {
    // Get random model
    const randomIndex = Math.floor(Math.random() * 5);
    const baseObstacle = baseObstacles[randomIndex];
    const config = buildingConfigs[randomIndex];

    // Assign model data
    assignModelToAgents([agent], baseObstacle, {
      ...config,
      color: [1.0, 1.0, 1.0, 1.0],
      programType: 'texture',
    });

    scene.addObject(agent);
  }

  // Traffic Lights
  const baseTrafficLight = createBaseModelWithMtl(
    12, gl, textureProgramInfo, 
    trafficLightModel.obj, trafficLightModel.mtl
  );

  for (const agent of trafficLights) {
    assignModelToAgents([agent], baseTrafficLight, {
      scale: { x: 1, y: 1, z: 1 },
      programType: 'texture',
      texture: complexBuildingTexture,
      rotRad: getTrafficLightRotation(agent.direction),
    });

    scene.addObject(agent);
  }

  // Roads
  const baseRoadStraight = createBaseModelWithMtl(
    14, gl, textureProgramInfo,
    roadStraightModel.obj, roadStraightModel.mtl
  );

  for (const agent of roads) {
    assignModelToAgents([agent], baseRoadStraight, {
      scale: { x: 0.5, y: 1, z: 0.5 },
      programType: 'texture',
      texture: complexBuildingTexture,
      shininess: 8.0,
      rotRad: getRotationByDirection(agent.direction),
    });

    scene.addObject(agent);
  }

  // Sidewalks
  for (const agent of sidewalks) {
    assignModelToAgents([agent], baseCubeTex, {
      scale: { x: 0.25, y: 0.1, z: 0.25 },
      programType: 'texture',
      texture: sidewalkTexture,
    });

    scene.addObject(agent);
  }

  // Hospitals
  const baseHospital = createBaseModelWithMtl(
    15, gl, textureProgramInfo,
    hospitalModel.obj, hospitalModel.mtl
  );

  for (const agent of hospitals) {
    assignModelToAgents([agent], baseHospital, {
      scale: { x: 1.6, y: 0.8, z: 1.6 },
      programType: 'texture',
      texture: simpleBuildingTexture,
    });

    scene.addObject(agent);
  }

  // Destinations
  const baseDestination = createBaseModelWithMtl(
    16, gl, textureProgramInfo, 
    destinationModel.obj, destinationModel.mtl
  );

  for (const agent of destinations) {
    assignModelToAgents([agent], baseDestination, {
      scale: { x: 2, y: 2, z: 2 },
      programType: 'texture',
      texture: simpleBuildingTexture,
    });

    scene.addObject(agent);
  }
}

// Function to draw an object with texture
function drawObjectTextured(gl, programInfo, object, viewProjectionMatrix, fract) {
  // Calculate interpolation for position
  const newPos = object.posArray;
  const oldPos = object.oldPosArray || newPos; // In the first frame oldPosArray is undefined

  // Prepare the vector for translation and scale
  let v3_tra = [
    // Basically, calculate the % of movement between old and new position
    oldPos[0] + (newPos[0] - oldPos[0]) * fract, // x
    oldPos[1] + (newPos[1] - oldPos[1]) * fract, // y
    oldPos[2] + (newPos[2] - oldPos[2]) * fract, // z
  ];
  let v3_sca = object.scaArray;

  // Calculate interpolation for rotation
  const newRot = object.targetRotY;
  const oldRot = object.oldRotY;
  
  // Handle rotation interpolation with angle wrapping (only for agents with targetRotY)
  let rotY = object.rotRad.y; // Default to current rotation
  if (newRot !== undefined && oldRot !== undefined) {
    let deltaRot = newRot - oldRot;

    // Wrap around if the difference is greater than 180 degrees
    if (deltaRot > Math.PI) {
      deltaRot -= 2 * Math.PI;
    } else if (deltaRot < -Math.PI) {
      deltaRot += 2 * Math.PI;
    }

    // Interpolate rotation
    rotY = oldRot + deltaRot * fract;
  }

  // Create the individual transform matrices
  const scaMat = M4.scale(v3_sca);
  const rotXMat = M4.rotationX(object.rotRad.x);
  const rotYMat = M4.rotationY(rotY);
  const rotZMat = M4.rotationZ(object.rotRad.z);
  const traMat = M4.translation(v3_tra);

  // Create the composite matrix with all transformations
  let transforms = M4.identity();
  transforms = M4.multiply(scaMat, transforms);
  transforms = M4.multiply(rotXMat, transforms);
  transforms = M4.multiply(rotYMat, transforms);
  transforms = M4.multiply(rotZMat, transforms);
  transforms = M4.multiply(traMat, transforms);

  object.matrix = transforms;

  // Apply the projection to the final matrix for the
  // World-View-Projection
  const wvpMat = M4.multiply(viewProjectionMatrix, transforms);
  const normalMat = M4.transpose(M4.inverse(transforms));

  // Model uniforms
  let uniforms = {
    u_world: transforms,
    u_worldInverseTransform: normalMat,
    u_worldViewProjection: wvpMat,
    
    u_texture: object.texture,
    u_shininess: object.shininess,
  };
  
  twgl.setUniforms(programInfo, uniforms);
  gl.bindVertexArray(object.vao);
  twgl.drawBufferInfo(gl, object.bufferInfo);
}

// Function to do the actual display of the objects
async function drawScene() {
  // Compute time elapsed since last frame
  let now = Date.now();
  let deltaTime = now - then;
  elapsed += deltaTime;
  let fract = Math.min(1.0, elapsed / duration);
  then = now;

  // Update traffic lights each frame
  updateTrafficLights(trafficLights, greenTexture, redTexture, scene, baseCubeTexGlobal);

  // Update ambulance siren lights position and state
  updateAmbulanceLights(ambulances, scene, fract);

  // Clear the canvas
  gl.clearColor(0, 0, 0, 1);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

  // tell webgl to cull faces
  gl.enable(gl.CULL_FACE);
  gl.enable(gl.DEPTH_TEST);

  scene.camera.checkKeys();
  const viewProjectionMatrix = setupViewProjection(gl);

  // Texture shader

  // Get lights close to camera
  const closeLights = getLightsCloseToCamera(NUM_LIGHTS - 1, scene);

  // Prepare light arrays for the shader
  let lightPositions = [];
  let diffuseLights = [];
  let specularLights = [];

  // Always add global light (scene.lights[0]) at index 0
  lightPositions.push(...scene.lights[0].posArray);
  diffuseLights.push(...scene.lights[0].diffuse);
  specularLights.push(...scene.lights[0].specular);

  // Add local lights after the global light
  for (const light of closeLights) {
    // Skip the global light (index 0) if it was included in closeLights
    if (light === scene.lights[0]) continue;
    
    // ... used to add the each number of the data individually
    // Since the shaders expects a plain array
    // Ref: https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Operators/Spread_syntax
    lightPositions.push(...light.posArray);
    diffuseLights.push(...light.diffuse);
    specularLights.push(...light.specular);
  }

  let textureUniforms = {
    u_viewWorldPosition: scene.camera.posArray,
    u_lightWorldPosition: lightPositions,
    
    // Global light
    u_ambientLight: scene.lights[0].ambient,
    u_globalDiffuseLight: scene.lights[0].diffuse,
    u_globalSpecularLight: scene.lights[0].specular,

    // Local lights
    u_diffuseLight: diffuseLights,
    u_specularLight: specularLights,
    
    // Attenuation
    u_constant: 1.0,
    u_linear: 0.15,
    u_quadratic: 0.15,
  };

  // Draw objects
  gl.useProgram(textureProgramInfo.program);
  twgl.setUniforms(textureProgramInfo, textureUniforms);

  for (let object of scene.objects) {
    if (object.programType === 'texture') {
      drawObjectTextured(gl, textureProgramInfo, object, viewProjectionMatrix, fract);
    }
  }

  // Update the scene after the elapsed duration
  if (elapsed >= duration) {
    elapsed = 0;
    await update();
  }
  
  requestAnimationFrame(drawScene);
}

function setupViewProjection(gl) {
  // Field of view of 60 degrees vertically, in radians
  const fov = 60 * Math.PI / 180;
  const aspect = gl.canvas.clientWidth / gl.canvas.clientHeight;

  // Matrices for the world view
  const projectionMatrix = M4.perspective(fov, aspect, 1, 200);

  const cameraPosition = scene.camera.posArray;
  const target = scene.camera.targetArray;
  const up = [0, 1, 0];

  const cameraMatrix = M4.lookAt(cameraPosition, target, up);
  const viewMatrix = M4.inverse(cameraMatrix);
  const viewProjectionMatrix = M4.multiply(projectionMatrix, viewMatrix);

  return viewProjectionMatrix;
}

// Setup a UI using lil-gui
function setupUI() {
  const gui = new GUI();

  // Actions
  const actions = {
    trackCar: function() {
      if (agents.length > 0) {
        // Get random car
        const randomIdx = Math.floor(Math.random() * agents.length);
        const car = agents[randomIdx];
        this.trackAgent(car);
      }
    },
    trackAmbulance: function() {
      if (ambulances.length > 0) {
        // Get random ambulance
        const randomIdx = Math.floor(Math.random() * ambulances.length);
        const ambulance = ambulances[randomIdx];
        this.trackAgent(ambulance);
      }
    },
    trackEmergency: function() {
      if (ambulances.length > 0) {
        for (const ambulance of ambulances) {
          if (ambulance.has_emergency) {
            this.trackAgent(ambulance);
            return;
          }
        }
      }
    },
    stopTracking: function() {
      scene.camera.setTargetObject(null);
    },
    resetCamera: function() {
      scene.camera.setTargetObject(null);
      scene.camera.distance = 10;
      scene.camera.elevation = 0.6;
      scene.camera.azimuth = 4;
      scene.camera.panOffset = [0, 8, 0];
      scene.camera.target = { x: 0, y: 0, z: 0 };
    },
    trackAgent: function(agent) {
      scene.camera.setTargetObject(agent);
      scene.camera.distance = 1;
      scene.camera.elevation = 1.5;
      scene.camera.azimuth = 3.14;
      scene.camera.panOffset = [0, 5, 0];
    },
    centerCamera: function() {
      scene.camera.setTargetObject(null);
      scene.camera.distance = 20;
      scene.camera.elevation = 1.5;
      scene.camera.azimuth = 3.14;
      scene.camera.panOffset = [0, 8, 0];
      scene.camera.target = { x: 11.5, y: 0, z: 11.5 };
    },
    restartSimulation: async function() {
      await restartSimulation();
    }
  };

  // Simulation Parameters
  const simFolder = gui.addFolder('Simulación');
  const durationObj = { value: duration };
  simFolder.add(durationObj, 'value', 1, 1000, 100).onChange((value) => { duration = value; })
    .name('Duración (ms)');
  simFolder.add(simulationParams, 'vehicle_spawn_rate', 1, 50, 1).name('Tasa Aparición')
  simFolder.add(simulationParams, 'vehicles_per_step', 1, 4, 1).name('Vehículos x Paso');
  simFolder.add(simulationParams, 'ambulance_per_step', 0, 4, 1).name('Ambulancias x Paso');
  simFolder.add(simulationParams, 'emergency_chance', 0, 1, 0.1).name('Prob. Emergencia');
  simFolder.add(actions, 'restartSimulation').name('Reiniciar Simulación');

  // Camera
  const cameraFolder = gui.addFolder('Cámara');
  cameraFolder.add(scene.camera, 'distance', 1, 50).decimals(2).name('Distancia');
  cameraFolder.add(scene.camera, 'elevation', -Math.PI / 2, Math.PI / 2).decimals(2).name('Elevación');
  cameraFolder.add(scene.camera, 'azimuth', -Math.PI, Math.PI).decimals(2).name('Azimut');
  cameraFolder.close();

  // Global light
  const globalLightFolder = gui.addFolder('Luz global');
  // Position
  const posFolder = globalLightFolder.addFolder('Posición');
  posFolder.add(scene.lights[0].position, 'x', -100, 100).decimals(2)
  posFolder.add(scene.lights[0].position, 'y', -100, 100).decimals(2)
  posFolder.add(scene.lights[0].position, 'z', -100, 100).decimals(2)
  // Colors
  const colorFolder = globalLightFolder.addFolder('Colores');
  colorFolder.addColor(scene.lights[0], 'ambient').name('Ambiental');
  colorFolder.addColor(scene.lights[0], 'diffuse').name('Difusa');
  colorFolder.addColor(scene.lights[0], 'specular').name('Especular');
  globalLightFolder.close();

  // Add actions to the folder
  const actionsFolder = gui.addFolder('Acciones');
  actionsFolder.add(actions, 'trackCar').name('Seguir Coche Aleatorio');
  actionsFolder.add(actions, 'trackAmbulance').name('Seguir Ambulancia Aleatoria');
  actionsFolder.add(actions, 'trackEmergency').name('Seguir Ambulancia en Emergencia');
  actionsFolder.add(actions, 'stopTracking').name('Detener Seguimiento');
  actionsFolder.add(actions, 'resetCamera').name('Reiniciar Cámara');
  actionsFolder.add(actions, 'centerCamera').name('Centrar Cámara');
}

main();
