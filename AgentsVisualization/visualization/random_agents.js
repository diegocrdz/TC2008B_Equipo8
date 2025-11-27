/*
 * Base program for a 3D scene that connects to an API to get the movement
 * of agents.
 * The scene shows colored cubes
 *
 * Diego Córdova Rodríguez
 * Lorena Estefanía Chewtat Torres
 * Aquiba Yudah Benarroch Bittán
 * 
 * 2025-11-27
 */

'use strict';

import * as twgl from 'twgl-base.js';
import GUI from 'lil-gui';
import { M4 } from '../libs/3d-lib';
import { Scene3D } from '../libs/scene3d';
import { Object3D } from '../libs/object3d';
import { Light3D } from '../libs/light3d';
import { Camera3D } from '../libs/camera3d';
import { cubeTextured } from '../libs/shapes';
import { loadMtl } from '../libs/obj_loader.js';

// Functions and arrays for the communication with the API
import {
  initAgentsModel, update,
  agents, getAgents,
  ambulances, getAmbulances,
  obstacles, getObstacles,
  trafficLights, getTrafficLights,
  roads, getRoads,
  hospitals, getHospitals,
  destinations, getDestinations,
  sidewalks, getSidewalks,
} from '../libs/api_connection.js';

// Define the shader code, using GLSL 3.00
// Texture shaders
import vsTex from '../assets/shaders/vs_multi_lights_attenuation.glsl?raw';
import fsTex from '../assets/shaders/fs_multi_lights_attenuation.glsl?raw';

// Import the models dictionaries
import {
  buildingModels, hospitalModel,
  trafficLightModel,
  carModels, ambulanceModel,
  destinationModel, roadStraightModel
} from './random_objects.js';

// Import helper functions
import {
  createTexture,
  getRotationFromDirection,
  getLightsCloseToCamera,
} from './utils.js';

// Create the 3D scene
const scene = new Scene3D();

// Global variables
let textureProgramInfo = undefined;
let gl = undefined;
const duration = 500; // ms
let elapsed = 0;
let then = 0;
const NUM_LIGHTS = 10;

// Global textures for traffic lights
let greenTexture = undefined;
let redTexture = undefined;

// Main function is async to be able to make the requests
async function main() {
  // Setup the canvas area
  const canvas = document.querySelector('canvas');
  gl = canvas.getContext('webgl2');
  twgl.resizeCanvasToDisplaySize(gl.canvas);
  gl.viewport(0, 0, gl.canvas.width, gl.canvas.height);

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
    [0.5, 0.5, 0.5, 1.0], // Ambient
    [0.5, 0.5, 0.5, 1.0], // Diffuse
    [0.5, 0.5, 0.5, 1.0], // Specular
  );
  scene.addLight(light);
}

function setupObjects(scene, gl) {
  // Create textures used for the models
  const skyboxTexture = createTexture(gl, '../assets/textures/Skybox/night.png');
  const sidewalkTexture = createTexture(gl, '../assets/textures/Sidewalk/sidewalk.png');
  const simpleBuildingTexture = createTexture(gl, '../assets/textures/Building/simple_buildings.png');
  const simpleBuildingTextureB = createTexture(gl, '../assets/textures/Building/simple_buildings_b.png');
  const simpleBuildingTextureA = createTexture(gl, '../assets/textures/Building/simple_buildings_a.png');
  const complexBuildingTexture = createTexture(gl, '../assets/textures/Building/citybits_texture.png');
  const car1Texture = createTexture(gl, '../assets/textures/Vehicles/car1.png');
  const car2Texture = createTexture(gl, '../assets/textures/Vehicles/car2.png');
  const car3Texture = createTexture(gl, '../assets/textures/Vehicles/car3.png');
  const ambulanceTexture = createTexture(gl, '../assets/textures/Vehicles/ambulance.png');
  greenTexture = createTexture(gl, '../assets/textures/Trafficlight/green.png');
  redTexture = createTexture(gl, '../assets/textures/Trafficlight/red.png');

  // Cube with texture
  // Used for skybox and sidewalks
  const baseCubeTex = new Object3D(1);
  baseCubeTex.arrays = cubeTextured(2);
  baseCubeTex.bufferInfo = twgl.createBufferInfoFromArrays(gl, baseCubeTex.arrays);
  baseCubeTex.vao = twgl.createVAOFromBufferInfo(gl, textureProgramInfo, baseCubeTex.bufferInfo);

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
  const car1 = new Object3D(4);
  const car2 = new Object3D(5);
  const car3 = new Object3D(6);

  // Load MTLs for each car
  loadMtl(carModels[0].mtl);
  loadMtl(carModels[1].mtl);
  loadMtl(carModels[2].mtl);

  // Prepare VAOs for each car
  car1.prepareVAO(gl, textureProgramInfo, carModels[0].obj);
  car2.prepareVAO(gl, textureProgramInfo, carModels[1].obj);
  car3.prepareVAO(gl, textureProgramInfo, carModels[2].obj);
  car1.programType = 'texture';
  car2.programType = 'texture';
  car3.programType = 'texture';

  for (const agent of agents) {
    // Get random model
    const randomIndex = Math.floor(Math.random() * 3);

    // Assign the base obstacle according to the random index
    const car = [car1, car2, car3][randomIndex];

    // Set model info
    agent.arrays = car.arrays;
    agent.bufferInfo = car.bufferInfo;
    agent.vao = car.vao;
    
    agent.scale = { x: 0.1, y: 0.1, z: 0.1 };
    agent.color = [1.0, 1.0, 1.0, 1.0];
    agent.shininess = 4.0;

    // Apply rotation based on direction
    if (agent.direction) {
      const rotationY = getRotationFromDirection(agent.direction);
      agent.rotRad.y = rotationY;
      agent.rotDeg.y = rotationY * 180 / Math.PI;
    }

    // Assign texture based on random index
    switch (randomIndex) {
      case 0:
        agent.texture = car1Texture;
        break;
      case 1:
        agent.texture = car2Texture;
        break;
      case 2:
        agent.texture = car3Texture;
        break;
      default:
        agent.texture = car1Texture;
    }
    
    agent.programType = 'texture';

    scene.addObject(agent);
  }

  // Ambulances
  const ambulance = new Object3D(-4);
  
  // Load MTL for ambulance
  loadMtl(ambulanceModel.mtl);

  // Prepare VAO for ambulance
  ambulance.prepareVAO(gl, textureProgramInfo, ambulanceModel.obj);

  for (const agent of ambulances) {
    // Set model info
    agent.arrays = ambulance.arrays;
    agent.bufferInfo = ambulance.bufferInfo;
    agent.vao = ambulance.vao;

    // Model config
    agent.scale = { x: 0.1, y: 0.1, z: 0.1 };
    agent.color = [1.0, 1.0, 1.0, 1.0];
    agent.shininess = 4.0;
    agent.texture = ambulanceTexture;
    agent.programType = 'texture';

    // Apply rotation based on direction
    if (agent.direction) {
      const rotationY = getRotationFromDirection(agent.direction);
      agent.rotRad.y = rotationY;
      agent.rotDeg.y = rotationY * 180 / Math.PI;
    }

    scene.addObject(agent);
  }

  // Obstacles
  const baseObstacle0 = new Object3D(7);
  const baseObstacle1 = new Object3D(8);
  const baseObstacle2 = new Object3D(9);
  const baseObstacle3 = new Object3D(10);
  const baseObstacle4 = new Object3D(11);

  // Load MTLs for each building model
  loadMtl(buildingModels[0].mtl);
  loadMtl(buildingModels[1].mtl);
  loadMtl(buildingModels[2].mtl);
  loadMtl(buildingModels[3].mtl);
  loadMtl(buildingModels[4].mtl);

  // Prepare VAOs for each building model
  baseObstacle0.prepareVAO(gl, textureProgramInfo, buildingModels[0].obj);
  baseObstacle1.prepareVAO(gl, textureProgramInfo, buildingModels[1].obj);
  baseObstacle2.prepareVAO(gl, textureProgramInfo, buildingModels[2].obj);
  baseObstacle3.prepareVAO(gl, textureProgramInfo, buildingModels[3].obj);
  baseObstacle4.prepareVAO(gl, textureProgramInfo, buildingModels[4].obj);
  baseObstacle0.programType = 'texture';
  baseObstacle1.programType = 'texture';
  baseObstacle2.programType = 'texture';
  baseObstacle3.programType = 'texture';
  baseObstacle4.programType = 'texture';

  for (const agent of obstacles) {
    // Get random model
    const randomIndex = Math.floor(Math.random() * 5);

    // Assign the base obstacle according to the random index
    const baseObstacle = [
      baseObstacle0, baseObstacle1, 
      baseObstacle2, baseObstacle3,
      baseObstacle4,
    ][randomIndex];

    // Set model info
    agent.arrays = baseObstacle.arrays;
    agent.bufferInfo = baseObstacle.bufferInfo;
    agent.vao = baseObstacle.vao;

    // Model config
    switch (randomIndex) {
      case 0:
        agent.scale = { x: 1, y: 1.2, z: 1 };
        agent.color = [1.0, 1.0, 1.0, 1.0];
        agent.shininess = 16.0;
        agent.texture = simpleBuildingTexture;
        break;
      case 1:
        agent.scale = { x: 1, y: 1.2, z: 1 };
        agent.color = [1.0, 1.0, 1.0, 1.0];
        agent.shininess = 16.0;
        agent.texture = simpleBuildingTextureB;
        break;
      case 2:
      case 3:
        agent.scale = { x: 0.35, y: 0.5, z: 0.35 };
        agent.color = [1.0, 1.0, 1.0, 1.0];
        agent.shininess = 32.0;
        agent.texture = complexBuildingTexture;
        break;
      case 4:
        agent.scale = { x: 1, y: 1.5, z: 1 };
        agent.color = [1.0, 1.0, 1.0, 1.0];
        agent.shininess = 16.0;
        agent.texture = simpleBuildingTextureA;
        break;
    }

    agent.programType = 'texture';
    scene.addObject(agent);
  }

  // Traffic Lights
  loadMtl(trafficLightModel.mtl);

  const baseTrafficLight = new Object3D(12);
  baseTrafficLight.prepareVAO(gl, textureProgramInfo, trafficLightModel.obj);

  for (const agent of trafficLights) {
    agent.arrays = baseTrafficLight.arrays;
    agent.bufferInfo = baseTrafficLight.bufferInfo;
    agent.vao = baseTrafficLight.vao;

    agent.scale = { x: 1, y: 1, z: 1 };

    // Adjust position and rotation based on direction
    switch (agent.direction) {
      case "Right":
        agent.rotRad = { x: 0, y: Math.PI * 3 / 2, z: 0 }; // 270 degrees
        break;
      case "Left":
        agent.rotRad = { x: 0, y: Math.PI * 3 / 2, z: 0 }; // 270 degrees
        break;
      case "Up":
        agent.rotRad = { x: 0, y: Math.PI, z: 0 }; // No rotation
        break;
      case "Down":
        agent.rotRad = { x: 0, y: Math.PI, z: 0 }; // No rotation
        break;
      default:
        agent.rotRad = { x: 0, y: 0, z: 0 };
    }

    agent.programType = 'texture';
    agent.texture = complexBuildingTexture;

    scene.addObject(agent);
  }

  // Create lights for each traffic light
  for (const tl of trafficLights) {
    const pos = tl.position;
    const heightOffset = pos.y + 0.8;

    // Get offset based on direction to align with semaphore
    let offsetX = 0;
    let offsetZ = 0;

    switch (tl.direction) {
      case "Right":
        offsetZ = 0.5;
        break;
      case "Left":
        offsetZ = 0.5;
        break;
      case "Up":
        offsetX = -0.5;
        break;
      case "Down":
        offsetX = -0.5;
        break;
    }

    // Select color based on state
    const lightColor = tl.state
      ? [0.0, 0.8, 0.0, 1.0] // Green
      : [0.8, 0.0, 0.0, 1.0]; // Red

    // Create the light
    let light = new Light3D(
      [pos.x + offsetX, heightOffset, pos.z + offsetZ],
      [0.1, 0.1, 0.1, 1.0], // Ambient
      lightColor, // Diffuse
      lightColor, // Specular
    );

    // Store reference to light in traffic light object
    tl.light = light;
    scene.addLight(light);
  }

  // Create light emitter cubes for each traffic light
  for (const tl of trafficLights) {
    const lightCube = new Object3D(13);
    const pos = tl.position;
    const heightOffset = pos.y + 0.8;

    // Get offset based on direction to align with semaphore
    let offsetX = 0;
    let offsetZ = 0;

    switch (tl.direction) {
      case "Right":
        offsetZ = 0.1;
        break;
      case "Left":
        offsetZ = 0.1;
        break;
      case "Up":
        offsetX = -0.1;
        break;
      case "Down":
        offsetX = -0.1;
        break;
    }

    lightCube.position = { x: pos.x + offsetX, y: heightOffset, z: pos.z + offsetZ };
    lightCube.scale = { x: 0.05, y: 0.1, z: 0.05 };

    // Set texture and color based on state
    lightCube.shininess = 16.0;
    lightCube.programType = 'texture';
    lightCube.color = [1.0, 1.0, 1.0, 1.0]; // White so texture shows properly

    if (tl.state) {
      lightCube.texture = greenTexture; // Green texture
    } else {
      lightCube.texture = redTexture; // Red texture
    }

    lightCube.arrays = baseCubeTex.arrays;
    lightCube.bufferInfo = baseCubeTex.bufferInfo;
    lightCube.vao = baseCubeTex.vao;

    // Store reference to cube in traffic light
    tl.lightCube = lightCube;
    scene.addObject(lightCube);
  }

  // Roads
  loadMtl(roadStraightModel.mtl);

  const baseRoadStraight = new Object3D(14);
  baseRoadStraight.prepareVAO(gl, textureProgramInfo, roadStraightModel.obj);

  for (const agent of roads) {
    // Set model info
    agent.arrays = baseRoadStraight.arrays;
    agent.bufferInfo = baseRoadStraight.bufferInfo;
    agent.vao = baseRoadStraight.vao;

    // Model config
    agent.scale = { x: 0.5, y: 1, z: 0.5 };
    agent.programType = 'texture';
    agent.texture = complexBuildingTexture;

    // Set rotation based on direction
    switch (agent.direction) {
      case "Right":
      case "Left":
        // Rotate 90 degrees
        agent.rotRad = { x: 0, y: Math.PI / 2, z: 0 };
        break;
      case "Up":
      case "Down":
        // No rotation
        agent.rotRad = { x: 0, y: 0, z: 0 };
        break;
      default:
        // By default, no rotation
        agent.rotRad = { x: 0, y: 0, z: 0 };
    }

    scene.addObject(agent);
  }

  // Sidewalks
  for (const agent of sidewalks) {
    // Set model info
    agent.arrays = baseCubeTex.arrays;
    agent.bufferInfo = baseCubeTex.bufferInfo;
    agent.vao = baseCubeTex.vao;

    // Model config
    agent.scale = { x: 0.25, y: 0.1, z: 0.25 };
    agent.programType = 'texture';
    agent.texture = sidewalkTexture;

    scene.addObject(agent);
  }

  // Load MTL for hospital model
  loadMtl(hospitalModel.mtl);

  // Prepare VAO for hospital model
  const baseHospital = new Object3D(15);
  baseHospital.prepareVAO(gl, textureProgramInfo, hospitalModel.obj);

  for (const agent of hospitals) {
    // Set model info
    agent.arrays = baseHospital.arrays;
    agent.bufferInfo = baseHospital.bufferInfo;
    agent.vao = baseHospital.vao;

    // Model config
    agent.scale = { x: 1.6, y: 0.8, z: 1.6 };
    agent.programType = 'texture';
    agent.texture = simpleBuildingTexture;

    scene.addObject(agent);
  }

  // Destinations
  loadMtl(destinationModel.mtl);

  const baseDestination = new Object3D(16);
  baseDestination.prepareVAO(gl, textureProgramInfo, destinationModel.obj);

  for (const agent of destinations) {
    // Set model info
    agent.arrays = baseDestination.arrays;
    agent.bufferInfo = baseDestination.bufferInfo;
    agent.vao = baseDestination.vao;

    // Model config
    agent.scale = { x: 2, y: 2, z: 2 };
    agent.programType = 'texture';
    agent.texture = simpleBuildingTexture;

    scene.addObject(agent);
  }
}

// Function to draw an object with texture
function drawObjectTextured(gl, programInfo, object, viewProjectionMatrix, fract) {
  // Calculate interpolation
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

  // Create the individual transform matrices
  const scaMat = M4.scale(v3_sca);
  const rotXMat = M4.rotationX(object.rotRad.x);
  const rotYMat = M4.rotationY(object.rotRad.y);
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

  // Update trafficlight lights each frame
  for (const tl of trafficLights) {
    if (tl.light) {
      const pos = tl.position;
      const heightOffset = pos.y + 0.8;

      // Get offset based on direction
      let offsetX = 0;
      let offsetZ = 0;

      switch (tl.direction) {
        case "Right":
          offsetZ = -0.65;
          break;
        case "Left":
          offsetZ = -0.65;
          break;
        case "Up":
          offsetX = 0.65;
          break;
        case "Down":
          offsetX = 0.65;
          break;
      }

      // Update light position
      tl.light.position = {
        x: pos.x + offsetX,
        y: heightOffset,
        z: pos.z + offsetZ
      };

      // Update light color based on current state
      const lightColor = tl.state
        ? [0.0, 0.8, 0.0, 1.0] // Green
        : [0.8, 0.0, 0.0, 1.0]; // Red

      tl.light.diffuse = lightColor;
      tl.light.specular = lightColor;
    }

    // Update light cube position and color
    if (tl.lightCube) {
      // Get offset based on direction
      let offsetX = 0;
      let offsetZ = 0;

      switch (tl.direction) {
        case "Right":
          offsetZ = -0.65;
          break;
        case "Left":
          offsetZ = -0.65;
          break;
        case "Up":
          offsetX = 0.65;
          break;
        case "Down":
          offsetX = 0.65;
          break;
      }

      tl.lightCube.position = {
        x: tl.position.x + offsetX,
        y: tl.position.y + 0.9,
        z: tl.position.z + offsetZ
      };

      // Update cube texture based on state
      if (tl.state) {
        tl.lightCube.texture = greenTexture;
      } else {
        tl.lightCube.texture = redTexture;
      }
    }
  }

  // Clear the canvas
  gl.clearColor(0, 0, 0, 1);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

  // tell webgl to cull faces
  gl.enable(gl.CULL_FACE);
  gl.enable(gl.DEPTH_TEST);

  scene.camera.checkKeys();
  const viewProjectionMatrix = setupViewProjection(gl);

  // Texture shader

  // Get closest lights to camera
  const closeLights = getLightsCloseToCamera(NUM_LIGHTS, scene);

  // Prepare light arrays
  let lightPositions = [];
  let diffuseLights = [];
  let specularLights = [];

  // Add all scene lights
  for (let i = 0; i < closeLights.length; i++) {
    // Get current light
    const light = closeLights[i];

    // Add light properties to arrays
    lightPositions.push(...light.posArray);
    diffuseLights.push(...light.diffuse);
    specularLights.push(...light.specular);
  }

  let textureUniforms = {
    u_viewWorldPosition: scene.camera.posArray,
    u_lightWorldPosition: lightPositions,
 
    u_ambientLight: scene.lights[0].ambient,
    u_diffuseLight: diffuseLights,
    u_specularLight: specularLights,
    
    // Attenuation parameters
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

    // Update rotation of all agents based on their direction
    for (const agent of agents) {
      if (agent.direction) {
        const rotationY = getRotationFromDirection(agent.direction);
        agent.rotRad.y = rotationY;
        agent.rotDeg.y = rotationY * 180 / Math.PI;
      }
    }

    // Update rotation of ambulances
    for (const ambulance of ambulances) {
      if (ambulance.direction) {
        const rotationY = getRotationFromDirection(ambulance.direction);
        ambulance.rotRad.y = rotationY;
        ambulance.rotDeg.y = rotationY * 180 / Math.PI;
      }
    }
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

  // By default, the GUI is closed
  gui.close();

  // Camera
  const cameraFolder = gui.addFolder('Camera');
  cameraFolder.add(scene.camera, 'distance', 1, 50).decimals(2);
  cameraFolder.add(scene.camera, 'elevation', -Math.PI / 2, Math.PI / 2).decimals(2);

  // Global light
  const globalLightFolder = gui.addFolder('Global Light');
  // Position
  const posFolder = globalLightFolder.addFolder('Position');
  posFolder.add(scene.lights[0].position, 'x', -100, 100).decimals(2)
  posFolder.add(scene.lights[0].position, 'y', -100, 100).decimals(2)
  posFolder.add(scene.lights[0].position, 'z', -100, 100).decimals(2)
  // Colors
  const colorFolder = globalLightFolder.addFolder('Colors');
  colorFolder.addColor(scene.lights[0], 'ambient');
  colorFolder.addColor(scene.lights[0], 'diffuse');
  colorFolder.addColor(scene.lights[0], 'specular');

  // Actions
  const actionsFolder = gui.addFolder('Actions');
  // Actions object
  const actions = {
    trackAmbulance: function() {
      if (ambulances.length > 0) {
        scene.camera.setTargetObject(ambulances[0]);
      }
    }
  };
  actionsFolder.add(actions, 'trackAmbulance').name('Track Ambulance');
}

main();
