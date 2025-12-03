/*
 * Utils functions for the visualization
 * These functions can be reutilized in random_agents.js and 
 * api_connection.js
 *
 * Diego Córdova Rodríguez
 * Lorena Estefanía Chewtat Torres
 * Aquiba Yudah Benarroch Bittán
 * 
 * 2025-11-27
 */

import * as twgl from 'twgl-base.js';
import { Object3D } from '../libs/object3d.js';
import { Light3D } from '../libs/light3d.js';
import { loadMtl } from '../libs/obj_loader.js';

// Helper function to create a texture
export function createTexture(gl, src) {
  return twgl.createTexture(gl, {
    min: gl.LINEAR,
    mag: gl.LINEAR,
    src: src,
  });
}

// Function to get the closest lights to the camera
export function getLightsCloseToCamera(numLights, scene) {
  // Get camera position
  const camPos = scene.camera.posArray;
  const [cx, cy, cz] = camPos;

  // Compute distances to all lights
  let lightDistances = [];
  for (let i = 0; i < scene.lights.length; i++) {
    const lightPos = scene.lights[i].posArray;

    // Calculate vector from camera to light
    const dx = lightPos[0] - cx;
    const dy = lightPos[1] - cy;
    const dz = lightPos[2] - cz;

    // Get the magnitude of the distance vector
    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

    // Store index and distance
    lightDistances.push({ index: i, distance: dist });
  }

  // Sort lights by distance
  lightDistances.sort((a, b) => a.distance - b.distance);

  // Get the closest numLights lights
  let closestLights = [];

  // Math.min lets us have less than numLights if there are not enough lights
  for (let i = 0; i < Math.min(numLights, lightDistances.length); i++) {
    closestLights.push(scene.lights[lightDistances[i].index]);
  }

  return closestLights;
}

// Helper function to create a light
export function createLight(x, y, z, offsetX, offsetY, offsetZ, color, intensity) {
  const light = new Light3D();
  light.position = { x: x + offsetX, y: y + offsetY, z: z + offsetZ };
  light.ambient = [0.1, 0.1, 0.1, 1.0];
  light.diffuse = color;
  light.specular = color;
  return light;
}

// Function that gets the direction of a car and returns the correct rotation
export function getRotation(oldPos, newPos) {
  const dx = newPos[0] - oldPos[0];
  const dz = newPos[2] - oldPos[2];

  // If no movement, return null to keep previous rotation
  if (Math.abs(dx) == 0 && Math.abs(dz) == 0) {
    return null;
  }

  // If diagonal movement (both dx and dz are non-zero)
  if (Math.abs(dx) > 0 && Math.abs(dz) > 0) {
    // up-right
    if (dx > 0 && dz > 0) {
      return Math.PI / 4; // 45 degrees
    } else if (dx < 0 && dz > 0) {
      // up-left
      return (3 * Math.PI) / 4; // 135 degrees
    } else if (dx < 0 && dz < 0) {
      // down-left
      return (-3 * Math.PI) / 4; // 225 degrees
    } else if (dx > 0 && dz < 0) {
      // down-right
      return -Math.PI / 4; // 315 degrees
    }
  }

  let direction = "";

  if (Math.abs(dx) > Math.abs(dz)) {
    // Horizontal movement (left/right)
    if (dx > 0) {
      direction = "Right";
    } else {
      direction = "Left";
    }
  } else {
    // Forward/backward movement (up/down in grid)
    if (dz > 0) {
      direction = "Up";
    } else {
      direction = "Down";
    }
  }

  switch (direction) {
    case "Right":
      return Math.PI / 2; // 90 degrees
    case "Left":
      return -Math.PI / 2; // 270 degrees
    case "Up":
      return 0; // 0 degrees
    case "Down":
      return Math.PI; // 180 degrees
    default:
      return 0;
  }
}

// Helper function to apply common properties to an agent/object
export function applyObjectProperties(obj, props) {
  if (props.scale) obj.scale = props.scale;
  if (props.color) obj.color = props.color;
  if (props.shininess !== undefined) obj.shininess = props.shininess;
  if (props.texture) obj.texture = props.texture;
  if (props.programType) obj.programType = props.programType;
  if (props.rotRad) obj.rotRad = props.rotRad;
  if (props.position) obj.position = props.position;
}

// Helper function to assign model data to multiple agents
export function assignModelToAgents(agents, baseModel, props = {}) {
  for (const agent of agents) {
    agent.arrays = baseModel.arrays;
    agent.bufferInfo = baseModel.bufferInfo;
    agent.vao = baseModel.vao;
    applyObjectProperties(agent, props);
  }
}

// Helper function to create a base model object
export function createBaseModel(id, gl, programInfo, modelObj, modelMtl, { loadMtl }) {
  // Create the base model
  const baseModel = new Object3D(id);

  // Load MTL if provided
  if (modelMtl) loadMtl(modelMtl);

  // Prepare VAO
  baseModel.prepareVAO(gl, programInfo, modelObj);
  baseModel.programType = 'texture';

  return baseModel;
}

// Helper function to get rotation of a road light based on direction
export function getRotationByDirection(direction) {
  switch (direction) {
    case "Right":
      return { x: 0, y: Math.PI / 2, z: 0 };
    case "Left":
      return { x: 0, y: -Math.PI / 2, z: 0 };
    case "Up":
      return { x: 0, y: 0, z: 0 };
    case "Down":
      return { x: 0, y: Math.PI, z: 0 };
    default:
      return { x: 0, y: 0, z: 0 };
  }
}

// Helper function to get rotation for traffic lights
// This is used to position the traffic lights correctly on sidewalks
export function getTrafficLightRotation(direction) {
  switch (direction) {
    case "Right":
      return { x: 0, y: Math.PI * 3 / 2, z: 0 }; // 270 degrees
    case "Left":
      return { x: 0, y: Math.PI * 3 / 2, z: 0 }; // 270 degrees
    case "Up":
      return { x: 0, y: Math.PI, z: 0 };
    case "Down":
      return { x: 0, y: Math.PI, z: 0 };
    default:
      return { x: 0, y: 0, z: 0 };
  }
}

// Helper function to create multiple base models already prepared with VAO
// Used to add multiple models at once, especially new vehicles in the scene
export function createBaseModels(gl, programInfo, models, { loadMtl }) {
  // Array to store base models
  const baseModels = {};
  let id = 100; // Starting ID for models

  // Create each base model
  for (const [key, model] of Object3D.entries(models)) {
    // Load MTL if provided
    if (model.mtl) loadMtl(model.mtl);

    // Create the base model
    const baseModel = new Object3D(id++);

    // Prepare VAO
    baseModel.prepareVAO(gl, programInfo, model.obj);
    baseModel.programType = 'texture';

    // Store in the base models array
    baseModels[key] = baseModel;
  }
  return baseModels;
}

// Helper function to create a single base model with MTL loading and VAO preparation
// Used to initially create models
export function createBaseModelWithMtl(id, gl, programInfo, modelObj, modelMtl) {  
  // Load MTL
  loadMtl(modelMtl);
  
  // Create the base model
  const baseModel = new Object3D(id);

  // Prepare VAO
  baseModel.prepareVAO(gl, programInfo, modelObj);

  // Set program type
  baseModel.programType = 'texture';
  
  return baseModel;
}

// Helper function to get offset based on traffic light direction
// Used to position the light correctly relative to the traffic light
export function getTrafficLightOffset(direction, distance = 0.5) {
  let offsetX = 0;
  let offsetZ = 0;

  switch (direction) {
    case "Right":
      offsetZ = distance;
      break;
    case "Left":
      offsetZ = distance;
      break;
    case "Up":
      offsetX = -distance;
      break;
    case "Down":
      offsetX = -distance;
      break;
  }

  return { offsetX, offsetZ };
}

// Helper function to get light color based on traffic light state
// Green for true, Red for false
export function getTrafficLightColor(state) {
  return state
    ? [0.0, 0.8, 0.0, 1.0] // Green
    : [0.8, 0.0, 0.0, 1.0]; // Red
}

// Helper function to update all traffic lights
export function updateTrafficLights(
  trafficLights,
  greenTexture,
  redTexture,
  scene,
  baseCube
) {
  // Distance and height offsets for light and cube
  const distance = -0.65;
  const offsetY = 0.8;
  const cubeOffsetY = 0.9;

  for (const tl of trafficLights) {
    // Apply position offset on first update if not yet applied
    if (!tl.positionOffsetApplied) {
      const posOffset = getTrafficLightOffset(tl.direction);
      tl.position = {
        x: tl.position.x + posOffset.offsetX,
        y: tl.position.y,
        z: tl.position.z + posOffset.offsetZ
      };
      tl.positionOffsetApplied = true;
    }

    // Set light position and color
    const pos = tl.position;
    const heightOffset = pos.y + offsetY;
    const lightColor = getTrafficLightColor(tl.state);
    const offset = getTrafficLightOffset(tl.direction, distance);

    // Create light if it doesn't exist
    if (!tl.light) {
      tl.light = new Light3D(
        [pos.x + offset.offsetX, heightOffset, pos.z + offset.offsetZ],
        [0.1, 0.1, 0.1, 1.0], // Ambient
        lightColor, // Diffuse
        lightColor // Specular
      );
      scene.addLight(tl.light);

    } else {
      // Update existing light position and color
      tl.light.position = {
        x: pos.x + offset.offsetX,
        y: heightOffset,
        z: pos.z + offset.offsetZ
      };

      tl.light.diffuse = lightColor;
      tl.light.specular = lightColor;
    }

    // Create light cube if it doesn't exist
    if (!tl.lightCube) {
      tl.lightCube = new Object3D();
      
      // Copy geometry data from base cube
      if (baseCube) {
        tl.lightCube.arrays = baseCube.arrays;
        tl.lightCube.bufferInfo = baseCube.bufferInfo;
        tl.lightCube.vao = baseCube.vao;
      }
      
      tl.lightCube.position = {
        x: pos.x + offset.offsetX,
        y: pos.y + cubeOffsetY,
        z: pos.z + offset.offsetZ
      };

      tl.lightCube.scale = { x: 0.05, y: 0.1, z: 0.05 };
      tl.lightCube.texture = tl.state ? greenTexture : redTexture;
      tl.lightCube.programType = 'texture';
      
      scene.addObject(tl.lightCube);
      
    } else {
      // Update existing light cube position and texture
      tl.lightCube.position = {
        x: pos.x + offset.offsetX,
        y: pos.y + cubeOffsetY,
        z: pos.z + offset.offsetZ
      };
      // Update texture based on current state
      tl.lightCube.texture = tl.state ? greenTexture : redTexture;
    }
  }
}

// Helper function to create/update ambulance siren lights
export function updateAmbulanceLights(ambulances, scene, fract) {
  const offsetY = 0.8;

  for (const ambulance of ambulances) {
    if (!ambulance.posArray) continue;

    // If the ambulance has been marked to have emergency siren on
    // and doesnt already have a siren light, create it
    if (ambulance.light && !ambulance.sirenLight) {
      // Set light position
      const pos = ambulance.position;
      const heightOffset = pos.y + offsetY;

      // Determine light color based on light state
      const lightColor = ambulance.light_state === "blue"
        ? [0.0, 0.0, 1.0, 1.0] // Blue
        : [1.0, 0.0, 0.0, 1.0]; // Red
      
      // Create siren light
      const light = new Light3D(
        [pos.x, heightOffset, pos.z],
        [0.1, 0.1, 0.1, 1.0], // Ambient
        lightColor, // Diffuse
        lightColor // Specular
      );

      ambulance.sirenLight = light;
      scene.addLight(light);

    } else if (ambulance.light && ambulance.sirenLight) {
      // If the siren exists, update its position and color
      // Set light position
      const pos = ambulance.position;
      const heightOffset = pos.y + offsetY;

      // Calculate light position with fract for interpolation
      const newPos = ambulance.posArray;
      const oldPos = ambulance.oldPosArray || newPos; // In the first frame oldPosArray is undefined

      // Interpolate position in x and z (y remains constant)
      const interpX = oldPos[0] + (newPos[0] - oldPos[0]) * fract;
      const interpZ = oldPos[2] + (newPos[2] - oldPos[2]) * fract;

      // Update light position
      ambulance.sirenLight.position = {
        x: interpX,
        y: heightOffset,
        z: interpZ
      };
      
      // Determine light color based on light state
      const lightColor = ambulance.light_state === "blue"
        ? [0.0, 0.0, 1.0, 1.0] // Blue
        : [1.0, 0.0, 0.0, 1.0]; // Red
      
      // Update light color
      ambulance.sirenLight.diffuse = lightColor;
      ambulance.sirenLight.specular = lightColor;

    } else if (!ambulance.light && ambulance.sirenLight) {
      // If the ambulance no longer has emergency siren on
      // but still has a siren light, remove it from the scene
      scene.removeLight(ambulance.sirenLight);
      ambulance.sirenLight = null;
    }
  }
}