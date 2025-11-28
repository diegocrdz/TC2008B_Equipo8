/*
 * Utils functions for the visualization
 *
 * Diego Córdova Rodríguez
 * Lorena Estefanía Chewtat Torres
 * Aquiba Yudah Benarroch Bittán
 * 
 * 2025-11-27
 */

import * as twgl from 'twgl-base.js';
import { Object3D } from '../libs/object3d.js';
import { loadMtl } from '../libs/obj_loader.js';

// Helper function to create a texture
export function createTexture(gl, src) {
  return twgl.createTexture(gl, {
    min: gl.LINEAR,
    mag: gl.LINEAR,
    src: src,
  });
}

// Function that gets the direction of a car and returns the correct rotation
export function getRotation(oldPos, newPos) {
  const dx = newPos[0] - oldPos[0];
  const dz = newPos[2] - oldPos[2];

  // If no movement, return null to keep previous rotation
  if (Math.abs(dx) == 0 && Math.abs(dz) == 0) {
    return null;
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
  const baseModel = new (require('../libs/object3d.js').Object3D)(id);
  if (modelMtl) loadMtl(modelMtl);
  baseModel.prepareVAO(gl, programInfo, modelObj);
  baseModel.programType = 'texture';
  return baseModel;
}

// Helper function to get rotation based on direction
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

// Helper function to get rotation for traffic lights (different from roads)
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

// Helper function to batch create models
export function createBaseModels(gl, programInfo, models, { loadMtl }) {
  const baseModels = {};
  const ObjectClass = require('../libs/object3d.js').Object3D;
  
  let id = 100;
  for (const [key, model] of Object.entries(models)) {
    if (model.mtl) loadMtl(model.mtl);
    const baseModel = new ObjectClass(id++);
    baseModel.prepareVAO(gl, programInfo, model.obj);
    baseModel.programType = 'texture';
    baseModels[key] = baseModel;
  }
  return baseModels;
}

// Helper function to create a single base model with MTL loading and VAO preparation
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
export function getTrafficLightColor(state) {
  return state
    ? [0.0, 0.8, 0.0, 1.0] // Green
    : [0.8, 0.0, 0.0, 1.0]; // Red
}

// Helper function to update all traffic lights
export function updateTrafficLights(trafficLights, greenTexture, redTexture) {
  for (const tl of trafficLights) {
    const pos = tl.position;
    const heightOffset = pos.y + 0.8;
    const lightColor = getTrafficLightColor(tl.state);

    // Update light
    if (tl.light) {
      const offset = getTrafficLightOffset(tl.direction, -0.65);
      
      tl.light.position = {
        x: pos.x + offset.offsetX,
        y: heightOffset,
        z: pos.z + offset.offsetZ
      };

      tl.light.diffuse = lightColor;
      tl.light.specular = lightColor;
    }

    // Update light cube
    if (tl.lightCube) {
      const offset = getTrafficLightOffset(tl.direction, -0.65);
      
      tl.lightCube.position = {
        x: tl.position.x + offset.offsetX,
        y: tl.position.y + 0.9,
        z: tl.position.z + offset.offsetZ
      };

      tl.lightCube.texture = tl.state ? greenTexture : redTexture;
    }
  }
}