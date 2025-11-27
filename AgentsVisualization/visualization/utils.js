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

// Helper function to create a texture
export function createTexture(gl, src) {
  return twgl.createTexture(gl, {
    min: gl.LINEAR,
    mag: gl.LINEAR,
    src: src,
  });
}

// Function that gets the direction of a car and returns the correct rotation
export function getRotationFromDirection(direction) {
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