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
      return Math.PI; // 180 degrees
    case "Down":
      return 0; // 0 degrees
    default:
      return 0;
  }
}

