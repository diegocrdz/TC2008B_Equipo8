/*
 * Dictionaries to store the loaded 3D models and their materials
 *
 * Aquiba Yudah Benarroch Bittan, A01783710 
 * Diego Córdova Rodríguez, A01781166 
 * Lorena Estefanía Chewtat Torres, A01785378
 * 
 * 2025-12-04
 */

'use strict';

// Utils
import { createTexture } from './utils.js';

// Import building models and materials

// Buildings
import building1Obj from '../assets/models/Buildings/building1.obj?raw';
import building1Mtl from '../assets/models/Buildings/building1.mtl?raw';

import building2Obj from '../assets/models/Buildings/building2.obj?raw';
import building2Mtl from '../assets/models/Buildings/building2.mtl?raw';

import building3Obj from '../assets/models/Buildings/building3.obj?raw';
import building3Mtl from '../assets/models/Buildings/building3.mtl?raw';

import building4Obj from '../assets/models/Buildings/building4.obj?raw';
import building4Mtl from '../assets/models/Buildings/building4.mtl?raw';

import building5Obj from '../assets/models/Buildings/building5.obj?raw';
import building5Mtl from '../assets/models/Buildings/building5.mtl?raw';

// Hospital
import hospitalObj from '../assets/models/Buildings/hospital.obj?raw';
import hospitalMtl from '../assets/models/Buildings/hospital.mtl?raw';

// Destination
import destinationObj from '../assets/models/Destinations/destination.obj?raw';
import destinationMtl from '../assets/models/Destinations/destination.mtl?raw';

// Traffic Lights
import trafficLightObj from '../assets/models/Trafficlights/trafficlight.obj?raw';
import trafficLightMtl from '../assets/models/Trafficlights/trafficlight.mtl?raw';

// Roads
import roadStraightObj from '../assets/models/Roads/road_straight.obj?raw';
import roadStraightMtl from '../assets/models/Roads/road_straight.mtl?raw';

// Cars
import car1Obj from '../assets/models/Vehicles/car1.obj?raw';
import car1Mtl from '../assets/models/Vehicles/car1.mtl?raw';

import car2Obj from '../assets/models/Vehicles/car2.obj?raw';
import car2Mtl from '../assets/models/Vehicles/car2.mtl?raw';

import car3Obj from '../assets/models/Vehicles/car3.obj?raw';
import car3Mtl from '../assets/models/Vehicles/car3.mtl?raw';

// Ambulance
import ambulanceObj from '../assets/models/Vehicles/ambulance.obj?raw';
import ambulanceMtl from '../assets/models/Vehicles/ambulance.mtl?raw';

// Export the models dictionaries

// Buildings (Obstacles)
export const buildingModels = {
    0: {
        obj: building1Obj,
        mtl: building1Mtl,
        type: 'texture',
    },
    1: {
        obj: building2Obj,
        mtl: building2Mtl,
        type: 'texture',
    },
    2: {
        obj: building3Obj,
        mtl: building3Mtl,
        type: 'texture',
    },
    3: {
        obj: building4Obj,
        mtl: building4Mtl,
        type: 'texture',
    },
    4: {
        obj: building5Obj,
        mtl: building5Mtl,
        type: 'texture',
    },
};

// Hospital model
export const hospitalModel = {
    obj: hospitalObj,
    mtl: hospitalMtl,
    type: 'phong',
};

// Destination model
export const destinationModel = {
    obj: destinationObj,
    mtl: destinationMtl,
    type: 'texture',
};

// Cars
export const carModels = {
    0: {
        obj: car1Obj,
        mtl: car1Mtl,
        type: 'texture',
    },
    1: {
        obj: car2Obj,
        mtl: car2Mtl,
        type: 'texture',
    },
    2: {
        obj: car3Obj,
        mtl: car3Mtl,
        type: 'texture',
    },
};

// Ambulance model
export const ambulanceModel = {
    obj: ambulanceObj,
    mtl: ambulanceMtl,
    type: 'texture',
};

// Traffic Light model
export const trafficLightModel = {
    obj: trafficLightObj,
    mtl: trafficLightMtl,
    type: 'texture',
};

// Road Straight model
export const roadStraightModel = {
    obj: roadStraightObj,
    mtl: roadStraightMtl,
    type: 'texture',
};

// Textures to be created with gl context
let skyboxTexture;
let sidewalkTexture;
let simpleBuildingTexture;
let simpleBuildingTextureB;
let simpleBuildingTextureA;
let complexBuildingTexture;
let car1Texture;
let car2Texture;
let car3Texture;
let ambulanceTexture;
let greenTexture;
let redTexture;

// Function to initialize all textures with gl context
export function initTextures(gl) {
  skyboxTexture = createTexture(gl, '../assets/textures/Skyboxes/space.jpg');
  sidewalkTexture = createTexture(gl, '../assets/textures/Sidewalks/sidewalk.png');
  simpleBuildingTexture = createTexture(gl, '../assets/textures/Buildings/simple_buildings.png');
  simpleBuildingTextureB = createTexture(gl, '../assets/textures/Buildings/simple_buildings_b.png');
  simpleBuildingTextureA = createTexture(gl, '../assets/textures/Buildings/simple_buildings_a.png');
  complexBuildingTexture = createTexture(gl, '../assets/textures/Buildings/citybits_texture.png');
  car1Texture = createTexture(gl, '../assets/textures/Vehicles/car1.png');
  car2Texture = createTexture(gl, '../assets/textures/Vehicles/car2.png');
  car3Texture = createTexture(gl, '../assets/textures/Vehicles/car3.png');
  ambulanceTexture = createTexture(gl, '../assets/textures/Vehicles/ambulance.png');
  greenTexture = createTexture(gl, '../assets/textures/Trafficlights/green.png');
  redTexture = createTexture(gl, '../assets/textures/Trafficlights/red.png');
}

// Export textures
export {
  skyboxTexture, sidewalkTexture, simpleBuildingTexture,
  simpleBuildingTextureA, simpleBuildingTextureB,
  complexBuildingTexture, car1Texture,
  car2Texture, car3Texture, ambulanceTexture,
  greenTexture, redTexture,
};