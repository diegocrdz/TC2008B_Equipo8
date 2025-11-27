/*
 * Dictionaries to store the loaded 3D models and their materials
 *
 * Diego Córdova Rodríguez
 * Lorena Estefanía Chewtat Torres
 * Aquiba Yudah Benarroch Bittán
 * 
 * 2025-11-27
 */

'use strict';

// Import building models and materials

// Buildings
import building1Obj from '../assets/models/building1.obj?raw';
import building1Mtl from '../assets/models/building1.mtl?raw';

import building2Obj from '../assets/models/building2.obj?raw';
import building2Mtl from '../assets/models/building2.mtl?raw';

import building3Obj from '../assets/models/building3.obj?raw';
import building3Mtl from '../assets/models/building3.mtl?raw';

import building4Obj from '../assets/models/building4.obj?raw';
import building4Mtl from '../assets/models/building4.mtl?raw';

import building5Obj from '../assets/models/building5.obj?raw';
import building5Mtl from '../assets/models/building5.mtl?raw';

// Hospital
import hospitalObj from '../assets/models/hospital.obj?raw';
import hospitalMtl from '../assets/models/hospital.mtl?raw';

// Destination
import destinationObj from '../assets/models/destination.obj?raw';
import destinationMtl from '../assets/models/destination.mtl?raw';

// Traffic Lights
import trafficLightObj from '../assets/models/trafficlight.obj?raw';
import trafficLightMtl from '../assets/models/trafficlight.mtl?raw';

// Roads
import roadStraightObj from '../assets/models/road_straight.obj?raw';
import roadStraightMtl from '../assets/models/road_straight.mtl?raw';

// Cars
import car1Obj from '../assets/models/car1.obj?raw';
import car1Mtl from '../assets/models/car1.mtl?raw';

import car2Obj from '../assets/models/car2.obj?raw';
import car2Mtl from '../assets/models/car2.mtl?raw';  

import car3Obj from '../assets/models/car3.obj?raw';
import car3Mtl from '../assets/models/car3.mtl?raw';

// Ambulance
import ambulanceObj from '../assets/models/ambulance.obj?raw';
import ambulanceMtl from '../assets/models/ambulance.mtl?raw';

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