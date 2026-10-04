/**
 * three.js - three.js r186 as the global THREE the renderer modules use.
 *
 * Imported first by js/main.js: ES modules run in import order, so THREE
 * exists before any module that builds three objects at load time
 * (TerrainManager's shared material, Scene3D's scratch vectors).
 */

import * as THREE_MODULE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

// The scene was tuned on r128, which drew colours as raw sRGB values: no
// conversion of THREE.Color inputs to linear here, and every renderer outputs
// LinearSRGBColorSpace (Scene3D, MagCal3D), so a hex colour is the colour drawn.
THREE_MODULE.ColorManagement.enabled = false;

window.THREE = { ...THREE_MODULE, GLTFLoader };
