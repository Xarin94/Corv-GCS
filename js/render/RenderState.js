import { ORIGIN } from '../core/constants.js';
import { RenderWorld } from './RenderWorld.js';
// App-owned state. The data modules themselves can run without a DOM or Three.
export const renderWorld = new RenderWorld(ORIGIN);
