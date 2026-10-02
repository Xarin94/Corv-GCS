// roslib loads 'ws' only where there is no global WebSocket (plain Node).
export const WebSocket = globalThis.WebSocket;
export default { WebSocket: globalThis.WebSocket };
