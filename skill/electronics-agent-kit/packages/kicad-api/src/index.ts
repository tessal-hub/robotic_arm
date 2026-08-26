/**
 * KiCad API Client
 *
 * TypeScript client for KiCad's Protobuf IPC API.
 * Communicates with running KiCad instances over Unix sockets.
 *
 * @module @electronics-agent-kit/kicad-api
 */

export { KiCadClient } from './client.js';
export { KiCadConnection } from './connection.js';
export * from './types.js';
export * from './errors.js';
