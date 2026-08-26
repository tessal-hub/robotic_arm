/**
 * KiCad Unix Socket Connection
 *
 * Low-level connection handler for KiCad's IPC socket.
 */

import * as net from 'node:net';
import * as fs from 'node:fs';
import { KiCadNotRunningError, KiCadTimeoutError } from './errors.js';

/**
 * Default socket paths for different platforms
 */
export const SOCKET_PATHS = {
  linux: '/tmp/kicad/api.sock',
  darwin: '/tmp/kicad/api.sock',
  // Windows uses named pipes, not yet supported
} as const;

/**
 * Connection options
 */
export interface ConnectionOptions {
  /** Path to KiCad's API socket */
  socketPath?: string;
  /** Connection timeout in milliseconds */
  timeout?: number;
  /** Client name for identification */
  clientName?: string;
}

const DEFAULT_TIMEOUT = 5000;
const DEFAULT_CLIENT_NAME = 'electronics-agent-kit';

/**
 * Low-level connection to KiCad's IPC socket
 */
export class KiCadConnection {
  private socket: net.Socket | null = null;
  private readonly socketPath: string;
  private readonly timeout: number;
  private readonly clientName: string;
  private connected = false;
  private responseBuffer = Buffer.alloc(0);
  private responseResolver: ((data: Buffer) => void) | null = null;

  constructor(options: ConnectionOptions = {}) {
    const platform = process.platform as keyof typeof SOCKET_PATHS;
    this.socketPath = options.socketPath ?? SOCKET_PATHS[platform] ?? SOCKET_PATHS.linux;
    this.timeout = options.timeout ?? DEFAULT_TIMEOUT;
    this.clientName = options.clientName ?? DEFAULT_CLIENT_NAME;
  }

  /**
   * Check if KiCad's API socket exists
   */
  isSocketAvailable(): boolean {
    try {
      fs.accessSync(this.socketPath, fs.constants.R_OK | fs.constants.W_OK);
      return true;
    } catch {
      return false;
    }
  }

  /**
   * Connect to KiCad's IPC socket
   */
  async connect(): Promise<void> {
    if (this.connected) {
      return;
    }

    if (!this.isSocketAvailable()) {
      throw new KiCadNotRunningError(this.socketPath);
    }

    return new Promise<void>((resolve, reject) => {
      const timeoutId = setTimeout(() => {
        this.cleanup();
        reject(new KiCadTimeoutError(this.timeout));
      }, this.timeout);

      this.socket = net.createConnection(this.socketPath, () => {
        clearTimeout(timeoutId);
        this.connected = true;
        resolve();
      });

      this.socket.on('data', (data) => {
        this.handleData(data);
      });

      this.socket.on('error', (err) => {
        clearTimeout(timeoutId);
        this.cleanup();
        reject(new KiCadNotRunningError(this.socketPath));
      });

      this.socket.on('close', () => {
        this.cleanup();
      });
    });
  }

  /**
   * Send a message and wait for response
   *
   * KiCad's protocol uses length-prefixed messages:
   * - 4 bytes: message length (big-endian uint32)
   * - N bytes: protobuf message
   */
  async sendMessage(data: Buffer): Promise<Buffer> {
    if (!this.connected || !this.socket) {
      await this.connect();
    }

    return new Promise<Buffer>((resolve, reject) => {
      const timeoutId = setTimeout(() => {
        this.responseResolver = null;
        reject(new KiCadTimeoutError(this.timeout));
      }, this.timeout);

      this.responseResolver = (responseData) => {
        clearTimeout(timeoutId);
        resolve(responseData);
      };

      // Create length-prefixed message
      const lengthBuffer = Buffer.alloc(4);
      lengthBuffer.writeUInt32BE(data.length, 0);
      const message = Buffer.concat([lengthBuffer, data]);

      this.socket!.write(message, (err) => {
        if (err) {
          clearTimeout(timeoutId);
          this.responseResolver = null;
          reject(err);
        }
      });
    });
  }

  /**
   * Handle incoming data from the socket
   */
  private handleData(data: Buffer): void {
    this.responseBuffer = Buffer.concat([this.responseBuffer, data]);

    // Check if we have a complete message
    while (this.responseBuffer.length >= 4) {
      const messageLength = this.responseBuffer.readUInt32BE(0);

      if (this.responseBuffer.length < 4 + messageLength) {
        // Wait for more data
        return;
      }

      // Extract the complete message
      const message = this.responseBuffer.subarray(4, 4 + messageLength);
      this.responseBuffer = this.responseBuffer.subarray(4 + messageLength);

      if (this.responseResolver) {
        const resolver = this.responseResolver;
        this.responseResolver = null;
        resolver(message);
      }
    }
  }

  /**
   * Disconnect from KiCad
   */
  disconnect(): void {
    this.cleanup();
  }

  private cleanup(): void {
    if (this.socket) {
      this.socket.destroy();
      this.socket = null;
    }
    this.connected = false;
    this.responseBuffer = Buffer.alloc(0);
    this.responseResolver = null;
  }

  /**
   * Check if connected
   */
  isConnected(): boolean {
    return this.connected;
  }

  /**
   * Get the socket path being used
   */
  getSocketPath(): string {
    return this.socketPath;
  }

  /**
   * Get the client name
   */
  getClientName(): string {
    return this.clientName;
  }
}
