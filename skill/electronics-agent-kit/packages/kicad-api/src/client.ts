/**
 * KiCad API Client
 *
 * High-level client for interacting with KiCad's IPC API.
 * Provides typed methods for common operations.
 */

import { KiCadConnection, ConnectionOptions } from './connection.js';
import { KiCadApiError, ApiStatusCode } from './errors.js';
import {
  DocumentType,
  BoardLayer,
  KiCadObjectType,
  Track,
  Via,
  Zone,
  FootprintInstance,
  Net,
  KiCadVersion,
  BoardStackup,
  Vector2,
  Distance,
  KIID,
  ItemCreationResult,
  mmToNm,
  nmToMm,
} from './types.js';

/**
 * Client options
 */
export interface KiCadClientOptions extends ConnectionOptions {
  /** Automatically connect on first request */
  autoConnect?: boolean;
}

/**
 * High-level KiCad API client
 *
 * @example
 * ```typescript
 * const client = new KiCadClient();
 *
 * // Check if KiCad is running
 * if (client.isAvailable()) {
 *   await client.connect();
 *
 *   // Get version
 *   const version = await client.getVersion();
 *   console.log(`KiCad ${version.fullVersion}`);
 *
 *   // Get all tracks
 *   const tracks = await client.getTracks();
 *
 *   // Create a new track
 *   await client.createTrack({
 *     start: { x: mmToNm(10), y: mmToNm(20) },
 *     end: { x: mmToNm(50), y: mmToNm(20) },
 *     width: { value: mmToNm(0.25) },
 *     layer: BoardLayer.F_Cu,
 *     net: { code: 1, name: 'VCC' }
 *   });
 *
 *   client.disconnect();
 * }
 * ```
 */
export class KiCadClient {
  private connection: KiCadConnection;
  private readonly autoConnect: boolean;
  private kicadToken: string | null = null;

  constructor(options: KiCadClientOptions = {}) {
    this.connection = new KiCadConnection(options);
    this.autoConnect = options.autoConnect ?? true;
  }

  // ============================================================================
  // Connection Management
  // ============================================================================

  /**
   * Check if KiCad's API socket is available
   */
  isAvailable(): boolean {
    return this.connection.isSocketAvailable();
  }

  /**
   * Connect to KiCad
   */
  async connect(): Promise<void> {
    await this.connection.connect();
  }

  /**
   * Disconnect from KiCad
   */
  disconnect(): void {
    this.connection.disconnect();
  }

  /**
   * Check if connected
   */
  isConnected(): boolean {
    return this.connection.isConnected();
  }

  // ============================================================================
  // Base Commands
  // ============================================================================

  /**
   * Ping KiCad to check connection
   */
  async ping(): Promise<boolean> {
    try {
      await this.sendCommand('Ping', {});
      return true;
    } catch {
      return false;
    }
  }

  /**
   * Get KiCad version information
   */
  async getVersion(): Promise<KiCadVersion> {
    const response = await this.sendCommand('GetVersion', {});
    return {
      major: response.version?.major ?? 0,
      minor: response.version?.minor ?? 0,
      patch: response.version?.patch ?? 0,
      fullVersion: response.version?.fullVersion ?? 'unknown',
    };
  }

  // ============================================================================
  // Document Commands
  // ============================================================================

  /**
   * Get list of open documents
   */
  async getOpenDocuments(type: DocumentType = DocumentType.BOARD): Promise<string[]> {
    const response = await this.sendCommand('GetOpenDocuments', { type });
    return response.documents?.map((d: any) => d.boardFilename) ?? [];
  }

  /**
   * Save the current document
   */
  async saveDocument(filename?: string): Promise<void> {
    const document = filename
      ? { type: DocumentType.BOARD, boardFilename: filename }
      : { type: DocumentType.BOARD };
    await this.sendCommand('SaveDocument', { document });
  }

  // ============================================================================
  // Board Commands
  // ============================================================================

  /**
   * Get board stackup information
   */
  async getBoardStackup(filename?: string): Promise<BoardStackup> {
    const board = this.makeDocumentSpec(filename);
    const response = await this.sendCommand('GetBoardStackup', { board });
    return {
      copperLayerCount: response.stackup?.copperLayerCount ?? 2,
      layers: response.stackup?.layers ?? [],
    };
  }

  /**
   * Get enabled board layers
   */
  async getEnabledLayers(filename?: string): Promise<BoardLayer[]> {
    const board = this.makeDocumentSpec(filename);
    const response = await this.sendCommand('GetBoardEnabledLayers', { board });
    return response.layers ?? [];
  }

  /**
   * Get all nets in the board
   */
  async getNets(filename?: string): Promise<Net[]> {
    const board = this.makeDocumentSpec(filename);
    const response = await this.sendCommand('GetNets', { board });
    return (response.nets ?? []).map((n: any) => ({
      code: n.code?.value ?? 0,
      name: n.name ?? '',
    }));
  }

  /**
   * Refill all zones or specific zones
   */
  async refillZones(zoneIds?: KIID[], filename?: string): Promise<void> {
    const board = this.makeDocumentSpec(filename);
    await this.sendCommand('RefillZones', {
      board,
      zones: zoneIds ?? [],
    });
  }

  // ============================================================================
  // Item CRUD Operations
  // ============================================================================

  /**
   * Get all tracks
   */
  async getTracks(filename?: string): Promise<Track[]> {
    return this.getItems<Track>(KiCadObjectType.TRACK, filename);
  }

  /**
   * Get all vias
   */
  async getVias(filename?: string): Promise<Via[]> {
    return this.getItems<Via>(KiCadObjectType.VIA, filename);
  }

  /**
   * Get all zones
   */
  async getZones(filename?: string): Promise<Zone[]> {
    return this.getItems<Zone>(KiCadObjectType.ZONE, filename);
  }

  /**
   * Get all footprints
   */
  async getFootprints(filename?: string): Promise<FootprintInstance[]> {
    return this.getItems<FootprintInstance>(KiCadObjectType.FOOTPRINT, filename);
  }

  /**
   * Get items by type
   */
  async getItems<T>(type: KiCadObjectType, filename?: string): Promise<T[]> {
    const header = this.makeItemHeader(filename);
    const response = await this.sendCommand('GetItems', {
      header,
      types: [type],
    });
    return response.items ?? [];
  }

  /**
   * Get items by ID
   */
  async getItemsById<T>(ids: KIID[], filename?: string): Promise<T[]> {
    const header = this.makeItemHeader(filename);
    const response = await this.sendCommand('GetItemsById', {
      header,
      items: ids,
    });
    return response.items ?? [];
  }

  /**
   * Create a new track
   */
  async createTrack(
    track: Omit<Track, 'id'> & { id?: KIID },
    filename?: string
  ): Promise<ItemCreationResult<Track>> {
    return this.createItem<Track>(track, filename);
  }

  /**
   * Create a new via
   */
  async createVia(
    via: Omit<Via, 'id'> & { id?: KIID },
    filename?: string
  ): Promise<ItemCreationResult<Via>> {
    return this.createItem<Via>(via, filename);
  }

  /**
   * Create items
   */
  async createItem<T>(item: any, filename?: string): Promise<ItemCreationResult<T>> {
    const results = await this.createItems<T>([item], filename);
    return results[0] ?? { status: 0 };
  }

  /**
   * Create multiple items
   */
  async createItems<T>(items: any[], filename?: string): Promise<ItemCreationResult<T>[]> {
    const header = this.makeItemHeader(filename);
    const response = await this.sendCommand('CreateItems', {
      header,
      items,
    });
    return (
      response.createdItems?.map((result: any) => ({
        status: result.status?.code ?? 0,
        errorMessage: result.status?.errorMessage,
        item: result.item,
      })) ?? []
    );
  }

  /**
   * Update items
   */
  async updateItems<T>(items: T[], filename?: string): Promise<ItemCreationResult<T>[]> {
    const header = this.makeItemHeader(filename);
    const response = await this.sendCommand('UpdateItems', {
      header,
      items,
    });
    return (
      response.updatedItems?.map((result: any) => ({
        status: result.status?.code ?? 0,
        errorMessage: result.status?.errorMessage,
        item: result.item,
      })) ?? []
    );
  }

  /**
   * Delete items by ID
   */
  async deleteItems(ids: KIID[], filename?: string): Promise<void> {
    const header = this.makeItemHeader(filename);
    await this.sendCommand('DeleteItems', {
      header,
      itemIds: ids,
    });
  }

  // ============================================================================
  // Selection Commands
  // ============================================================================

  /**
   * Get currently selected items
   */
  async getSelection<T>(types?: KiCadObjectType[], filename?: string): Promise<T[]> {
    const header = this.makeItemHeader(filename);
    const response = await this.sendCommand('GetSelection', {
      header,
      types: types ?? [],
    });
    return response.items ?? [];
  }

  /**
   * Add items to selection
   */
  async addToSelection(ids: KIID[], filename?: string): Promise<void> {
    const header = this.makeItemHeader(filename);
    await this.sendCommand('AddToSelection', {
      header,
      items: ids,
    });
  }

  /**
   * Remove items from selection
   */
  async removeFromSelection(ids: KIID[], filename?: string): Promise<void> {
    const header = this.makeItemHeader(filename);
    await this.sendCommand('RemoveFromSelection', {
      header,
      items: ids,
    });
  }

  /**
   * Clear selection
   */
  async clearSelection(filename?: string): Promise<void> {
    const header = this.makeItemHeader(filename);
    await this.sendCommand('ClearSelection', { header });
  }

  // ============================================================================
  // Commit Management
  // ============================================================================

  /**
   * Begin a commit (batch changes)
   */
  async beginCommit(): Promise<KIID> {
    const response = await this.sendCommand('BeginCommit', {});
    return response.id ?? { value: '' };
  }

  /**
   * End a commit
   */
  async endCommit(id: KIID, commit: boolean = true, message?: string): Promise<void> {
    await this.sendCommand('EndCommit', {
      id,
      action: commit ? 1 : 2, // CMA_COMMIT = 1, CMA_DROP = 2
      message: message ?? '',
    });
  }

  /**
   * Execute a batch of operations as a single undoable action
   */
  async withCommit<T>(message: string, fn: () => Promise<T>): Promise<T> {
    const commitId = await this.beginCommit();
    try {
      const result = await fn();
      await this.endCommit(commitId, true, message);
      return result;
    } catch (error) {
      await this.endCommit(commitId, false);
      throw error;
    }
  }

  // ============================================================================
  // Editor Commands
  // ============================================================================

  /**
   * Refresh the editor display
   */
  async refreshEditor(): Promise<void> {
    await this.sendCommand('RefreshEditor', { frame: 2 }); // PCB editor frame
  }

  /**
   * Get the active layer
   */
  async getActiveLayer(filename?: string): Promise<BoardLayer> {
    const board = this.makeDocumentSpec(filename);
    const response = await this.sendCommand('GetActiveLayer', { board });
    return response.layer ?? BoardLayer.UNKNOWN;
  }

  /**
   * Set the active layer
   */
  async setActiveLayer(layer: BoardLayer, filename?: string): Promise<void> {
    const board = this.makeDocumentSpec(filename);
    await this.sendCommand('SetActiveLayer', { board, layer });
  }

  /**
   * Get visible layers
   */
  async getVisibleLayers(filename?: string): Promise<BoardLayer[]> {
    const board = this.makeDocumentSpec(filename);
    const response = await this.sendCommand('GetVisibleLayers', { board });
    return response.layers ?? [];
  }

  /**
   * Set visible layers
   */
  async setVisibleLayers(layers: BoardLayer[], filename?: string): Promise<void> {
    const board = this.makeDocumentSpec(filename);
    await this.sendCommand('SetVisibleLayers', { board, layers });
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  private makeDocumentSpec(filename?: string) {
    return {
      type: DocumentType.BOARD,
      boardFilename: filename ?? '',
    };
  }

  private makeItemHeader(filename?: string) {
    return {
      document: this.makeDocumentSpec(filename),
    };
  }

  /**
   * Send a command to KiCad
   *
   * This is the low-level method that handles the protobuf encoding/decoding.
   * Currently uses a simplified JSON-like approach for development.
   * Production version should use proper protobuf encoding.
   */
  private async sendCommand(command: string, payload: any): Promise<any> {
    if (this.autoConnect && !this.connection.isConnected()) {
      await this.connection.connect();
    }

    // TODO: Replace with proper protobuf encoding when proto files are compiled
    // For now, this is a placeholder that shows the intended API structure.
    //
    // The actual implementation would:
    // 1. Create the appropriate protobuf message for the command
    // 2. Wrap it in an ApiRequest envelope
    // 3. Serialize to binary
    // 4. Send via connection.sendMessage()
    // 5. Deserialize the ApiResponse
    // 6. Extract and return the inner message

    const request = {
      header: {
        kicadToken: this.kicadToken ?? '',
        clientName: this.connection.getClientName(),
      },
      message: {
        '@type': `type.googleapis.com/kiapi.${this.getPackageForCommand(command)}.${command}`,
        ...payload,
      },
    };

    // Placeholder: In production, this would be protobuf-encoded
    const requestBuffer = Buffer.from(JSON.stringify(request));

    const responseBuffer = await this.connection.sendMessage(requestBuffer);

    // Placeholder: In production, this would be protobuf-decoded
    try {
      const response = JSON.parse(responseBuffer.toString());

      if (response.status?.status !== ApiStatusCode.OK && response.status?.status !== undefined) {
        throw new KiCadApiError(
          response.status.status,
          response.status.errorMessage ?? 'Unknown error',
          response.status.details
        );
      }

      // Store the KiCad token for future requests
      if (response.header?.kicadToken) {
        this.kicadToken = response.header.kicadToken;
      }

      return response.message ?? {};
    } catch (error) {
      if (error instanceof KiCadApiError) {
        throw error;
      }
      // If JSON parsing fails, the response wasn't in our expected format
      // This would happen if we're actually connected to real KiCad with protobuf
      throw new KiCadApiError(
        ApiStatusCode.BAD_REQUEST,
        'Failed to parse response - protobuf encoding not yet implemented'
      );
    }
  }

  /**
   * Get the package name for a command
   */
  private getPackageForCommand(command: string): string {
    // Board-specific commands
    const boardCommands = [
      'GetBoardStackup',
      'GetBoardEnabledLayers',
      'GetNets',
      'RefillZones',
      'GetActiveLayer',
      'SetActiveLayer',
      'GetVisibleLayers',
      'SetVisibleLayers',
    ];

    if (boardCommands.includes(command)) {
      return 'board.commands';
    }

    // Editor commands
    const editorCommands = [
      'RefreshEditor',
      'GetOpenDocuments',
      'SaveDocument',
      'BeginCommit',
      'EndCommit',
      'CreateItems',
      'GetItems',
      'GetItemsById',
      'UpdateItems',
      'DeleteItems',
      'GetSelection',
      'AddToSelection',
      'RemoveFromSelection',
      'ClearSelection',
    ];

    if (editorCommands.includes(command)) {
      return 'common.commands';
    }

    // Base commands
    return 'common.commands';
  }
}
