# @electronics-agent-kit/kicad-api

TypeScript client for KiCad's Protobuf IPC API.

## Status

**Experimental** - This package is under active development. The API may change.

Currently, this package provides:
- Connection handling for KiCad's Unix socket IPC
- Type definitions for common KiCad objects
- High-level client methods for board operations

**Note:** Full protobuf encoding is not yet implemented. The client structure is in place, but actual communication with KiCad requires compiling the proto files.

## Requirements

- KiCad 8.x or later (with IPC API enabled)
- Node.js 20+
- Linux or macOS (Windows named pipes not yet supported)

## Installation

```bash
npm install @electronics-agent-kit/kicad-api
```

## Usage

```typescript
import { KiCadClient, BoardLayer, mmToNm } from '@electronics-agent-kit/kicad-api';

const client = new KiCadClient();

// Check if KiCad is running
if (client.isAvailable()) {
  await client.connect();

  // Get KiCad version
  const version = await client.getVersion();
  console.log(`Connected to KiCad ${version.fullVersion}`);

  // Get all tracks
  const tracks = await client.getTracks();
  console.log(`Found ${tracks.length} tracks`);

  // Get all nets
  const nets = await client.getNets();
  console.log(`Found ${nets.length} nets`);

  // Create a track as a single undoable operation
  await client.withCommit('Add power trace', async () => {
    await client.createTrack({
      start: { x: mmToNm(10), y: mmToNm(20) },
      end: { x: mmToNm(50), y: mmToNm(20) },
      width: { value: mmToNm(0.5) },
      layer: BoardLayer.F_Cu,
      net: { code: 1, name: 'VCC' },
      locked: false,
    });
  });

  client.disconnect();
} else {
  console.log('KiCad is not running or API socket not found');
}
```

## API Reference

### KiCadClient

The main client class for interacting with KiCad.

#### Connection Methods

- `isAvailable()` - Check if KiCad's API socket exists
- `connect()` - Connect to KiCad
- `disconnect()` - Disconnect from KiCad
- `isConnected()` - Check connection status
- `ping()` - Test connection

#### Document Methods

- `getOpenDocuments(type?)` - Get list of open documents
- `saveDocument(filename?)` - Save current document

#### Board Methods

- `getBoardStackup(filename?)` - Get board stackup info
- `getEnabledLayers(filename?)` - Get enabled layers
- `getNets(filename?)` - Get all nets
- `refillZones(zoneIds?, filename?)` - Refill zones

#### Item CRUD Methods

- `getTracks(filename?)` - Get all tracks
- `getVias(filename?)` - Get all vias
- `getZones(filename?)` - Get all zones
- `getFootprints(filename?)` - Get all footprints
- `getItems<T>(type, filename?)` - Get items by type
- `getItemsById<T>(ids, filename?)` - Get items by ID
- `createTrack(track, filename?)` - Create a track
- `createVia(via, filename?)` - Create a via
- `createItems<T>(items, filename?)` - Create multiple items
- `updateItems<T>(items, filename?)` - Update items
- `deleteItems(ids, filename?)` - Delete items by ID

#### Selection Methods

- `getSelection<T>(types?, filename?)` - Get selected items
- `addToSelection(ids, filename?)` - Add to selection
- `removeFromSelection(ids, filename?)` - Remove from selection
- `clearSelection(filename?)` - Clear selection

#### Commit Methods

- `beginCommit()` - Start a commit
- `endCommit(id, commit?, message?)` - End a commit
- `withCommit<T>(message, fn)` - Execute operations as single undo

#### Editor Methods

- `refreshEditor()` - Refresh display
- `getActiveLayer(filename?)` - Get active layer
- `setActiveLayer(layer, filename?)` - Set active layer
- `getVisibleLayers(filename?)` - Get visible layers
- `setVisibleLayers(layers, filename?)` - Set visible layers

### Utility Functions

```typescript
// Unit conversions
mmToNm(mm: number): number       // Millimeters to nanometers
nmToMm(nm: number): number       // Nanometers to millimeters
milsToNm(mils: number): number   // Mils to nanometers
nmToMils(nm: number): number     // Nanometers to mils
degreesToTenths(deg: number): number  // Degrees to tenths
tenthsToDegrees(tenths: number): number

// Convenience constructors
vector2FromMm(x: number, y: number): Vector2
distanceFromMm(mm: number): Distance
```

## How It Works

KiCad 8.x exposes an IPC API via Unix sockets (or named pipes on Windows). The API uses Protocol Buffers for message encoding.

The socket is typically located at:
- Linux/macOS: `/tmp/kicad/api.sock`

### Message Format

Messages are length-prefixed:
1. 4 bytes: message length (big-endian uint32)
2. N bytes: protobuf-encoded ApiRequest/ApiResponse

### Current Limitations

1. **Protobuf encoding not yet implemented** - The client structure is ready but needs proto file compilation
2. **Windows not supported** - Named pipes require different handling
3. **Schematic API is limited** - KiCad's schematic API only supports queries, not modifications

## Development

```bash
# Install dependencies
npm install

# Build
npm run build

# Run tests
npm test

# Generate protobuf types (requires buf)
npm run generate-proto
```

## Next Steps

1. Set up buf.build for proto file compilation
2. Implement proper protobuf encoding/decoding
3. Add comprehensive tests with mocked socket
4. Add schematic-specific client methods

## License

MIT
