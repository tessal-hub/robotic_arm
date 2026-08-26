/**
 * Type definitions for KiCad API
 *
 * These types mirror the Protobuf definitions but are easier to work with in TypeScript.
 */

// ============================================================================
// Base Types
// ============================================================================

/**
 * 2D vector/point in nanometers (KiCad's internal unit)
 */
export interface Vector2 {
  x: number;
  y: number;
}

/**
 * Distance in nanometers
 */
export interface Distance {
  value: number;
}

/**
 * Angle in tenths of degrees
 */
export interface Angle {
  value: number;
}

/**
 * Unique identifier for KiCad objects
 */
export interface KIID {
  value: string;
}

/**
 * Document specifier
 */
export interface DocumentSpecifier {
  type: DocumentType;
  boardFilename?: string;
}

export enum DocumentType {
  UNKNOWN = 0,
  BOARD = 1,
  SCHEMATIC = 2,
  SYMBOL_LIBRARY = 3,
  FOOTPRINT_LIBRARY = 4,
}

// ============================================================================
// Board Types
// ============================================================================

/**
 * Board layers
 */
export enum BoardLayer {
  UNKNOWN = 0,
  UNDEFINED = 1,
  F_Cu = 3,
  In1_Cu = 4,
  In2_Cu = 5,
  B_Cu = 34,
  B_Adhes = 35,
  F_Adhes = 36,
  B_Paste = 37,
  F_Paste = 38,
  B_SilkS = 39,
  F_SilkS = 40,
  B_Mask = 41,
  F_Mask = 42,
  Dwgs_User = 43,
  Cmts_User = 44,
  Eco1_User = 45,
  Eco2_User = 46,
  Edge_Cuts = 47,
  Margin = 48,
  B_CrtYd = 49,
  F_CrtYd = 50,
  B_Fab = 51,
  F_Fab = 52,
}

/**
 * Net information
 */
export interface Net {
  code: number;
  name: string;
}

/**
 * Track segment
 */
export interface Track {
  id: KIID;
  start: Vector2;
  end: Vector2;
  width: Distance;
  locked: boolean;
  layer: BoardLayer;
  net: Net;
}

/**
 * Arc track
 */
export interface Arc {
  id: KIID;
  start: Vector2;
  mid: Vector2;
  end: Vector2;
  width: Distance;
  locked: boolean;
  layer: BoardLayer;
  net: Net;
}

/**
 * Via types
 */
export enum ViaType {
  UNKNOWN = 0,
  THROUGH = 1,
  BLIND_BURIED = 2,
  MICRO = 3,
  BLIND = 4,
  BURIED = 5,
}

/**
 * Via
 */
export interface Via {
  id: KIID;
  position: Vector2;
  locked: boolean;
  net: Net;
  type: ViaType;
  startLayer: BoardLayer;
  endLayer: BoardLayer;
  diameter: number;
  drillDiameter: number;
}

/**
 * Pad type
 */
export enum PadType {
  UNKNOWN = 0,
  PTH = 1,
  SMD = 2,
  EDGE_CONNECTOR = 3,
  NPTH = 4,
}

/**
 * Pad shape
 */
export enum PadShape {
  UNKNOWN = 0,
  CIRCLE = 1,
  RECTANGLE = 2,
  OVAL = 3,
  TRAPEZOID = 4,
  ROUNDRECT = 5,
  CHAMFERED_RECT = 6,
  CUSTOM = 7,
}

/**
 * Pad
 */
export interface Pad {
  id: KIID;
  locked: boolean;
  number: string;
  net: Net;
  type: PadType;
  position: Vector2;
  size: Vector2;
  shape: PadShape;
  orientation: Angle;
  drillSize?: Vector2;
}

/**
 * Zone type
 */
export enum ZoneType {
  UNKNOWN = 0,
  COPPER = 1,
  GRAPHICAL = 2,
  RULE_AREA = 3,
  TEARDROP = 4,
}

/**
 * Zone connection style
 */
export enum ZoneConnectionStyle {
  UNKNOWN = 0,
  INHERITED = 1,
  NONE = 2,
  THERMAL = 3,
  FULL = 4,
  PTH_THERMAL = 5,
}

/**
 * Zone
 */
export interface Zone {
  id: KIID;
  type: ZoneType;
  layers: BoardLayer[];
  name: string;
  priority: number;
  filled: boolean;
  locked: boolean;
  net?: Net;
  clearance?: Distance;
  minThickness?: Distance;
}

/**
 * Footprint instance on a board
 */
export interface FootprintInstance {
  id: KIID;
  position: Vector2;
  orientation: Angle;
  layer: BoardLayer;
  locked: boolean;
  reference: string;
  value: string;
  libraryId?: string;
}

// ============================================================================
// Request/Response Types
// ============================================================================

/**
 * Request header
 */
export interface ApiRequestHeader {
  kicadToken?: string;
  clientName: string;
}

/**
 * Response status
 */
export interface ApiResponseStatus {
  status: import('./errors.js').ApiStatusCode;
  errorMessage?: string;
}

/**
 * KiCad version info
 */
export interface KiCadVersion {
  major: number;
  minor: number;
  patch: number;
  fullVersion: string;
}

/**
 * Board stackup information
 */
export interface BoardStackup {
  copperLayerCount: number;
  layers: BoardStackupLayer[];
}

export interface BoardStackupLayer {
  layer: BoardLayer;
  material?: string;
  thickness?: Distance;
  color?: string;
}

/**
 * Item creation result
 */
export interface ItemCreationResult<T> {
  status: ItemStatusCode;
  errorMessage?: string;
  item?: T;
}

export enum ItemStatusCode {
  UNKNOWN = 0,
  OK = 1,
  INVALID_TYPE = 2,
  EXISTING = 3,
  NONEXISTENT = 4,
  IMMUTABLE = 5,
  INVALID_DATA = 7,
}

/**
 * Item types that can be queried/created
 */
export enum KiCadObjectType {
  UNKNOWN = 0,
  TRACK = 1,
  ARC = 2,
  VIA = 3,
  PAD = 4,
  ZONE = 5,
  FOOTPRINT = 6,
  GRAPHIC_SHAPE = 7,
  TEXT = 8,
  TEXTBOX = 9,
  DIMENSION = 10,
  GROUP = 11,
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Convert millimeters to nanometers (KiCad's internal unit)
 */
export function mmToNm(mm: number): number {
  return Math.round(mm * 1_000_000);
}

/**
 * Convert nanometers to millimeters
 */
export function nmToMm(nm: number): number {
  return nm / 1_000_000;
}

/**
 * Convert mils to nanometers
 */
export function milsToNm(mils: number): number {
  return Math.round(mils * 25_400);
}

/**
 * Convert nanometers to mils
 */
export function nmToMils(nm: number): number {
  return nm / 25_400;
}

/**
 * Convert degrees to tenths of degrees (KiCad's internal unit)
 */
export function degreesToTenths(degrees: number): number {
  return Math.round(degrees * 10);
}

/**
 * Convert tenths of degrees to degrees
 */
export function tenthsToDegrees(tenths: number): number {
  return tenths / 10;
}

/**
 * Create a Vector2 from millimeters
 */
export function vector2FromMm(xMm: number, yMm: number): Vector2 {
  return {
    x: mmToNm(xMm),
    y: mmToNm(yMm),
  };
}

/**
 * Create a Distance from millimeters
 */
export function distanceFromMm(mm: number): Distance {
  return { value: mmToNm(mm) };
}
