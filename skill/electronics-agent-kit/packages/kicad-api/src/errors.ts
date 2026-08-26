/**
 * KiCad API Error classes
 */

/**
 * Base error class for KiCad API errors
 */
export class KiCadError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'KiCadError';
  }
}

/**
 * Error thrown when KiCad is not running or socket is not available
 */
export class KiCadNotRunningError extends KiCadError {
  constructor(socketPath: string) {
    super(`KiCad is not running or socket not found at: ${socketPath}`);
    this.name = 'KiCadNotRunningError';
  }
}

/**
 * Error thrown when connection to KiCad times out
 */
export class KiCadTimeoutError extends KiCadError {
  constructor(timeoutMs: number) {
    super(`Connection to KiCad timed out after ${timeoutMs}ms`);
    this.name = 'KiCadTimeoutError';
  }
}

/**
 * Error thrown when KiCad returns an error response
 */
export class KiCadApiError extends KiCadError {
  readonly code: ApiStatusCode;
  readonly details?: string;

  constructor(code: ApiStatusCode, message: string, details?: string) {
    super(message);
    this.name = 'KiCadApiError';
    this.code = code;
    this.details = details;
  }
}

/**
 * API status codes from KiCad
 */
export enum ApiStatusCode {
  UNKNOWN = 0,
  OK = 1,
  TIMEOUT = 2,
  BAD_REQUEST = 3,
  NOT_READY = 4,
  UNHANDLED = 5,
  TOKEN_MISMATCH = 6,
  BUSY = 7,
  UNIMPLEMENTED = 8,
}
