#pragma once
#include <cstdint>
#include <esp_attr.h>

/**
 * @brief 6-Axis Motion Block using Q32.32 Fixed-Point 2nd-Order Taylor/DDA Integration
 * 
 * Each block represents an S-curve sub-phase or path segment.
 * 
 * Fixed-point format:
 * - ddaStepFraction: Q32.32 (Upper 32 bits = integer steps per tick, Lower 32 bits = sub-step fraction).
 * - ddaStepAccel:    Signed 32-bit rate increment per tick.
 * - ddaStepJerk:     Signed 32-bit acceleration increment per tick.
 */
struct MotionBlock {
    uint32_t totalTicks;             ///< Duration in 50kHz ticks (20us per tick)
    uint32_t currentTick;            ///< Elapsed ticks within this block
    int32_t  targetAbsSteps[6];      ///< Absolute destination steps for exact Snap-to-Target
    uint64_t ddaStepFraction[6];     ///< Q32.32 initial step fraction / velocity
    int32_t  ddaStepAccel[6];        ///< Acceleration per tick
    int32_t  ddaStepJerk[6];         ///< Jerk per tick
    uint8_t  dirMask;                ///< Bit 0..5: Direction for axes J1..J6 (1 = positive, 0 = negative)
    bool     isLastSegment;          ///< True if this is the final block in a trajectory move

    MotionBlock() {
        totalTicks = 0;
        currentTick = 0;
        dirMask = 0;
        isLastSegment = true;
        for (int i = 0; i < 6; i++) {
            targetAbsSteps[i] = 0;
            ddaStepFraction[i] = 0;
            ddaStepAccel[i] = 0;
            ddaStepJerk[i] = 0;
        }
    }
};
