/*
 * sync_tracker.c
 *
 *  Adaptive Period Tracking and Temperature-Compensated Holdover Module
 *  Target: STM32G474CET3
 */

#include "sync_tracker.h"
#include <math.h>

#define ALPHA_INITIAL       0.50f
#define BETA_INITIAL        0.10f
#define ALPHA_STEADY        0.20f
#define BETA_STEADY         0.03f

/* DPLL Phase Tracking Gains */
#define DPLL_KP             0.35f   /* Proportional phase pull gain */
#define DPLL_KI             0.05f   /* Integral phase error gain */
#define MAX_DPLL_PULL_TICKS 5000.0f /* Max slew rate per cycle (~59 us) to prevent jitter */

#define MAX_HOLD_STEPS      300U   /* Up to 10 minutes holdover */

void SyncTracker_Init(SyncTracker_t *tracker)
{
    tracker->state = SYNC_STATE_SEARCH;
    tracker->filtered_period = (float)NOMINAL_SYNC_PERIOD_TICKS;
    tracker->drift_rate = 0.0f;
    tracker->temp_coeff_kT = 0.0f;
    tracker->phase_error_ticks = 0.0f;
    tracker->phase_integrator = 0.0f;
    tracker->hard_reset_required = true;
    tracker->last_locked_period = (float)NOMINAL_SYNC_PERIOD_TICKS;
    tracker->last_locked_temp = 25.0f;
    tracker->lock_count = 0;
    tracker->missed_count = 0;
    tracker->last_raw_period = (float)NOMINAL_SYNC_PERIOD_TICKS;
    tracker->last_temperature = 25.0f;
    tracker->accumulated_drift_us = 0.0f;
}

/**
 * @brief Process new sync detection from EM Barker sequence with DPLL soft phase tracking
 * @param raw_period_ticks Total CPU ticks elapsed since previous sync pulse
 * @param raw_phase_cnt Current TIM2->CNT value at the exact moment of correlation peak
 * @param current_arr Current TIM2->ARR reload value
 * @param current_temp Current temperature from TMP235
 * @retval Recommended new period for TIM2 (ticks)
 */
uint32_t SyncTracker_OnSyncDetected(SyncTracker_t *tracker, uint32_t raw_period_ticks, uint32_t raw_phase_cnt, uint32_t current_arr, float current_temp)
{
    float raw_f = (float)raw_period_ticks;
    
    // If recovering from prolonged Holdover (casing / cavern), ignore wrapped DWT ticks
    if (tracker->state == SYNC_STATE_HOLDOVER) {
        raw_f = tracker->filtered_period;
    }
    // Normalize if multiple 2.0-second periods elapsed during short blackout (< 50s)
    else if (raw_f > 1.5f * (float)NOMINAL_SYNC_PERIOD_TICKS) {
        float num_periods = roundf(raw_f / (float)NOMINAL_SYNC_PERIOD_TICKS);
        if (num_periods >= 1.0f) {
            raw_f = raw_f / num_periods;
        }
    }

    // Check if raw period is within physical plausibility limits
    if (raw_f < (float)MIN_VALID_PERIOD_TICKS || raw_f > (float)MAX_VALID_PERIOD_TICKS) {
        raw_f = tracker->filtered_period;
    }

    if (tracker->state == SYNC_STATE_SEARCH || tracker->lock_count == 0) {
        // Initial hard acquisition
        tracker->filtered_period = raw_f;
        tracker->drift_rate = 0.0f;
        tracker->phase_error_ticks = 0.0f;
        tracker->phase_integrator = 0.0f;
        tracker->hard_reset_required = true; // Signal caller to do one-time hard reset of TIM2->CNT
        tracker->last_locked_period = raw_f;
        tracker->last_locked_temp = current_temp;
        tracker->lock_count = 1;
        tracker->missed_count = 0;
        tracker->state = SYNC_STATE_LOCKED;
    } else {
        tracker->hard_reset_required = false; // Soft DPLL active - do NOT reset CNT!
        
        // Compute circular phase error: raw_phase_cnt relative to ideal 0
        float arr_f = (float)current_arr;
        float phase_err = (float)raw_phase_cnt;
        if (phase_err > arr_f * 0.5f) {
            phase_err = phase_err - arr_f; // Negative error (pulse arrived slightly early)
        }
        tracker->phase_error_ticks = phase_err;
        
        // DPLL PI phase error accumulator
        tracker->phase_integrator += phase_err;
        // Anti-windup limit
        if (tracker->phase_integrator > 10000.0f) tracker->phase_integrator = 10000.0f;
        if (tracker->phase_integrator < -10000.0f) tracker->phase_integrator = -10000.0f;

        // Online Tracking Filter (Alpha-Beta Filter for base frequency)
        float alpha = (tracker->lock_count < 8) ? ALPHA_INITIAL : ALPHA_STEADY;
        float beta  = (tracker->lock_count < 8) ? BETA_INITIAL  : BETA_STEADY;
        
        // Predict
        float pred_period = tracker->filtered_period + tracker->drift_rate;
        float residual = raw_f - pred_period;
        
        // Update frequency
        tracker->filtered_period = pred_period + alpha * residual;
        tracker->drift_rate = tracker->drift_rate + beta * residual;
        
        // Online temperature sensitivity estimation (kT = dN / dT)
        float delta_T = current_temp - tracker->last_temperature;
        float delta_N = raw_f - tracker->last_raw_period;
        
        if (fabsf(delta_T) >= 0.05f && fabsf(delta_T) <= 5.0f) {
            float measured_kT = delta_N / delta_T;
            if (measured_kT > -2000.0f && measured_kT < 2000.0f) {
                tracker->temp_coeff_kT = 0.85f * tracker->temp_coeff_kT + 0.15f * measured_kT;
            }
        }
        
        tracker->lock_count++;
        tracker->missed_count = 0;
        tracker->state = SYNC_STATE_LOCKED;
        tracker->last_locked_period = tracker->filtered_period;
        tracker->last_locked_temp = current_temp;
    }

    tracker->last_raw_period = raw_f;
    tracker->last_temperature = current_temp;
    tracker->accumulated_drift_us = (tracker->filtered_period - (float)NOMINAL_SYNC_PERIOD_TICKS) / 84.0f;

    // DPLL Soft Correction: adjust ARR for the next period to pull phase error to zero
    // If phase_error > 0 (pulse arrived after rollover), increase ARR to delay next rollover and align phase
    float dpll_pull = DPLL_KP * tracker->phase_error_ticks + DPLL_KI * tracker->phase_integrator;
    if (dpll_pull > MAX_DPLL_PULL_TICKS)  dpll_pull = MAX_DPLL_PULL_TICKS;
    if (dpll_pull < -MAX_DPLL_PULL_TICKS) dpll_pull = -MAX_DPLL_PULL_TICKS;

    float recommended_arr = tracker->filtered_period + dpll_pull;
    return (uint32_t)roundf(recommended_arr);
}

/**
 * @brief Autonomous Holdover prediction when sync packet is lost in conductive layer
 * @retval Recommended extrapolated period for TIM2 (ticks)
 */
uint32_t SyncTracker_OnSyncMissed(SyncTracker_t *tracker, float current_temp)
{
    tracker->missed_count++;
    tracker->state = SYNC_STATE_HOLDOVER;
    tracker->hard_reset_required = false;
    
    // Extrapolate period based on temperature change since last lock + steady drift rate
    float delta_T = current_temp - tracker->last_locked_temp;
    float temp_compensation = tracker->temp_coeff_kT * delta_T;
    
    // Damp drift rate over prolonged holdover to avoid divergence
    float damping = 1.0f / (1.0f + 0.05f * (float)tracker->missed_count);
    float drift_compensation = tracker->drift_rate * damping;
    
    float predicted_period = tracker->last_locked_period + temp_compensation + drift_compensation;
    
    // Bound check
    if (predicted_period < (float)MIN_VALID_PERIOD_TICKS) predicted_period = (float)MIN_VALID_PERIOD_TICKS;
    if (predicted_period > (float)MAX_VALID_PERIOD_TICKS) predicted_period = (float)MAX_VALID_PERIOD_TICKS;
    
    tracker->filtered_period = predicted_period;
    tracker->last_temperature = current_temp;
    tracker->accumulated_drift_us = (predicted_period - (float)NOMINAL_SYNC_PERIOD_TICKS) / 84.0f;

    return (uint32_t)roundf(predicted_period);
}

SyncState_t SyncTracker_GetState(const SyncTracker_t *tracker)
{
    return tracker->state;
}

float SyncTracker_GetFilteredPeriod(const SyncTracker_t *tracker)
{
    return tracker->filtered_period;
}

float SyncTracker_GetDifferenceUs(const SyncTracker_t *tracker)
{
    return tracker->accumulated_drift_us;
}

bool SyncTracker_IsHardResetRequired(const SyncTracker_t *tracker)
{
    return tracker->hard_reset_required;
}

