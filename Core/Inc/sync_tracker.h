/*
 * sync_tracker.h
 *
 *  Adaptive Period Tracking and Temperature-Compensated Holdover Module
 *  Target: STM32G474CET3
 */

#ifndef INC_SYNC_TRACKER_H_
#define INC_SYNC_TRACKER_H_

#include <stdint.h>
#include <stdbool.h>

#define NOMINAL_SYNC_PERIOD_TICKS   168000000UL /* 2.0s at 84 MHz */
#define MIN_VALID_PERIOD_TICKS      167000000UL /* Max frequency offset limit */
#define MAX_VALID_PERIOD_TICKS      169000000UL

typedef enum {
    SYNC_STATE_SEARCH = 0,    /* Searching for first sync packet */
    SYNC_STATE_LOCKED = 1,    /* Stable lock, continuous tracking */
    SYNC_STATE_HOLDOVER = 2   /* Signal lost in conductive layer, running on predictive model */
} SyncState_t;

typedef struct {
    SyncState_t state;
    
    /* Filtered states */
    float filtered_period;      /* Current estimated period in CPU ticks */
    float drift_rate;           /* Frequency drift per 2-second cycle (ticks/step) */
    float temp_coeff_kT;        /* Temperature coefficient (ticks / °C) */
    
    /* Last valid lock snapshot */
    float last_locked_period;
    float last_locked_temp;
    uint32_t lock_count;        /* Consecutive successful sync detections */
    uint32_t missed_count;      /* Consecutive missed sync periods */
    
    /* Diagnostics */
    float last_raw_period;
    float last_temperature;
    float accumulated_drift_us; /* Total estimated time drift in microseconds */
} SyncTracker_t;

/* Public Functions */
void SyncTracker_Init(SyncTracker_t *tracker);
uint32_t SyncTracker_OnSyncDetected(SyncTracker_t *tracker, uint32_t raw_period_ticks, float current_temp);
uint32_t SyncTracker_OnSyncMissed(SyncTracker_t *tracker, float current_temp);
SyncState_t SyncTracker_GetState(const SyncTracker_t *tracker);
float SyncTracker_GetFilteredPeriod(const SyncTracker_t *tracker);
float SyncTracker_GetDifferenceUs(const SyncTracker_t *tracker);

#endif /* INC_SYNC_TRACKER_H_ */
