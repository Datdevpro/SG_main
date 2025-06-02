#ifndef FLEX_SENSOR_H
#define FLEX_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the flex sensors
void flex_sensor_init(void);

// Check which flex sensor is activated
// Returns:
//   1-4: Sensor number that is activated
//   -1: No sensor activated
int flex_sensor_check(void);

#ifdef __cplusplus
}
#endif

#endif // FLEX_SENSOR_H

