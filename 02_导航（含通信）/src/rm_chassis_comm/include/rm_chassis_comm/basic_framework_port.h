#ifndef RM_BASIC_FRAMEWORK_NAV_PORT_H
#define RM_BASIC_FRAMEWORK_NAV_PORT_H
#include "rm_chassis_comm/mcu_link.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Calibration is mandatory: the checked-in legacy wheel controller does NOT
 * establish a complete SI -> motor RPM conversion. Zero initialization is safe. */
typedef struct {
  bool verified;
  float scale_vx,scale_vy,scale_wz;
  float max_abs_vx,max_abs_vy,max_abs_wz;
} rm_basic_calibration_t;
typedef struct {bool navigation_enabled;float vx,vy,wz,offset_angle;} rm_basic_values_t;
/* Map body-frame SI speeds into the legacy controller's calibrated internal
 * units. Signed scales express verified axis signs. Not a motor driver. */
int rm_basic_map(const rm_output_t *,const rm_basic_calibration_t *,rm_basic_values_t *);
#ifdef __cplusplus
}
#endif
#endif
