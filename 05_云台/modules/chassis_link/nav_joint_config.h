#ifndef NAV_JOINT_CONFIG_H
#define NAV_JOINT_CONFIG_H
/* Encoder values in the supplied tracking firmware are relative to power-on,
 * not calibrated chassis-relative joints. Do not assert validity yet. */
#define NAV_JOINT_CALIBRATED 0
#if NAV_JOINT_CALIBRATED
#error Implement measured absolute chassis-relative encoder joints before enabling calibration.
#endif
/* Reserved for a measured, chassis-relative absolute encoder implementation. */
#endif
