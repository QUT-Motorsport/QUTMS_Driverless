#pragma once

#include <stdint.h>

#include "sbg_can_dt.hpp"

// STATUS
typedef struct SBG_ECAN_MSG_STATUS_01 {
	uint32_t id;
	uint8_t data[8];
} SBG_ECAN_MSG_STATUS_01_t;

SBG_ECAN_MSG_STATUS_01_t Compose_SBG_ECAN_MSG_STATUS_01(uint32_t id, uint32_t time_stamp, uint16_t general_status,
														uint16_t clock_status);
void Parse_SBG_ECAN_MSG_STATUS_01(uint8_t* data, uint32_t* id, uint32_t* time_stamp, uint16_t* general_status,
								  uint16_t* clock_status);

typedef struct SBG_ECAN_MSG_STATUS_02 {
	uint32_t id;
	uint8_t data[8];
} SBG_ECAN_MSG_STATUS_02_t;

SBG_ECAN_MSG_STATUS_02_t Compose_SBG_ECAN_MSG_STATUS_02(uint32_t id, uint32_t com_status, uint32_t aiding_status);
void Parse_SBG_ECAN_MSG_STATUS_02(uint8_t* data, uint32_t* id, uint32_t* com_status, uint32_t* aiding_status);

typedef struct SBG_ECAN_MSG_STATUS_03 {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_STATUS_03_t;

SBG_ECAN_MSG_STATUS_03_t Compose_SBG_ECAN_MSG_STATUS_03(uint32_t id, uint32_t solution_status, uint16_t heave_status);
void Parse_SBG_ECAN_MSG_STATUS_03(uint8_t* data, uint32_t* id, uint32_t* solution_status, uint16_t* heave_status);

// UTC
typedef struct SBG_ECAN_MSG_UTC_0 {
	uint32_t id;
	uint8_t data[8];
} SBG_ECAN_MSG_UTC_0_t;

SBG_ECAN_MSG_UTC_0_t Compose_SBG_ECAN_MSG_UTC_0(uint32_t id, uint32_t time_stamp, uint32_t gps_tow);
void Parse_SBG_ECAN_MSG_UTC_0(uint8_t* data, uint32_t* id, uint32_t* time_stamp, uint32_t* gps_tow);

typedef struct SBG_ECAN_MSG_UTC_1 {
	uint32_t id;
	uint8_t data[8];
} SBG_ECAN_MSG_UTC_1_t;

SBG_ECAN_MSG_UTC_1_t Compose_SBG_ECAN_MSG_UTC_1(uint32_t id, uint8_t year, uint8_t month, uint8_t day, uint8_t hour,
												uint8_t min, uint8_t sec, uint16_t micro_sec);
void Parse_SBG_ECAN_MSG_UTC_1(uint8_t* data, uint32_t* id, uint8_t* year, uint8_t* month, uint8_t* day, uint8_t* hour,
							  uint8_t* min, uint8_t* sec, uint16_t* micro_sec);

// IMU
typedef struct SBG_ECAN_MSG_IMU_INFO {
	uint32_t id;
	uint8_t data[8];
} SBG_ECAN_MSG_IMU_INFO_t;

SBG_ECAN_MSG_IMU_INFO_t Compose_SBG_ECAN_MSG_IMU_INFO(uint32_t id, uint32_t time_stamp, uint16_t imu_status,
													  uint16_t temperature);
void Parse_SBG_ECAN_MSG_IMU_INFO(uint8_t* data, uint32_t* id, uint32_t* time_stamp, uint16_t* imu_status,
								 float* temperature);

typedef struct SBG_ECAN_MSG_IMU_ACCEL {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_IMU_ACCEL_t;

SBG_ECAN_MSG_IMU_ACCEL_t Compose_SBG_ECAN_MSG_IMU_ACCEL(uint32_t id, int16_t accel_x, int16_t accel_y, int16_t accel_z);
void Parse_SBG_ECAN_MSG_IMU_ACCEL(uint8_t* data, uint32_t* id, float* accel_x, float* accel_y, float* accel_z);

typedef struct SBG_ECAN_MSG_IMU_GYRO {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_IMU_GYRO_t;

SBG_ECAN_MSG_IMU_GYRO_t Compose_SBG_ECAN_MSG_IMU_GYRO(uint32_t id, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z);
void Parse_SBG_ECAN_MSG_IMU_GYRO(uint8_t* data, uint32_t* id, float* gyro_x, float* gyro_y, float* gyro_z);

typedef struct SBG_ECAN_MSG_IMU_DELTA_VEL {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_IMU_DELTA_VEL_t;

SBG_ECAN_MSG_IMU_DELTA_VEL_t Compose_SBG_ECAN_MSG_IMU_DELTA_VEL(uint32_t id, int16_t delta_vel_x, int16_t delta_vel_y,
																int16_t delta_vel_z);
void Parse_SBG_ECAN_MSG_IMU_DELTA_VEL(uint8_t* data, uint32_t* id, float* delta_vel_x, float* delta_vel_y,
									  float* delta_vel_z);

typedef struct SBG_ECAN_MSG_IMU_DELTA_ANGLE {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_IMU_DELTA_ANGLE_t;

SBG_ECAN_MSG_IMU_DELTA_ANGLE_t Compose_SBG_ECAN_MSG_IMU_DELTA_ANGLE(uint32_t id, int16_t delta_angle_x,
																	int16_t delta_angle_y, int16_t delta_angle_z);
void Parse_SBG_ECAN_MSG_IMU_DELTA_ANGLE(uint8_t* data, uint32_t* id, float* delta_angle_x, float* delta_angle_y,
										float* delta_angle_z);

// EKF
typedef struct SBG_ECAN_MSG_EKF_INFO {
	uint32_t id;
	uint8_t data[4];
} SBG_ECAN_MSG_EKF_INFO_t;

SBG_ECAN_MSG_EKF_INFO_t Compose_SBG_ECAN_MSG_EKF_INFO(uint32_t id, uint32_t time_stamp);
void Parse_SBG_ECAN_MSG_EKF_INFO(uint8_t* data, uint32_t* id, uint32_t* time_stamp);

typedef struct SBG_ECAN_MSG_EKF_QUAT {
	uint32_t id;
	uint8_t data[8];
} SBG_ECAN_MSG_EKF_QUAT_t;

SBG_ECAN_MSG_EKF_QUAT_t Compose_SBG_ECAN_MSG_EKF_QUAT(uint32_t id, int16_t q0, int16_t q1, int16_t q2, int16_t q3);
void Parse_SBG_ECAN_MSG_EKF_QUAT(uint8_t* data, uint32_t* id, float* q0, float* q1, float* q2, float* q3);

typedef struct SBG_ECAN_MSG_EKF_EULER {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_EKF_EULER_t;

SBG_ECAN_MSG_EKF_EULER_t Compose_SBG_ECAN_MSG_EKF_EULER(uint32_t id, int16_t roll, int16_t pitch, int16_t yaw);
void Parse_SBG_ECAN_MSG_EKF_EULER(uint8_t* data, uint32_t* id, float* roll, float* pitch, float* yaw);

typedef struct SBG_ECAN_MSG_EKF_ORIENTATION_ACC {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_EKF_ORIENTATION_ACC_t;

SBG_ECAN_MSG_EKF_ORIENTATION_ACC_t Compose_SBG_ECAN_MSG_EKF_ORIENTATION_ACC(uint32_t id, int16_t roll_acc,
																			int16_t pitch_acc, int16_t yaw_acc);
void Parse_SBG_ECAN_MSG_EKF_ORIENTATION_ACC(uint8_t* data, uint32_t* id, float* roll_acc, float* pitch_acc,
											float* yaw_acc);

typedef struct SBG_ECAN_MSG_EKF_POS {
	uint32_t id;
	uint8_t data[8];
} SBG_ECAN_MSG_EKF_POS_t;

SBG_ECAN_MSG_EKF_POS_t Compose_SBG_ECAN_MSG_EKF_POS(uint32_t id, int32_t latitude, int32_t longitude);
void Parse_SBG_ECAN_MSG_EKF_POS(uint8_t* data, uint32_t* id, float* latitude, float* longitude);

typedef struct SBG_ECAN_MSG_EKF_ALTITUDE {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_EKF_ALTITUDE_t;

SBG_ECAN_MSG_EKF_ALTITUDE_t Compose_SBG_ECAN_MSG_EKF_ALTITUDE(uint32_t id, int32_t altitude, int16_t undulation);
void Parse_SBG_ECAN_MSG_EKF_ALTITUDE(uint8_t* data, uint32_t* id, float* altitude, float* undulation);

typedef struct SBG_ECAN_MSG_EKF_POS_ACC {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_EKF_POS_ACC_t;

SBG_ECAN_MSG_EKF_POS_ACC_t Compose_SBG_ECAN_MSG_EKF_POS_ACC(uint32_t id, int16_t latitude_acc, int16_t longitude_acc,
															int16_t altitude_acc);
void Parse_SBG_ECAN_MSG_EKF_POS_ACC(uint8_t* data, uint32_t* id, int16_t* latitude_acc, int16_t* longitude_acc,
									int16_t* altitude_acc);

typedef struct SBG_ECAN_MSG_EKF_VEL_NED {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_EKF_VEL_NED_t;

SBG_ECAN_MSG_EKF_VEL_NED_t Compose_SBG_ECAN_MSG_EKF_VEL_NED(uint32_t id, int16_t velocity_n, int16_t velocity_e,
															int16_t velocity_d);
void Parse_SBG_ECAN_MSG_EKF_VEL_NED(uint8_t* data, uint32_t* id, int16_t* velocity_n, int16_t* velocity_e,
									int16_t* velocity_d);

typedef struct SBG_ECAN_MSG_EKF_VEL_NED_ACC {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_EKF_VEL_NED_ACC_t;

SBG_ECAN_MSG_EKF_VEL_NED_ACC_t Compose_SBG_ECAN_MSG_EKF_VEL_NED_ACC(uint32_t id, uint16_t velocity_acc_n,
																	uint16_t velocity_acc_e, uint16_t velocity_acc_d);
void Parse_SBG_ECAN_MSG_EKF_VEL_NED_ACC(uint8_t* data, uint32_t* id, uint16_t* velocity_acc_n, uint16_t* velocity_acc_e,
										uint16_t* velocity_acc_d);

typedef struct SBG_ECAN_MSG_EKF_VEL_BODY {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_EKF_VEL_BODY_t;

SBG_ECAN_MSG_EKF_VEL_BODY_t Compose_SBG_ECAN_MSG_EKF_VEL_BODY(uint32_t id, int16_t velocity_x, int16_t velocity_y,
															  int16_t velocity_z);
void Parse_SBG_ECAN_MSG_EKF_VEL_BODY(uint8_t* data, uint32_t* id, int16_t* velocity_x, int16_t* velocity_y,
									 int16_t* velocity_z);

// Vehicle
typedef struct SBG_ECAN_MSG_AUTO_TRACK_SLIP_CURV {
	uint32_t id;
	uint8_t data[7];
} SBG_ECAN_MSG_AUTO_TRACK_SLIP_CURV_t;

SBG_ECAN_MSG_AUTO_TRACK_SLIP_CURV_t Compose_SBG_ECAN_MSG_AUTO_TRACK_SLIP_CURV(uint32_t id, int16_t angle_track,
																			  int16_t angle_slip,
																			  uint16_t curvature_radius,
																			  uint8_t auto_status);
void Parse_SBG_ECAN_MSG_AUTO_TRACK_SLIP_CURV(uint8_t* data, uint32_t* id, int16_t* angle_track, int16_t* angle_slip,
											 uint16_t* curvature_radius, uint8_t* auto_status);

// Magnetic
typedef struct SBG_ECAN_MSG_MAG_0 {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_MAG_0_t;

SBG_ECAN_MSG_MAG_0_t Compose_SBG_ECAN_MSG_MAG_0(uint32_t id, uint32_t time_stamp, uint16_t mag_status);
void Parse_SBG_ECAN_MSG_MAG_0(uint8_t* data, uint32_t* id, uint32_t* time_stamp, uint16_t* mag_status);

typedef struct SBG_ECAN_MSG_MAG_1 {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_MAG_1_t;

SBG_ECAN_MSG_MAG_1_t Compose_SBG_ECAN_MSG_MAG_1(uint32_t id, int16_t mag_x, int16_t mag_y, int16_t mag_z);
void Parse_SBG_ECAN_MSG_MAG_1(uint8_t* data, uint32_t* id, int16_t* mag_x, int16_t* mag_y, int16_t* mag_z);

typedef struct SBG_ECAN_MSG_MAG_2 {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_MAG_2_t;

SBG_ECAN_MSG_MAG_2_t Compose_SBG_ECAN_MSG_MAG_2(uint32_t id, int16_t acc_x, int16_t acc_y, int16_t acc_z);
void Parse_SBG_ECAN_MSG_MAG_2(uint8_t* data, uint32_t* id, int16_t* acc_x, int16_t* acc_y, int16_t* acc_z);

// Odometer
typedef struct SBG_ECAN_MSG_ODO_INFO {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_ODO_INFO_t;

SBG_ECAN_MSG_ODO_INFO_t Compose_SBG_ECAN_MSG_INFO(uint32_t id, uint32_t time_stamp, uint16_t odo_status);
void Parse_SBG_ECAN_MSG_INFO(uint8_t* data, uint32_t* id, uint32_t* time_stamp, uint16_t* odo_status);

typedef struct SBG_ECAN_MSG_ODO_VEL {
	uint32_t id;
	uint8_t data[2];
} SBG_ECAN_MSG_ODO_VEL_t;

SBG_ECAN_MSG_ODO_VEL_t Compose_SBG_ECAN_MSG_ODO_VEL(uint32_t id, int16_t velocity);
void Parse_SBG_ECAN_MSG_ODO_VEL(uint8_t* data, uint32_t* id, int16_t* velocity);

// Air
typedef struct SBG_ECAN_MSG_AIR_DATA_INFO {
	uint32_t id;
	uint8_t data[7];
} SBG_ECAN_MSG_AIR_DATA_INFO_t;

SBG_ECAN_MSG_AIR_DATA_INFO_t Compose_SBG_ECAN_MSG_AIR_DATA_INFO(uint32_t id, uint32_t time_stamp,
																uint8_t air_data_status, int16_t air_temperature);
void Parse_SBG_ECAN_MSG_AIR_DATA_INFO(uint8_t* data, uint32_t* id, uint32_t* time_stamp, uint8_t* air_data_status,
									  int16_t* air_temperature);

typedef struct SBG_ECAN_MSG_AIR_DATA_ALTITUDE {
	uint32_t id;
	uint8_t data[8];
} SBG_ECAN_MSG_AIR_DATA_ALTITUDE_t;

SBG_ECAN_MSG_AIR_DATA_ALTITUDE_t Compose_SBG_ECAN_MSG_AIR_DATA_ALTITUDE(uint32_t id, uint32_t pressure_abs,
																		int32_t altitude);
void Parse_SBG_ECAN_MSG_AIR_DATA_ALTITUDE(uint8_t* data, uint32_t* id, uint32_t* pressure_abs, int32_t* altitude);

typedef struct SBG_ECAN_MSG_AIR_DATA_AIRSPEED {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_AIR_DATA_AIRSPEED_t;

SBG_ECAN_MSG_AIR_DATA_AIRSPEED_t Compose_SBG_ECAN_MSG_AIR_DATA_AIRSPEED(uint32_t id, int32_t pressure_diff,
																		int16_t airspeed);
void Parse_SBG_ECAN_MSG_AIR_DATA_AIRSPEED(uint8_t* data, uint32_t* id, int32_t* pressure_diff, int16_t* airspeed);

// GPS1
typedef struct SBG_ECAN_MSG_GPS1_VEL_INFO {
	uint32_t id;
	uint8_t data[8];
} SBG_ECAN_MSG_GPS1_VEL_INFO_t;

SBG_ECAN_MSG_GPS1_VEL_INFO_t Compose_SBG_ECAN_MSG_GPS1_VEL_INFO(uint32_t id, uint32_t time_stamp,
																uint32_t gps_vel_status);
void Parse_SBG_ECAN_MSG_GPS1_VEL_INFO(uint8_t* data, uint32_t* id, uint32_t* time_stamp, uint32_t* gps_vel_status);

typedef struct SBG_ECAN_MSG_GPS1_VEL {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_GPS1_VEL_t;

SBG_ECAN_MSG_GPS1_VEL_t Compose_SBG_ECAN_MSG_GPS1_VEL(uint32_t id, int16_t vel_n, int16_t vel_e, int16_t vel_d);
void Parse_SBG_ECAN_MSG_GPS1_VEL(uint8_t* data, uint32_t* id, float* vel_n, float* vel_e, float* vel_d);

typedef struct SBG_ECAN_MSG_GPS1_VEL_ACC {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_GPS1_VEL_ACC_t;

SBG_ECAN_MSG_GPS1_VEL_ACC_t Compose_SGB_ECAN_MSG_GPS1_VEL_ACC(uint32_t id, uint16_t vel_acc_n, uint16_t vel_acc_e,
															  uint16_t vel_acc_d);
void Parse_SBG_ECAN_MSG_GPS1_VEL_ACC(uint8_t* data, uint32_t* id, uint16_t* vell_acc_n, uint16_t* vell_acc_e,
									 uint16_t* vel_acc_d);

typedef struct SBG_ECAN_MSG_GPS1_COURSE {
	uint32_t id;
	uint8_t data[4];
} SBG_ECAN_MSG_GPS1_COURSE_t;

SBG_ECAN_MSG_GPS1_COURSE_t Compose_SBG_ECAN_MSG_GPS1_COURSE(uint32_t id, uint16_t course, uint16_t course_acc);
void Parse_SBG_ECAN_MSG_GPS1_COURSE(uint8_t* data, uint32_t* id, float* course, float* course_acc);

typedef struct SBG_ECAN_MSG_GPS1_POS_INFO {
	uint32_t id;
	uint8_t data[8];
} SBG_ECAN_MSG_GPS1_POS_INFO_t;

SBG_ECAN_MSG_GPS1_POS_INFO_t Compose_SBG_ECAN_MSG_GPS1_POS_INFO(uint32_t id, uint32_t time_stamp,
																uint32_t gps_pos_status);
void Parse_SBG_ECAN_MSG_GPS1_POS_INFO(uint8_t* data, uint32_t* id, uint32_t* time_stamp, uint32_t* gps_pos_status);

typedef struct SBG_ECAN_MSG_GPS1_POS {
	uint32_t id;
	uint8_t data[8];
} SBG_ECAN_MSG_GPS1_POS_t;

SBG_ECAN_MSG_GPS1_POS_t Compose_SBG_ECAN_MSG_GPS1_POS(uint32_t id, int32_t latitude, int32_t longitude);
void Parse_SBG_ECAN_MSG_GPS1_POS(uint8_t* data, uint32_t* id, float* latitude, float* longitude);

typedef struct SBG_ECAN_MSG_GPS1_POS_ALT {
	uint32_t id;
	uint8_t data[8];
} SBG_ECAN_MSG_GPS1_POS_ALT_t;

SBG_ECAN_MSG_GPS1_POS_ALT_t Compose_SBG_ECAN_MSG_GPS1_POS_ALT(uint32_t id, int32_t altitude, int16_t undulation,
															  uint8_t num_sv, uint8_t diff_corr_age);
void Parse_SBG_ECAN_MSG_GPS1_POS_ALT(uint8_t* data, uint32_t* id, int32_t* altitude, int16_t* undulation,
									 uint8_t* num_sv, uint8_t* diff_corr_age);

typedef struct SBG_ECAN_MSG_GPS1_POS_ACC {
	uint32_t id;
	uint8_t data[8];
} SBG_ECAN_MSG_GPS1_POS_ACC_t;

SBG_ECAN_MSG_GPS1_POS_ACC_t Compose_SBG_ECAN_MSG_GPS1_POS_ACC(uint32_t id, uint16_t lat_acc, uint16_t long_acc,
															  uint16_t alt_acc, uint16_t base_station_id);
void Parse_SBG_ECAN_MSG_GPS1_POS_ACC(uint8_t* data, uint32_t* id, uint16_t* lat_acc, uint16_t* long_acc,
									 uint16_t* alt_acc, uint16_t* base_station_id);

// Events
typedef struct SBG_ECAN_MSG_EVENT_INFO_A {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_EVENT_INFO_A_t;

SBG_ECAN_MSG_EVENT_INFO_A_t Compose_SBG_ECAN_MSG_EVENT_INFO_A(uint32_t id, uint32_t time_stamp, uint16_t event_status);
void Parse_SBG_ECAN_MSG_EVENT_INFO_A(uint8_t* data, uint32_t* id, uint32_t* time_stamp, uint16_t* event_status);

typedef struct SBG_ECAN_MSG_EVENT_TIME_A {
	uint32_t id;
	uint8_t data[8];
} SBG_ECAN_MSG_EVENT_TIME_A_t;

SBG_ECAN_MSG_EVENT_TIME_A_t Compose_SBG_ECAN_MSG_EVEN_TIME_A(uint32_t id, uint16_t time_offset_0,
															 uint16_t time_offset_1, uint16_t time_offset_2,
															 uint16_t time_offset_3);
void Parse_SBG_ECAN_MSG_EVENT_TIME_A(uint8_t* data, uint32_t* id, uint16_t* time_offset_0, uint16_t* time_offset_1,
									 uint16_t* time_offset_2, uint16_t* time_offset_3);

typedef struct SBG_ECAN_MSG_EVENT_INFO_B {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_EVENT_INFO_B_t;

SBG_ECAN_MSG_EVENT_INFO_B_t Compose_SBG_ECAN_MSG_EVENT_INFO_B(uint32_t id, uint32_t time_stamp, uint16_t event_status);
void Parse_SBG_ECAN_MSG_EVENT_INFO_B(uint8_t* data, uint32_t* id, uint32_t* time_stamp, uint16_t* event_status);

typedef struct SBG_ECAN_MSG_EVENT_TIME_B {
	uint32_t id;
	uint8_t data[8];
} SBG_ECAN_MSG_EVENT_TIME_B_t;

SBG_ECAN_MSG_EVENT_TIME_B_t Compose_SBG_ECAN_MSG_EVEN_TIME_B(uint32_t id, uint16_t time_offset_0,
															 uint16_t time_offset_1, uint16_t time_offset_2,
															 uint16_t time_offset_3);
void Parse_SBG_ECAN_MSG_EVENT_TIME_B(uint8_t* data, uint32_t* id, uint16_t* time_offset_0, uint16_t* time_offset_1,
									 uint16_t* time_offset_2, uint16_t* time_offset_3);

typedef struct SBG_ECAN_MSG_EVENT_INFO_C {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_EVENT_INFO_C_t;

SBG_ECAN_MSG_EVENT_INFO_C_t Compose_SBG_ECAN_MSG_EVENT_INFO_C(uint32_t id, uint32_t time_stamp, uint16_t event_status);
void Parse_SBG_ECAN_MSG_EVENT_INFO_C(uint8_t* data, uint32_t* id, uint32_t* time_stamp, uint16_t* event_status);

typedef struct SBG_ECAN_MSG_EVENT_TIME_C {
	uint32_t id;
	uint8_t data[8];
} SBG_ECAN_MSG_EVENT_TIME_C_t;

SBG_ECAN_MSG_EVENT_TIME_C_t Compose_SBG_ECAN_MSG_EVEN_TIME_C(uint32_t id, uint16_t time_offset_0,
															 uint16_t time_offset_1, uint16_t time_offset_2,
															 uint16_t time_offset_3);
void Parse_SBG_ECAN_MSG_EVENT_TIME_C(uint8_t* data, uint32_t* id, uint16_t* time_offset_0, uint16_t* time_offset_1,
									 uint16_t* time_offset_2, uint16_t* time_offset_3);

typedef struct SBG_ECAN_MSG_EVENT_INFO_D {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_EVENT_INFO_D_t;

SBG_ECAN_MSG_EVENT_INFO_D_t Compose_SBG_ECAN_MSG_EVENT_INFO_D(uint32_t id, uint32_t time_stamp, uint16_t event_status);
void Parse_SBG_ECAN_MSG_EVENT_INFO_D(uint8_t* data, uint32_t* id, uint32_t* time_stamp, uint16_t* event_status);

typedef struct SBG_ECAN_MSG_EVENT_TIME_D {
	uint32_t id;
	uint8_t data[8];
} SBG_ECAN_MSG_EVENT_TIME_D_t;

SBG_ECAN_MSG_EVENT_TIME_D_t Compose_SBG_ECAN_MSG_EVEN_TIME_D(uint32_t id, uint16_t time_offset_0,
															 uint16_t time_offset_1, uint16_t time_offset_2,
															 uint16_t time_offset_3);
void Parse_SBG_ECAN_MSG_EVENT_TIME_D(uint8_t* data, uint32_t* id, uint16_t* time_offset_0, uint16_t* time_offset_1,
									 uint16_t* time_offset_2, uint16_t* time_offset_3);

typedef struct SBG_ECAN_MSG_EVENT_INFO_E {
	uint32_t id;
	uint8_t data[6];
} SBG_ECAN_MSG_EVENT_INFO_E_t;

SBG_ECAN_MSG_EVENT_INFO_E_t Compose_SBG_ECAN_MSG_EVENT_INFO_E(uint32_t id, uint32_t time_stamp, uint16_t event_status);
void Parse_SBG_ECAN_MSG_EVENT_INFO_E(uint8_t* data, uint32_t* id, uint32_t* time_stamp, uint16_t* event_status);

typedef struct SBG_ECAN_MSG_EVENT_TIME_E {
	uint32_t id;
	uint8_t data[8];
} SBG_ECAN_MSG_EVENT_TIME_E_t;

SBG_ECAN_MSG_EVENT_TIME_E_t Compose_SBG_ECAN_MSG_EVEN_TIME_E(uint32_t id, uint16_t time_offset_0,
															 uint16_t time_offset_1, uint16_t time_offset_2,
															 uint16_t time_offset_3);
void Parse_SBG_ECAN_MSG_EVENT_TIME_E(uint8_t* data, uint32_t* id, uint16_t* time_offset_0, uint16_t* time_offset_1,
									 uint16_t* time_offset_2, uint16_t* time_offset_3);