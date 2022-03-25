#include "sbg_can.hpp"

SBG_ECAN_MSG_STATUS_01_t Compose_SBG_ECAN_MSG_STATUS_01(uint32_t id, uint32_t time_stamp, uint16_t general_status,
														uint16_t clock_status) {
	SBG_ECAN_MSG_STATUS_01_t msg;
	msg.id = id;
	// time_stamp
	msg.data[0] = (time_stamp >> 0) & 0xFF;
	msg.data[1] = (time_stamp >> 8) & 0xFF;
	msg.data[2] = (time_stamp >> 16) & 0xFF;
	msg.data[3] = (time_stamp >> 24) & 0xFF;

	// general_status
	msg.data[4] = (general_status >> 0) & 0xFF;
	msg.data[5] = (general_status >> 8) & 0xFF;

	// clock_status
	msg.data[6] = (clock_status >> 0) & 0xFF;
	msg.data[7] = (clock_status >> 8) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_STATUS_01(uint8_t* data, uint32_t* id, uint32_t* time_stamp, uint16_t* general_status,
								  uint16_t* clock_status) {
	*id = SBG_ECAN_MSG_STATUS_01_ID;
	*time_stamp = (uint32_t)((data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
	*general_status = (uint16_t)((data[4] << 0) | (data[5] << 8));
	*clock_status = (uint16_t)((data[6] << 0) | (data[7] << 8));
}

SBG_ECAN_MSG_STATUS_02_t Compose_SBG_ECAN_MSG_STATUS_02(uint32_t id, uint32_t com_status, uint32_t aiding_status) {
	SBG_ECAN_MSG_STATUS_02_t msg;
	msg.id = id;

	// com_status
	msg.data[0] = (com_status >> 0) & 0xFF;
	msg.data[1] = (com_status >> 8) & 0xFF;
	msg.data[2] = (com_status >> 16) & 0xFF;
	msg.data[3] = (com_status >> 24) & 0xFF;

	// aiding_status
	msg.data[4] = (aiding_status >> 0) & 0xFF;
	msg.data[5] = (aiding_status >> 8) & 0xFF;
	msg.data[6] = (aiding_status >> 16) & 0xFF;
	msg.data[7] = (aiding_status >> 24) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_STATUS_02(uint8_t* data, uint32_t* id, uint32_t* com_status, uint32_t* aiding_status) {
	*id = SBG_ECAN_MSG_STATUS_02_ID;
	*com_status = (uint32_t)((data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
	*aiding_status = (uint32_t)((data[4] << 0) | (data[5] << 8) | (data[6] << 16) | (data[7] << 24));
}

SBG_ECAN_MSG_STATUS_03_t Compose_SBG_ECAN_MSG_STATUS_03(uint32_t id, uint32_t solution_status, uint16_t heave_status) {
	SBG_ECAN_MSG_STATUS_03_t msg;
	msg.id = id;

	// solution_status
	msg.data[0] = (solution_status >> 0) & 0xFF;
	msg.data[1] = (solution_status >> 8) & 0xFF;
	msg.data[2] = (solution_status >> 16) & 0xFF;
	msg.data[3] = (solution_status >> 24) & 0xFF;

	// heave_status
	msg.data[4] = (heave_status >> 0) & 0xFF;
	msg.data[5] = (heave_status >> 8) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_STATUS_03(uint8_t* data, uint32_t* id, uint32_t* solution_status, uint16_t* heave_status) {
	*id = SBG_ECAN_MSG_STATUS_03_ID;
	*solution_status = (uint32_t)((data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
	*heave_status = (uint16_t)((data[4] << 0) | (data[5] << 8));
}

SBG_ECAN_MSG_UTC_0_t Compose_SBG_ECAN_MSG_UTC_0(uint32_t id, uint32_t time_stamp, uint32_t gps_tow) {
	SBG_ECAN_MSG_UTC_0_t msg;
	msg.id = id;

	// time_stamp
	msg.data[0] = (time_stamp >> 0) & 0xFF;
	msg.data[1] = (time_stamp >> 8) & 0xFF;
	msg.data[2] = (time_stamp >> 16) & 0xFF;
	msg.data[3] = (time_stamp >> 24) & 0xFF;

	// gps_tow
	msg.data[0] = (gps_tow >> 0) & 0xFF;
	msg.data[1] = (gps_tow >> 8) & 0xFF;
	msg.data[2] = (gps_tow >> 16) & 0xFF;
	msg.data[3] = (gps_tow >> 24) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_UTC_0(uint8_t* data, uint32_t* id, uint32_t* time_stamp, uint32_t* gps_tow) {
	*id = SBG_ECAN_MSG_UTC_0_ID;
	*time_stamp = (uint32_t)((data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
	*gps_tow = (uint32_t)((data[4] << 0) | (data[5] << 8) | (data[6] << 16) | (data[7] << 24));
}

SBG_ECAN_MSG_UTC_1_t Compose_SBG_ECAN_MSG_UTC_1(uint32_t id, uint8_t year, uint8_t month, uint8_t day, uint8_t hour,
												uint8_t min, uint8_t sec, uint16_t micro_sec) {
	SBG_ECAN_MSG_UTC_1_t msg;
	msg.id = id;

	msg.data[0] = year;
	msg.data[1] = month;
	msg.data[2] = day;
	msg.data[3] = hour;
	msg.data[4] = min;
	msg.data[5] = sec;
	msg.data[6] = (micro_sec >> 0) & 0xFF;
	msg.data[7] = (micro_sec >> 8) & 0xFF;

	micro_sec /= 100.f;

	return msg;
}

void Parse_SBG_ECAN_MSG_UTC_1(uint8_t* data, uint32_t* id, uint8_t* year, uint8_t* month, uint8_t* day, uint8_t* hour,
							  uint8_t* min, uint8_t* sec, uint16_t* micro_sec) {
	*id = SBG_ECAN_MSG_UTC_1_ID;
	*year = data[0];
	*month = data[1];
	*day = data[2];
	*hour = data[3];
	*min = data[4];
	*sec = data[5];
	*micro_sec = (uint16_t)((data[6] << 0) | (data[7] << 8));
	*micro_sec *= 100.f;
}

SBG_ECAN_MSG_IMU_INFO_t Compose_SBG_ECAN_MSG_IMU_INFO(uint32_t id, uint32_t time_stamp, uint16_t imu_status,
													  uint16_t temperature) {
	temperature /= (float)10e-2;

	SBG_ECAN_MSG_IMU_INFO_t msg;
	msg.id = id;

	// time_stamp
	msg.data[0] = (time_stamp >> 0) & 0xFF;
	msg.data[1] = (time_stamp >> 8) & 0xFF;
	msg.data[2] = (time_stamp >> 16) & 0xFF;
	msg.data[3] = (time_stamp >> 24) & 0xFF;

	// imu_status
	msg.data[4] = (imu_status >> 0) & 0xFF;
	msg.data[5] = (imu_status >> 8) & 0xFF;

	// temperature
	msg.data[6] = (temperature >> 0) & 0xFF;
	msg.data[7] = (temperature >> 8) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_IMU_INFO(uint8_t* data, uint32_t* id, uint32_t* time_stamp, uint16_t* imu_status,
								 float* temperature) {
	*id = SBG_ECAN_MSG_IMU_INFO_ID;
	*time_stamp = (uint32_t)((data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
	*imu_status = (uint16_t)((data[4] << 0) | (data[5] << 8));
	*temperature = (uint16_t)((data[6] << 0) | (data[7] << 8));
	*temperature *= 0.01;
}

SBG_ECAN_MSG_IMU_ACCEL_t Compose_SBG_ECAN_MSG_IMU_ACCEL(uint32_t id, int16_t accel_x, int16_t accel_y,
														int16_t accel_z) {
	accel_x /= 0.01;
	accel_y /= 0.01;
	accel_z /= 0.01;

	SBG_ECAN_MSG_IMU_ACCEL_t msg;
	msg.id = id;

	// accel_x
	msg.data[0] = (accel_x >> 0) & 0xFF;
	msg.data[1] = (accel_x >> 8) & 0xFF;

	// accel_y
	msg.data[2] = (accel_y >> 0) & 0xFF;
	msg.data[3] = (accel_y >> 8) & 0xFF;

	// accel_z
	msg.data[4] = (accel_z >> 0) & 0xFF;
	msg.data[6] = (accel_z >> 8) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_IMU_ACCEL(uint8_t* data, uint32_t* id, float* accel_x, float* accel_y, float* accel_z) {
	*id = SBG_ECAN_MSG_IMU_ACCEL_ID;
	*accel_x = (int16_t)((data[0] << 0) | (data[1] << 8));
	*accel_y = (int16_t)((data[2] << 0) | (data[3] << 8));
	*accel_z = (int16_t)((data[4] << 0) | (data[5] << 8));

	*accel_x *= 0.01;
	*accel_y *= 0.01;
	*accel_z *= 0.01;
}

SBG_ECAN_MSG_IMU_GYRO_t Compose_SBG_ECAN_MSG_IMU_GYRO(uint32_t id, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z) {
	gyro_x /= 0.001;
	gyro_y /= 0.001;
	gyro_z /= 0.001;

	SBG_ECAN_MSG_IMU_GYRO_t msg;
	msg.id = id;

	// gyro_x
	msg.data[0] = (gyro_x >> 0) & 0xFF;
	msg.data[1] = (gyro_x >> 8) & 0xFF;

	// gyro_y
	msg.data[2] = (gyro_y >> 0) & 0xFF;
	msg.data[3] = (gyro_y >> 8) & 0xFF;

	// gyro_z
	msg.data[4] = (gyro_z >> 0) & 0xFF;
	msg.data[5] = (gyro_z >> 8) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_IMU_GYRO(uint8_t* data, uint32_t* id, float* gyro_x, float* gyro_y, float* gyro_z) {
	*id = SBG_ECAN_MSG_IMU_GYRO_ID;
	*gyro_x = (int16_t)((data[0] << 0) | (data[1] << 8));
	*gyro_y = (int16_t)((data[2] << 0) | (data[3] << 8));
	*gyro_z = (int16_t)((data[4] << 0) | (data[5] << 8));

	*gyro_x *= 0.001;
	*gyro_y *= 0.001;
	*gyro_z *= 0.001;
}

SBG_ECAN_MSG_IMU_DELTA_VEL_t Compose_SBG_ECAN_MSG_IMU_DELTA_VEL(uint32_t id, int16_t delta_vel_x, int16_t delta_vel_y,
																int16_t delta_vel_z) {
	delta_vel_x /= 0.01;
	delta_vel_y /= 0.01;
	delta_vel_z /= 0.01;

	SBG_ECAN_MSG_IMU_DELTA_VEL_t msg;
	msg.id = id;

	// delta_vel_x
	msg.data[0] = (delta_vel_x >> 0) & 0xFF;
	msg.data[1] = (delta_vel_x >> 8) & 0xFF;

	// delta_vel_y
	msg.data[2] = (delta_vel_y >> 0) & 0xFF;
	msg.data[3] = (delta_vel_y >> 8) & 0xFF;

	// delta_vel_z
	msg.data[4] = (delta_vel_z >> 0) & 0xFF;
	msg.data[5] = (delta_vel_z >> 8) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_IMU_DELTA_VEL(uint8_t* data, uint32_t* id, float* delta_vel_x, float* delta_vel_y,
									  float* delta_vel_z) {
	*id = SBG_ECAN_MSG_IMU_DELTA_VEL_ID;
	*delta_vel_x = (int16_t)((data[0] << 0) | (data[1] << 8));
	*delta_vel_y = (int16_t)((data[2] << 0) | (data[3] << 8));
	*delta_vel_z = (int16_t)((data[4] << 0) | (data[5] << 8));

	*delta_vel_x *= 0.01;
	*delta_vel_y *= 0.01;
	*delta_vel_z *= 0.01;
}

SBG_ECAN_MSG_IMU_DELTA_ANGLE_t Compose_SBG_ECAN_MSG_IMU_DELTA_ANGLE(uint32_t id, int16_t delta_angle_x,
																	int16_t delta_angle_y, int16_t delta_angle_z) {
	delta_angle_x /= 0.001;
	delta_angle_y /= 0.001;
	delta_angle_z /= 0.001;

	SBG_ECAN_MSG_IMU_DELTA_ANGLE_t msg;
	msg.id = id;

	// delta_angle_x
	msg.data[0] = (delta_angle_x >> 0) & 0xFF;
	msg.data[1] = (delta_angle_x >> 8) & 0xFF;

	// delta_angle_y
	msg.data[2] = (delta_angle_y >> 0) & 0xFF;
	msg.data[3] = (delta_angle_y >> 8) & 0xFF;

	// delta_angle_z
	msg.data[4] = (delta_angle_z >> 0) & 0xFF;
	msg.data[5] = (delta_angle_z >> 8) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_IMU_DELTA_ANGLE(uint8_t* data, uint32_t* id, float* delta_angle_x, float* delta_angle_y,
										float* delta_angle_z) {
	*id = SBG_ECAN_MSG_IMU_DELTA_ANGLE_ID;
	*delta_angle_x = (int16_t)((data[0] << 0) | (data[1] << 8));
	*delta_angle_y = (int16_t)((data[2] << 0) | (data[3] << 8));
	*delta_angle_z = (int16_t)((data[4] << 0) | (data[5] << 8));

	*delta_angle_x *= 0.001;
	*delta_angle_y *= 0.001;
	*delta_angle_z *= 0.001;
}

SBG_ECAN_MSG_EKF_INFO_t Compose_SBG_ECAN_MSG_EKF_INFO(uint32_t id, uint32_t time_stamp) {
	SBG_ECAN_MSG_EKF_INFO_t msg;
	msg.id = id;

	// time_stamp
	msg.data[0] = (time_stamp >> 0) & 0xFF;
	msg.data[1] = (time_stamp >> 8) & 0xFF;
	msg.data[2] = (time_stamp >> 16) & 0xFF;
	msg.data[3] = (time_stamp >> 24) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_EKF_INFO(uint8_t* data, uint32_t* id, uint32_t* time_stamp) {
	*id = SBG_ECAN_MSG_EKF_INFO_ID;
	*time_stamp = (uint32_t)((data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
}

SBG_ECAN_MSG_EKF_QUAT_t Compose_SBG_ECAN_MSG_EKF_QUAT(uint32_t id, int16_t q0, int16_t q1, int16_t q2, int16_t q3) {
	q0 /= 0.000030519;
	q1 /= 0.000030519;
	q2 /= 0.000030519;
	q3 /= 0.000030519;

	SBG_ECAN_MSG_EKF_QUAT_t msg;
	msg.id = id;

	// q0
	msg.data[0] = (q0 >> 0) & 0xFF;
	msg.data[1] = (q0 >> 8) & 0xFF;

	// q1
	msg.data[2] = (q1 >> 0) & 0xFF;
	msg.data[3] = (q1 >> 8) & 0xFF;

	// q2
	msg.data[4] = (q2 >> 0) & 0xFF;
	msg.data[5] = (q2 >> 8) & 0xFF;

	// q3
	msg.data[6] = (q3 >> 0) & 0xFF;
	msg.data[7] = (q3 >> 8) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_EKF_QUAT(uint8_t* data, uint32_t* id, float* q0, float* q1, float* q2, float* q3) {
	*id = SBG_ECAN_MSG_EKF_QUAT_ID;
	*q0 = (int16_t)((data[0] << 0) | data[1] << 8);
	*q1 = (int16_t)((data[2] << 0) | data[3] << 8);
	*q2 = (int16_t)((data[4] << 0) | data[5] << 8);
	*q3 = (int16_t)((data[6] << 0) | data[7] << 8);

	*q0 *= 0.000030519;
	*q1 *= 0.000030519;
	*q2 *= 0.000030519;
	*q3 *= 0.000030519;
}

SBG_ECAN_MSG_EKF_EULER_t Compose_SBG_ECAN_MSG_EKF_EULER(uint32_t id, int16_t roll, int16_t pitch, int16_t yaw) {
	roll /= 0.0001;
	pitch /= 0.0001;
	yaw /= 0.0001;

	SBG_ECAN_MSG_EKF_EULER_t msg;
	msg.id = id;

	// roll
	msg.data[0] = (roll >> 0) & 0xFF;
	msg.data[1] = (roll >> 8) & 0xFF;

	// pitch
	msg.data[2] = (pitch >> 0) & 0xFF;
	msg.data[3] = (pitch >> 8) & 0xFF;

	// yaw
	msg.data[4] = (yaw >> 0) & 0xFF;
	msg.data[5] = (yaw >> 8) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_EKF_EULER(uint8_t* data, uint32_t* id, float* roll, float* pitch, float* yaw) {
	*id = SBG_ECAN_MSG_EKF_EULER_ID;
	*roll = (int16_t)((data[0] << 0) | (data[1] << 8));
	*pitch = (int16_t)((data[2] << 0) | (data[3] << 8));
	*yaw = (int16_t)((data[4] << 0) | (data[5] << 8));

	*roll *= 0.0001;
	*pitch *= 0.0001;
	*yaw *= 0.0001;
}

SBG_ECAN_MSG_EKF_ORIENTATION_ACC_t Compose_SBG_ECAN_MSG_EKF_ORIENTATION_ACC(uint32_t id, int16_t roll_acc,
																			int16_t pitch_acc, int16_t yaw_acc) {
	roll_acc /= 0.0001;
	pitch_acc /= 0.0001;
	yaw_acc /= 0.0001;
	SBG_ECAN_MSG_EKF_ORIENTATION_ACC_t msg;
	msg.id = id;

	// roll_acc
	msg.data[0] = (roll_acc >> 0) & 0xFF;
	msg.data[1] = (roll_acc >> 8) & 0xFF;

	// pitch_acc
	msg.data[2] = (pitch_acc >> 0) & 0xFF;
	msg.data[3] = (pitch_acc >> 8) & 0xFF;

	// yaw_acc
	msg.data[4] = (yaw_acc >> 0) & 0xFF;
	msg.data[5] = (yaw_acc >> 8) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_EKF_ORIENTATION_ACC(uint8_t* data, uint32_t* id, float* roll_acc, float* pitch_acc,
											float* yaw_acc) {
	*id = SBG_ECAN_MSG_EKF_ORIENTATION_ACC_ID;
	*roll_acc = (int16_t)((data[0] << 0) | data[1] << 8);
	*pitch_acc = (int16_t)((data[2] << 0) | data[3] << 8);
	*yaw_acc = (int16_t)((data[4] << 0) | data[5] << 8);

	*roll_acc *= 0.0001;
	*pitch_acc *= 0.0001;
	*yaw_acc *= 0.0001;
}

SBG_ECAN_MSG_EKF_POS_t Compose_SBG_ECAN_MSG_EKF_POS(uint32_t id, int32_t latitude, int32_t longitude) {
	latitude /= 0.0000001;
	longitude /= 0.0000001;
	SBG_ECAN_MSG_EKF_POS_t msg;
	msg.id = id;

	// latitude
	msg.data[0] = (latitude >> 0) & 0xFF;
	msg.data[1] = (latitude >> 8) & 0xFF;
	msg.data[2] = (latitude >> 16) & 0xFF;
	msg.data[3] = (latitude >> 24) & 0xFF;

	// longitude
	msg.data[4] = (longitude >> 0) & 0xFF;
	msg.data[5] = (longitude >> 8) & 0xFF;
	msg.data[6] = (longitude >> 16) & 0xFF;
	msg.data[7] = (longitude >> 24) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_EKF_POS(uint8_t* data, uint32_t* id, float* latitude, float* longitude) {
	*id = SBG_ECAN_MSG_EKF_POS_ID;
	*latitude = (int32_t)((data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
	*longitude = (int32_t)((data[4] << 0) | (data[5] << 8) | (data[6] << 16) | (data[7] << 24));

	*latitude *= 0.0000001;
	*longitude *= 0.0000001;
}

SBG_ECAN_MSG_EKF_ALTITUDE_t Compose_SBG_ECAN_MSG_EKF_ALTITUDE(uint32_t id, int32_t altitude, int16_t undulation) {
	altitude /= 0.001;
	undulation /= 0.005;
	SBG_ECAN_MSG_EKF_ALTITUDE_t msg;
	msg.id = id;

	// altitude
	msg.data[0] = (altitude >> 0) & 0xFF;
	msg.data[1] = (altitude >> 8) & 0xFF;
	msg.data[2] = (altitude >> 16) & 0xFF;
	msg.data[3] = (altitude >> 24) & 0xFF;

	// undulation
	msg.data[4] = (altitude >> 0) & 0xFF;
	msg.data[5] = (altitude >> 8) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_EKF_ALTITUDE(uint8_t* data, uint32_t* id, float* altitude, float* undulation) {
	*id = SBG_ECAN_MSG_EKF_ALTITUDE_ID;
	*altitude = (int32_t)((data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
	*undulation = (int16_t)((data[4] << 0) | (data[5] << 8));

	*altitude *= 0.001;
	*undulation *= 0.005;
}

SBG_ECAN_MSG_GPS1_VEL_INFO_t Compose_SBG_ECAN_MSG_GPS1_VEL_INFO(uint32_t id, uint32_t time_stamp,
																uint32_t gps_vel_status) {
	SBG_ECAN_MSG_GPS1_VEL_INFO_t msg;
	msg.id = id;

	// time_stamp
	msg.data[0] = (time_stamp >> 0) & 0xFF;
	msg.data[1] = (time_stamp >> 8) & 0xFF;
	msg.data[2] = (time_stamp >> 16) & 0xFF;
	msg.data[3] = (time_stamp >> 24) & 0xFF;

	// gps_vel_status
	msg.data[4] = (gps_vel_status >> 0) & 0xFF;
	msg.data[5] = (gps_vel_status >> 8) & 0xFF;
	msg.data[6] = (gps_vel_status >> 16) & 0xFF;
	msg.data[7] = (gps_vel_status >> 24) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_GPS1_VEL_INFO(uint8_t* data, uint32_t* id, uint32_t* time_stamp, uint32_t* gps_vel_status) {
	*id = SBG_ECAN_MSG_GPS1_VEL_INFO_ID;
	*time_stamp = (uint32_t)((data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
	*gps_vel_status = (uint32_t)((data[4] << 0) | (data[5] << 8) | (data[6] << 16) | (data[7] << 24));
}

SBG_ECAN_MSG_GPS1_VEL_t Compose_SBG_ECAN_MSG_GPS1_VEL(uint32_t id, int16_t vel_n, int16_t vel_e, int16_t vel_d) {
	vel_n /= 0.01;
	vel_e /= 0.01;
	vel_d /= 0.01;

	SBG_ECAN_MSG_GPS1_VEL_t msg;
	msg.id = id;

	// vel_n
	msg.data[0] = (vel_n >> 0) & 0xFF;
	msg.data[1] = (vel_n >> 8) & 0xFF;

	// vel_e
	msg.data[2] = (vel_e >> 0) & 0xFF;
	msg.data[3] = (vel_e >> 8) & 0xFF;

	// vel_d
	msg.data[4] = (vel_d >> 0) & 0xFF;
	msg.data[5] = (vel_d >> 8) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_GPS1_VEL(uint8_t* data, uint32_t* id, float* vel_n, float* vel_e, float* vel_d) {
	*id = SBG_ECAN_MSG_GPS1_VEL_ID;
	*vel_n = (int16_t)((data[0] << 0) | data[1] << 8);
	*vel_e = (int16_t)((data[2] << 0) | data[3] << 8);
	*vel_d = (int16_t)((data[4] << 0) | data[5] << 8);

	*vel_n *= 0.01;
	*vel_e *= 0.01;
	*vel_d *= 0.01;
}

SBG_ECAN_MSG_GPS1_COURSE_t Compose_SBG_ECAN_MSG_GPS1_COURSE(uint32_t id, uint16_t course, uint16_t course_acc) {
	course /= 0.01;
	course_acc /= 0.01;

	SBG_ECAN_MSG_GPS1_COURSE_t msg;
	msg.id = id;

	// course
	msg.data[0] = (course << 0) & 0xFF;
	msg.data[1] = (course << 8) & 0xFF;

	// course_acc
	msg.data[2] = (course_acc << 0) & 0xFF;
	msg.data[3] = (course_acc << 8) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_GPS1_COURSE(uint8_t* data, uint32_t* id, float* course, float* course_acc) {
	*id = SBG_ECAN_MSG_GPS1_COURSE_ID;
	*course = (uint16_t)((data[0] << 0) | data[1] << 8);
	*course_acc = (uint16_t)((data[2] << 0) | data[3] << 8);

	*course *= 0.01;
	*course_acc *= 0.01;
}

SBG_ECAN_MSG_GPS1_POS_INFO_t Compose_SBG_ECAN_MSG_GPS1_POS_INFO(uint32_t id, uint32_t time_stamp,
																uint32_t gps_pos_status) {
	SBG_ECAN_MSG_GPS1_POS_INFO_t msg;
	msg.id = id;

	// time_stamp
	msg.data[0] = (time_stamp >> 0) & 0xFF;
	msg.data[1] = (time_stamp >> 8) & 0xFF;
	msg.data[2] = (time_stamp >> 16) & 0xFF;
	msg.data[3] = (time_stamp >> 24) & 0xFF;

	// gps_pos_status
	msg.data[4] = (gps_pos_status >> 0) & 0xFF;
	msg.data[5] = (gps_pos_status >> 8) & 0xFF;
	msg.data[6] = (gps_pos_status >> 16) & 0xFF;
	msg.data[7] = (gps_pos_status >> 24) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_GPS1_POS_INFO(uint8_t* data, uint32_t* id, uint32_t* time_stamp, uint32_t* gps_pos_status) {
	*id = SBG_ECAN_MSG_GPS1_POS_INFO_ID;
	*time_stamp = (uint32_t)((data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
	*gps_pos_status = (uint32_t)((data[4] << 0) | (data[5] << 8) | (data[6] << 16) | (data[7] << 24));
}

SBG_ECAN_MSG_GPS1_POS_t Compose_SBG_ECAN_MSG_GPS1_POS(uint32_t id, int32_t latitude, int32_t longitude) {
	latitude /= 0.0000001;
	longitude /= 0.0000001;

	SBG_ECAN_MSG_GPS1_POS_t msg;
	msg.id = id;

	// latitude
	msg.data[0] = (latitude >> 0) & 0xFF;
	msg.data[1] = (latitude >> 8) & 0xFF;
	msg.data[2] = (latitude >> 16) & 0xFF;
	msg.data[3] = (latitude >> 24) & 0xFF;

	// longitude
	msg.data[4] = (longitude >> 0) & 0xFF;
	msg.data[5] = (longitude >> 8) & 0xFF;
	msg.data[6] = (longitude >> 16) & 0xFF;
	msg.data[7] = (longitude >> 24) & 0xFF;

	return msg;
}

void Parse_SBG_ECAN_MSG_GPS1_POS(uint8_t* data, uint32_t* id, float* latitude, float* longitude) {
	*id = SBG_ECAN_MSG_GPS1_POS_ID;
	*latitude = (int32_t)((data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
	*longitude = (int32_t)((data[4] << 0) | (data[5] << 8) | (data[6] << 16) | (data[7] << 24));

	*latitude *= 0.0000001;
	*longitude *= 0.0000001;
}
