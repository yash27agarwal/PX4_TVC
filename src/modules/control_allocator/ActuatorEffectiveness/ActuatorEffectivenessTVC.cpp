#include "ActuatorEffectivenessTVC.hpp"
#include <mathlib/mathlib.h> // For math::constrain and other math functions

// using namespace matrix;

ActuatorEffectivenessTVC::ActuatorEffectivenessTVC(ModuleParams *parent)
    : ModuleParams(parent)
{
	// Find motor parameter handles
	for (int i = 0; i < NUM_MOTORS; ++i) {
		char buffer[32]; // Buffer for param name
		snprintf(buffer, sizeof(buffer), "CA_TVC_M%d_CT", i);
		_param_handles.motor_ct[i] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_TVC_M%d_KMR", i);
		_param_handles.motor_kmr[i] = param_find(buffer);
	}

	// Find servo parameter handles
	for (int i = 0; i < NUM_SERVOS; ++i) {
		char buffer[32];
		snprintf(buffer, sizeof(buffer), "CA_TVC_S%d_GAIN", i);
		_param_handles.servo_gain[i] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_TVC_S%d_MAX", i);
		_param_handles.servo_max_limit[i] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_TVC_S%d_MIN", i);
		_param_handles.servo_min_limit[i] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_TVC_S%d_GEO_A", i);
		_param_handles.servo_geo_a[i] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_TVC_S%d_GEO_B", i);
		_param_handles.servo_geo_b[i] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_TVC_S%d_GEO_C", i);
		_param_handles.servo_geo_c[i] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_TVC_S%d_GEO_D", i);
		_param_handles.servo_geo_d[i] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_TVC_S%d_GEO_E", i);
		_param_handles.servo_geo_e[i] = param_find(buffer);
	}

	_param_handles.gimbal_com_distance = param_find("CA_TVC_COM_DIS");

	updateParams(); // Perform initial parameter load
}

void ActuatorEffectivenessTVC::updateParams()
{
	ModuleParams::updateParams(); // Call base class method

	// Load motor parameters
	for (int i = 0; i < NUM_MOTORS; ++i) {
		if (_param_handles.motor_ct[i] != PARAM_INVALID) {
		param_get(_param_handles.motor_ct[i], &_geometry.motors[i].ct);
		}

		if (_param_handles.motor_kmr[i] != PARAM_INVALID) {
		param_get(_param_handles.motor_kmr[i], &_geometry.motors[i].kmr);
		}
	}

	// Load servo parameters
	for (int i = 0; i < NUM_SERVOS; ++i) {
		if (_param_handles.servo_gain[i] != PARAM_INVALID) {
		param_get(_param_handles.servo_gain[i], &_geometry.servos[i].gain);
		}

		if (_param_handles.servo_max_limit[i] != PARAM_INVALID) {
		param_get(_param_handles.servo_max_limit[i], &_geometry.servos[i].max_limit);
		}

		if (_param_handles.servo_min_limit[i] != PARAM_INVALID) {
		param_get(_param_handles.servo_min_limit[i], &_geometry.servos[i].min_limit);
		}

		if (_param_handles.servo_geo_a[i] != PARAM_INVALID) {
		param_get(_param_handles.servo_geo_a[i], &_geometry.servos[i].geo_a);
		}

		if (_param_handles.servo_geo_b[i] != PARAM_INVALID) {
		param_get(_param_handles.servo_geo_b[i], &_geometry.servos[i].geo_b);
		}

		if (_param_handles.servo_geo_c[i] != PARAM_INVALID) {
		param_get(_param_handles.servo_geo_c[i], &_geometry.servos[i].geo_c);
		}

		if (_param_handles.servo_geo_d[i] != PARAM_INVALID) {
		param_get(_param_handles.servo_geo_d[i], &_geometry.servos[i].geo_d);
		}

		if (_param_handles.servo_geo_e[i] != PARAM_INVALID) {
		param_get(_param_handles.servo_geo_e[i], &_geometry.servos[i].geo_e);
		}


	}

	if (_param_handles.gimbal_com_distance != PARAM_INVALID) {
		param_get(_param_handles.gimbal_com_distance, &_geometry.gimbal_com_distance);
	}

	_geometry_updated = true; // Signal that geometry might have changed
}

bool ActuatorEffectivenessTVC::getEffectivenessMatrix(Configuration &configuration,
                                EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE && !_geometry_updated) {
		return false;
	}

	// Motors
	// Motor 0 (e.g., Top)
	// The effectiveness matrix entries are simplified here.
	// Your updateSetpoint method will contain the detailed non-linear logic.
	// Thrust is along -Z. Roll moment depends on KMR.
	_motor1_idx = configuration.addActuator(ActuatorType::MOTORS, matrix::Vector3f{}, matrix::Vector3f{});   // Thrust Z
	// Motor 1 (e.g., Bottom)
	_motor2_idx = configuration.addActuator(ActuatorType::MOTORS,
				matrix::Vector3f{}, matrix::Vector3f{});   // Thrust Z

	// Servos
	// Assuming servo 0 is for roll and servo 1 is for pitch.
	// The matrix entries here are nominal, as updateSetpoint will handle the precise calculations.
	_servo_roll_idx = configuration.addActuator(ActuatorType::SERVOS,
				matrix::Vector3f{}, // Nominal pitch effectiveness
				matrix::Vector3f{});
	_servo_pitch_idx   = configuration.addActuator(ActuatorType::SERVOS,
				matrix::Vector3f{},
				matrix::Vector3f{});

	// Note: If you have trim parameters for servos, you would load them in updateParams()
	// and apply them here to configuration.trim[configuration.selected_matrix](servo_idx)

	_geometry_updated = false;
	// _params_updated is handled by the base class and will trigger updateParams()
	return true;
}

void ActuatorEffectivenessTVC::calculateGimbalAngles(const ControlSetpoint &control_sp,
						    float &gimbal_roll_cmd, float &gimbal_pitch_cmd)
{
	float roll{0.0f};
	float pitch{0.0f};

	float thrust_norm = sqrtf(
			control_sp.t_x * control_sp.t_x +
			control_sp.t_y * control_sp.t_y +
			control_sp.t_z * control_sp.t_z);

	roll = -asinf(control_sp.t_y / sqrt(
		thrust_norm * thrust_norm - control_sp.t_x * control_sp.t_x));
	roll = math::constrain(roll, _geometry.servos[0].min_limit, _geometry.servos[0].max_limit);

	pitch = asinf(control_sp.t_x / thrust_norm);
	pitch  = math::constrain(pitch, _geometry.servos[1].min_limit, _geometry.servos[1].max_limit);

	gimbal_roll_cmd = roll;
	gimbal_pitch_cmd = pitch;
}

void ActuatorEffectivenessTVC::calculateMotorSpeeds(const ControlSetpoint &control_sp,
						    float &motor1_pwm, float &motor2_pwm)
{
	// Calculate motor speeds based on the control setpoint
	// This is a simplified example; you would replace this with your cubic equation logic.
	float thrust_mag_in_gimbal_frame = sqrtf(control_sp.t_x * control_sp.t_x +
				control_sp.t_y * control_sp.t_y +
				control_sp.t_z * control_sp.t_z);

	float motor1_speed_sq = (thrust_mag_in_gimbal_frame / 2) / _geometry.motors[0].ct; // Example calculation
	float motor2_speed_sq = (thrust_mag_in_gimbal_frame / 2) / _geometry.motors[1].ct; // Example calculation

	// Check for negative motor speed squared values
	if (motor1_speed_sq < 0.0f || motor2_speed_sq < 0.0f) {
		motor1_pwm = NAN;
		motor2_pwm = NAN;
		PX4_WARN("Negative motor speed squared detected: M1: %.2f, M2: %.2f. Replacing with NaN.",
				(double)motor1_speed_sq, (double)motor2_speed_sq);
		return;
	}

	float motor1_speed = sqrtf(motor1_speed_sq);
	float motor2_speed = sqrtf(motor2_speed_sq);

	// Convert to PWM signal range (1000 to 2000)
	float pwm_1 = p1 * motor1_speed * motor1_speed  +
		      p2 * motor1_speed + p3;

	float pwm_2 = p1 * motor2_speed * motor2_speed  +
			  p2 * motor2_speed + p3;

	// Constrain PWM signals to be between 1100 and 1900
	pwm_1 = math::constrain(pwm_1, 1100.0f, 1900.0f);
	pwm_2 = math::constrain(pwm_2, 1100.0f, 1900.0f);

	// Normalize to 0 to 1 range
	motor1_speed = (pwm_1 - 1000.0f) / 1000.0f; // Assuming 1000 is the min PWM value and 2000 is the max PWM value
	motor2_speed = (pwm_2 - 1000.0f) / 1000.0f; // Assuming 1000 is the min PWM value and 2000 is the max PWM value

	// Ensure speeds are within 0 to 1 range
	motor1_pwm = math::constrain(motor1_speed, 0.0f, 1.0f);
	motor2_pwm = math::constrain(motor2_speed, 0.0f, 1.0f);
}


void ActuatorEffectivenessTVC::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
                ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
                const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{

	// Follows NED frame convention

	// _tvc_control_sp.tau_x = control_sp(0);
	// _tvc_control_sp.tau_y = control_sp(1);
	// _tvc_control_sp.tau_z = control_sp(2);
	// _tvc_control_sp.t_x = control_sp(3);
	// _tvc_control_sp.t_y = - _tvc_control_sp.tau_z / _geometry.gimbal_com_distance; // Invert Y thrust for correct direction
	// _tvc_control_sp.t_z = _tvc_control_sp.tau_y / _geometry.gimbal_com_distance; // Invert Z thrust for correct direction

	_tvc_control_sp.tau_x = control_sp(0);
	_tvc_control_sp.tau_y = control_sp(1);
	_tvc_control_sp.tau_z = control_sp(2);
	_tvc_control_sp.t_x = control_sp(3);
	_tvc_control_sp.t_y = control_sp(4);
	_tvc_control_sp.t_z = control_sp(5);

	// // Test case
	// _tvc_control_sp.tau_x = 0.0f;
	// _tvc_control_sp.tau_y = 0.0f;
	// _tvc_control_sp.tau_z = 0.0f;
	// _tvc_control_sp.t_x = 0.6f * 9.81f * 1.1f;
	// _tvc_control_sp.t_y = 0.0f;
	// _tvc_control_sp.t_z = 0.0f;

	calculateGimbalAngles(_tvc_control_sp, gimbal_roll_target_angle, gimbal_pitch_target_angle);
	actuator_sp(_servo_roll_idx)   = gimbal_roll_target_angle;
	actuator_sp(_servo_pitch_idx) = gimbal_pitch_target_angle;

	// 3. Calculate propeller speeds
	calculateMotorSpeeds(_tvc_control_sp, motor1_target_pwm, motor2_target_pwm);

	// 4. Apply motor speeds (already normalized 0 to 1 by calculateMotorSpeeds)
	// The actuator_min/max for motors are typically 0 to 1 (or -1 to 1 if reversible).
	actuator_sp(_motor1_idx) = motor1_target_pwm;
	actuator_sp(_motor2_idx) = motor2_target_pwm;

}
