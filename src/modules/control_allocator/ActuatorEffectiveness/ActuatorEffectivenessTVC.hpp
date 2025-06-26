#pragma once

#include "ActuatorEffectiveness.hpp"
#include <px4_platform_common/module_params.h>
#include <matrix/matrix/math.hpp> // For matrix types

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
// #include <uORB/topics/vehicle_torque_setpoint.h>
// #include <uORB/topics/vehicle_thrust_setpoint.h>

#include <px4_platform_common/log.h>

class ActuatorEffectivenessTVC : public ModuleParams, public ActuatorEffectiveness
{
public:
	struct ControlSetpoint {
		float t_x;
		float t_y;
		float t_z;
		float tau_x;
		float tau_y;
		float tau_z;
	};

	ActuatorEffectivenessTVC(ModuleParams *parent);
	virtual ~ActuatorEffectivenessTVC() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
			const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) override;

	const char *name() const override { return "TVC"; } // Changed from "GimbalCoaxial" to "TVC" to match params

	// Helper functions
	void calculateGimbalAngles(const ControlSetpoint &control_sp,
						    float &gimbal_pitch_cmd, float &gimbal_yaw_cmd);
	void calculateMotorSpeeds(const ControlSetpoint &control_sp,
						    float &motor1_pwm, float &motor2_pwm);
private:
	void updateParams() override;

	static constexpr int NUM_MOTORS = 2;
	static constexpr int NUM_SERVOS = 2; // Assuming 2 gimbal servos

	struct MotorGeometry {
		float ct;  // Thrust coefficient
		float kmr; // Roll moment coefficient
	};

	struct ServoGeometry {
		float gain;
		float max_limit;
		float min_limit;
		float geo_a;
		float geo_b;
		float geo_c;
		float geo_d;
		float geo_e;
	};

	struct Geometry {
		MotorGeometry motors[NUM_MOTORS];
		ServoGeometry servos[NUM_SERVOS];
		float gimbal_com_distance; // Distance from gimbal center to motors
	};

	struct ParamHandles {
		param_t motor_ct[NUM_MOTORS];
		param_t motor_kmr[NUM_MOTORS];
		param_t servo_gain[NUM_SERVOS];
		param_t servo_max_limit[NUM_SERVOS];
		param_t servo_min_limit[NUM_SERVOS];
		param_t servo_geo_a[NUM_SERVOS];
		param_t servo_geo_b[NUM_SERVOS];
		param_t servo_geo_c[NUM_SERVOS];
		param_t servo_geo_d[NUM_SERVOS];
		param_t servo_geo_e[NUM_SERVOS];
		param_t gimbal_com_distance; // Parameter for gimbal center of mass distance
	};



	ParamHandles _param_handles{};
	Geometry _geometry{};
	ControlSetpoint _tvc_control_sp{}; // Current control setpoint

	float gimbal_pitch_target_angle{};
	float gimbal_yaw_target_angle{};

	float motor1_target_pwm{}; // Output of your cubic equation (0 to 1)
	float motor2_target_pwm{}; // Output of your cubic equation (0 to 1)

	// Actuator indices
	int _motor1_idx{};
	int _motor2_idx{};
	int _servo_pitch_idx{}; // Assuming servo 0 is pitch
	int _servo_yaw_idx{};   // Assuming servo 1 is yaw

	float p1 {3.1352e-4f};
	float p2 {0.1352f};
	float p3 {996.9672f};

	bool _geometry_updated{true}; // Flag to indicate if geometry needs recalculation
	// _params_updated is handled by ModuleParams base class

	hrt_abstime _last_run{0};

	// uORB::Subscription _vehicle_torque_setpoint1_sub{ORB_ID(vehicle_torque_setpoint), 1};  /**< vehicle torque setpoint subscription (2. instance) */
	// uORB::Subscription _vehicle_thrust_setpoint1_sub{ORB_ID(vehicle_thrust_setpoint), 1};	 /**< vehicle thrust setpoint subscription (2. instance) */
};
