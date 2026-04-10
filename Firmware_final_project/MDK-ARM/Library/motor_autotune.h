#ifndef MOTOR_AUTOTUNE_H
#define MOTOR_AUTOTUNE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MOTOR_AUTOTUNE_CAPTURE_SAMPLES 600u

typedef enum
{
	MOTOR_AUTOTUNE_STATE_IDLE = 0u,
	MOTOR_AUTOTUNE_STATE_RS = 1u,
	MOTOR_AUTOTUNE_STATE_LS = 2u,
	MOTOR_AUTOTUNE_STATE_FLUX = 3u,
	MOTOR_AUTOTUNE_STATE_DONE = 4u,
	MOTOR_AUTOTUNE_STATE_ERROR = 5u
} MotorAutoTuneState_e;

typedef enum
{
	MOTOR_AUTOTUNE_ERROR_NONE = 0u,
	MOTOR_AUTOTUNE_ERROR_OVERCURRENT = 1u,
	MOTOR_AUTOTUNE_ERROR_STALL = 2u,
	MOTOR_AUTOTUNE_ERROR_INVALID_CONFIG = 3u,
	MOTOR_AUTOTUNE_ERROR_SIGNAL = 4u
} MotorAutoTuneError_e;

typedef enum
{
	MOTOR_AUTOTUNE_OUTPUT_DISABLED = 0u,
	MOTOR_AUTOTUNE_OUTPUT_CURRENT_LOOP = 1u,
	MOTOR_AUTOTUNE_OUTPUT_DIRECT_D_VOLTAGE = 2u,
	MOTOR_AUTOTUNE_OUTPUT_OPEN_LOOP_VF = 3u
} MotorAutoTuneOutputMode_e;

typedef enum
{
	MOTOR_AUTOTUNE_CHART_NONE = 0u,
	MOTOR_AUTOTUNE_CHART_RS = 1u,
	MOTOR_AUTOTUNE_CHART_LS = 2u
} MotorAutoTuneChartStage_e;

typedef struct
{
	float rs_current_low_a;
	float rs_current_high_a;
	float ls_step_voltage_v;
	float flux_frequency_hz;
	float flux_voltage_v;
	float current_bandwidth_hz;
	float speed_bandwidth_hz;
	float position_bandwidth_hz;
} MotorAutoTuneConfig_t;

typedef struct
{
	float id_current_a;
	float iq_current_a;
	float vd_voltage_v;
	float vq_voltage_v;
	float phase_voltage_u_v;
	float phase_voltage_v_v;
	float phase_voltage_w_v;
	float phase_u_a;
	float phase_v_a;
	float phase_w_a;
	float bus_voltage_v;
	float electrical_theta_rad;
	float mechanical_position_counts;
	float mechanical_speed_rpm;
	float encoder_resolution_counts;
	float rated_current_a;
	float rotor_inertia;
} MotorAutoTuneInputs_t;

typedef struct
{
	MotorAutoTuneOutputMode_e mode;
	float id_ref_a;
	float iq_ref_a;
	float vd_voltage_v;
	float vq_voltage_v;
	float voltage_limit_v;
	float vf_frequency_hz;
	float vf_voltage_v;
	uint8_t isolate_q_axis;
} MotorAutoTuneOutputs_t;

typedef struct
{
	MotorAutoTuneState_e state;
	MotorAutoTuneError_e error;
	MotorAutoTuneConfig_t config;

	uint8_t enabled;
	uint8_t progress_percent;
	uint8_t tuning_data_ready;
	uint8_t chart_stage;

	uint8_t substep;
	uint8_t chart_decimation;
	uint8_t chart_transfer_active;
	uint8_t reserved;

	uint32_t counter;
	uint32_t phase_counter;
	uint32_t oc_event_count;

	float loop_frequency_hz;
	float dt_s;
	float overcurrent_threshold_a;

	float measured_Rs;
	float measured_Ls;
	float measured_Ke;
	float measured_Flux;
	float measured_PolePairs;

	float tuned_current_kp;
	float tuned_current_ki;
	float tuned_speed_kp;
	float tuned_speed_ki;
	float tuned_position_kp;

	float rs_low_v_sum;
	float rs_low_i_sum;
	float rs_high_v_sum;
	float rs_high_i_sum;
	uint32_t rs_low_samples;
	uint32_t rs_high_samples;

	float ls_initial_current_a;
	float ls_final_current_a;
	float ls_current_slope_a_per_s;
	float ls_step_voltage_applied_v;

	float flux_voltage_sq_sum;
	float flux_speed_rpm_sum;
	uint32_t flux_samples;

	float mechanical_position_start_counts;
	float commanded_electrical_turns;
	float estimated_mechanical_turns;

	uint16_t chart_length;
	uint16_t chart_send_index;
	float chart_sample_period_s;
	float chart_primary[MOTOR_AUTOTUNE_CAPTURE_SAMPLES];
	float chart_secondary[MOTOR_AUTOTUNE_CAPTURE_SAMPLES];
	float chart_tertiary[MOTOR_AUTOTUNE_CAPTURE_SAMPLES];
} MotorAutoTune_t;

void MotorAutoTune_SetDefaultConfig(MotorAutoTuneConfig_t *config);
void MotorAutoTune_Reset(MotorAutoTune_t *handle);
uint8_t MotorAutoTune_Start(
	MotorAutoTune_t *handle,
	const MotorAutoTuneConfig_t *config,
	float loop_frequency_hz,
	float overcurrent_threshold_a);
void MotorAutoTune_Stop(MotorAutoTune_t *handle);
void MotorAutoTune_ClearDataReady(MotorAutoTune_t *handle);
void MotorAutoTune_SetError(MotorAutoTune_t *handle, MotorAutoTuneError_e error);
void MotorAutoTune_Process(
	MotorAutoTune_t *handle,
	const MotorAutoTuneInputs_t *inputs,
	MotorAutoTuneOutputs_t *outputs);
uint8_t MotorAutoTune_ApplyEstimatedParameters(
	const MotorAutoTune_t *handle,
	float *driver_parameters,
	uint32_t driver_parameter_count,
	float *motor_parameters,
	uint32_t motor_parameter_count);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_AUTOTUNE_H */
