#include "motor_autotune.h"

#include <math.h>
#include <string.h>

#include "define.h"

#define MOTOR_AUTOTUNE_MIN_LOOP_HZ              100.0f
#define MOTOR_AUTOTUNE_MIN_OC_THRESHOLD_A       0.1f
#define MOTOR_AUTOTUNE_MIN_RS_CURRENT_A         0.05f
#define MOTOR_AUTOTUNE_MIN_STEP_VOLTAGE_V       0.25f
#define MOTOR_AUTOTUNE_MIN_LS_FREQUENCY_HZ      10.0f
#define MOTOR_AUTOTUNE_MIN_FLUX_FREQUENCY_HZ    1.0f
#define MOTOR_AUTOTUNE_MIN_FLUX_VOLTAGE_V       1.0f

#define MOTOR_AUTOTUNE_RS_SETTLE_S              0.20f
#define MOTOR_AUTOTUNE_RS_AVERAGE_S             0.12f
#define MOTOR_AUTOTUNE_RS_PROGRESS              30u

#define MOTOR_AUTOTUNE_LS_PREPARE_S             0.10f
#define MOTOR_AUTOTUNE_LS_SETTLE_CYCLES         5.0f
#define MOTOR_AUTOTUNE_LS_MEASURE_CYCLES        6.0f
#define MOTOR_AUTOTUNE_LS_MIN_CURRENT_PEAK_A    0.03f
#define MOTOR_AUTOTUNE_LS_PROGRESS              60u

#define MOTOR_AUTOTUNE_PP_TEST_TIMEOUT_S        5.0f
#define MOTOR_AUTOTUNE_PP_SETTLE_S              0.20f
#define MOTOR_AUTOTUNE_PP_TARGET_TURNS          1.0f
#define MOTOR_AUTOTUNE_PP_MIN_MOVEMENT_TURNS    0.15f
#define MOTOR_AUTOTUNE_PP_PROGRESS              78u

#define MOTOR_AUTOTUNE_FLUX_SETTLE_S            0.60f
#define MOTOR_AUTOTUNE_FLUX_AVERAGE_S           0.35f
#define MOTOR_AUTOTUNE_FLUX_MIN_SPEED_RPM       20.0f
#define MOTOR_AUTOTUNE_FLUX_PROGRESS            75u

#define MOTOR_AUTOTUNE_MECH_MIN_FREQUENCY_HZ    0.2f
#define MOTOR_AUTOTUNE_MECH_MAX_FREQUENCY_HZ    10.0f
#define MOTOR_AUTOTUNE_MECH_MIN_SPEED_LPF_HZ    2.0f
#define MOTOR_AUTOTUNE_MECH_MIN_SPEED_RPM       20.0f
#define MOTOR_AUTOTUNE_MECH_SETTLE_CYCLES       2.0f
#define MOTOR_AUTOTUNE_MECH_SOFTSTART_CYCLES    0.35f
#define MOTOR_AUTOTUNE_MECH_J_WINDOW_TARGET     4u
#define MOTOR_AUTOTUNE_MECH_B_WINDOW_TARGET     6u
#define MOTOR_AUTOTUNE_MECH_TIMEOUT_S           12.0f
#define MOTOR_AUTOTUNE_MECH_PROGRESS_J_BASE     82u
#define MOTOR_AUTOTUNE_MECH_PROGRESS_B_BASE     92u
#define MOTOR_AUTOTUNE_MECH_PROGRESS_B_SPAN     6u
#define MOTOR_AUTOTUNE_MECH_J_ZERO_RELEASE_RPM  1.0f
#define MOTOR_AUTOTUNE_MECH_J_ZERO_SWITCH_RPM   4.0f
#define MOTOR_AUTOTUNE_MAX_J_KG_M2              1.0f
#define MOTOR_AUTOTUNE_MAX_B_NM_S_PER_RAD       10.0f

#define MOTOR_AUTOTUNE_DEFAULT_RS_LOW_A         0.20f
#define MOTOR_AUTOTUNE_DEFAULT_RS_HIGH_A        0.45f
#define MOTOR_AUTOTUNE_DEFAULT_LS_STEP_V        2.0f
#define MOTOR_AUTOTUNE_DEFAULT_LS_FREQ_HZ       200.0f
#define MOTOR_AUTOTUNE_DEFAULT_FLUX_FREQ_HZ     15.0f
#define MOTOR_AUTOTUNE_DEFAULT_FLUX_VOLT_V      6.0f
#define MOTOR_AUTOTUNE_DEFAULT_CURRENT_BW_HZ    200.0f
#define MOTOR_AUTOTUNE_DEFAULT_SPEED_BW_HZ      20.0f
#define MOTOR_AUTOTUNE_DEFAULT_POSITION_BW_HZ   5.0f
#define MOTOR_AUTOTUNE_DEFAULT_MECH_IQ_A        0.0f
#define MOTOR_AUTOTUNE_DEFAULT_MECH_FREQ_HZ     0.8f
#define MOTOR_AUTOTUNE_DEFAULT_MECH_SPEED_LPF_HZ 10.0f

#define MOTOR_AUTOTUNE_RS_WAIT_HOST_SUBSTEP     4u
#define MOTOR_AUTOTUNE_LS_WAIT_HOST_SUBSTEP     3u

static float MotorAutoTune_CurrentLoopVoltageLimit(const MotorAutoTuneInputs_t *inputs);
static void MotorAutoTune_AdvanceToStage(MotorAutoTune_t *handle, MotorAutoTuneState_e next_state);
static void MotorAutoTune_Finish(MotorAutoTune_t *handle, const MotorAutoTuneInputs_t *inputs);
void MotorAutoTune_SetError(MotorAutoTune_t *handle, MotorAutoTuneError_e error);

static float MotorAutoTune_Abs(float value)
{
	return (value >= 0.0f) ? value : -value;
}

static float MotorAutoTune_Clamp(float value, float lower, float upper)
{
	if (value < lower)
	{
		return lower;
	}
	if (value > upper)
	{
		return upper;
	}
	return value;
}

static uint32_t MotorAutoTune_SecondsToTicks(const MotorAutoTune_t *handle, float seconds)
{
	float ticks_f;

	if ((handle == 0) || (handle->loop_frequency_hz <= 1.0f) || (seconds <= 0.0f))
	{
		return 1u;
	}

	ticks_f = seconds * handle->loop_frequency_hz;
	if (ticks_f < 1.0f)
	{
		ticks_f = 1.0f;
	}
	return (uint32_t)(ticks_f + 0.5f);
}

static float MotorAutoTune_LpfAlpha(float cutoff_hz, float dt_s)
{
	float omega_dt;

	if ((cutoff_hz <= 0.0f) || (dt_s <= 0.0f))
	{
		return 1.0f;
	}

	omega_dt = 2.0f * PI * cutoff_hz * dt_s;
	if (omega_dt <= 0.0f)
	{
		return 1.0f;
	}
	return omega_dt / (1.0f + omega_dt);
}

static void MotorAutoTune_UpdateCurrentGains(MotorAutoTune_t *handle)
{
	float current_bw_rad_s;

	if (handle == 0)
	{
		return;
	}
	if ((handle->measured_Rs <= 0.0f) || (handle->measured_Ls <= 0.0f))
	{
		return;
	}

	current_bw_rad_s = 2.0f * PI * MotorAutoTune_Clamp(
		handle->config.current_bandwidth_hz,
		1.0f,
		2000.0f);
	handle->tuned_current_kp = handle->measured_Ls * current_bw_rad_s;
	handle->tuned_current_ki = handle->measured_Rs * current_bw_rad_s;
}

static float MotorAutoTune_GetMechanicalFrequencyHz(const MotorAutoTune_t *handle)
{
	float frequency_hz = MOTOR_AUTOTUNE_DEFAULT_MECH_FREQ_HZ;

	if (handle != 0)
	{
		frequency_hz = handle->config.mechanical_frequency_hz;
	}
	return MotorAutoTune_Clamp(
		frequency_hz,
		MOTOR_AUTOTUNE_MECH_MIN_FREQUENCY_HZ,
		MOTOR_AUTOTUNE_MECH_MAX_FREQUENCY_HZ);
}

static float MotorAutoTune_GetMechanicalSpeedLpfHz(const MotorAutoTune_t *handle)
{
	float cutoff_hz = MOTOR_AUTOTUNE_DEFAULT_MECH_SPEED_LPF_HZ;

	if (handle != 0)
	{
		cutoff_hz = handle->config.mechanical_speed_lpf_hz;
	}
	return MotorAutoTune_Clamp(
		cutoff_hz,
		MOTOR_AUTOTUNE_MECH_MIN_SPEED_LPF_HZ,
		120.0f);
}

static float MotorAutoTune_DefaultMechanicalIqAmplitude(
	const MotorAutoTune_t *handle,
	const MotorAutoTuneInputs_t *inputs)
{
	float rated_current_a = MOTOR_AUTOTUNE_DEFAULT_RS_HIGH_A;
	float current_limit_a;
	float iq_amplitude_a;

	if ((inputs != 0) && (inputs->rated_current_a > 0.0f))
	{
		rated_current_a = inputs->rated_current_a;
	}
	current_limit_a = rated_current_a * 0.30f;
	if ((handle != 0) && (handle->overcurrent_threshold_a > 0.0f))
	{
		float fault_limited_a = handle->overcurrent_threshold_a * 0.22f;
		if (fault_limited_a < current_limit_a)
		{
			current_limit_a = fault_limited_a;
		}
	}
	if (current_limit_a < 0.08f)
	{
		current_limit_a = 0.08f;
	}

	iq_amplitude_a = rated_current_a * 0.12f;
	if (iq_amplitude_a < 0.08f)
	{
		iq_amplitude_a = 0.08f;
	}
	if (iq_amplitude_a > current_limit_a)
	{
		iq_amplitude_a = current_limit_a;
	}
	return iq_amplitude_a;
}

static void MotorAutoTune_ResetMechanicalRuntime(
	MotorAutoTune_t *handle,
	const MotorAutoTuneInputs_t *inputs,
	uint8_t preserve_torque_sign)
{
	float initial_speed_rad_s = 0.0f;

	if (handle == 0)
	{
		return;
	}
	if (inputs != 0)
	{
		initial_speed_rad_s = inputs->mechanical_speed_rpm * ((2.0f * PI) / 60.0f);
	}

	handle->mechanical_speed_filtered_rad_s = initial_speed_rad_s;
	handle->mechanical_speed_prev_rad_s = initial_speed_rad_s;
	handle->mechanical_accel_filtered_rad_s2 = 0.0f;
	handle->mechanical_peak_speed_rpm = MotorAutoTune_Abs(
		(initial_speed_rad_s * 60.0f) / (2.0f * PI));
	handle->mechanical_window_numerator = 0.0f;
	handle->mechanical_window_denominator = 0.0f;
	handle->mechanical_window_estimate_sum = 0.0f;
	handle->mechanical_regression_sw2 = 0.0f;
	handle->mechanical_regression_sw = 0.0f;
	handle->mechanical_regression_s11 = 0.0f;
	handle->mechanical_regression_swy = 0.0f;
	handle->mechanical_regression_sy = 0.0f;
	handle->mechanical_zero_cross_count = 0u;
	handle->mechanical_window_count = 0u;
	handle->mechanical_window_active = 0u;
	handle->mechanical_zero_cross_armed = 0u;
	handle->mechanical_prev_speed_sign = 0;
	handle->counter = 0u;
	handle->phase_counter = 0u;
	handle->mechanical_timeout_ticks = MotorAutoTune_SecondsToTicks(
		handle,
		MOTOR_AUTOTUNE_MECH_TIMEOUT_S);
	if (preserve_torque_sign == 0u)
	{
		handle->mechanical_torque_sign = 0.0f;
	}
}

static void MotorAutoTune_UpdateMechanicalObservables(
	MotorAutoTune_t *handle,
	const MotorAutoTuneInputs_t *inputs)
{
	float speed_raw_rad_s;
	float speed_alpha;
	float accel_alpha;
	float accel_raw_rad_s2;
	float accel_lpf_hz;
	float speed_rpm_abs;

	if ((handle == 0) || (inputs == 0))
	{
		return;
	}

	speed_raw_rad_s = inputs->mechanical_speed_rpm * ((2.0f * PI) / 60.0f);
	speed_alpha = MotorAutoTune_LpfAlpha(
		MotorAutoTune_GetMechanicalSpeedLpfHz(handle),
		handle->dt_s);
	handle->mechanical_speed_filtered_rad_s += speed_alpha *
		(speed_raw_rad_s - handle->mechanical_speed_filtered_rad_s);
	accel_raw_rad_s2 = (
		handle->mechanical_speed_filtered_rad_s -
		handle->mechanical_speed_prev_rad_s) / handle->dt_s;
	accel_lpf_hz = MotorAutoTune_GetMechanicalSpeedLpfHz(handle) * 1.5f;
	{
		float excitation_lpf_min_hz = MotorAutoTune_GetMechanicalFrequencyHz(handle) * 6.0f;
		if (accel_lpf_hz < excitation_lpf_min_hz)
		{
			accel_lpf_hz = excitation_lpf_min_hz;
		}
	}
	accel_alpha = MotorAutoTune_LpfAlpha(accel_lpf_hz, handle->dt_s);
	handle->mechanical_accel_filtered_rad_s2 += accel_alpha *
		(accel_raw_rad_s2 - handle->mechanical_accel_filtered_rad_s2);
	handle->mechanical_speed_prev_rad_s = handle->mechanical_speed_filtered_rad_s;

	speed_rpm_abs = MotorAutoTune_Abs(
		(handle->mechanical_speed_filtered_rad_s * 60.0f) / (2.0f * PI));
	if (speed_rpm_abs > handle->mechanical_peak_speed_rpm)
	{
		handle->mechanical_peak_speed_rpm = speed_rpm_abs;
	}
}

static void MotorAutoTune_CommandMechanicalSine(
	MotorAutoTune_t *handle,
	const MotorAutoTuneInputs_t *inputs,
	MotorAutoTuneOutputs_t *outputs)
{
	float elapsed_s;
	float electrical_omega;
	float softstart_scale = 1.0f;
	uint32_t softstart_ticks;

	if ((handle == 0) || (outputs == 0))
	{
		return;
	}

	elapsed_s = ((float)handle->counter) * handle->dt_s;
	electrical_omega = 2.0f * PI * MotorAutoTune_GetMechanicalFrequencyHz(handle);
	softstart_ticks = MotorAutoTune_SecondsToTicks(
		handle,
		MOTOR_AUTOTUNE_MECH_SOFTSTART_CYCLES /
			MotorAutoTune_GetMechanicalFrequencyHz(handle));
	if ((softstart_ticks > 0u) && (handle->counter < softstart_ticks))
	{
		softstart_scale = (float)handle->counter / (float)softstart_ticks;
	}

	outputs->mode = MOTOR_AUTOTUNE_OUTPUT_CURRENT_LOOP;
	outputs->id_ref_a = 0.0f;
	outputs->iq_ref_a =
		(handle->mechanical_iq_amplitude_a * softstart_scale) *
		sinf(electrical_omega * elapsed_s);
	outputs->voltage_limit_v = MotorAutoTune_CurrentLoopVoltageLimit(inputs);
	outputs->isolate_q_axis = 0u;
}

static void MotorAutoTune_ResetOutputs(MotorAutoTuneOutputs_t *outputs)
{
	if (outputs == 0)
	{
		return;
	}

	memset(outputs, 0, sizeof(*outputs));
	outputs->mode = MOTOR_AUTOTUNE_OUTPUT_DISABLED;
}

static float MotorAutoTune_CurrentLoopVoltageLimit(const MotorAutoTuneInputs_t *inputs)
{
	float limit = 4.0f;

	if (inputs == 0)
	{
		return limit;
	}

	if (inputs->bus_voltage_v > 1.0f)
	{
		limit = inputs->bus_voltage_v * 0.30f;
	}
	if (limit < 2.0f)
	{
		limit = 2.0f;
	}
	if (limit > 12.0f)
	{
		limit = 12.0f;
	}
	return limit;
}

static float MotorAutoTune_OpenLoopPolePairFrequency(const MotorAutoTuneConfig_t *config)
{
	float frequency = MOTOR_AUTOTUNE_DEFAULT_FLUX_FREQ_HZ * 0.25f;

	if (config != 0)
	{
		frequency = config->flux_frequency_hz * 0.25f;
	}

	if (frequency < 3.0f)
	{
		frequency = 3.0f;
	}
	if (frequency > 8.0f)
	{
		frequency = 8.0f;
	}
	return frequency;
}

static float MotorAutoTune_OpenLoopPolePairVoltage(const MotorAutoTuneConfig_t *config)
{
	float voltage = MOTOR_AUTOTUNE_DEFAULT_FLUX_VOLT_V * 0.7f;

	if (config != 0)
	{
		voltage = config->flux_voltage_v * 0.7f;
	}

	if (voltage < 3.0f)
	{
		voltage = 3.0f;
	}
	if (voltage > 10.0f)
	{
		voltage = 10.0f;
	}
	return voltage;
}

static void MotorAutoTune_PrepareChart(
	MotorAutoTune_t *handle,
	MotorAutoTuneChartStage_e stage,
	uint32_t expected_ticks)
{
	uint32_t decimation = 1u;

	if (handle == 0)
	{
		return;
	}

	memset(handle->chart_primary, 0, sizeof(handle->chart_primary));
	memset(handle->chart_secondary, 0, sizeof(handle->chart_secondary));
	memset(handle->chart_tertiary, 0, sizeof(handle->chart_tertiary));
	handle->chart_length = 0u;
	handle->chart_send_index = 0u;
	handle->chart_stage = (uint8_t)stage;
	handle->chart_transfer_active = 0u;
	handle->tuning_data_ready = 0u;

	if (expected_ticks > MOTOR_AUTOTUNE_CAPTURE_SAMPLES)
	{
		decimation = (expected_ticks + MOTOR_AUTOTUNE_CAPTURE_SAMPLES - 1u) /
			MOTOR_AUTOTUNE_CAPTURE_SAMPLES;
	}
	if (decimation > 255u)
	{
		decimation = 255u;
	}

	handle->chart_decimation = (uint8_t)decimation;
	handle->chart_sample_period_s = handle->dt_s * (float)handle->chart_decimation;
}

static void MotorAutoTune_PushChart(MotorAutoTune_t *handle, float primary, float secondary, float tertiary)
{
	if ((handle == 0) || (handle->chart_stage == MOTOR_AUTOTUNE_CHART_NONE))
	{
		return;
	}
	if (handle->chart_length >= MOTOR_AUTOTUNE_CAPTURE_SAMPLES)
	{
		return;
	}
	if ((handle->chart_decimation > 1u) && ((handle->counter % handle->chart_decimation) != 0u))
	{
		return;
	}

	handle->chart_primary[handle->chart_length] = primary;
	handle->chart_secondary[handle->chart_length] = secondary;
	handle->chart_tertiary[handle->chart_length] = tertiary;
	handle->chart_length++;
}

static void MotorAutoTune_ArmChartTransfer(MotorAutoTune_t *handle, uint8_t progress_percent)
{
	if (handle == 0)
	{
		return;
	}

	handle->progress_percent = progress_percent;
	handle->tuning_data_ready = 1u;
	handle->chart_transfer_active = 1u;
	handle->chart_send_index = 0u;
}

static void MotorAutoTune_AdvanceToStage(MotorAutoTune_t *handle, MotorAutoTuneState_e next_state)
{
	if (handle == 0)
	{
		return;
	}

	handle->state = next_state;
	handle->substep = 0u;
	handle->counter = 0u;
	handle->phase_counter = 0u;
	handle->chart_send_index = 0u;
	handle->chart_transfer_active = 0u;
}

static void MotorAutoTune_Finish(MotorAutoTune_t *handle, const MotorAutoTuneInputs_t *inputs)
{
	float speed_bw_rad_s;
	float position_bw_rad_s;
	float rotor_inertia;
	float pole_pairs;
	float torque_constant;
	float encoder_resolution_counts;
	float position_output_scale;

	if (handle == 0)
	{
		return;
	}

	speed_bw_rad_s = 2.0f * PI * MotorAutoTune_Clamp(
		handle->config.speed_bandwidth_hz,
		0.5f,
		500.0f);
	position_bw_rad_s = 2.0f * PI * MotorAutoTune_Clamp(
		handle->config.position_bandwidth_hz,
		0.1f,
		100.0f);

	MotorAutoTune_UpdateCurrentGains(handle);

	rotor_inertia = handle->measured_J;
	if ((rotor_inertia <= 0.0f) && (inputs != 0))
	{
		rotor_inertia = inputs->rotor_inertia;
	}
	if (rotor_inertia <= 0.0f)
	{
		/* Conservative fallback when no trustworthy inertia estimate exists. */
		rotor_inertia = 1.0e-4f;
	}

	pole_pairs = (handle->measured_PolePairs > 0.5f) ? handle->measured_PolePairs : 1.0f;
	torque_constant = 1.5f * pole_pairs * MotorAutoTune_Clamp(handle->measured_Flux, 1.0e-5f, 10.0f);
	handle->tuned_speed_kp = (rotor_inertia * speed_bw_rad_s) / torque_constant;
	handle->tuned_speed_ki = handle->tuned_speed_kp * (speed_bw_rad_s * 0.25f);

	encoder_resolution_counts = (inputs != 0) ? inputs->encoder_resolution_counts : 0.0f;
	encoder_resolution_counts = MotorAutoTune_Clamp(
		encoder_resolution_counts,
		1.0f,
		16777216.0f);
	/* Position PI works on encoder counts and outputs RPM. Scale the requested
	   bandwidth into count-based gains so Apply Estimated Gains matches the
	   runtime position PI controller instead of the legacy P-only path. */
	position_output_scale = 60.0f / encoder_resolution_counts;
	handle->tuned_position_kp = position_output_scale * (2.0f * position_bw_rad_s);
	handle->tuned_position_ki = position_output_scale * (position_bw_rad_s * position_bw_rad_s);

	handle->progress_percent = 100u;
	handle->state = MOTOR_AUTOTUNE_STATE_DONE;
	handle->enabled = 0u;
}

static void MotorAutoTune_ProcessRs(
	MotorAutoTune_t *handle,
	const MotorAutoTuneInputs_t *inputs,
	MotorAutoTuneOutputs_t *outputs)
{
	uint32_t settle_ticks;
	uint32_t average_ticks;
	float current_target;
	float current_limit;
	float low_i_avg;
	float low_v_avg;
	float high_i_avg;
	float high_v_avg;
	float delta_i;
	float delta_v;

	settle_ticks = MotorAutoTune_SecondsToTicks(handle, MOTOR_AUTOTUNE_RS_SETTLE_S);
	average_ticks = MotorAutoTune_SecondsToTicks(handle, MOTOR_AUTOTUNE_RS_AVERAGE_S);
	current_limit = MotorAutoTune_CurrentLoopVoltageLimit(inputs);
	current_target = handle->config.rs_current_low_a;

	outputs->mode = MOTOR_AUTOTUNE_OUTPUT_CURRENT_LOOP;
	outputs->id_ref_a = current_target;
	outputs->iq_ref_a = 0.0f;
	outputs->voltage_limit_v = current_limit;
	outputs->isolate_q_axis = 1u;

	if (handle->substep < 2u)
	{
		MotorAutoTune_PushChart(handle, inputs->id_current_a, inputs->vd_voltage_v, current_target);
	}

	switch (handle->substep)
	{
		case 0u:
			if (handle->counter == 0u)
			{
				MotorAutoTune_PrepareChart(
					handle,
					MOTOR_AUTOTUNE_CHART_RS,
					(2u * settle_ticks) + (2u * average_ticks));
			}
			if (++handle->counter >= settle_ticks)
			{
				handle->counter = 0u;
				handle->substep = 1u;
				handle->rs_low_v_sum = 0.0f;
				handle->rs_low_i_sum = 0.0f;
				handle->rs_low_samples = 0u;
			}
			break;

		case 1u:
			handle->rs_low_v_sum += inputs->vd_voltage_v;
			handle->rs_low_i_sum += inputs->id_current_a;
			handle->rs_low_samples++;
			if (++handle->counter >= average_ticks)
			{
				handle->counter = 0u;
				handle->substep = 2u;
			}
			break;

		case 2u:
			current_target = handle->config.rs_current_high_a;
			outputs->id_ref_a = current_target;
			if (++handle->counter >= settle_ticks)
			{
				handle->counter = 0u;
				handle->substep = 3u;
				handle->rs_high_v_sum = 0.0f;
				handle->rs_high_i_sum = 0.0f;
				handle->rs_high_samples = 0u;
			}
			break;

		case 3u:
			current_target = handle->config.rs_current_high_a;
			outputs->id_ref_a = current_target;
			handle->rs_high_v_sum += inputs->vd_voltage_v;
			handle->rs_high_i_sum += inputs->id_current_a;
			handle->rs_high_samples++;
			if (++handle->counter >= average_ticks)
			{
				low_i_avg = (handle->rs_low_samples > 0u) ?
					(handle->rs_low_i_sum / (float)handle->rs_low_samples) : 0.0f;
				low_v_avg = (handle->rs_low_samples > 0u) ?
					(handle->rs_low_v_sum / (float)handle->rs_low_samples) : 0.0f;
				high_i_avg = (handle->rs_high_samples > 0u) ?
					(handle->rs_high_i_sum / (float)handle->rs_high_samples) : 0.0f;
				high_v_avg = (handle->rs_high_samples > 0u) ?
					(handle->rs_high_v_sum / (float)handle->rs_high_samples) : 0.0f;
				delta_i = high_i_avg - low_i_avg;
				delta_v = high_v_avg - low_v_avg;

				if (MotorAutoTune_Abs(delta_i) > MOTOR_AUTOTUNE_MIN_RS_CURRENT_A)
				{
					handle->measured_Rs = MotorAutoTune_Abs(delta_v / delta_i);
				}
				else if (MotorAutoTune_Abs(high_i_avg) > MOTOR_AUTOTUNE_MIN_RS_CURRENT_A)
				{
					handle->measured_Rs = MotorAutoTune_Abs(high_v_avg / high_i_avg);
				}
				else
				{
					MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_SIGNAL);
					return;
				}

				if ((handle->measured_Rs <= 0.0f) || (handle->measured_Rs > 1000.0f))
				{
					MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_SIGNAL);
					return;
				}

				handle->substep = MOTOR_AUTOTUNE_RS_WAIT_HOST_SUBSTEP;
				handle->counter = 0u;
				MotorAutoTune_ArmChartTransfer(handle, MOTOR_AUTOTUNE_RS_PROGRESS);
			}
			break;

		case MOTOR_AUTOTUNE_RS_WAIT_HOST_SUBSTEP:
		default:
			outputs->id_ref_a = 0.0f;
			break;
	}
}

static void MotorAutoTune_ProcessLs(
	MotorAutoTune_t *handle,
	const MotorAutoTuneInputs_t *inputs,
	MotorAutoTuneOutputs_t *outputs)
{
	uint32_t prepare_ticks;
	uint32_t settle_ticks;
	uint32_t measure_ticks;
	uint32_t total_ticks;
	float frequency_hz;
	float omega;
	float voltage_peak_v;
	float elapsed_s;
	float commanded_vd;
	float sample_marker;
	float current_sample_a;
	float current_rms_a;
	float current_peak_a;
	float impedance_mag_ohm;
	float reactive_term_sq;

	prepare_ticks = MotorAutoTune_SecondsToTicks(handle, MOTOR_AUTOTUNE_LS_PREPARE_S);
	frequency_hz = MotorAutoTune_Clamp(
		handle->config.ls_frequency_hz,
		MOTOR_AUTOTUNE_MIN_LS_FREQUENCY_HZ,
		1000.0f);
	voltage_peak_v = MotorAutoTune_Abs(handle->config.ls_step_voltage_v);
	settle_ticks = MotorAutoTune_SecondsToTicks(
		handle,
		MOTOR_AUTOTUNE_LS_SETTLE_CYCLES / frequency_hz);
	measure_ticks = MotorAutoTune_SecondsToTicks(
		handle,
		MOTOR_AUTOTUNE_LS_MEASURE_CYCLES / frequency_hz);
	total_ticks = settle_ticks + measure_ticks;
	omega = 2.0f * PI * frequency_hz;

	switch (handle->substep)
	{
		case 0u:
			if (handle->counter == 0u)
			{
				MotorAutoTune_PrepareChart(
					handle,
					MOTOR_AUTOTUNE_CHART_LS,
					prepare_ticks + total_ticks);
			}
			outputs->mode = MOTOR_AUTOTUNE_OUTPUT_CURRENT_LOOP;
			outputs->id_ref_a = 0.0f;
			outputs->iq_ref_a = 0.0f;
			outputs->voltage_limit_v = MotorAutoTune_CurrentLoopVoltageLimit(inputs);
			outputs->isolate_q_axis = 1u;
			MotorAutoTune_PushChart(handle, inputs->id_current_a, inputs->vd_voltage_v, 0.0f);
			if (++handle->counter >= prepare_ticks)
			{
				handle->counter = 0u;
				handle->ls_step_voltage_applied_v = voltage_peak_v;
				handle->ls_excitation_frequency_hz = frequency_hz;
				handle->ls_current_sq_sum = 0.0f;
				handle->ls_current_peak_a = 0.0f;
				handle->ls_measure_samples = 0u;
				handle->ls_initial_current_a = 0.0f;
				handle->ls_final_current_a = 0.0f;
				handle->ls_current_slope_a_per_s = 0.0f;
				handle->substep = 1u;
			}
			break;

		case 1u:
			outputs->mode = MOTOR_AUTOTUNE_OUTPUT_DIRECT_D_VOLTAGE;
			elapsed_s = ((float)handle->counter) * handle->dt_s;
			commanded_vd = voltage_peak_v * sinf(omega * elapsed_s);
			outputs->vd_voltage_v = commanded_vd;
			outputs->vq_voltage_v = 0.0f;
			outputs->isolate_q_axis = 1u;
			sample_marker = (handle->counter >= settle_ticks) ? 1.0f : 0.0f;
			MotorAutoTune_PushChart(handle, inputs->id_current_a, commanded_vd, sample_marker);

			if (handle->counter >= settle_ticks)
			{
				current_sample_a = inputs->id_current_a;
				handle->ls_current_sq_sum += current_sample_a * current_sample_a;
				current_sample_a = MotorAutoTune_Abs(current_sample_a);
				if (current_sample_a > handle->ls_current_peak_a)
				{
					handle->ls_current_peak_a = current_sample_a;
				}
				handle->ls_measure_samples++;
			}

			if (++handle->counter >= total_ticks)
			{
				if (handle->ls_measure_samples < 4u)
				{
					MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_SIGNAL);
					return;
				}

				current_rms_a = sqrtf(handle->ls_current_sq_sum / (float)handle->ls_measure_samples);
				current_peak_a = current_rms_a * sqrtf(2.0f);
				handle->ls_initial_current_a = current_rms_a;
				handle->ls_final_current_a = current_peak_a;
				handle->ls_current_slope_a_per_s = handle->ls_current_peak_a;

				if (current_peak_a < MOTOR_AUTOTUNE_LS_MIN_CURRENT_PEAK_A)
				{
					MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_SIGNAL);
					return;
				}

				impedance_mag_ohm = voltage_peak_v / current_peak_a;
				reactive_term_sq =
					(impedance_mag_ohm * impedance_mag_ohm) -
					(handle->measured_Rs * handle->measured_Rs);

				if ((omega <= 0.0f) || (reactive_term_sq <= 0.0f))
				{
					MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_SIGNAL);
					return;
				}

				handle->measured_Ls = sqrtf(reactive_term_sq) / omega;
				if ((handle->measured_Ls <= 0.0f) || (handle->measured_Ls > 10.0f))
				{
					MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_SIGNAL);
					return;
				}
				MotorAutoTune_UpdateCurrentGains(handle);

				handle->substep = MOTOR_AUTOTUNE_LS_WAIT_HOST_SUBSTEP;
				handle->counter = 0u;
				MotorAutoTune_ArmChartTransfer(handle, MOTOR_AUTOTUNE_LS_PROGRESS);
			}
			break;

		case MOTOR_AUTOTUNE_LS_WAIT_HOST_SUBSTEP:
		default:
			outputs->mode = MOTOR_AUTOTUNE_OUTPUT_CURRENT_LOOP;
			outputs->id_ref_a = 0.0f;
			outputs->iq_ref_a = 0.0f;
			outputs->voltage_limit_v = MotorAutoTune_CurrentLoopVoltageLimit(inputs);
			outputs->isolate_q_axis = 1u;
			break;
	}
}

static void MotorAutoTune_ProcessFlux(
	MotorAutoTune_t *handle,
	const MotorAutoTuneInputs_t *inputs,
	MotorAutoTuneOutputs_t *outputs)
{
	float encoder_resolution;
	float test_frequency_hz;
	float test_voltage_v;
	float mechanical_turns;
	uint32_t timeout_ticks;
	uint32_t settle_ticks;
	uint32_t average_ticks;

	encoder_resolution = MotorAutoTune_Clamp(inputs->encoder_resolution_counts, 1.0f, 16777216.0f);
	test_frequency_hz = MotorAutoTune_OpenLoopPolePairFrequency(&handle->config);
	test_voltage_v = MotorAutoTune_OpenLoopPolePairVoltage(&handle->config);
	timeout_ticks = MotorAutoTune_SecondsToTicks(handle, MOTOR_AUTOTUNE_PP_TEST_TIMEOUT_S);
	settle_ticks = MotorAutoTune_SecondsToTicks(handle, MOTOR_AUTOTUNE_FLUX_SETTLE_S);
	average_ticks = MotorAutoTune_SecondsToTicks(handle, MOTOR_AUTOTUNE_FLUX_AVERAGE_S);

	switch (handle->substep)
	{
		case 0u:
			if (handle->counter == 0u)
			{
				handle->mechanical_position_start_counts = inputs->mechanical_position_counts;
				handle->commanded_electrical_turns = 0.0f;
				handle->estimated_mechanical_turns = 0.0f;
			}

			outputs->mode = MOTOR_AUTOTUNE_OUTPUT_OPEN_LOOP_VF;
			outputs->vf_frequency_hz = test_frequency_hz;
			outputs->vf_voltage_v = test_voltage_v;
			handle->commanded_electrical_turns += MotorAutoTune_Abs(test_frequency_hz) * handle->dt_s;
			mechanical_turns = MotorAutoTune_Abs(
				(inputs->mechanical_position_counts - handle->mechanical_position_start_counts) /
				encoder_resolution);
			handle->estimated_mechanical_turns = mechanical_turns;

			if (mechanical_turns >= MOTOR_AUTOTUNE_PP_TARGET_TURNS)
			{
				float ratio = 0.0f;

				if (mechanical_turns > 0.01f)
				{
					ratio = handle->commanded_electrical_turns / mechanical_turns;
				}
				if (ratio < 0.5f)
				{
					MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_SIGNAL);
					return;
				}

				handle->measured_PolePairs = floorf(ratio + 0.5f);
				handle->measured_PolePairs = MotorAutoTune_Clamp(handle->measured_PolePairs, 1.0f, 64.0f);
				handle->progress_percent = MOTOR_AUTOTUNE_PP_PROGRESS;
				handle->counter = 0u;
				handle->substep = 1u;
				return;
			}

			if (++handle->counter >= timeout_ticks)
			{
				if (mechanical_turns < MOTOR_AUTOTUNE_PP_MIN_MOVEMENT_TURNS)
				{
					MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_STALL);
				}
				else
				{
					MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_SIGNAL);
				}
			}
			break;

		case 1u:
			outputs->mode = MOTOR_AUTOTUNE_OUTPUT_OPEN_LOOP_VF;
			outputs->vf_frequency_hz = 0.0f;
			outputs->vf_voltage_v = 0.0f;
			if (++handle->counter >= MotorAutoTune_SecondsToTicks(handle, MOTOR_AUTOTUNE_PP_SETTLE_S))
			{
				handle->counter = 0u;
				handle->substep = 2u;
				handle->flux_voltage_sq_sum = 0.0f;
				handle->flux_current_sq_sum = 0.0f;
				handle->flux_speed_rpm_sum = 0.0f;
				handle->flux_samples = 0u;
			}
			break;

		case 2u:
			outputs->mode = MOTOR_AUTOTUNE_OUTPUT_OPEN_LOOP_VF;
			outputs->vf_frequency_hz = handle->config.flux_frequency_hz;
			outputs->vf_voltage_v = handle->config.flux_voltage_v;
			if (++handle->counter >= settle_ticks)
			{
				handle->counter = 0u;
				handle->substep = 3u;
			}
			break;

		case 3u:
		{
			float phase_rms_sq;
			float phase_current_rms_sq;
			float average_rpm;
			float electrical_omega_rad_s;
			float phase_rms;
			float phase_current_rms;
			float resistive_drop_rms;
			float compensated_phase_rms_sq;

			outputs->mode = MOTOR_AUTOTUNE_OUTPUT_OPEN_LOOP_VF;
			outputs->vf_frequency_hz = handle->config.flux_frequency_hz;
			outputs->vf_voltage_v = handle->config.flux_voltage_v;

			phase_rms_sq = (
				(inputs->phase_voltage_u_v * inputs->phase_voltage_u_v) +
				(inputs->phase_voltage_v_v * inputs->phase_voltage_v_v) +
				(inputs->phase_voltage_w_v * inputs->phase_voltage_w_v)) / 3.0f;
			phase_current_rms_sq = (
				(inputs->phase_u_a * inputs->phase_u_a) +
				(inputs->phase_v_a * inputs->phase_v_a) +
				(inputs->phase_w_a * inputs->phase_w_a)) / 3.0f;
			handle->flux_voltage_sq_sum += phase_rms_sq;
			handle->flux_current_sq_sum += phase_current_rms_sq;
			handle->flux_speed_rpm_sum += MotorAutoTune_Abs(inputs->mechanical_speed_rpm);
			handle->flux_samples++;

			if (++handle->counter < average_ticks)
			{
				break;
			}

			if (handle->flux_samples == 0u)
			{
				MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_SIGNAL);
				return;
			}

			average_rpm = handle->flux_speed_rpm_sum / (float)handle->flux_samples;
			if (average_rpm < MOTOR_AUTOTUNE_FLUX_MIN_SPEED_RPM)
			{
				MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_STALL);
				return;
			}

			phase_rms = sqrtf(handle->flux_voltage_sq_sum / (float)handle->flux_samples);
			phase_current_rms = sqrtf(handle->flux_current_sq_sum / (float)handle->flux_samples);
			resistive_drop_rms = phase_current_rms * MotorAutoTune_Clamp(handle->measured_Rs, 0.0f, 1000.0f);
			compensated_phase_rms_sq = (phase_rms * phase_rms) - (resistive_drop_rms * resistive_drop_rms);
			/* Best-effort Rs compensation for back-EMF estimation. If the computed
			   correction becomes non-physical because voltage is model-derived
			   from PWM or the operating point is too slow/noisy, fall back to the
			   original terminal-voltage estimate instead of aborting the flow. */
			if (compensated_phase_rms_sq > (phase_rms * phase_rms * 0.05f))
			{
				phase_rms = sqrtf(compensated_phase_rms_sq);
			}
			electrical_omega_rad_s = 2.0f * PI * average_rpm * handle->measured_PolePairs / 60.0f;
			if (electrical_omega_rad_s <= 1.0e-6f)
			{
				MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_SIGNAL);
				return;
			}

			handle->measured_Flux = (phase_rms * 1.41421356f) / electrical_omega_rad_s;
			handle->measured_Ke = handle->measured_Flux;
			handle->progress_percent = MOTOR_AUTOTUNE_FLUX_PROGRESS;
			MotorAutoTune_AdvanceToStage(handle, MOTOR_AUTOTUNE_STATE_J);
			break;
		}

		default:
			break;
	}
}

static void MotorAutoTune_ProcessJ(
	MotorAutoTune_t *handle,
	const MotorAutoTuneInputs_t *inputs,
	MotorAutoTuneOutputs_t *outputs)
{
	float frequency_hz;
	float settle_ticks;
	float min_cross_ticks;
	float current_speed_rad_s;
	float raw_torque_nm;
	float iq_amplitude_limit_a;
	float pole_pairs;
	float speed_release_threshold_rad_s = 0.0f;
	float speed_switch_threshold_rad_s = 0.0f;
	int8_t current_sign = 0;

	if ((handle == 0) || (inputs == 0) || (outputs == 0))
	{
		return;
	}

	frequency_hz = MotorAutoTune_GetMechanicalFrequencyHz(handle);
	settle_ticks = (float)MotorAutoTune_SecondsToTicks(
		handle,
		MOTOR_AUTOTUNE_MECH_SETTLE_CYCLES / frequency_hz);
	min_cross_ticks = (float)MotorAutoTune_SecondsToTicks(handle, 0.20f / frequency_hz);

	if ((handle->substep == 0u) && (handle->counter == 0u))
	{
		iq_amplitude_limit_a = MotorAutoTune_DefaultMechanicalIqAmplitude(handle, inputs);
		handle->mechanical_iq_amplitude_a = MotorAutoTune_Abs(handle->config.mechanical_iq_amplitude_a);
		if (handle->mechanical_iq_amplitude_a <= 0.0f)
		{
			handle->mechanical_iq_amplitude_a = iq_amplitude_limit_a;
		}
		if (handle->mechanical_iq_amplitude_a > iq_amplitude_limit_a)
		{
			handle->mechanical_iq_amplitude_a = iq_amplitude_limit_a;
		}

		pole_pairs = (handle->measured_PolePairs > 0.5f) ? handle->measured_PolePairs : 0.0f;
		if ((pole_pairs <= 0.0f) || (handle->measured_Flux <= 0.0f))
		{
			MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_SIGNAL);
			return;
		}

		handle->mechanical_torque_constant_nm_per_a = 1.5f * pole_pairs * handle->measured_Flux;
		MotorAutoTune_ResetMechanicalRuntime(handle, inputs, 0u);
		handle->progress_percent = MOTOR_AUTOTUNE_MECH_PROGRESS_J_BASE;
	}

	MotorAutoTune_CommandMechanicalSine(handle, inputs, outputs);
	MotorAutoTune_UpdateMechanicalObservables(handle, inputs);
	current_speed_rad_s = handle->mechanical_speed_filtered_rad_s;
	raw_torque_nm = handle->mechanical_torque_constant_nm_per_a * inputs->iq_current_a;
	speed_release_threshold_rad_s =
		MOTOR_AUTOTUNE_MECH_J_ZERO_RELEASE_RPM * ((2.0f * PI) / 60.0f);
	speed_switch_threshold_rad_s =
		MOTOR_AUTOTUNE_MECH_J_ZERO_SWITCH_RPM * ((2.0f * PI) / 60.0f);
	if (current_speed_rad_s > speed_switch_threshold_rad_s)
	{
		current_sign = 1;
	}
	else if (current_speed_rad_s < -speed_switch_threshold_rad_s)
	{
		current_sign = -1;
	}

	if (handle->substep == 0u)
	{
		handle->counter++;
		if ((float)handle->counter >= settle_ticks)
		{
			MotorAutoTune_ResetMechanicalRuntime(handle, inputs, 0u);
			handle->substep = 1u;
		}
		return;
	}

	if (handle->mechanical_window_active != 0u)
	{
		handle->mechanical_window_numerator +=
			raw_torque_nm * handle->mechanical_accel_filtered_rad_s2 * handle->dt_s;
		handle->mechanical_window_denominator +=
			handle->mechanical_accel_filtered_rad_s2 *
			handle->mechanical_accel_filtered_rad_s2 * handle->dt_s;
	}

	if (MotorAutoTune_Abs(current_speed_rad_s) <= speed_release_threshold_rad_s)
	{
		handle->mechanical_zero_cross_armed = 1u;
	}

	if ((current_sign != 0) &&
		(handle->mechanical_prev_speed_sign != 0) &&
		(current_sign != handle->mechanical_prev_speed_sign) &&
		(handle->mechanical_zero_cross_armed != 0u) &&
		(handle->phase_counter >= (uint32_t)min_cross_ticks))
	{
		handle->mechanical_zero_cross_count++;
		if (handle->mechanical_window_active == 0u)
		{
			handle->mechanical_window_active = 1u;
			handle->mechanical_window_numerator = 0.0f;
			handle->mechanical_window_denominator = 0.0f;
			handle->mechanical_window_count = 0u;
		}
		else
		{
			handle->mechanical_window_count++;
			handle->progress_percent = (uint8_t)(
				MOTOR_AUTOTUNE_MECH_PROGRESS_J_BASE +
				((handle->mechanical_window_count * 8u) / MOTOR_AUTOTUNE_MECH_J_WINDOW_TARGET));

			if (handle->mechanical_window_count >= MOTOR_AUTOTUNE_MECH_J_WINDOW_TARGET)
			{
				if (handle->mechanical_window_denominator <= 1.0e-9f)
				{
					MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_SIGNAL);
					return;
				}

				handle->mechanical_torque_sign =
					(handle->mechanical_window_numerator >= 0.0f) ? 1.0f : -1.0f;
				handle->measured_J = MotorAutoTune_Abs(
					handle->mechanical_window_numerator /
					handle->mechanical_window_denominator);
				if ((!isfinite(handle->measured_J)) ||
					(handle->measured_J <= 0.0f) ||
					(handle->measured_J > MOTOR_AUTOTUNE_MAX_J_KG_M2))
				{
					MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_SIGNAL);
					return;
				}

				MotorAutoTune_AdvanceToStage(handle, MOTOR_AUTOTUNE_STATE_B);
				return;
			}
		}

		handle->mechanical_prev_speed_sign = current_sign;
		handle->mechanical_zero_cross_armed = 0u;
		handle->phase_counter = 0u;
	}
	else if ((current_sign != 0) && (handle->mechanical_prev_speed_sign == 0))
	{
		handle->mechanical_prev_speed_sign = current_sign;
		handle->mechanical_zero_cross_armed = 0u;
	}

	handle->counter++;
	handle->phase_counter++;
	if (handle->counter >= handle->mechanical_timeout_ticks)
	{
		MotorAutoTune_SetError(
			handle,
			(handle->mechanical_peak_speed_rpm < MOTOR_AUTOTUNE_MECH_MIN_SPEED_RPM) ?
				MOTOR_AUTOTUNE_ERROR_STALL :
				MOTOR_AUTOTUNE_ERROR_SIGNAL);
	}
}

static void MotorAutoTune_ProcessB(
	MotorAutoTune_t *handle,
	const MotorAutoTuneInputs_t *inputs,
	MotorAutoTuneOutputs_t *outputs)
{
	float frequency_hz;
	float settle_ticks;
	float min_cross_ticks;
	float current_speed_rad_s;
	float corrected_torque_nm;
	float regression_output;
	float determinant;
	float speed_release_threshold_rad_s = 0.0f;
	float speed_switch_threshold_rad_s = 0.0f;
	int8_t current_sign = 0;

	if ((handle == 0) || (inputs == 0) || (outputs == 0))
	{
		return;
	}
	if (handle->measured_J <= 0.0f)
	{
		MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_SIGNAL);
		return;
	}

	frequency_hz = MotorAutoTune_GetMechanicalFrequencyHz(handle);
	settle_ticks = (float)MotorAutoTune_SecondsToTicks(
		handle,
		1.0f / frequency_hz);
	min_cross_ticks = (float)MotorAutoTune_SecondsToTicks(handle, 0.20f / frequency_hz);

	if ((handle->substep == 0u) && (handle->counter == 0u))
	{
		MotorAutoTune_ResetMechanicalRuntime(handle, inputs, 1u);
		handle->progress_percent = MOTOR_AUTOTUNE_MECH_PROGRESS_B_BASE;
	}

	MotorAutoTune_CommandMechanicalSine(handle, inputs, outputs);
	MotorAutoTune_UpdateMechanicalObservables(handle, inputs);
	current_speed_rad_s = handle->mechanical_speed_filtered_rad_s;
	corrected_torque_nm =
		handle->mechanical_torque_sign *
		handle->mechanical_torque_constant_nm_per_a *
		inputs->iq_current_a;
	regression_output = corrected_torque_nm -
		(handle->measured_J * handle->mechanical_accel_filtered_rad_s2);
	speed_release_threshold_rad_s =
		MOTOR_AUTOTUNE_MECH_J_ZERO_RELEASE_RPM * ((2.0f * PI) / 60.0f);
	speed_switch_threshold_rad_s =
		MOTOR_AUTOTUNE_MECH_J_ZERO_SWITCH_RPM * ((2.0f * PI) / 60.0f);
	if (current_speed_rad_s > speed_switch_threshold_rad_s)
	{
		current_sign = 1;
	}
	else if (current_speed_rad_s < -speed_switch_threshold_rad_s)
	{
		current_sign = -1;
	}

	if (handle->substep == 0u)
	{
		handle->counter++;
		if ((float)handle->counter >= settle_ticks)
		{
			MotorAutoTune_ResetMechanicalRuntime(handle, inputs, 1u);
			handle->substep = 1u;
		}
		return;
	}

	if (handle->mechanical_window_active != 0u)
	{
		handle->mechanical_regression_sw2 +=
			current_speed_rad_s * current_speed_rad_s * handle->dt_s;
		handle->mechanical_regression_sw += current_speed_rad_s * handle->dt_s;
		handle->mechanical_regression_s11 += handle->dt_s;
		handle->mechanical_regression_swy +=
			current_speed_rad_s * regression_output * handle->dt_s;
		handle->mechanical_regression_sy += regression_output * handle->dt_s;
	}

	if (MotorAutoTune_Abs(current_speed_rad_s) <= speed_release_threshold_rad_s)
	{
		handle->mechanical_zero_cross_armed = 1u;
	}

	if ((current_sign != 0) &&
		(handle->mechanical_prev_speed_sign != 0) &&
		(current_sign != handle->mechanical_prev_speed_sign) &&
		(handle->mechanical_zero_cross_armed != 0u) &&
		(handle->phase_counter >= (uint32_t)min_cross_ticks))
	{
		handle->mechanical_zero_cross_count++;
		if (handle->mechanical_window_active == 0u)
		{
			handle->mechanical_window_active = 1u;
			handle->mechanical_regression_sw2 = 0.0f;
			handle->mechanical_regression_sw = 0.0f;
			handle->mechanical_regression_s11 = 0.0f;
			handle->mechanical_regression_swy = 0.0f;
			handle->mechanical_regression_sy = 0.0f;
			handle->mechanical_window_count = 0u;
		}
		else
		{
			handle->mechanical_window_count++;
			handle->progress_percent = (uint8_t)(
				MOTOR_AUTOTUNE_MECH_PROGRESS_B_BASE +
				((handle->mechanical_window_count * MOTOR_AUTOTUNE_MECH_PROGRESS_B_SPAN) /
				 MOTOR_AUTOTUNE_MECH_B_WINDOW_TARGET));
			if (handle->mechanical_window_count >= MOTOR_AUTOTUNE_MECH_B_WINDOW_TARGET)
			{
				determinant =
					(handle->mechanical_regression_sw2 * handle->mechanical_regression_s11) -
					(handle->mechanical_regression_sw * handle->mechanical_regression_sw);
				if ((!isfinite(determinant)) || (determinant <= 1.0e-9f))
				{
					MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_SIGNAL);
					return;
				}

				handle->measured_B = (
					(handle->mechanical_regression_swy * handle->mechanical_regression_s11) -
					(handle->mechanical_regression_sy * handle->mechanical_regression_sw)) /
					determinant;
				handle->estimated_LoadTorque = (
					(handle->mechanical_regression_sy * handle->mechanical_regression_sw2) -
					(handle->mechanical_regression_swy * handle->mechanical_regression_sw)) /
					determinant;
				if ((handle->measured_B < 0.0f) && (handle->measured_B > -0.01f))
				{
					handle->measured_B = 0.0f;
				}
				if ((!isfinite(handle->measured_B)) ||
					(handle->measured_B < 0.0f) ||
					(handle->measured_B > MOTOR_AUTOTUNE_MAX_B_NM_S_PER_RAD))
				{
					MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_SIGNAL);
					return;
				}

				MotorAutoTune_Finish(handle, inputs);
				return;
			}
		}

		handle->mechanical_prev_speed_sign = current_sign;
		handle->mechanical_zero_cross_armed = 0u;
		handle->phase_counter = 0u;
	}
	else if ((current_sign != 0) && (handle->mechanical_prev_speed_sign == 0))
	{
		handle->mechanical_prev_speed_sign = current_sign;
		handle->mechanical_zero_cross_armed = 0u;
	}

	handle->counter++;
	handle->phase_counter++;
	if (handle->counter >= handle->mechanical_timeout_ticks)
	{
		MotorAutoTune_SetError(
			handle,
			(handle->mechanical_peak_speed_rpm < MOTOR_AUTOTUNE_MECH_MIN_SPEED_RPM) ?
				MOTOR_AUTOTUNE_ERROR_STALL :
				MOTOR_AUTOTUNE_ERROR_SIGNAL);
	}
}

void MotorAutoTune_SetDefaultConfig(MotorAutoTuneConfig_t *config)
{
	if (config == 0)
	{
		return;
	}

	config->rs_current_low_a = MOTOR_AUTOTUNE_DEFAULT_RS_LOW_A;
	config->rs_current_high_a = MOTOR_AUTOTUNE_DEFAULT_RS_HIGH_A;
	config->ls_step_voltage_v = MOTOR_AUTOTUNE_DEFAULT_LS_STEP_V;
	config->ls_frequency_hz = MOTOR_AUTOTUNE_DEFAULT_LS_FREQ_HZ;
	config->flux_frequency_hz = MOTOR_AUTOTUNE_DEFAULT_FLUX_FREQ_HZ;
	config->flux_voltage_v = MOTOR_AUTOTUNE_DEFAULT_FLUX_VOLT_V;
	config->current_bandwidth_hz = MOTOR_AUTOTUNE_DEFAULT_CURRENT_BW_HZ;
	config->speed_bandwidth_hz = MOTOR_AUTOTUNE_DEFAULT_SPEED_BW_HZ;
	config->position_bandwidth_hz = MOTOR_AUTOTUNE_DEFAULT_POSITION_BW_HZ;
	config->mechanical_iq_amplitude_a = MOTOR_AUTOTUNE_DEFAULT_MECH_IQ_A;
	config->mechanical_frequency_hz = MOTOR_AUTOTUNE_DEFAULT_MECH_FREQ_HZ;
	config->mechanical_speed_lpf_hz = MOTOR_AUTOTUNE_DEFAULT_MECH_SPEED_LPF_HZ;
}

void MotorAutoTune_Reset(MotorAutoTune_t *handle)
{
	if (handle == 0)
	{
		return;
	}

	memset(handle, 0, sizeof(*handle));
	MotorAutoTune_SetDefaultConfig(&handle->config);
	handle->state = MOTOR_AUTOTUNE_STATE_IDLE;
	handle->error = MOTOR_AUTOTUNE_ERROR_NONE;
	handle->chart_stage = MOTOR_AUTOTUNE_CHART_NONE;
}

uint8_t MotorAutoTune_Start(
	MotorAutoTune_t *handle,
	const MotorAutoTuneConfig_t *config,
	float loop_frequency_hz,
	float overcurrent_threshold_a)
{
	MotorAutoTuneConfig_t local_config;
	float ls_frequency_max_hz = 1000.0f;

	if (handle == 0)
	{
		return 0u;
	}

	MotorAutoTune_SetDefaultConfig(&local_config);
	if (config != 0)
	{
		local_config = *config;
	}

	local_config.rs_current_low_a = MotorAutoTune_Clamp(local_config.rs_current_low_a, MOTOR_AUTOTUNE_MIN_RS_CURRENT_A, 10.0f);
	local_config.rs_current_high_a = MotorAutoTune_Clamp(local_config.rs_current_high_a, local_config.rs_current_low_a + 0.05f, 20.0f);
	local_config.ls_step_voltage_v = MotorAutoTune_Clamp(local_config.ls_step_voltage_v, MOTOR_AUTOTUNE_MIN_STEP_VOLTAGE_V, 40.0f);
	if (loop_frequency_hz > MOTOR_AUTOTUNE_MIN_LOOP_HZ)
	{
		float dynamic_max_hz = loop_frequency_hz * 0.10f;
		if (dynamic_max_hz < ls_frequency_max_hz)
		{
			ls_frequency_max_hz = dynamic_max_hz;
		}
	}
	if (ls_frequency_max_hz < MOTOR_AUTOTUNE_MIN_LS_FREQUENCY_HZ)
	{
		ls_frequency_max_hz = MOTOR_AUTOTUNE_MIN_LS_FREQUENCY_HZ;
	}
	local_config.ls_frequency_hz = MotorAutoTune_Clamp(
		local_config.ls_frequency_hz,
		MOTOR_AUTOTUNE_MIN_LS_FREQUENCY_HZ,
		ls_frequency_max_hz);
	local_config.flux_frequency_hz = MotorAutoTune_Clamp(local_config.flux_frequency_hz, MOTOR_AUTOTUNE_MIN_FLUX_FREQUENCY_HZ, 200.0f);
	local_config.flux_voltage_v = MotorAutoTune_Clamp(local_config.flux_voltage_v, MOTOR_AUTOTUNE_MIN_FLUX_VOLTAGE_V, 40.0f);
	local_config.current_bandwidth_hz = MotorAutoTune_Clamp(local_config.current_bandwidth_hz, 1.0f, 2000.0f);
	local_config.speed_bandwidth_hz = MotorAutoTune_Clamp(local_config.speed_bandwidth_hz, 0.1f, 500.0f);
	local_config.position_bandwidth_hz = MotorAutoTune_Clamp(local_config.position_bandwidth_hz, 0.05f, 100.0f);
	local_config.mechanical_iq_amplitude_a = MotorAutoTune_Abs(local_config.mechanical_iq_amplitude_a);
	local_config.mechanical_frequency_hz = MotorAutoTune_Clamp(
		local_config.mechanical_frequency_hz,
		MOTOR_AUTOTUNE_MECH_MIN_FREQUENCY_HZ,
		MOTOR_AUTOTUNE_MECH_MAX_FREQUENCY_HZ);
	local_config.mechanical_speed_lpf_hz = MotorAutoTune_Clamp(
		local_config.mechanical_speed_lpf_hz,
		MOTOR_AUTOTUNE_MECH_MIN_SPEED_LPF_HZ,
		120.0f);

	if (loop_frequency_hz < MOTOR_AUTOTUNE_MIN_LOOP_HZ)
	{
		handle->config = local_config;
		MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_INVALID_CONFIG);
		return 0u;
	}

	memset(handle, 0, sizeof(*handle));
	handle->config = local_config;
	handle->state = MOTOR_AUTOTUNE_STATE_RS;
	handle->error = MOTOR_AUTOTUNE_ERROR_NONE;
	handle->enabled = 1u;
	handle->progress_percent = 1u;
	handle->loop_frequency_hz = loop_frequency_hz;
	handle->dt_s = 1.0f / loop_frequency_hz;
	handle->overcurrent_threshold_a = MotorAutoTune_Clamp(
		overcurrent_threshold_a,
		MOTOR_AUTOTUNE_MIN_OC_THRESHOLD_A,
		1000.0f);
	handle->chart_stage = MOTOR_AUTOTUNE_CHART_NONE;
	return 1u;
}

void MotorAutoTune_Stop(MotorAutoTune_t *handle)
{
	if (handle == 0)
	{
		return;
	}

	handle->enabled = 0u;
	handle->state = MOTOR_AUTOTUNE_STATE_IDLE;
	handle->error = MOTOR_AUTOTUNE_ERROR_NONE;
	handle->progress_percent = 0u;
	handle->tuning_data_ready = 0u;
	handle->chart_transfer_active = 0u;
	handle->chart_send_index = 0u;
	handle->chart_stage = MOTOR_AUTOTUNE_CHART_NONE;
	handle->counter = 0u;
	handle->phase_counter = 0u;
}

void MotorAutoTune_ClearDataReady(MotorAutoTune_t *handle)
{
	if (handle == 0)
	{
		return;
	}

	handle->tuning_data_ready = 0u;
	handle->chart_transfer_active = 0u;
	handle->chart_send_index = 0u;

	if ((handle->state == MOTOR_AUTOTUNE_STATE_RS) &&
		(handle->substep == MOTOR_AUTOTUNE_RS_WAIT_HOST_SUBSTEP))
	{
		MotorAutoTune_AdvanceToStage(handle, MOTOR_AUTOTUNE_STATE_LS);
	}
	else if ((handle->state == MOTOR_AUTOTUNE_STATE_LS) &&
		(handle->substep == MOTOR_AUTOTUNE_LS_WAIT_HOST_SUBSTEP))
	{
		MotorAutoTune_AdvanceToStage(handle, MOTOR_AUTOTUNE_STATE_FLUX);
	}
}

void MotorAutoTune_SetError(MotorAutoTune_t *handle, MotorAutoTuneError_e error)
{
	if (handle == 0)
	{
		return;
	}

	handle->enabled = 0u;
	handle->state = MOTOR_AUTOTUNE_STATE_ERROR;
	handle->error = error;
	handle->chart_transfer_active = 0u;
	handle->tuning_data_ready = 0u;
}

void MotorAutoTune_Process(
	MotorAutoTune_t *handle,
	const MotorAutoTuneInputs_t *inputs,
	MotorAutoTuneOutputs_t *outputs)
{
	float phase_current_abs_max;

	MotorAutoTune_ResetOutputs(outputs);
	if ((handle == 0) || (inputs == 0))
	{
		return;
	}

	if (handle->enabled == 0u)
	{
		return;
	}

	phase_current_abs_max = MotorAutoTune_Abs(inputs->phase_u_a);
	if (MotorAutoTune_Abs(inputs->phase_v_a) > phase_current_abs_max)
	{
		phase_current_abs_max = MotorAutoTune_Abs(inputs->phase_v_a);
	}
	if (MotorAutoTune_Abs(inputs->phase_w_a) > phase_current_abs_max)
	{
		phase_current_abs_max = MotorAutoTune_Abs(inputs->phase_w_a);
	}
	if (phase_current_abs_max > handle->overcurrent_threshold_a)
	{
		handle->oc_event_count++;
		MotorAutoTune_SetError(handle, MOTOR_AUTOTUNE_ERROR_OVERCURRENT);
		return;
	}

	switch (handle->state)
	{
		case MOTOR_AUTOTUNE_STATE_RS:
			MotorAutoTune_ProcessRs(handle, inputs, outputs);
			break;

		case MOTOR_AUTOTUNE_STATE_LS:
			MotorAutoTune_ProcessLs(handle, inputs, outputs);
			break;

		case MOTOR_AUTOTUNE_STATE_FLUX:
			MotorAutoTune_ProcessFlux(handle, inputs, outputs);
			break;

		case MOTOR_AUTOTUNE_STATE_J:
			MotorAutoTune_ProcessJ(handle, inputs, outputs);
			break;

		case MOTOR_AUTOTUNE_STATE_B:
			MotorAutoTune_ProcessB(handle, inputs, outputs);
			break;

		case MOTOR_AUTOTUNE_STATE_DONE:
		case MOTOR_AUTOTUNE_STATE_ERROR:
		case MOTOR_AUTOTUNE_STATE_IDLE:
		default:
			break;
	}
}

uint8_t MotorAutoTune_ApplyEstimatedParameters(
	const MotorAutoTune_t *handle,
	float *driver_parameters,
	uint32_t driver_parameter_count,
	float *motor_parameters,
	uint32_t motor_parameter_count)
{
	if ((handle == 0) || (driver_parameters == 0) || (motor_parameters == 0))
	{
		return 0u;
	}
	if ((handle->state != MOTOR_AUTOTUNE_STATE_DONE) && (handle->progress_percent < 100u))
	{
		return 0u;
	}
	if ((driver_parameter_count <= POSITION_I_GAIN) || (motor_parameter_count <= MOTOR_VISCOUS_FRICTION))
	{
		return 0u;
	}

	motor_parameters[MOTOR_RESISTANCE] = handle->measured_Rs * 1000.0f;
	motor_parameters[MOTOR_INDUCTANCE] = handle->measured_Ls * 1000000.0f;
	motor_parameters[MOTOR_BACK_EMF_CONSTANT] = handle->measured_Ke * 1000.0f;
	motor_parameters[MOTOR_ROTOR_INERTIA] = handle->measured_J * 1000.0f;
	motor_parameters[MOTOR_NUMBER_POLE_PAIRS] = handle->measured_PolePairs;
	motor_parameters[MOTOR_VISCOUS_FRICTION] = handle->measured_B * 1000.0f;
	motor_parameters[MOTOR_CURRENT_P_GAIN] = handle->tuned_current_kp;
	motor_parameters[MOTOR_CURRENT_I_GAIN] = handle->tuned_current_ki;

	driver_parameters[SPEED_P_GAIN] = handle->tuned_speed_kp;
	driver_parameters[SPEED_I_GAIN] = handle->tuned_speed_ki;
	driver_parameters[POSITION_P_GAIN] = handle->tuned_position_kp;
	driver_parameters[POSITION_I_GAIN] = handle->tuned_position_ki;

	return 1u;
}
