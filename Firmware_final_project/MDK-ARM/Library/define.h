#ifndef DEFINE_H
#define DEFINE_H


#define FPGA_START_ADDR  (uint32_t)0x64000000
#define	REG_CURRENT_PHASE_U									FPGA_START_ADDR + 2
#define	REG_CURRENT_PHASE_V									FPGA_START_ADDR + 4
#define REG_TEMPARATURE_SENSOR							FPGA_START_ADDR + 6
#define REG_DC_BUS_VOLTAGE									FPGA_START_ADDR + 8
#define REG_ENCODER_ID											FPGA_START_ADDR + 10
#define REG_ENCODER_RX_WORD0									FPGA_START_ADDR + 12
#define REG_ENCODER_RX_WORD1									FPGA_START_ADDR + 14
#define REG_ENCODER_RX_WORD2									FPGA_START_ADDR + 16
#define REG_ENCODER_RX_WORD3									FPGA_START_ADDR + 18
#define REG_ENCODER_RX_WORD4									FPGA_START_ADDR + 20
#define REG_ENCODER_RX_WORD5									FPGA_START_ADDR + 22
#define REG_ENCODER_RX_WORD6									FPGA_START_ADDR + 24
#define REG_FPGA_ERROR												FPGA_START_ADDR + 26

#define PI 3.141592654f


//Define Motor encoder ID
#define TAMAGAWA_SERIAL_ABS_SINGLE_TURN				1
#define TAMAGAWA_SERIAL_ABS_MULTI_TURN				2
#define	PANASONIC_MINAS_A5_SERIAL_INC					3
#define PANASONIC_MINAS_A6_SERIAL_ABS					4
#define	QUADRATURE_ABZ_HALL										5
#define	YASKAWA_SIGMA_5_SERIAL_INC_HALL				6
#define YASKAWA_SIGMA_5_ABS_SINGLE_TURN				7
#define YASKAWA_SIGMA_7_SERIAL_INC						8
#define YASKAWA_SIGMA_5_SGMAV_ROBOT						9
#define YASKAWA_SIGMA_SGMAS_06A2A2C						10
#define YASKAWA_SIGMA_SERIAL_ENCODER					6
#define PANASONIC_MINAS_A5_SERIAL_ABS					11
#define PANASONIC_MINAS_A4_A5_SERIAL_ENC			3
#define TAMAGAWA_SERIAL_ABS_MULTI_TURN_23BIT	12
#define	PANASONIC_MINAS_A4_SERIAL_INC					13


#define OFFSET 0x7FFF
#define Resolution16bits 65536.0f
#define INPUT_RANGE_VDC 1367.91f // 2137.36*0.64 (0.64 là tam do -320->320mV, 2137.36 la Vdc bi chia ap sau khi qua mach chia ap )
#define INPUT_RANGE_TEMPARATURE 400.0f //  15.(24)*0.64 (0.64 là tam do -320->320mV, 15.(24) la VTS bi chia ap sau khi qua mach chia ap )
#define INPUT_RANGE_I 21.33f

/*Fault define*/	
#define NO_ERROR 														(uint16_t)(0x0000u)
#define NO_FAULTS														(uint16_t)(0x0000u)
#define ERROR_OVER_CURRENT									(uint16_t)(0x0001u)
#define ERROR_OVER_VOLTAGE 									(uint16_t)(0x0002u)
#define ERROR_UNDER_VOLTAGE 								(uint16_t)(0x0004u)
#define ERROR_OVER_TEMPERATURE 							(uint16_t)(0x0008u)
#define ERROR_START_UP_FAIL									(uint16_t)(0x0010u)
#define ERROR_SPEED_FOLLOWING_ERROR					(uint16_t)(0x0020u)
#define SOFTWARE_ERROR 											(uint16_t)(0x0040u)
#define READ_FLASH_MEMORY_ERROR							(uint16_t)(0x0080u)
#define ERROR_CALIB_TIMEOUT								(uint16_t)(0x0100u)
#define ERROR_CURRENT_OFFSET_INVALID				(uint16_t)(0x0200u)

#define HALF_PWM_PERIOD						5250
#define FULL_PWM_PERIOD						10500
#define MOTOR_POWER_CONVERSION_FACTOR			1
#define SPEED_LOOP_FREQUENCY				(float)8000.0f 		//Hz	//8000
#define CURRENT_LOOP_FREQUENCY			(float)16000.0f		//Hz
#define CONTROL_TIMING_MODE_16KHZ		0u
#define USER_ISR_FREQUENCY_16KHZ		CURRENT_LOOP_FREQUENCY
/* Runtime control is fixed at 16 kHz across firmware and GUI. */
#define USER_DEFAULT_CONTROL_TIMING_MODE	CONTROL_TIMING_MODE_16KHZ
#define USER_SELECTED_ISR_FREQUENCY		USER_ISR_FREQUENCY_16KHZ
#define USER_EFFECTIVE_CURRENT_LOOP_FREQUENCY	USER_SELECTED_ISR_FREQUENCY
#define USER_EFFECTIVE_SPEED_LOOP_FREQUENCY		(USER_EFFECTIVE_CURRENT_LOOP_FREQUENCY * 0.5f)
#define POS_MAX_CNT									1
#define POSITION_LOOP_FREQUENCY			8000
/*Motor Speed Sensor*/
#define USE_HALL_SENSOR				0
#define TICK_PER_TIMER				7200
#define SECTOR_RAD						(float)(1.0471975512)
//#define JOG_STEP_SPEED_CONVERSION				12 // 60*1000/(ENC_RESOLUTION)/0.5ms
#define SPEED_CONVERSION_FACTOR   MOTOR_ENC_RES/60// RPM to Cnt Per Sec
#define CNT_PER_S_TO_RPM_FACTOR 0.00045776367f
#define MOTOR_ENC_RES						1048576 // 20-bit incremental encoder pulses/rev
#define POSITION_MAX_ERR			MOTOR_ENC_RES/5

/*Flash Memory Address*/
#define PAGE_ADDRESS_DRIVER_PARAMETER				(uint32_t)0x0805F000	//2kbyte: 0x0803F000 --> 0x0803F7FF: page 126
#define PAGE_ADDRESS_MOTOR_PARAMETER				(uint32_t)0x08060000	//2kbyte: 0x08040000 --> 0x080407FF: page 127
#define FLASH_USER_START_ADDR								PAGE_ADDRESS_DRIVER_PARAMETER
#define FLASH_USER_END_ADDR								(PAGE_ADDRESS_MOTOR_PARAMETER + 0x000007FF)

/*Driver sensor scale*/
#define OVER_VOLTAGE_THRESHOLD_VOLT_UNIT							55 //Volts
#define UNDER_VOLTAGE_THRESHOLD_VOLT_UNIT							36 //Volts
#define OVER_TEMPERATURE_THRESHOLD_DEGREE_UNIT				80 //degree celcious
#define OVER_TEMPERATURE_DEADBAND_DEGREE_UNIT					75 //degree celcious

#define DIRVER_OVER_CURRENT_THRESHOLD_AMPERE_UNIT			6.0f  //Ampere
#define CURRENT_SENSOR_CALIB_SAMPLES									5000u
#define CURRENT_SENSOR_CALIB_TIMEOUT_SECONDS					2.0f
#define CURRENT_SENSOR_OFFSET_THRESHOLD_COUNTS				2048u

#define TUNING_DATA_LENGTH 600

#define POSITION_CONTROL_MODE			0x02
#define	SPEED_CONTROL_MODE				0x01
#define FFT_DATA_LEN_TH						8192
#define TRACE_DATA_LENGTH					1000
#define AUTO_TUNING_DATA_LENGTH		2000
#define DEFAULT_MOTOR_ENCODER_ID		PANASONIC_MINAS_A5_SERIAL_INC
#define DEFAULT_MOTOR_RATED_CURRENT_RMS	1.6f
#define DEFAULT_MOTOR_PEAK_CURRENT_RMS	3.2f
#define DEFAULT_MOTOR_MAXIMUM_POWER_W	200.0f
#define DEFAULT_MOTOR_MAXIMUM_VOLTAGE_V	91.0f
#define DEFAULT_MOTOR_RATED_ELECTRICAL_FREQUENCY_HZ	200.0f
#define DEFAULT_MOTOR_RATED_SPEED_RPM	3000.0f
#define INITIAL_MOTOR_POLE_PAIRS 	4
/*
 * Empirical electrical zero compensation captured during Id tuning/alignment.
 * Encoder alignment should capture the rotor electrical zero directly.
 * Keep this at 0 deg unless hardware verification proves a fixed phase
 * compensation is required across every motor/driver pairing.
 */
#define DEFAULT_ELECTRICAL_ALIGNMENT_OFFSET_DEG	90.05f //Alex tunning so f**king tierd!!!
#define DRIVER_PARAMETER_COUNT					17u
#define POSITION_TRACKING_MODE_SINGLE_TURN	0u
#define POSITION_TRACKING_MODE_MULTI_TURN	1u
typedef enum
{
	DEVICE_ID,															/*00:						32 bit*/
	CONTROL_MODE,														/*01:						32 bit*/
	POSITION_P_GAIN,												/*02:						32 bit*/
	POSITION_FF_GAIN,												/*03:						32 bit*/
	POSITION_FF_FILTER,											/*04:						32 BIT*/
	SPEED_P_GAIN,														/*05:						32 bit*/
	SPEED_I_GAIN,														/*06:						32 bit*/
	SPEED_FF_GAIN,													/*07:						32 bit*/
	SPEED_FF_FILTER,												/*08:						32 bit*/
	SPEED_DETECTION_FILTER_FREQUENCY,				/*09:Hz:				32 bit*/
	ACCELERATION_TIME,											/*10:ms:				32 bit*/
	DECELERATION_TIME,											/*11:ms:				32 bit*/
	MAXIMUM_SPEED,													/*12:RPM				32 bit*/
	SPEED_MOVING_THRESHOLD,									/*13:RPM:				32 bit*/
	SPEED_UNIT,															/*14:						32 bit*/
	TORQUE_FILTER_FREQUENCY,								/*15:Hz:				32 bit*/
	POSITION_TRACKING_MODE,								/*16:0=single,1=multi	32 bit*/
//	SPEED_UNIT,															/*16:						32 bit*/
}DRIVER_PARAMETER_ID;

#define POSITION_I_GAIN SPEED_FF_GAIN

typedef enum
{
	MOTOR_RATED_CURRENT_RMS,			/*00:mili ampere:		32 bit*/
	MOTOR_PEAK_CURRENT_RMS,				/*01:mili ampere:		32 bit*/
	MOTOR_RESISTANCE,							/*02:mili Ohm: 			32 bit*/
	MOTOR_INDUCTANCE,							/*03:micro Henri:		32 bit*/
	MOTOR_BACK_EMF_CONSTANT,			/*04:mili unit			32 bit*/
	MOTOR_ROTOR_INERTIA,					/*05:mili unit			32 bit*/	
	MOTOR_ABS_ENCODER_MODE,				/*06:mili unit			32 bit*/
	MOTOR_ENCODER_ID,							/*07:								32 bit*/
	MOTOR_ENCODER_RESOLUTION,			/*08:pulses/Rev:		32 bit*/
	MOTOR_MAXIMUM_POWER,					/*09:								32 bit*/
	MOTOR_MAXIMUM_VOLTAGE,				/*10:Volt:					32 bit*/
	MOTOR_NUMBER_POLE_PAIRS,			/*11:								32 bit*/
	MOTOR_RATED_TORQUE,						/*12:mN.m						32 bit*/
	MOTOR_MAXIMUM_SPEED,					/*13:RPM:						32 bit*/
	MOTOR_OVERLOAD_TORQUE,				/*14:%:							32 bit*/
	MOTOR_OVERLOAD_TIME,					/*15:ms:						32 bit*/
	MOTOR_CURRENT_P_GAIN,					/*16:								32 bit*/
	MOTOR_CURRENT_I_GAIN,					/*17:								32 bit*/
	MOTOR_HALL_OFFSET,						/*18:Pulse:					32 bit*/
	MOTOR_CURRENT_CTRL_DIRECTION,	/*19:0 or 1					32 bit*/
	MOTOR_FORWARD_HALL_0,					/*20:Pulses					32 bit*/
	MOTOR_FORWARD_HALL_1,					/*21:Pulses					32 bit*/
	MOTOR_FORWARD_HALL_2,					/*22:Pulses					32 bit*/
	MOTOR_FORWARD_HALL_3,					/*23:Pulses					32 bit*/
	MOTOR_FORWARD_HALL_4,					/*24:Pulses					32 bit*/
	MOTOR_FORWARD_HALL_5,					/*25:Pulses					32 bit*/
	MOTOR_REVERSE_HALL_0,					/*26:Pulses					32 bit*/
	MOTOR_REVERSE_HALL_1,					/*27:Pulses					32 bit*/
	MOTOR_REVERSE_HALL_2,					/*28:Pulses					32 bit*/
	MOTOR_REVERSE_HALL_3,					/*29:Pulses					32 bit*/
	MOTOR_REVERSE_HALL_4,					/*30:Pulses					32 bit*/
	MOTOR_REVERSE_HALL_5					/*31:Pulses					32 bit*/
}MOTOR_PARAMETER_ID;

#endif

