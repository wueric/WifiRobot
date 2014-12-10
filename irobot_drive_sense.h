#ifndef IROBOT_DRIVE_SENSE
#define IROBOT_DRIVE_SENSE

#include "mbed.h"

namespace irobotDriveSense {

/// OpCodes for iRobot open interface
typedef enum{
	OP_START				= 128,	///< Start mode
	OP_BAUD					= 129,	///< Change baud rate
	OP_CONTROL				= 130,	///< Control mode
	OP_SAFE					= 131,	///< Safe mode
	OP_FULL					= 132,	///< Full mode
	OP_DEMO					= 136,	///< Demo mode
	OP_DEMO_COVER			= 135,	///< Cover demo
	OP_DEMO_COVER_DOCK		= 143,	///< Dock & cover demo
	OP_DEMO_SPOT			= 134,	///< Spot demo
	OP_DRIVE				= 137,	///< Drive a turn radius and speed
	OP_DRIVE_DIRECT			= 145,	///< Directly drive left and right wheels
	OP_LEDS					= 139,	///< Control LEDs
	OP_DIGITAL_OUTPUTS		= 147,	///< Write digital outputs
	OP_PWM_LOW_SIDE_DRIVERS	= 144,	///< Write PWM low-side drivers
	OP_LOW_SIDE_DRIVERS		= 138,	///< Write analog low-side drivers
	OP_SEND_IR				= 151,	///< Send IR signal
	OP_SONG					= 140,	///< Define song
	OP_PLAY_SONG			= 141,	///< Play song
	OP_SENSORS				= 142,	///< Query sensors
	OP_QUERY_LIST			= 149,	///< Set list of sensors to query
	OP_STREAM				= 148,	///< Sensor stream
	OP_PAUSE_RESUME_STREAM	= 150,	///< Pause or resume sensor stream
	OP_SCRIPT				= 152,	///< Define action script
	OP_PLAY_SCRIPT			= 153,	///< Play action script
	OP_SHOW_SCRIPT			= 154,	///< Read the current action script
	OP_WAIT_TIME			= 155,	///< Script: instruct the robot to wait for a specified time
	OP_WAIT_DISTANCE		= 156,	///< Script: instruct the robot to wait for a specified distance
	OP_WAIT_ANGLE			= 157,	///< Script: instruct the robot to wait for a specified angle
	OP_WAIT_EVENT			= 158	///< Script: instruct the robot to wait for an event
} irobotOpcode_t;

#define OP_START_SIZE					1	///< Size of the START opcode and payload
#define OP_BAUD_SIZE					2	///< Size of the BAUD opcode and payload
#define OP_CONTROL_SIZE					1	///< Size of the CONTROL opcode and payload
#define OP_SAFE_SIZE					1	///< Size of the SAFE opcode and payload
#define OP_FULL_SIZE					1	///< Size of the FULL opcode and payload
#define OP_DEMO_SIZE					2	///< Size of the DEMO opcode and payload
#define OP_DEMO_COVER_SIZE				1	///< Size of the DEMO_COVER opcode and payload
#define OP_DEMO_COVER_DOCK_SIZE			1	///< Size of the DEMO_COVER_DOCK opcode and payload
#define OP_DEMO_SPOT_SIZE				1	///< Size of the DEMO_SPOT opcode and payload
#define OP_DRIVE_SIZE					5	///< Size of the DRIVE opcode and payload
#define OP_DRIVE_DIRECT_SIZE			5	///< Size of the DRIVE_DIRECT opcode and payload
#define OP_LEDS_SIZE					4	///< Size of the LEDs opcode and payload
#define OP_DIGITAL_OUTPUTS_SIZE			2	///< Size of the DIGITAL_OUTPUTS opcode and payload
#define OP_PWM_LOW_SIDE_DRIVERS_SIZE	4	///< Size of the PWM_LOW_SIDE_DRIVERS opcode and payload
#define OP_LOW_SIDE_DRIVERS_SIZE		2	///< Size of the LOW_SIDE_DRIVERS opcode and payload
#define OP_SEND_IR_SIZE					2	///< Size of the SEND_IR opcode and payload
// OP_SONG size is variable
#define OP_PLAY_SONG_SIZE				2	///< Size of the PLAY_SONG opcode and payload
#define OP_SENSORS_SIZE					2	///< Size of the SENSORS opcode and payload
// OP_QUERY_LIST size is variable
// OP_STREAM size is variable
#define OP_PAUSE_RESUME_STREAM_SIZE		2	///< Size of the PAUSE_RESUME_STREAM opcode and payload
// OP_SCRIPT size is variable
#define OP_PLAY_SCRIPT_SIZE				1	///< Size of the PLAY_SCRIPT opcode and payload
#define OP_SHOW_SCRIPT_SIZE				1	///< Size of the SHOW_SCRIPT opcode and payload
#define OP_WAIT_TIME_SIZE				2	///< Size of the WAIT_TIME opcode and payload
#define OP_WAIT_DISTANCE_SIZE			3	///< Size of the WAIT_DISTANCE opcode and payload
#define OP_WAIT_ANGLE_SIZE				3	///< Size of the WAIT_ANGLE opcode and payload
#define OP_WAIT_EVENT_SIZE				2	///< Size of the WAIT_EVENT opcode and payload


typedef enum{
	SENSOR_GROUP0							= 0,	///< Sensor Group 0
	SENSOR_GROUP1							= 1,	///< Sensor Group 1
	SENSOR_GROUP2							= 2,	///< Sensor Group 2
	SENSOR_GROUP3							= 3,	///< Sensor Group 3
	SENSOR_GROUP4							= 4,	///< Sensor Group 4
	SENSOR_GROUP5							= 5,	///< Sensor Group 5
	SENSOR_GROUP6							= 6,	///< Sensor Group 6
	SENSOR_BUMPS_WHEELDROPS					= 7,	///< Bumps & wheel drops
	SENSOR_WALL								= 8,	///< Wall
	SENSOR_CLIFF_LEFT						= 9,	///< Left cliff
	SENSOR_CLIFF_FRONT_LEFT					= 10,	///< Front left cliff
	SENSOR_CLIFF_FRONT_RIGHT				= 11,	///< Front right cliff
	SENSOR_CLIFF_RIGHT						= 12,	///< Right cliff
	SENSOR_VIRTUAL_WALL						= 13,	///< Virtual wall
	SENSOR_LOW_SIDE_DRIVER_WHEEL_OVERDRIVE	= 14,	///< Low-side driver and wheel overdrive
	SENSOR_UNUSED0							= 15,	///< Unused
	SENSOR_UNUSED1							= 16,	///< Unused
	SENSOR_INFRARED							= 17,	///< Infrared receiver
	SENSOR_BUTTONS							= 18,	///< Buttons
	SENSOR_DISTANCE							= 19,	///< Distance travelled
	SENSOR_ANGLE							= 20,	///< Angle turned
	SENSOR_CHARGING_STATE					= 21,	///< Charging state
	SENSOR_VOLTAGE							= 22,	///< Battery voltage
	SENSOR_CURRENT							= 23,	///< Battery current
	SENSOR_BATTERY_TEMPERATURE				= 24,	///< Battery temperature
	SENSOR_BATTERY_CHARGE					= 25,	///< Battery charge
	SENSOR_BATTERY_CAPACITY					= 26,	///< Battery capacity
	SENSOR_WALL_SIGNAL						= 27,	///< Wall analog signal
	SENSOR_CLIFF_LEFT_SIGNAL				= 28,	///< Left cliff analog signal
	SENSOR_CLIFF_FRONT_LEFT_SIGNAL			= 29,	///< Front left cliff analog signal
	SENSOR_CLIFF_FRONT_RIGHT_SIGNAL			= 30,	///< Front right cliff analog signal
	SENSOR_CLIFF_RIGHT_SIGNAL				= 31,	///< Right cliff analog signal
	SENSOR_CARGO_BAY_DIGITAL_INPUTS			= 32,	///< Cargo bay digital inputs
	SENSOR_CARGO_BAY_ANALOG_SIGNAL			= 33,	///< Cargo bay analog input
	SENSOR_CHARGING_SOURCES_AVAILABLE		= 34,	///< Charging sources available
	SENSOR_OI_MODE							= 35,	///< Open Interface (OI) mode
	SENSOR_SONG_NUMBER						= 36,	///< Song number
	SENSOR_SONG_PLAYING						= 37,	///< Song playing?
	SENSOR_NUMBER_OF_STREAM_PACKETS			= 38,	///< Number of packets in sensor stream
	SENSOR_REQUESTED_VELOCITY				= 39,	///< Requested drive velocity
	SENSOR_REQUESTED_RADIUS					= 40,	///< Requested drive radius
	SENSOR_REQUESTED_RIGHT_VELOCITY			= 41,	///< Requested right wheel velocity
	SENSOR_REQUESTED_LEFT_VELOCITY			= 42	///< Requested left wheel velocity
} irobotSensorCode;

// iRobot sensor packet sizes
#define SENSOR_GROUP0_SIZE							26	///< Size of the SENSOR_GROUP0 packet
#define SENSOR_GROUP1_SIZE							10	///< Size of the SENSOR_GROUP1 packet
#define SENSOR_GROUP2_SIZE							6	///< Size of the SENSOR_GROUP2 packet
#define SENSOR_GROUP3_SIZE							10	///< Size of the SENSOR_GROUP3 packet
#define SENSOR_GROUP4_SIZE							14	///< Size of the SENSOR_GROUP4 packet
#define SENSOR_GROUP5_SIZE							12	///< Size of the SENSOR_GROUP5 packet
#define SENSOR_GROUP6_SIZE							52	///< Size of the SENSOR_GROUP6 packet
#define SENSOR_BUMPS_WHEELDROPS_SIZE				1	///< Size of the BUMPS_WHEELDROPS packet
#define SENSOR_WALL_SIZE							1	///< Size of the WALL_SIZE packet
#define SENSOR_CLIFF_LEFT_SIZE						1	///< Size of the CLIFF_LEFT packet
#define SENSOR_CLIFF_FRONT_LEFT_SIZE				1	///< Size of the CLIFF_FRONT_LEFT packet
#define SENSOR_CLIFF_FRONT_RIGHT_SIZE				1 	///< Size of the CLIFF_FRONT_RIGHT packet
#define SENSOR_CLIFF_RIGHT_SIZE						1 	///< Size of the CLIFF_RIGHT packet
#define SENSOR_VIRTUAL_WALL_SIZE					1 	///< Size of the VIRUAL_WALL packet
#define SENSOR_LOW_SIDE_DRIVER_WHEEL_OVERDRIVE_SIZE	1 	///< Size of the LOW_SIDE_DRIVER_WHEEL_OVERDRIVE packet
#define SENSOR_UNUSED0_SIZE							1 	///< Size of the UNUSED0 packet
#define SENSOR_UNUSED1_SIZE							1 	///< Size of the UNUSED1 packet
#define SENSOR_INFRARED_SIZE						1 	///< Size of the INFRARED packet
#define SENSOR_BUTTONS_SIZE							1 	///< Size of the BUTTONS packet
#define SENSOR_DISTANCE_SIZE						2 	///< Size of the DISTANCE packet
#define SENSOR_ANGLE_SIZE							2 	///< Size of the ANGLE packet
#define SENSOR_CHARGING_STATE_SIZE					1 	///< Size of the CHARGING_STATE packet
#define SENSOR_VOLTAGE_SIZE							2 	///< Size of the VOLTAGE packet
#define SENSOR_CURRENT_SIZE							2 	///< Size of the CURRENT packet
#define SENSOR_BATTERY_TEMPERATURE_SIZE				1 	///< Size of the BATTERY_TEMPERATURE packet
#define SENSOR_BATTERY_CHARGE_SIZE					2 	///< Size of the BATTERY_CHARGE packet
#define SENSOR_BATTERY_CAPACITY_SIZE				2 	///< Size of the BATTERY_CAPACITY packet
#define SENSOR_WALL_SIGNAL_SIZE						2 	///< Size of the WALL_SIGNAL packet
#define SENSOR_CLIFF_LEFT_SIGNAL_SIZE				2 	///< Size of the CLIFF_LEFT_SIGNAL packet
#define SENSOR_CLIFF_FRONT_LEFT_SIGNAL_SIZE			2 	///< Size of the CLIFF_FRONT_LEFT_SIGNAL packet
#define SENSOR_CLIFF_FRONT_RIGHT_SIGNAL_SIZE		2 	///< Size of the CLIFF_FRONT_RIGHT_SIGNAL packet
#define SENSOR_CLIFF_RIGHT_SIGNAL_SIZE				2 	///< Size of the CLIFF_RIGHT_SIGNAL packet
#define SENSOR_CARGO_BAY_DIGITAL_INPUTS_SIZE		1 	///< Size of the CARGO_BAY_DIGITAL_INPUTS packet
#define SENSOR_CARGO_BAY_ANALOG_SIGNAL_SIZE			2 	///< Size of the CARGO_BAY_ANALOG_SIGNAL packet
#define SENSOR_CHARGING_SOURCES_AVAILABLE_SIZE		1 	///< Size of the CHARGING_SOURCES_AVAILABLE packet
#define SENSOR_OI_MODE_SIZE							1 	///< Size of the OI_MODE packet
#define SENSOR_SONG_NUMBER_SIZE						1 	///< Size of the SONG_NUMBER packet
#define SENSOR_SONG_PLAYING_SIZE					1 	///< Size of the SONG_PLAYING packet
#define SENSOR_NUMBER_OF_STREAM_PACKETS_SIZE		1 	///< Size of the NUMBER_OF_STREAM_PACKETS packet
#define SENSOR_REQUESTED_VELOCITY_SIZE				2 	///< Size of the REQUESTED_VELOCITY packet
#define SENSOR_REQUESTED_RADIUS_SIZE				2 	///< Size of the REQUESTED_RADIUS packet
#define SENSOR_REQUESTED_RIGHT_VELOCITY_SIZE		2 	///< Size of the REQUESTED_RIGHT_VELOCITY packet
#define SENSOR_REQUESTED_LEFT_VELOCITY_SIZE			2 	///< Size of the REQUESTED_LEFT_VELOCITY packet


void start(Serial& device);

void setWheelSpeed (Serial& device,
    int16_t rightWheel, 
    int16_t leftWheel);

void poll_netAngle (Serial& device,
    int16_t* angle);

void poll_netDistance(Serial& device,
    int16_t* distance);

}; // closing namespace irobotDriveSense

#endif

