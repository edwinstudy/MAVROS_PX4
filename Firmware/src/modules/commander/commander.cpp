/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file commander.cpp
 *
 * Main state machine / business logic
 *
 * @author Petri Tanskanen	<petri.tanskanen@inf.ethz.ch>
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Thomas Gubler	<thomas@px4.io>
 * @author Julian Oes		<julian@oes.ch>
 * @author Anton Babushkin	<anton@px4.io>
 * @author Sander Smeets	<sander@droneslab.com>
 */

#define  CHANGE_STATE //yuli
#define UI_STRIVE
#include <cmath>	// NAN

#ifdef  CHANGE_STATE
#include <drivers/drv_hrt.h>
#define MC_ID 1      //本机的ID号（1、2、3、4）
#endif
#define emergency_stop
#define RECEIVE_STATUS
/* commander module headers */
#include "accelerometer_calibration.h"
#include "airspeed_calibration.h"
#include "baro_calibration.h"
#include "calibration_routines.h"
#include "commander_helper.h"
#include "esc_calibration.h"
#include "gyro_calibration.h"
#include "mag_calibration.h"
#include "PreflightCheck.h"
#include "px4_custom_mode.h"
#include "rc_calibration.h"
#include "state_machine_helper.h"

/* PX4 headers */
#include <dataman/dataman.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>
#include <drivers/drv_tone_alarm.h>
#include <geo/geo.h>
#include <navigator/navigation.h>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <systemlib/circuit_breaker.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/param/param.h>
#include <systemlib/rc_check.h>
#include <systemlib/state_table.h>
#include <systemlib/systemlib.h>
#include <systemlib/hysteresis/hysteresis.h>

#include <sys/stat.h>
#include <string.h>
#include <math.h>
#include <poll.h>
#include <float.h>
#include <matrix/math.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_roi.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <v2.0/mavlink_types.h>
#ifdef UI_STRIVE
#include <uORB/topics/ui_strive_formation.h>
//#include <uORB/topics/ui_strive_formation_status.h>
#define Instance_MC_1 0             //formation status from mc1
#define Instance_MC_2 1             //formation status from mc2
#define Instance_MC_3 2             //formation status from mc3
#define Instance_MC_4 3             //formation status from mc4
#endif
#ifdef RECEIVE_STATUS
#include <uORB/topics/follow_to_commander.h>
#endif

typedef enum VEHICLE_MODE_FLAG
{
	VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED=1, /* 0b00000001 Reserved for future use. | */
	VEHICLE_MODE_FLAG_TEST_ENABLED=2, /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
	VEHICLE_MODE_FLAG_AUTO_ENABLED=4, /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
	VEHICLE_MODE_FLAG_GUIDED_ENABLED=8, /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
	VEHICLE_MODE_FLAG_STABILIZE_ENABLED=16, /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
	VEHICLE_MODE_FLAG_HIL_ENABLED=32, /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
	VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED=64, /* 0b01000000 remote control input is enabled. | */
	VEHICLE_MODE_FLAG_SAFETY_ARMED=128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. | */
	VEHICLE_MODE_FLAG_ENUM_END=129, /*  | */
} VEHICLE_MODE_FLAG;

static constexpr uint8_t COMMANDER_MAX_GPS_NOISE = 60;		/**< Maximum percentage signal to noise ratio allowed for GPS reception */

/* Decouple update interval and hysteresis counters, all depends on intervals */
#define COMMANDER_MONITORING_INTERVAL 10000
#define COMMANDER_MONITORING_LOOPSPERMSEC (1/(COMMANDER_MONITORING_INTERVAL/1000.0f))

#define MAVLINK_OPEN_INTERVAL 50000

#define STICK_ON_OFF_LIMIT 0.9f

#define POSITION_TIMEOUT		(1 * 1000 * 1000)	/**< consider the local or global position estimate invalid after 1000ms */
#define FAILSAFE_DEFAULT_TIMEOUT	(3 * 1000 * 1000)	/**< hysteresis time - the failsafe will trigger after 3 seconds in this state */
#define OFFBOARD_TIMEOUT		500000
#define DIFFPRESS_TIMEOUT		2000000

#define HOTPLUG_SENS_TIMEOUT		(8 * 1000 * 1000)	/**< wait for hotplug sensors to come online for upto 8 seconds */

#define PRINT_INTERVAL	5000000
#define PRINT_MODE_REJECT_INTERVAL	500000

#define INAIR_RESTART_HOLDOFF_INTERVAL	500000

#define HIL_ID_MIN 1000
#define HIL_ID_MAX 1999

/* Mavlink log uORB handle */
static orb_advert_t mavlink_log_pub = 0;

/* System autostart ID */
static int autostart_id;

/* flags */
static bool commander_initialized = false;
static volatile bool thread_should_exit = false;	/**< daemon exit flag */
static volatile bool thread_running = false;		/**< daemon status flag */
static int daemon_task;					/**< Handle of daemon task / thread */
static bool need_param_autosave = false;		/**< Flag set to true if parameters should be autosaved in next iteration (happens on param update and if functionality is enabled) */
static bool _usb_telemetry_active = false;
static hrt_abstime commander_boot_timestamp = 0;

static unsigned int leds_counter;
/* To remember when last notification was sent */
static uint64_t last_print_mode_reject_time = 0;

static systemlib::Hysteresis auto_disarm_hysteresis(false);

static float eph_threshold = 5.0f;
static float epv_threshold = 10.0f;

/* pre-flight EKF checks */
static float max_ekf_pos_ratio = 0.5f;
static float max_ekf_vel_ratio = 0.5f;
static float max_ekf_hgt_ratio = 0.5f;
static float max_ekf_yaw_ratio = 0.5f;
static float max_ekf_dvel_bias = 2.0e-3f;
static float max_ekf_dang_bias = 3.5e-4f;

/* pre-flight IMU consistency checks */
static float max_imu_acc_diff = 0.7f;
static float max_imu_gyr_diff = 0.09f;

static struct vehicle_status_s status = {};
static struct vehicle_roi_s _roi = {};
static struct battery_status_s battery = {};
static struct actuator_armed_s armed = {};
static struct safety_s safety = {};
static struct vehicle_control_mode_s control_mode = {};
static struct offboard_control_mode_s offboard_control_mode = {};
static struct home_position_s _home = {};
static int32_t _flight_mode_slots[manual_control_setpoint_s::MODE_SLOT_MAX];
static struct commander_state_s internal_state = {};

struct mission_result_s _mission_result;

static uint8_t main_state_before_rtl = commander_state_s::MAIN_STATE_MAX;
static unsigned _last_mission_instance = 0;
struct manual_control_setpoint_s sp_man = {};		///< the current manual control setpoint
static manual_control_setpoint_s _last_sp_man = {};	///< the manual control setpoint valid at the last mode switch

static struct vtol_vehicle_status_s vtol_status = {};
static struct cpuload_s cpuload = {};


static uint8_t main_state_prev = 0;
static bool warning_action_on = false;

static struct status_flags_s status_flags = {};

static uint64_t rc_signal_lost_timestamp;		// Time at which the RC reception was lost

static float avionics_power_rail_voltage;		// voltage of the avionics power rail

static bool can_arm_without_gps = false;

#ifdef CHANGE_STATE
hrt_abstime task_planning_time = 0;
hrt_abstime task_start_time = 0;
hrt_abstime takeoff_time = 2e6;
hrt_abstime followtarget_time = 10e6;
hrt_abstime catch_time = 15e6;
hrt_abstime docked_time = 20e6;
hrt_abstime pos_ctl_time = 25e6;
hrt_abstime return_time = 300e6;
bool plan_time_flag = false;
#endif

#ifdef UI_STRIVE
    ui_strive_formation_s formation ,formation1 ; //formation1, formation2, formation3, formation4;  //data from 4 different vehicles    *****zjm
    int _formation_sub;     //各个飞机之间不通信，各个飞机分别与地面站通信
//    int _formation_sub1;
//    int _formation_sub2;
//    int _formation_sub3;
//    int _formation_sub4;
    orb_advert_t _formation_status_pub;     //publish formation status      *****zjm
    //ui_strive_formation_status_s formation_status;
#endif
    extern mavlink_system_t mavlink_system;
#ifdef RECEIVE_STATUS
follow_to_commander_s return_flag = {};
int return_flag_sub = -1;
#endif
/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 *
 * @ingroup apps
 */
extern "C" __EXPORT int commander_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
void usage(const char *reason);

/**
 * React to commands that are sent e.g. from the mavlink module.
 */
bool handle_command(struct vehicle_status_s *status, const struct safety_s *safety, struct vehicle_command_s *cmd,
		    struct actuator_armed_s *armed, struct home_position_s *home, struct vehicle_global_position_s *global_pos,
		    struct vehicle_local_position_s *local_pos, struct vehicle_attitude_s *attitude, orb_advert_t *home_pub,
		    orb_advert_t *command_ack_pub, struct vehicle_command_ack_s *command_ack, struct vehicle_roi_s *roi,
			orb_advert_t *roi_pub);

/**
 * Mainloop of commander.
 */
int commander_thread_main(int argc, char *argv[]);

void control_status_leds(vehicle_status_s *status_local, const actuator_armed_s *actuator_armed, bool changed,
			 battery_status_s *battery_local, const cpuload_s *cpuload_local);

void get_circuit_breaker_params();

void check_valid(hrt_abstime timestamp, hrt_abstime timeout, bool valid_in, bool *valid_out, bool *changed);

transition_result_t set_main_state_rc(struct vehicle_status_s *status);

void set_control_mode();

bool stabilization_required();

void print_reject_mode(struct vehicle_status_s *current_status, const char *msg);

void print_reject_arm(const char *msg);

void print_status();

#ifdef CHANGE_STATE
void change_state(uint8_t _state);
void change_state(uint8_t _state)
{
   internal_state.main_state =  _state;

}
void task_plan();
void task_plan()
{
    if(task_planning_time > return_time || return_flag.status == 5) //return_flag.status来自于follow_target
    {
         change_state(5/*MAIN_STATE_AUTO_RTL*/);
         PX4_INFO("MAIN_STATE_AUTO_RTL");
    }

    ///
//    else if(task_planning_time > pos_ctl_time)
//    {
//        change_state(2/*MAIN_STATE_POSCTL*/);
//        PX4_INFO("MAIN_STATE_POSCTL");
//    }
//    ///
//    else if(task_planning_time > docked_time)
//    {
//        change_state(2/*MAIN_STATE_POSCTL*/);
//        PX4_INFO("MAIN_STATE_POSCTL");
//    }
    ///

//    else if(task_planning_time > catch_time)
//    {
//        change_state(12/*MAIN_STATE_AUTO_FOLLOW_TARGET*/);
//        PX4_INFO("MAIN_STATE_AUTO_FOLLOW_TARGET");

//    }
   ///
    else if(task_planning_time > followtarget_time)
    {
        change_state(12/*MAIN_STATE_AUTO_FOLLOW_TARGET*/);
        PX4_INFO("MAIN_STATE_AUTO_FOLLOW_TARGET");
    }
    ///
    else if(task_planning_time > takeoff_time)
    {
        change_state(10/*MAIN_STATE_AUTO_TAKEOFF*/);
        PX4_INFO("MAIN_STATE_AUTO_TAKEOFF");
    }
}
#endif

transition_result_t arm_disarm(bool arm, orb_advert_t *mavlink_log_pub, const char *armedBy);

/**
* @brief This function initializes the home position of the vehicle. This happens first time we get a good GPS fix and each
*		 time the vehicle is armed with a good GPS fix.
**/
static void commander_set_home_position(orb_advert_t &homePub, home_position_s &home,
					const vehicle_local_position_s &localPosition, const vehicle_global_position_s &globalPosition,
					const vehicle_attitude_s &attitude);

/**
 * Loop that runs at a lower rate and priority for calibration and parameter tasks.
 */
void *commander_low_prio_loop(void *arg);

void answer_command(struct vehicle_command_s &cmd, unsigned result,
					orb_advert_t &command_ack_pub, vehicle_command_ack_s &command_ack);

/**
 * check whether autostart ID is in the reserved range for HIL setups
 */
static bool is_hil_setup(int id) {
	return (id >= HIL_ID_MIN) && (id <= HIL_ID_MAX);
}


int commander_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("commander",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_DEFAULT + 40,
					     3200,
					     commander_thread_main,
					     (char * const *)&argv[0]);

		unsigned constexpr max_wait_us = 1000000;
		unsigned constexpr max_wait_steps = 2000;

		unsigned i;
		for (i = 0; i < max_wait_steps; i++) {
			usleep(max_wait_us / max_wait_steps);
			if (thread_running) {
				break;
			}
		}

		return !(i < max_wait_steps);
	}

	if (!strcmp(argv[1], "stop")) {

		if (!thread_running) {
			warnx("commander already stopped");
			return 0;
		}

		thread_should_exit = true;

		while (thread_running) {
			usleep(200000);
			warnx(".");
		}

		warnx("terminated.");

		return 0;
	}

	/* commands needing the app to run below */
	if (!thread_running) {
		warnx("\tcommander not started");
		return 1;
	}

	if (!strcmp(argv[1], "status")) {
		print_status();
		return 0;
	}

	if (!strcmp(argv[1], "calibrate")) {
		if (argc > 2) {
			int calib_ret = OK;
			if (!strcmp(argv[2], "mag")) {
				calib_ret = do_mag_calibration(&mavlink_log_pub);
			} else if (!strcmp(argv[2], "accel")) {
				calib_ret = do_accel_calibration(&mavlink_log_pub);
			} else if (!strcmp(argv[2], "gyro")) {
				calib_ret = do_gyro_calibration(&mavlink_log_pub);
			} else if (!strcmp(argv[2], "level")) {
				calib_ret = do_level_calibration(&mavlink_log_pub);
			} else if (!strcmp(argv[2], "esc")) {
				calib_ret = do_esc_calibration(&mavlink_log_pub, &armed);
			} else if (!strcmp(argv[2], "airspeed")) {
				calib_ret = do_airspeed_calibration(&mavlink_log_pub);
			} else {
				warnx("argument %s unsupported.", argv[2]);
			}

			if (calib_ret) {
				warnx("calibration failed, exiting.");
				return 1;
			} else {
				return 0;
			}
		} else {
			warnx("missing argument");
		}
	}

	if (!strcmp(argv[1], "check")) {
		int checkres = 0;
		checkres = preflight_check(&status, &mavlink_log_pub, false, true, &status_flags, &battery, false, hrt_elapsed_time(&commander_boot_timestamp));
		warnx("Preflight check: %s", (checkres == 0) ? "OK" : "FAILED");
		checkres = preflight_check(&status, &mavlink_log_pub, true, true, &status_flags, &battery, true, hrt_elapsed_time(&commander_boot_timestamp));
		warnx("Prearm check: %s", (checkres == 0) ? "OK" : "FAILED");
		return 0;
	}

	if (!strcmp(argv[1], "arm")) {
		if (TRANSITION_CHANGED != arm_disarm(true, &mavlink_log_pub, "command line")) {
			warnx("arming failed");
		}
		return 0;
	}

	if (!strcmp(argv[1], "disarm")) {
		if (TRANSITION_DENIED == arm_disarm(false, &mavlink_log_pub, "command line")) {
			warnx("rejected disarm");
		}
		return 0;
	}

	if (!strcmp(argv[1], "takeoff")) {

		/* see if we got a home position */
		if (status_flags.condition_home_position_valid) {

			if (TRANSITION_DENIED != arm_disarm(true, &mavlink_log_pub, "command line")) {

				vehicle_command_s cmd = {};
				cmd.target_system = status.system_id;
				cmd.target_component = status.component_id;

				cmd.command = vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF;
				cmd.param1 = NAN; /* minimum pitch */
				/* param 2-3 unused */
				cmd.param2 = NAN;
				cmd.param3 = NAN;
				cmd.param4 = NAN;
				cmd.param5 = NAN;
				cmd.param6 = NAN;
				cmd.param7 = NAN;

				orb_advert_t h = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
				(void)orb_unadvertise(h);

			} else {
				warnx("arming failed");
			}

		} else {
			warnx("rejecting takeoff, no position lock yet. Please retry..");
		}

		return 0;
	}

	if (!strcmp(argv[1], "land")) {

		vehicle_command_s cmd = {};
		cmd.target_system = status.system_id;
		cmd.target_component = status.component_id;

		cmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
		/* param 2-3 unused */
		cmd.param2 = NAN;
		cmd.param3 = NAN;
		cmd.param4 = NAN;
		cmd.param5 = NAN;
		cmd.param6 = NAN;
		cmd.param7 = NAN;

		orb_advert_t h = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
		(void)orb_unadvertise(h);

		return 0;
	}

	if (!strcmp(argv[1], "transition")) {

		vehicle_command_s cmd = {};
		cmd.target_system = status.system_id;
		cmd.target_component = status.component_id;

		cmd.command = vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION;
		/* transition to the other mode */
		cmd.param1 = (status.is_rotary_wing) ? vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW : vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
		/* param 2-3 unused */
		cmd.param2 = NAN;
		cmd.param3 = NAN;
		cmd.param4 = NAN;
		cmd.param5 = NAN;
		cmd.param6 = NAN;
		cmd.param7 = NAN;

		orb_advert_t h = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
		(void)orb_unadvertise(h);

		return 0;
	}

	if (!strcmp(argv[1], "mode")) {
		if (argc > 2) {
			uint8_t new_main_state = commander_state_s::MAIN_STATE_MAX;
			if (!strcmp(argv[2], "manual")) {
				new_main_state = commander_state_s::MAIN_STATE_MANUAL;
			} else if (!strcmp(argv[2], "altctl")) {
				new_main_state = commander_state_s::MAIN_STATE_ALTCTL;
			} else if (!strcmp(argv[2], "posctl")) {
				new_main_state = commander_state_s::MAIN_STATE_POSCTL;
			} else if (!strcmp(argv[2], "auto:mission")) {
				new_main_state = commander_state_s::MAIN_STATE_AUTO_MISSION;
			} else if (!strcmp(argv[2], "auto:loiter")) {
				new_main_state = commander_state_s::MAIN_STATE_AUTO_LOITER;
			} else if (!strcmp(argv[2], "auto:rtl")) {
				new_main_state = commander_state_s::MAIN_STATE_AUTO_RTL;
			} else if (!strcmp(argv[2], "acro")) {
				new_main_state = commander_state_s::MAIN_STATE_ACRO;
			} else if (!strcmp(argv[2], "offboard")) {
				new_main_state = commander_state_s::MAIN_STATE_OFFBOARD;
			} else if (!strcmp(argv[2], "stabilized")) {
				new_main_state = commander_state_s::MAIN_STATE_STAB;
			} else if (!strcmp(argv[2], "rattitude")) {
				new_main_state = commander_state_s::MAIN_STATE_RATTITUDE;
			} else if (!strcmp(argv[2], "auto:takeoff")) {
				new_main_state = commander_state_s::MAIN_STATE_AUTO_TAKEOFF;
			} else if (!strcmp(argv[2], "auto:land")) {
				new_main_state = commander_state_s::MAIN_STATE_AUTO_LAND;
			} else {
				warnx("argument %s unsupported.", argv[2]);
			}

			if (TRANSITION_DENIED == main_state_transition(&status, new_main_state, main_state_prev,  &status_flags, &internal_state)) {
				warnx("mode change failed");
			}
			return 0;

		} else {
			warnx("missing argument");
		}
	}

	if (!strcmp(argv[1], "lockdown")) {

		if (argc < 3) {
			usage("not enough arguments, missing [on, off]");
			return 1;
		}

		vehicle_command_s cmd = {};
		cmd.target_system = status.system_id;
		cmd.target_component = status.component_id;

		cmd.command = vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION;
		/* if the comparison matches for off (== 0) set 0.0f, 2.0f (on) else */
		cmd.param1 = strcmp(argv[2], "off") ? 2.0f : 0.0f; /* lockdown */

		orb_advert_t h = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
		(void)orb_unadvertise(h);

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

void usage(const char *reason)
{
	if (reason && *reason > 0) {
		PX4_INFO("%s", reason);
	}

	PX4_INFO("usage: commander {start|stop|status|calibrate|check|arm|disarm|takeoff|land|transition|mode}\n");
}

void print_status()
{
	warnx("type: %s", (status.is_rotary_wing) ? "symmetric motion" : "forward motion");
	warnx("safety: USB enabled: %s, power state valid: %s", (status_flags.usb_connected) ? "[OK]" : "[NO]",
	      (status_flags.condition_power_input_valid) ? " [OK]" : "[NO]");
	warnx("avionics rail: %6.2f V", (double)avionics_power_rail_voltage);
	warnx("home: lat = %.7f, lon = %.7f, alt = %.2f, yaw: %.2f", _home.lat, _home.lon, (double)_home.alt, (double)_home.yaw);
	warnx("home: x = %.7f, y = %.7f, z = %.2f ", (double)_home.x, (double)_home.y, (double)_home.z);
	warnx("datalink: %s", (status.data_link_lost) ? "LOST" : "OK");

#ifdef __PX4_POSIX
	warnx("main state: %d", internal_state.main_state);
	warnx("nav state: %d", status.nav_state);
#endif

	/* read all relevant states */
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	struct vehicle_status_s state;
	orb_copy(ORB_ID(vehicle_status), state_sub, &state);

	const char *armed_str;

	switch (status.arming_state) {
	case vehicle_status_s::ARMING_STATE_INIT:
		armed_str = "INIT";
		break;

	case vehicle_status_s::ARMING_STATE_STANDBY:
		armed_str = "STANDBY";
		break;

	case vehicle_status_s::ARMING_STATE_ARMED:
		armed_str = "ARMED";
		break;

	case vehicle_status_s::ARMING_STATE_ARMED_ERROR:
		armed_str = "ARMED_ERROR";
		break;

	case vehicle_status_s::ARMING_STATE_STANDBY_ERROR:
		armed_str = "STANDBY_ERROR";
		break;

	case vehicle_status_s::ARMING_STATE_REBOOT:
		armed_str = "REBOOT";
		break;

	case vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE:
		armed_str = "IN_AIR_RESTORE";
		break;

	default:
		armed_str = "ERR: UNKNOWN STATE";
		break;
	}

	px4_close(state_sub);


	warnx("arming: %s", armed_str);
}

static orb_advert_t status_pub;

transition_result_t arm_disarm(bool arm, orb_advert_t *mavlink_log_pub_local, const char *armedBy)
{
	transition_result_t arming_res = TRANSITION_NOT_CHANGED;

	// For HIL platforms, require that simulated sensors are connected
	if (arm && hrt_absolute_time() > commander_boot_timestamp + INAIR_RESTART_HOLDOFF_INTERVAL &&
		is_hil_setup(autostart_id) && status.hil_state != vehicle_status_s::HIL_STATE_ON) {

		mavlink_log_critical(mavlink_log_pub_local, "HIL platform: Connect to simulator before arming");
		return TRANSITION_DENIED;
	}

	// Transition the armed state. By passing mavlink_log_pub to arming_state_transition it will
	// output appropriate error messages if the state cannot transition.
	arming_res = arming_state_transition(&status,
					     &battery,
					     &safety,
					     arm ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_STANDBY,
					     &armed,
					     true /* fRunPreArmChecks */,
					     mavlink_log_pub_local,
					     &status_flags,
					     avionics_power_rail_voltage,
					     can_arm_without_gps,
					     hrt_elapsed_time(&commander_boot_timestamp));

	if (arming_res == TRANSITION_CHANGED) {
		mavlink_log_info(mavlink_log_pub_local, "%s by %s", arm ? "ARMED" : "DISARMED", armedBy);

	} else if (arming_res == TRANSITION_DENIED) {
		tune_negative(true);
	}

	return arming_res;
}

bool handle_command(struct vehicle_status_s *status_local, const struct safety_s *safety_local,
		    struct vehicle_command_s *cmd, struct actuator_armed_s *armed_local,
		    struct home_position_s *home, struct vehicle_global_position_s *global_pos,
		    struct vehicle_local_position_s *local_pos, struct vehicle_attitude_s *attitude, orb_advert_t *home_pub,
		    orb_advert_t *command_ack_pub, struct vehicle_command_ack_s *command_ack,
			struct vehicle_roi_s *roi, orb_advert_t *roi_pub)
{
	/* only handle commands that are meant to be handled by this system and component */
	if (cmd->target_system != status_local->system_id || ((cmd->target_component != status_local->component_id)
			&& (cmd->target_component != 0))) { // component_id 0: valid for all components
		return false;
	}

	/* result of the command */
	unsigned cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED;

	/* request to set different system mode */
	switch (cmd->command) {
	case vehicle_command_s::VEHICLE_CMD_DO_REPOSITION: {

		// Just switch the flight mode here, the navigator takes care of
		// doing something sensible with the coordinates. Its designed
		// to not require navigator and command to receive / process
		// the data at the exact same time.

		// Check if a mode switch had been requested
		if ((((uint32_t)cmd->param2) & 1) > 0) {
			transition_result_t main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_LOITER, main_state_prev, &status_flags, &internal_state);

			if ((main_ret != TRANSITION_DENIED)) {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				mavlink_log_critical(&mavlink_log_pub, "Rejecting reposition command");
			}
		} else {
			cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
		}
	}
	break;
	case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE: {
			uint8_t base_mode = (uint8_t)cmd->param1;
			uint8_t custom_main_mode = (uint8_t)cmd->param2;
			uint8_t custom_sub_mode = (uint8_t)cmd->param3;

			transition_result_t arming_ret = TRANSITION_NOT_CHANGED;

			transition_result_t main_ret = TRANSITION_NOT_CHANGED;

			/* set HIL state */
			hil_state_t new_hil_state = (base_mode & VEHICLE_MODE_FLAG_HIL_ENABLED) ? vehicle_status_s::HIL_STATE_ON : vehicle_status_s::HIL_STATE_OFF;
			transition_result_t hil_ret = hil_state_transition(new_hil_state, status_pub, status_local, &mavlink_log_pub);

			// Transition the arming state
			bool cmd_arm = base_mode & VEHICLE_MODE_FLAG_SAFETY_ARMED;

			arming_ret = arm_disarm(cmd_arm, &mavlink_log_pub, "set mode command");

			/* update home position on arming if at least 500 ms from commander start spent to avoid setting home on in-air restart */
			if (cmd_arm && (arming_ret == TRANSITION_CHANGED) &&
				(hrt_absolute_time() > (commander_boot_timestamp + INAIR_RESTART_HOLDOFF_INTERVAL))) {

				commander_set_home_position(*home_pub, *home, *local_pos, *global_pos, *attitude);
			}

			if (base_mode & VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED) {
				/* use autopilot-specific mode */
				if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_MANUAL) {
					/* MANUAL */
					main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, main_state_prev, &status_flags, &internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_ALTCTL) {
					/* ALTCTL */
					main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_ALTCTL, main_state_prev, &status_flags, &internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_POSCTL) {
					/* POSCTL */
					main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_POSCTL, main_state_prev, &status_flags, &internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_AUTO) {
					/* AUTO */
					if (custom_sub_mode > 0) {
						switch(custom_sub_mode) {
						case PX4_CUSTOM_SUB_MODE_AUTO_LOITER:
							main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_LOITER, main_state_prev, &status_flags, &internal_state);
							break;
						case PX4_CUSTOM_SUB_MODE_AUTO_MISSION:
							main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_MISSION, main_state_prev, &status_flags, &internal_state);
							break;
						case PX4_CUSTOM_SUB_MODE_AUTO_RTL:
							main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_RTL, main_state_prev, &status_flags, &internal_state);
							break;
						case PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF:
							main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_TAKEOFF, main_state_prev, &status_flags, &internal_state);
							break;
						case PX4_CUSTOM_SUB_MODE_AUTO_LAND:
							main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_LAND, main_state_prev, &status_flags, &internal_state);
							break;
						case PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET:
							main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET, main_state_prev, &status_flags, &internal_state);
							break;

						default:
							main_ret = TRANSITION_DENIED;
							mavlink_log_critical(&mavlink_log_pub, "Unsupported auto mode");
							break;
						}

					} else {
						main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_MISSION, main_state_prev, &status_flags, &internal_state);
					}

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_ACRO) {
					/* ACRO */
					main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_ACRO, main_state_prev, &status_flags, &internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_RATTITUDE) {
					/* RATTITUDE */
					main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_RATTITUDE, main_state_prev, &status_flags, &internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_STABILIZED) {
					/* STABILIZED */
					main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_STAB, main_state_prev, &status_flags, &internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD) {
					/* OFFBOARD */
					main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_OFFBOARD, main_state_prev, &status_flags, &internal_state);
				}

			} else {
				/* use base mode */
				if (base_mode & VEHICLE_MODE_FLAG_AUTO_ENABLED) {
					/* AUTO */
					main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_MISSION, main_state_prev, &status_flags, &internal_state);

				} else if (base_mode & VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED) {
					if (base_mode & VEHICLE_MODE_FLAG_GUIDED_ENABLED) {
						/* POSCTL */
						main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_POSCTL, main_state_prev, &status_flags, &internal_state);

					} else if (base_mode & VEHICLE_MODE_FLAG_STABILIZE_ENABLED) {
						/* STABILIZED */
						main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_STAB, main_state_prev, &status_flags, &internal_state);
					} else {
						/* MANUAL */
						main_ret = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, main_state_prev, &status_flags, &internal_state);
					}
				}
			}

			if ((hil_ret != TRANSITION_DENIED) && (arming_ret != TRANSITION_DENIED) && (main_ret != TRANSITION_DENIED)) {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

				if (arming_ret == TRANSITION_DENIED) {
					mavlink_log_critical(&mavlink_log_pub, "Rejecting arming cmd");
				}
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM: {

			// Adhere to MAVLink specs, but base on knowledge that these fundamentally encode ints
			// for logic state parameters
			if (static_cast<int>(cmd->param1 + 0.5f) != 0 && static_cast<int>(cmd->param1 + 0.5f) != 1) {
				mavlink_log_critical(&mavlink_log_pub, "Unsupported ARM_DISARM param: %.3f", (double)cmd->param1);

			} else {

				bool cmd_arms = (static_cast<int>(cmd->param1 + 0.5f) == 1);

				// Flick to inair restore first if this comes from an onboard system
				if (cmd->source_system == status_local->system_id && cmd->source_component == status_local->component_id) {
					status.arming_state = vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE;

				} else {
					// Refuse to arm if preflight checks have failed
					if ((!status_local->hil_state) != vehicle_status_s::HIL_STATE_ON && !status_flags.condition_system_sensors_initialized) {
						mavlink_log_critical(&mavlink_log_pub, "Arming DENIED. Preflight checks have failed.");
						cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;
						break;
					}

					// Refuse to arm if in manual with non-zero throttle
					if (cmd_arms
						&& (status_local->nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL
						|| status_local->nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO
						|| status_local->nav_state == vehicle_status_s::NAVIGATION_STATE_STAB
						|| status_local->nav_state == vehicle_status_s::NAVIGATION_STATE_RATTITUDE)
						&& (sp_man.z > 0.1f)) {

						mavlink_log_critical(&mavlink_log_pub, "Arming DENIED. Manual throttle non-zero.");
						cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;
						break;
					}
				}

				transition_result_t arming_res = arm_disarm(cmd_arms, &mavlink_log_pub, "arm/disarm component command");

				if (arming_res == TRANSITION_DENIED) {
					mavlink_log_critical(&mavlink_log_pub, "REJECTING component arm cmd");
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

				} else {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

					/* update home position on arming if at least 500 ms from commander start spent to avoid setting home on in-air restart */
					if (cmd_arms && (arming_res == TRANSITION_CHANGED) &&
						(hrt_absolute_time() > (commander_boot_timestamp + INAIR_RESTART_HOLDOFF_INTERVAL))) {

						commander_set_home_position(*home_pub, *home, *local_pos, *global_pos, *attitude);
					}
				}
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_OVERRIDE_GOTO: {
			// TODO listen vehicle_command topic directly from navigator (?)

			// Increase by 0.5f and rely on the integer cast
			// implicit floor(). This is the *safest* way to
			// convert from floats representing small ints to actual ints.
			unsigned int mav_goto = (cmd->param1 + 0.5f);

			if (mav_goto == 0) {	// MAV_GOTO_DO_HOLD
				status_local->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
				mavlink_log_critical(&mavlink_log_pub, "Pause mission cmd");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else if (mav_goto == 1) {	// MAV_GOTO_DO_CONTINUE
				status_local->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
				mavlink_log_critical(&mavlink_log_pub, "Continue mission cmd");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&mavlink_log_pub, "REJ CMD: %.1f %.1f %.1f %.1f %.1f %.1f %.1f",
						     (double)cmd->param1,
						     (double)cmd->param2,
						     (double)cmd->param3,
						     (double)cmd->param4,
						     (double)cmd->param5,
						     (double)cmd->param6,
						     (double)cmd->param7);
			}
		}
		break;

		/* Flight termination */
	case vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION: {
			if (cmd->param1 > 1.5f) {
				armed_local->lockdown = true;
				warnx("forcing lockdown (motors off)");

			} else if (cmd->param1 > 0.5f) {
				//XXX update state machine?
				armed_local->force_failsafe = true;
				warnx("forcing failsafe (termination)");

				/* param2 is currently used for other failsafe modes */
				status_local->engine_failure_cmd = false;
				status_flags.data_link_lost_cmd = false;
				status_flags.gps_failure_cmd = false;
				status_flags.rc_signal_lost_cmd = false;
				status_flags.vtol_transition_failure_cmd = false;

				if ((int)cmd->param2 <= 0) {
					/* reset all commanded failure modes */
					warnx("reset all non-flighttermination failsafe commands");

				} else if ((int)cmd->param2 == 1) {
					/* trigger engine failure mode */
					status_local->engine_failure_cmd = true;
					warnx("engine failure mode commanded");

				} else if ((int)cmd->param2 == 2) {
					/* trigger data link loss mode */
					status_flags.data_link_lost_cmd = true;
					warnx("data link loss mode commanded");

				} else if ((int)cmd->param2 == 3) {
					/* trigger gps loss mode */
					status_flags.gps_failure_cmd = true;
					warnx("GPS loss mode commanded");

				} else if ((int)cmd->param2 == 4) {
					/* trigger rc loss mode */
					status_flags.rc_signal_lost_cmd = true;
					warnx("RC loss mode commanded");

				} else if ((int)cmd->param2 == 5) {
					/* trigger vtol transition failure mode */
					status_flags.vtol_transition_failure_cmd = true;
					warnx("vtol transition failure mode commanded");
				}

			} else {
				armed_local->force_failsafe = false;
				armed_local->lockdown = false;
				warnx("disabling failsafe and lockdown");
			}

			cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_SET_HOME: {
			bool use_current = cmd->param1 > 0.5f;

			if (use_current) {
				/* use current position */
				if (status_flags.condition_global_position_valid) {
					home->lat = global_pos->lat;
					home->lon = global_pos->lon;
					home->alt = global_pos->alt;

					home->timestamp = hrt_absolute_time();

					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}

			} else {
				/* use specified position */
				home->lat = cmd->param5;
				home->lon = cmd->param6;
				home->alt = cmd->param7;

				home->timestamp = hrt_absolute_time();

				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
			}

			if (cmd_result == vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED) {
				mavlink_and_console_log_info(&mavlink_log_pub, "Home position: %.7f, %.7f, %.2f", home->lat, home->lon, (double)home->alt);

				/* announce new home position */
				if (*home_pub != nullptr) {
					orb_publish(ORB_ID(home_position), *home_pub, home);

				} else {
					*home_pub = orb_advertise(ORB_ID(home_position), home);
				}

				/* mark home position as set */
				status_flags.condition_home_position_valid = true;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_GUIDED_ENABLE: {
			transition_result_t res = TRANSITION_DENIED;
			static main_state_t main_state_pre_offboard = commander_state_s::MAIN_STATE_MANUAL;

			if (internal_state.main_state != commander_state_s::MAIN_STATE_OFFBOARD) {
				main_state_pre_offboard = internal_state.main_state;
			}

			if (cmd->param1 > 0.5f) {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_OFFBOARD, main_state_prev, &status_flags, &internal_state);

				if (res == TRANSITION_DENIED) {
					print_reject_mode(status_local, "OFFBOARD");
					status_flags.offboard_control_set_by_command = false;

				} else {
					/* Set flag that offboard was set via command, main state is not overridden by rc */
					status_flags.offboard_control_set_by_command = true;
				}

			} else {
				/* If the mavlink command is used to enable or disable offboard control:
				 * switch back to previous mode when disabling */
				res = main_state_transition(status_local, main_state_pre_offboard, main_state_prev, &status_flags, &internal_state);
				status_flags.offboard_control_set_by_command = false;
			}

			if (res == TRANSITION_DENIED) {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

			} else {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH: {
			if (TRANSITION_CHANGED == main_state_transition(&status, commander_state_s::MAIN_STATE_AUTO_RTL, main_state_prev, &status_flags, &internal_state)) {
				mavlink_and_console_log_info(&mavlink_log_pub, "Returning to launch");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&mavlink_log_pub, "Return to launch denied");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF: {
			/* ok, home set, use it to take off */
			if (TRANSITION_CHANGED == main_state_transition(&status, commander_state_s::MAIN_STATE_AUTO_TAKEOFF, main_state_prev, &status_flags, &internal_state)) {
				mavlink_and_console_log_info(&mavlink_log_pub, "Taking off");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&mavlink_log_pub, "Takeoff denied, disarm and re-try");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_LAND: {
			if (TRANSITION_CHANGED == main_state_transition(&status, commander_state_s::MAIN_STATE_AUTO_LAND, main_state_prev, &status_flags, &internal_state)) {
				mavlink_and_console_log_info(&mavlink_log_pub, "Landing at current position");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&mavlink_log_pub, "Landing denied, land manually");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_ROI:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI: {

		roi->mode = cmd->param1;

		if (roi->mode == vehicle_roi_s::VEHICLE_ROI_WPINDEX) {
			roi->mission_seq =  cmd->param2;
		}
		else if (roi->mode == vehicle_roi_s::VEHICLE_ROI_LOCATION) {
			roi->lat = cmd->param5;
			roi->lon = cmd->param6;
			roi->alt = cmd->param7;
		}
		else if (roi->mode == vehicle_roi_s::VEHICLE_ROI_TARGET) {
			roi->target_seq = cmd->param2;
		}

		if (*roi_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_roi), *roi_pub, roi);

		} else {
			*roi_pub = orb_advertise(ORB_ID(vehicle_roi), roi);
		}

		cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

		break;
	}
	case vehicle_command_s::VEHICLE_CMD_MISSION_START: {

		cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;

		// check if current mission and first item are valid
		if (_mission_result.valid) {

			// requested first mission item valid
			if (PX4_ISFINITE(cmd->param1) && (cmd->param1 >= 0) && (cmd->param1 < _mission_result.seq_total)) {

				// switch to AUTO_MISSION and ARM
				if ((TRANSITION_DENIED != main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_MISSION, main_state_prev, &status_flags, &internal_state))
					&& (TRANSITION_DENIED != arm_disarm(true, &mavlink_log_pub, "mission start command"))) {

					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
				} else {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
					mavlink_log_critical(&mavlink_log_pub, "Mission start denied");
				}
			}
		} else {
			mavlink_log_critical(&mavlink_log_pub, "Mission start denied, no valid mission");
		}
	}
	break;
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_0:
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_1:
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_2:
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL:
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE:
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL_QUAT:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_STORAGE:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_UAVCAN:
	case vehicle_command_s::VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY:
	case vehicle_command_s::VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY:
	case vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL:
	case vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION:
	case vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST:
	case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED:
	case vehicle_command_s::VEHICLE_CMD_DO_LAND_START:
	case vehicle_command_s::VEHICLE_CMD_DO_GO_AROUND:
	case vehicle_command_s::VEHICLE_CMD_START_RX_PAIR:
	case vehicle_command_s::VEHICLE_CMD_LOGGING_START:
	case vehicle_command_s::VEHICLE_CMD_LOGGING_STOP:
		/* ignore commands that handled in low prio loop */
		break;

	default:
		/* Warn about unsupported commands, this makes sense because only commands
		 * to this component ID (or all) are passed by mavlink. */
		answer_command(*cmd, vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED, *command_ack_pub, *command_ack);
		break;
	}

	if (cmd_result != vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED) {
		/* already warned about unsupported commands in "default" case */
		answer_command(*cmd, cmd_result, *command_ack_pub, *command_ack);
	}

	return true;
}

/**
* @brief This function initializes the home position of the vehicle. This happens first time we get a good GPS fix and each
*		 time the vehicle is armed with a good GPS fix.
**/
static void commander_set_home_position(orb_advert_t &homePub, home_position_s &home,
					const vehicle_local_position_s &localPosition, const vehicle_global_position_s &globalPosition,
					const vehicle_attitude_s &attitude)
{
	//Need global position fix to be able to set home
	if (!status_flags.condition_global_position_valid) {
		return;
	}

	//Ensure that the GPS accuracy is good enough for intializing home
	if (globalPosition.eph > eph_threshold || globalPosition.epv > epv_threshold) {
		return;
	}

	//Set home position
	home.timestamp = hrt_absolute_time();
	home.lat = globalPosition.lat;
	home.lon = globalPosition.lon;
	home.alt = globalPosition.alt;

	home.x = localPosition.x;
	home.y = localPosition.y;
	home.z = localPosition.z;

	matrix::Eulerf euler = matrix::Quatf(attitude.q);
	home.yaw = euler.psi();

    PX4_INFO("home: %.7f, %.7f, %.2f", home.lat, home.lon, (double)home.alt);

	/* announce new home position */
	if (homePub != nullptr) {
		orb_publish(ORB_ID(home_position), homePub, &home);
        PX4_INFO("orb_publish(ORB_ID(home_position)");

	} else {
		homePub = orb_advertise(ORB_ID(home_position), &home);
	}

	//Play tune first time we initialize HOME
	if (!status_flags.condition_home_position_valid) {
		tune_home_set(true);
	}

	/* mark home position as set */
	status_flags.condition_home_position_valid = true;
}

int commander_thread_main(int argc, char *argv[])
{
	/* not yet initialized */
	commander_initialized = false;

	bool sensor_fail_tune_played = false;
	bool arm_tune_played = false;
	bool was_landed = true;
	bool was_falling = false;
	bool was_armed = false;

	bool startup_in_hil = false;

	// XXX for now just set sensors as initialized
	status_flags.condition_system_sensors_initialized = true;

#ifdef __PX4_NUTTX
	/* NuttX indicates 3 arguments when only 2 are present */
	argc -= 1;
	argv += 1;
#endif

	if (argc > 2) {
		if (!strcmp(argv[2],"-hil")) {
			startup_in_hil = true;
		} else {
			PX4_ERR("Argument %s not supported, abort.", argv[2]);
			thread_should_exit = true;
		}
	}

	/* set parameters */
	param_t _param_sys_type = param_find("MAV_TYPE");
	param_t _param_system_id = param_find("MAV_SYS_ID");
	param_t _param_component_id = param_find("MAV_COMP_ID");
	param_t _param_enable_datalink_loss = param_find("NAV_DLL_ACT");
	param_t _param_offboard_loss_act = param_find("COM_OBL_ACT");
	param_t _param_offboard_loss_rc_act = param_find("COM_OBL_RC_ACT");
	param_t _param_enable_rc_loss = param_find("NAV_RCL_ACT");
	param_t _param_datalink_loss_timeout = param_find("COM_DL_LOSS_T");
	param_t _param_rc_loss_timeout = param_find("COM_RC_LOSS_T");
	param_t _param_datalink_regain_timeout = param_find("COM_DL_REG_T");
	param_t _param_ef_throttle_thres = param_find("COM_EF_THROT");
	param_t _param_ef_current2throttle_thres = param_find("COM_EF_C2T");
	param_t _param_ef_time_thres = param_find("COM_EF_TIME");
	param_t _param_autostart_id = param_find("SYS_AUTOSTART");
	param_t _param_autosave_params = param_find("COM_AUTOS_PAR");
	param_t _param_rc_in_off = param_find("COM_RC_IN_MODE");
	param_t _param_rc_arm_hyst = param_find("COM_RC_ARM_HYST");
	param_t _param_eph = param_find("COM_HOME_H_T");
	param_t _param_epv = param_find("COM_HOME_V_T");
	param_t _param_geofence_action = param_find("GF_ACTION");
	param_t _param_disarm_land = param_find("COM_DISARM_LAND");
	param_t _param_low_bat_act = param_find("COM_LOW_BAT_ACT");
	param_t _param_offboard_loss_timeout = param_find("COM_OF_LOSS_T");
	param_t _param_arm_without_gps = param_find("COM_ARM_WO_GPS");

	param_t _param_fmode_1 = param_find("COM_FLTMODE1");
	param_t _param_fmode_2 = param_find("COM_FLTMODE2");
	param_t _param_fmode_3 = param_find("COM_FLTMODE3");
	param_t _param_fmode_4 = param_find("COM_FLTMODE4");
	param_t _param_fmode_5 = param_find("COM_FLTMODE5");
	param_t _param_fmode_6 = param_find("COM_FLTMODE6");

	/* pre-flight EKF checks */
	param_t _param_max_ekf_pos_ratio = param_find("COM_ARM_EKF_POS");
	param_t _param_max_ekf_vel_ratio = param_find("COM_ARM_EKF_VEL");
	param_t _param_max_ekf_hgt_ratio = param_find("COM_ARM_EKF_HGT");
	param_t _param_max_ekf_yaw_ratio = param_find("COM_ARM_EKF_YAW");
	param_t _param_max_ekf_dvel_bias = param_find("COM_ARM_EKF_AB");
	param_t _param_max_ekf_dang_bias = param_find("COM_ARM_EKF_GB");

	/* pre-flight IMU consistency checks */
	param_t _param_max_imu_acc_diff = param_find("COM_ARM_IMU_ACC");
	param_t _param_max_imu_gyr_diff = param_find("COM_ARM_IMU_GYR");

	// These are too verbose, but we will retain them a little longer
	// until we are sure we really don't need them.

	// const char *main_states_str[commander_state_s::MAIN_STATE_MAX];
	// main_states_str[commander_state_s::MAIN_STATE_MANUAL]			= "MANUAL";
	// main_states_str[commander_state_s::MAIN_STATE_ALTCTL]			= "ALTCTL";
	// main_states_str[commander_state_s::MAIN_STATE_POSCTL]			= "POSCTL";
	// main_states_str[commander_state_s::MAIN_STATE_AUTO_MISSION]		= "AUTO_MISSION";
	// main_states_str[commander_state_s::MAIN_STATE_AUTO_LOITER]			= "AUTO_LOITER";
	// main_states_str[commander_state_s::MAIN_STATE_AUTO_RTL]			= "AUTO_RTL";
	// main_states_str[commander_state_s::MAIN_STATE_ACRO]			= "ACRO";
	// main_states_str[commander_state_s::MAIN_STATE_STAB]			= "STAB";
	// main_states_str[commander_state_s::MAIN_STATE_OFFBOARD]			= "OFFBOARD";

	// const char *nav_states_str[vehicle_status_s::NAVIGATION_STATE_MAX];
	// nav_states_str[vehicle_status_s::NAVIGATION_STATE_MANUAL]			= "MANUAL";
	// nav_states_str[vehicle_status_s::NAVIGATION_STATE_STAB]				= "STAB";
	// nav_states_str[vehicle_status_s::NAVIGATION_STATE_RATTITUDE]		= "RATTITUDE";
	// nav_states_str[vehicle_status_s::NAVIGATION_STATE_ALTCTL]			= "ALTCTL";
	// nav_states_str[vehicle_status_s::NAVIGATION_STATE_POSCTL]			= "POSCTL";
	// nav_states_str[vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION]		= "AUTO_MISSION";
	// nav_states_str[vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER]		= "AUTO_LOITER";
	// nav_states_str[vehicle_status_s::NAVIGATION_STATE_AUTO_RTL]		= "AUTO_RTL";
	// nav_states_str[vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF]		= "AUTO_TAKEOFF";
	// nav_states_str[vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER]		= "AUTO_RCRECOVER";
	// nav_states_str[vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS]		= "AUTO_RTGS";
	// nav_states_str[vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL]	= "AUTO_LANDENGFAIL";
	// nav_states_str[vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL]	= "AUTO_LANDGPSFAIL";
	// nav_states_str[vehicle_status_s::NAVIGATION_STATE_ACRO]			= "ACRO";
	// nav_states_str[vehicle_status_s::NAVIGATION_STATE_AUTO_LAND]			= "LAND";
	// nav_states_str[vehicle_status_s::NAVIGATION_STATE_DESCEND]		= "DESCEND";
	// nav_states_str[vehicle_status_s::NAVIGATION_STATE_TERMINATION]		= "TERMINATION";
	// nav_states_str[vehicle_status_s::NAVIGATION_STATE_OFFBOARD]		= "OFFBOARD";

	/* pthread for slow low prio thread */
	pthread_t commander_low_prio_thread;

	/* initialize */
	if (led_init() != OK) {
		PX4_WARN("LED init failed");
	}

	if (buzzer_init() != OK) {
		PX4_WARN("Buzzer init failed");
	}

	/* vehicle status topic */
	memset(&status, 0, sizeof(status));

	// We want to accept RC inputs as default
	status_flags.rc_input_blocked = false;
	status.rc_input_mode = vehicle_status_s::RC_IN_MODE_DEFAULT;
	internal_state.main_state = commander_state_s::MAIN_STATE_MANUAL;
	internal_state.timestamp = hrt_absolute_time();
	main_state_prev = commander_state_s::MAIN_STATE_MAX;
	status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
	status.arming_state = vehicle_status_s::ARMING_STATE_INIT;

	if (startup_in_hil) {
		status.hil_state = vehicle_status_s::HIL_STATE_ON;
	} else {
		status.hil_state = vehicle_status_s::HIL_STATE_OFF;
	}
	status.failsafe = false;

	/* neither manual nor offboard control commands have been received */
	status_flags.offboard_control_signal_found_once = false;
	status_flags.rc_signal_found_once = false;

	/* assume we don't have a valid baro on startup */
	status_flags.barometer_failure = true;
	status_flags.ever_had_barometer_data = false;

	/* mark all signals lost as long as they haven't been found */
	status.rc_signal_lost = true;
	status_flags.offboard_control_signal_lost = true;
	status.data_link_lost = true;
	status_flags.offboard_control_loss_timeout = false;

	status_flags.condition_system_prearm_error_reported = false;
	status_flags.condition_system_hotplug_timeout = false;

	status.timestamp = hrt_absolute_time();

	status_flags.condition_power_input_valid = true;
	avionics_power_rail_voltage = -1.0f;
	status_flags.usb_connected = false;

	// CIRCUIT BREAKERS
	status_flags.circuit_breaker_engaged_power_check = false;
	status_flags.circuit_breaker_engaged_airspd_check = false;
	status_flags.circuit_breaker_engaged_enginefailure_check = false;
	status_flags.circuit_breaker_engaged_gpsfailure_check = false;
	get_circuit_breaker_params();

	// initialize gps failure to false if circuit breaker enabled
	if (status_flags.circuit_breaker_engaged_gpsfailure_check) {
		status_flags.gps_failure = false;
	} else {
		status_flags.gps_failure = true;
	}

	/* publish initial state */
	status_pub = orb_advertise(ORB_ID(vehicle_status), &status);

	if (status_pub == nullptr) {
		warnx("ERROR: orb_advertise for topic vehicle_status failed (uorb app running?).\n");
		warnx("exiting.");
		px4_task_exit(PX4_ERROR);
	}

	/* Initialize armed with all false */
	memset(&armed, 0, sizeof(armed));
	/* armed topic */
	orb_advert_t armed_pub = orb_advertise(ORB_ID(actuator_armed), &armed);

	/* vehicle control mode topic */
	memset(&control_mode, 0, sizeof(control_mode));
	orb_advert_t control_mode_pub = orb_advertise(ORB_ID(vehicle_control_mode), &control_mode);

	/* home position */
	orb_advert_t home_pub = nullptr;
	memset(&_home, 0, sizeof(_home));

	/* region of interest */
	orb_advert_t roi_pub = nullptr;
	memset(&_roi, 0, sizeof(_roi));

	/* command ack */
	orb_advert_t command_ack_pub = nullptr;
	struct vehicle_command_ack_s command_ack;
	memset(&command_ack, 0, sizeof(command_ack));

	/* init mission state, do it here to allow navigator to use stored mission even if mavlink failed to start */
	orb_advert_t mission_pub = nullptr;
	mission_s mission;

	orb_advert_t commander_state_pub = nullptr;

	if (dm_read(DM_KEY_MISSION_STATE, 0, &mission, sizeof(mission_s)) == sizeof(mission_s)) {
		if (mission.dataman_id >= 0 && mission.dataman_id <= 1) {
			if (mission.count > 0) {
				mavlink_log_info(&mavlink_log_pub, "[cmd] Mission #%d loaded, %u WPs, curr: %d",
						 mission.dataman_id, mission.count, mission.current_seq);
			}

		} else {
			const char *missionfail = "reading mission state failed";
			warnx("%s", missionfail);
			mavlink_log_critical(&mavlink_log_pub, missionfail);

			/* initialize mission state in dataman */
			mission.dataman_id = 0;
			mission.count = 0;
			mission.current_seq = 0;
			dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission, sizeof(mission_s));
		}

		mission_pub = orb_advertise(ORB_ID(offboard_mission), &mission);
		orb_publish(ORB_ID(offboard_mission), mission_pub, &mission);
	}

	int ret;

	/* Start monitoring loop */
	unsigned counter = 0;
	unsigned stick_off_counter = 0;
	unsigned stick_on_counter = 0;

	bool low_battery_voltage_actions_done = false;
	bool critical_battery_voltage_actions_done = false;

	bool status_changed = true;
	bool param_init_forced = true;

	bool updated = false;

	/* Subscribe to safety topic */
	int safety_sub = orb_subscribe(ORB_ID(safety));
	memset(&safety, 0, sizeof(safety));
	safety.safety_switch_available = false;
	safety.safety_off = false;

	/* Subscribe to mission result topic */
	int mission_result_sub = orb_subscribe(ORB_ID(mission_result));

	/* Subscribe to geofence result topic */
	int geofence_result_sub = orb_subscribe(ORB_ID(geofence_result));
	struct geofence_result_s geofence_result;
	memset(&geofence_result, 0, sizeof(geofence_result));

	/* Subscribe to manual control data */
	int sp_man_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	memset(&sp_man, 0, sizeof(sp_man));

	/* Subscribe to offboard control data */
	int offboard_control_mode_sub = orb_subscribe(ORB_ID(offboard_control_mode));
	memset(&offboard_control_mode, 0, sizeof(offboard_control_mode));

	/* Subscribe to telemetry status topics */
	int telemetry_subs[ORB_MULTI_MAX_INSTANCES];
	uint64_t telemetry_last_heartbeat[ORB_MULTI_MAX_INSTANCES];
	uint64_t telemetry_last_dl_loss[ORB_MULTI_MAX_INSTANCES];
	bool telemetry_preflight_checks_reported[ORB_MULTI_MAX_INSTANCES];
	bool telemetry_lost[ORB_MULTI_MAX_INSTANCES];

	for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		telemetry_subs[i] = -1;
		telemetry_last_heartbeat[i] = 0;
		telemetry_last_dl_loss[i] = 0;
		telemetry_lost[i] = true;
		telemetry_preflight_checks_reported[i] = false;
	}

	/* Subscribe to global position */
	int global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	struct vehicle_global_position_s global_position;
	memset(&global_position, 0, sizeof(global_position));
	/* Init EPH and EPV */
	global_position.eph = 1000.0f;
	global_position.epv = 1000.0f;

	/* Subscribe to local position data */
	int local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	struct vehicle_local_position_s local_position = {};

	/* Subscribe to attitude data */
	int attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	struct vehicle_attitude_s attitude = {};

	/* Subscribe to land detector */
	int land_detector_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	struct vehicle_land_detected_s land_detector = {};
	land_detector.landed = true;

	/*
	 * The home position is set based on GPS only, to prevent a dependency between
	 * position estimator and commander. RAW GPS is more than good enough for a
	 * non-flying vehicle.
	 */

	/* Subscribe to GPS topic */
	int gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	struct vehicle_gps_position_s gps_position;
	memset(&gps_position, 0, sizeof(gps_position));
	gps_position.eph = FLT_MAX;
	gps_position.epv = FLT_MAX;

	/* Subscribe to sensor topic */
	int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	struct sensor_combined_s sensors;
	memset(&sensors, 0, sizeof(sensors));

	/* Subscribe to differential pressure topic */
	int diff_pres_sub = orb_subscribe(ORB_ID(differential_pressure));
	struct differential_pressure_s diff_pres;
	memset(&diff_pres, 0, sizeof(diff_pres));

	/* Subscribe to command topic */
	int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
	struct vehicle_command_s cmd;
	memset(&cmd, 0, sizeof(cmd));

	/* Subscribe to parameters changed topic */
	int param_changed_sub = orb_subscribe(ORB_ID(parameter_update));

	/* Subscribe to battery topic */
	int battery_sub = orb_subscribe(ORB_ID(battery_status));
	memset(&battery, 0, sizeof(battery));

	/* Subscribe to subsystem info topic */
	int subsys_sub = orb_subscribe(ORB_ID(subsystem_info));
	struct subsystem_info_s info;
	memset(&info, 0, sizeof(info));

	/* Subscribe to position setpoint triplet */
	int pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	struct position_setpoint_triplet_s pos_sp_triplet;
	memset(&pos_sp_triplet, 0, sizeof(pos_sp_triplet));

	/* Subscribe to system power */
	int system_power_sub = orb_subscribe(ORB_ID(system_power));
	struct system_power_s system_power;
	memset(&system_power, 0, sizeof(system_power));

	/* Subscribe to actuator controls (outputs) */
	int actuator_controls_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	struct actuator_controls_s actuator_controls;
	memset(&actuator_controls, 0, sizeof(actuator_controls));

	/* Subscribe to vtol vehicle status topic */
	int vtol_vehicle_status_sub = orb_subscribe(ORB_ID(vtol_vehicle_status));
	//struct vtol_vehicle_status_s vtol_status;
	memset(&vtol_status, 0, sizeof(vtol_status));
	vtol_status.vtol_in_rw_mode = true;		//default for vtol is rotary wing

	int cpuload_sub = orb_subscribe(ORB_ID(cpuload));
	memset(&cpuload, 0, sizeof(cpuload));

	control_status_leds(&status, &armed, true, &battery, &cpuload);

	/* now initialized */
	commander_initialized = true;
	thread_running = true;

	/* update vehicle status to find out vehicle type (required for preflight checks) */
	param_get(_param_sys_type, &(status.system_type)); // get system type
	status.is_rotary_wing = is_rotary_wing(&status) || is_vtol(&status);
	status.is_vtol = is_vtol(&status);

	bool checkAirspeed = false;
	/* Perform airspeed check only if circuit breaker is not
	 * engaged and it's not a rotary wing */
	if (!status_flags.circuit_breaker_engaged_airspd_check &&
	    (!status.is_rotary_wing || status.is_vtol)) {
		checkAirspeed = true;
	}

	// Run preflight check
	int32_t rc_in_off = 0;
	bool hotplug_timeout = hrt_elapsed_time(&commander_boot_timestamp) > HOTPLUG_SENS_TIMEOUT;
	int32_t arm_without_gps = 0;
	param_get(_param_autostart_id, &autostart_id);
	param_get(_param_rc_in_off, &rc_in_off);
	param_get(_param_arm_without_gps, &arm_without_gps);
	can_arm_without_gps = (arm_without_gps == 1);
	status.rc_input_mode = rc_in_off;
	if (is_hil_setup(autostart_id)) {
		// HIL configuration selected: real sensors will be disabled
		status_flags.condition_system_sensors_initialized = false;
		set_tune_override(TONE_STARTUP_TUNE); //normal boot tune
	} else {
			// sensor diagnostics done continuously, not just at boot so don't warn about any issues just yet
			status_flags.condition_system_sensors_initialized = Commander::preflightCheck(&mavlink_log_pub, true, true, true, true,
				checkAirspeed, (status.rc_input_mode == vehicle_status_s::RC_IN_MODE_DEFAULT), !status_flags.circuit_breaker_engaged_gpsfailure_check,
				/* checkDynamic */ false, is_vtol(&status), /* reportFailures */ false, /* prearm */ false, hrt_elapsed_time(&commander_boot_timestamp));
			set_tune_override(TONE_STARTUP_TUNE); //normal boot tune
	}

	// user adjustable duration required to assert arm/disarm via throttle/rudder stick
	int32_t rc_arm_hyst = 100;
	param_get(_param_rc_arm_hyst, &rc_arm_hyst);
	rc_arm_hyst *= COMMANDER_MONITORING_LOOPSPERMSEC;

	commander_boot_timestamp = hrt_absolute_time();

	transition_result_t arming_ret;

	int32_t datalink_loss_enabled = 0;
	int32_t rc_loss_enabled = 0;
	int32_t datalink_loss_timeout = 10;
	float rc_loss_timeout = 0.5;
	int32_t datalink_regain_timeout = 0;
	float offboard_loss_timeout = 0.0f;
	int32_t offboard_loss_act = 0;
	int32_t offboard_loss_rc_act = 0;

	int32_t geofence_action = 0;

	/* Thresholds for engine failure detection */
	int32_t ef_throttle_thres = 1.0f;
	int32_t ef_current2throttle_thres = 0.0f;
	int32_t ef_time_thres = 1000.0f;
	uint64_t timestamp_engine_healthy = 0; /**< absolute time when engine was healty */

	int autosave_params; /**< Autosave of parameters enabled/disabled, loaded from parameter */

	int32_t disarm_when_landed = 0;
	int32_t low_bat_action = 0;

	/* check which state machines for changes, clear "changed" flag */
	bool arming_state_changed = false;
	bool main_state_changed = false;
	bool failsafe_old = false;

	bool have_taken_off_since_arming = false;

	/* initialize low priority thread */
	pthread_attr_t commander_low_prio_attr;
	pthread_attr_init(&commander_low_prio_attr);
	pthread_attr_setstacksize(&commander_low_prio_attr, PX4_STACK_ADJUSTED(3000));

#ifndef __PX4_QURT
	// This is not supported by QURT (yet).
	struct sched_param param;
	(void)pthread_attr_getschedparam(&commander_low_prio_attr, &param);

	/* low priority */
	param.sched_priority = SCHED_PRIORITY_DEFAULT - 50;
	(void)pthread_attr_setschedparam(&commander_low_prio_attr, &param);
#endif

	pthread_create(&commander_low_prio_thread, &commander_low_prio_attr, commander_low_prio_loop, NULL);
	pthread_attr_destroy(&commander_low_prio_attr);

#ifdef UI_STRIVE
    _formation_sub = -1;

//    _formation_sub1 = -1;
//    _formation_sub2 = -1;
//    _formation_sub3 = -1;
//    _formation_sub4 = -1;
    _formation_status_pub = nullptr;
#endif
#ifdef UI_STRIVE

      memset(&formation, 0, sizeof(formation));
      memset(&formation1, 0, sizeof(formation1));
//    memset(&formation2, 0, sizeof(formation2));
//    memset(&formation3, 0, sizeof(formation3));
//    memset(&formation4, 0, sizeof(formation4));
//   memset(&formation_status, 0, sizeof(formation_status));
#endif



#ifdef UI_STRIVE
    if (_formation_sub < 0) {
        _formation_sub = orb_subscribe(ORB_ID(ui_strive_formation));//来自于mavlink这个线程
    }
//    if (_formation_sub1 < 0) {
//        _formation_sub1 = orb_subscribe_multi(ORB_ID(ui_strive_formation), Instance_MC_1);
//    }
//    if (_formation_sub2 < 0) {
//        _formation_sub2 = orb_subscribe_multi(ORB_ID(ui_strive_formation), Instance_MC_2);
//    }
//    if (_formation_sub3 < 0) {
//        _formation_sub3 = orb_subscribe_multi(ORB_ID(ui_strive_formation), Instance_MC_3);
//    }
//    if (_formation_sub4 < 0) {
//        _formation_sub4 = orb_subscribe_multi(ORB_ID(ui_strive_formation), Instance_MC_4);
//    }
#endif
#ifdef RECEIVE_STATUS
    if(return_flag_sub < 0)
    {
        return_flag_sub = orb_subscribe(ORB_ID(follow_to_commander));  //订阅这个消息－－来自于follow_target线程
    }
#endif


	while (!thread_should_exit) {

		arming_ret = TRANSITION_NOT_CHANGED;
#ifdef UI_STRIVE
        orb_check(_formation_sub, &updated);
        if(updated)
        {
            orb_copy(ORB_ID(ui_strive_formation), _formation_sub, &formation);
             if(formation.sysid == 1)
             {
                 formation1 = formation;

             }
//
            if(mavlink_system.sysid == formation.sysid + 1)
            {
               // formation_status.status = formation.status;
            }
        }
    #endif

		/* update parameters */
		orb_check(param_changed_sub, &updated);

		if (updated || param_init_forced) {

			/* parameters changed */
			struct parameter_update_s param_changed;
			orb_copy(ORB_ID(parameter_update), param_changed_sub, &param_changed);

			/* update parameters */
			if (!armed.armed) {
				if (param_get(_param_sys_type, &(status.system_type)) != OK) {
					warnx("failed getting new system type");
				}

				/* disable manual override for all systems that rely on electronic stabilization */
				if (is_rotary_wing(&status) || (is_vtol(&status) && vtol_status.vtol_in_rw_mode)) {
					status.is_rotary_wing = true;

				} else {
					status.is_rotary_wing = false;
				}

				/* set vehicle_status.is_vtol flag */
				status.is_vtol = is_vtol(&status);

				/* check and update system / component ID */
				param_get(_param_system_id, &(status.system_id));
				param_get(_param_component_id, &(status.component_id));

				get_circuit_breaker_params();

				status_changed = true;
			}

			/* Safety parameters */
			param_get(_param_enable_datalink_loss, &datalink_loss_enabled);
			param_get(_param_enable_rc_loss, &rc_loss_enabled);
			param_get(_param_datalink_loss_timeout, &datalink_loss_timeout);
			param_get(_param_rc_loss_timeout, &rc_loss_timeout);
			param_get(_param_rc_in_off, &rc_in_off);
			status.rc_input_mode = rc_in_off;
			param_get(_param_rc_arm_hyst, &rc_arm_hyst);
			rc_arm_hyst *= COMMANDER_MONITORING_LOOPSPERMSEC;
			param_get(_param_datalink_regain_timeout, &datalink_regain_timeout);
			param_get(_param_ef_throttle_thres, &ef_throttle_thres);
			param_get(_param_ef_current2throttle_thres, &ef_current2throttle_thres);
			param_get(_param_ef_time_thres, &ef_time_thres);
			param_get(_param_geofence_action, &geofence_action);
			param_get(_param_disarm_land, &disarm_when_landed);

			// If we update parameters the first time
			// make sure the hysteresis time gets set.
			// After that it will be set in the main state
			// machine based on the arming state.
			if (param_init_forced) {
				auto_disarm_hysteresis.set_hysteresis_time_from(false,
									(hrt_abstime)disarm_when_landed * 1000000);
			}

			param_get(_param_low_bat_act, &low_bat_action);
			param_get(_param_offboard_loss_timeout, &offboard_loss_timeout);
			param_get(_param_offboard_loss_act, &offboard_loss_act);
			param_get(_param_offboard_loss_rc_act, &offboard_loss_rc_act);
			param_get(_param_arm_without_gps, &arm_without_gps);
			can_arm_without_gps = (arm_without_gps == 1);

			/* Autostart id */
			param_get(_param_autostart_id, &autostart_id);

			/* Parameter autosave setting */
			param_get(_param_autosave_params, &autosave_params);

			/* EPH / EPV */
			param_get(_param_eph, &eph_threshold);
			param_get(_param_epv, &epv_threshold);

			/* flight mode slots */
			param_get(_param_fmode_1, &_flight_mode_slots[0]);
			param_get(_param_fmode_2, &_flight_mode_slots[1]);
			param_get(_param_fmode_3, &_flight_mode_slots[2]);
			param_get(_param_fmode_4, &_flight_mode_slots[3]);
			param_get(_param_fmode_5, &_flight_mode_slots[4]);
			param_get(_param_fmode_6, &_flight_mode_slots[5]);

			/* pre-flight EKF checks */
			param_get(_param_max_ekf_pos_ratio, &max_ekf_pos_ratio);
			param_get(_param_max_ekf_vel_ratio, &max_ekf_vel_ratio);
			param_get(_param_max_ekf_hgt_ratio, &max_ekf_hgt_ratio);
			param_get(_param_max_ekf_yaw_ratio, &max_ekf_yaw_ratio);
			param_get(_param_max_ekf_dvel_bias, &max_ekf_dvel_bias);
			param_get(_param_max_ekf_dang_bias, &max_ekf_dang_bias);

			/* pre-flight IMU consistency checks */
			param_get(_param_max_imu_acc_diff, &max_imu_acc_diff);
			param_get(_param_max_imu_gyr_diff, &max_imu_gyr_diff);

			param_init_forced = false;

			/* Set flag to autosave parameters if necessary */
			if (updated && autosave_params != 0 && param_changed.saved == false) {
				/* trigger an autosave */
				need_param_autosave = true;
			}
		}

		orb_check(sp_man_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(manual_control_setpoint), sp_man_sub, &sp_man);
		}

		orb_check(offboard_control_mode_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(offboard_control_mode), offboard_control_mode_sub, &offboard_control_mode);
		}

		if (offboard_control_mode.timestamp != 0 &&
		    offboard_control_mode.timestamp + OFFBOARD_TIMEOUT > hrt_absolute_time()) {
			if (status_flags.offboard_control_signal_lost) {
				status_flags.offboard_control_signal_lost = false;
				status_flags.offboard_control_loss_timeout = false;
				status_changed = true;
			}

		} else {
			if (!status_flags.offboard_control_signal_lost) {
				status_flags.offboard_control_signal_lost = true;
				status_changed = true;
			}

			/* check timer if offboard was there but now lost */
			if (!status_flags.offboard_control_loss_timeout && offboard_control_mode.timestamp != 0) {
				if (offboard_loss_timeout < FLT_EPSILON) {
					/* execute loss action immediately */
					status_flags.offboard_control_loss_timeout = true;

				} else {
					/* wait for timeout if set */
					status_flags.offboard_control_loss_timeout = offboard_control_mode.timestamp +
						OFFBOARD_TIMEOUT + offboard_loss_timeout * 1e6f < hrt_absolute_time();
				}

				if (status_flags.offboard_control_loss_timeout) {
					status_changed = true;
				}
			}
		}

		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {

			if (telemetry_subs[i] < 0 && (OK == orb_exists(ORB_ID(telemetry_status), i))) {
				telemetry_subs[i] = orb_subscribe_multi(ORB_ID(telemetry_status), i);
			}

			orb_check(telemetry_subs[i], &updated);

			if (updated) {
				struct telemetry_status_s telemetry;
				memset(&telemetry, 0, sizeof(telemetry));

				orb_copy(ORB_ID(telemetry_status), telemetry_subs[i], &telemetry);

				/* perform system checks when new telemetry link connected */
				if (/* we first connect a link or re-connect a link after loosing it or haven't yet reported anything */
				    (telemetry_last_heartbeat[i] == 0 || (hrt_elapsed_time(&telemetry_last_heartbeat[i]) > 3 * 1000 * 1000)
				        || !telemetry_preflight_checks_reported[i]) &&
				    /* and this link has a communication partner */
				    (telemetry.heartbeat_time > 0) &&
				    /* and it is still connected */
				    (hrt_elapsed_time(&telemetry.heartbeat_time) < 2 * 1000 * 1000) &&
				    /* and the system is not already armed (and potentially flying) */
				    !armed.armed) {

					hotplug_timeout = hrt_elapsed_time(&commander_boot_timestamp) > HOTPLUG_SENS_TIMEOUT;
					/* flag the checks as reported for this link when we actually report them */
					telemetry_preflight_checks_reported[i] = hotplug_timeout;

					/* provide RC and sensor status feedback to the user */
					if (is_hil_setup(autostart_id)) {
						/* HIL configuration: check only RC input */
						(void)Commander::preflightCheck(&mavlink_log_pub, false, false, false, false, false,
								(status.rc_input_mode == vehicle_status_s::RC_IN_MODE_DEFAULT), false,
								 /* checkDynamic */ true, is_vtol(&status), /* reportFailures */ false, /* prearm */ false, hrt_elapsed_time(&commander_boot_timestamp));
					} else {
						/* check sensors also */
						(void)Commander::preflightCheck(&mavlink_log_pub, true, true, true, true, checkAirspeed,
								(status.rc_input_mode == vehicle_status_s::RC_IN_MODE_DEFAULT), !status_flags.circuit_breaker_engaged_gpsfailure_check,
								 /* checkDynamic */ true, is_vtol(&status), /* reportFailures */ hotplug_timeout, /* prearm */ false, hrt_elapsed_time(&commander_boot_timestamp));
					}
				}

				/* set (and don't reset) telemetry via USB as active once a MAVLink connection is up */
				if (telemetry.type == telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_USB) {
					_usb_telemetry_active = true;
				}

				if (telemetry.heartbeat_time > 0) {
					telemetry_last_heartbeat[i] = telemetry.heartbeat_time;
				}
			}
		}

		orb_check(sensor_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensors);

			/* Check if the barometer is healthy and issue a warning in the GCS if not so.
			 * Because the barometer is used for calculating AMSL altitude which is used to ensure
			 * vertical separation from other airtraffic the operator has to know when the
			 * barometer is inoperational.
			 * */
			hrt_abstime baro_timestamp = sensors.timestamp + sensors.baro_timestamp_relative;
			if (hrt_elapsed_time(&baro_timestamp) < FAILSAFE_DEFAULT_TIMEOUT) {
				/* handle the case where baro was regained */
				if (status_flags.barometer_failure) {
					status_flags.barometer_failure = false;
					status_changed = true;
					if (status_flags.ever_had_barometer_data) {
						mavlink_log_critical(&mavlink_log_pub, "baro healthy");
					}
					status_flags.ever_had_barometer_data = true;
				}

			} else {
				if (!status_flags.barometer_failure) {
					status_flags.barometer_failure = true;
					status_changed = true;
					mavlink_log_critical(&mavlink_log_pub, "baro failed");
				}
			}
		}

		orb_check(diff_pres_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(differential_pressure), diff_pres_sub, &diff_pres);
		}

		orb_check(system_power_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(system_power), system_power_sub, &system_power);

			if (hrt_elapsed_time(&system_power.timestamp) < 200000) {
				if (system_power.servo_valid &&
				    !system_power.brick_valid &&
				    !system_power.usb_connected) {
					/* flying only on servo rail, this is unsafe */
					status_flags.condition_power_input_valid = false;

				} else {
					status_flags.condition_power_input_valid = true;
				}

				/* copy avionics voltage */
				avionics_power_rail_voltage = system_power.voltage5V_v;

				/* if the USB hardware connection went away, reboot */
				if (status_flags.usb_connected && !system_power.usb_connected) {
					/*
					 * apparently the USB cable went away but we are still powered,
					 * so lets reset to a classic non-usb state.
					 */
					mavlink_log_critical(&mavlink_log_pub, "USB disconnected, rebooting.")
					usleep(400000);
					px4_systemreset(false);
				}

				/* finally judge the USB connected state based on software detection */
				status_flags.usb_connected = _usb_telemetry_active;
			}
		}

		check_valid(diff_pres.timestamp, DIFFPRESS_TIMEOUT, true, &(status_flags.condition_airspeed_valid), &status_changed);

		/* update safety topic */
		orb_check(safety_sub, &updated);

		if (updated) {
			bool previous_safety_off = safety.safety_off;
			orb_copy(ORB_ID(safety), safety_sub, &safety);

			/* disarm if safety is now on and still armed */
			if (status.hil_state == vehicle_status_s::HIL_STATE_OFF && safety.safety_switch_available && !safety.safety_off && armed.armed) {
				arming_state_t new_arming_state = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED ? vehicle_status_s::ARMING_STATE_STANDBY :
								   vehicle_status_s::ARMING_STATE_STANDBY_ERROR);

				if (TRANSITION_CHANGED == arming_state_transition(&status,
										  &battery,
										  &safety,
										  new_arming_state,
										  &armed,
										  true /* fRunPreArmChecks */,
										  &mavlink_log_pub,
										  &status_flags,
										  avionics_power_rail_voltage,
										  can_arm_without_gps,
										  hrt_elapsed_time(&commander_boot_timestamp))) {
					mavlink_log_info(&mavlink_log_pub, "DISARMED by safety switch");
					arming_state_changed = true;
				}
			}

			//Notify the user if the status of the safety switch changes
			if (safety.safety_switch_available && previous_safety_off != safety.safety_off) {

				if (safety.safety_off) {
					set_tune(TONE_NOTIFY_POSITIVE_TUNE);

				} else {
					tune_neutral(true);
				}

				status_changed = true;
			}
		}

		/* update vtol vehicle status*/
		orb_check(vtol_vehicle_status_sub, &updated);

		if (updated) {
			/* vtol status changed */
			orb_copy(ORB_ID(vtol_vehicle_status), vtol_vehicle_status_sub, &vtol_status);
			status.vtol_fw_permanent_stab = vtol_status.fw_permanent_stab;

			/* Make sure that this is only adjusted if vehicle really is of type vtol */
			if (is_vtol(&status)) {
				status.is_rotary_wing = vtol_status.vtol_in_rw_mode;
				status.in_transition_mode = vtol_status.vtol_in_trans_mode;
				status.in_transition_to_fw = vtol_status.in_transition_to_fw;
				status_flags.vtol_transition_failure = vtol_status.vtol_transition_failsafe;
				status_flags.vtol_transition_failure_cmd = vtol_status.vtol_transition_failsafe;
			}

			status_changed = true;
		}

		/* update global position estimate */
		orb_check(global_position_sub, &updated);

		if (updated) {
			/* position changed */
			vehicle_global_position_s gpos;
			orb_copy(ORB_ID(vehicle_global_position), global_position_sub, &gpos);

			/* copy to global struct if valid, with hysteresis */

			// XXX consolidate this with local position handling and timeouts after release
			// but we want a low-risk change now.
			if (status_flags.condition_global_position_valid) {
				if (gpos.eph < eph_threshold * 2.5f) {
					orb_copy(ORB_ID(vehicle_global_position), global_position_sub, &global_position);
				}
			} else {
				if (gpos.eph < eph_threshold) {
					orb_copy(ORB_ID(vehicle_global_position), global_position_sub, &global_position);
				}
			}
		}

		/* update local position estimate */
		orb_check(local_position_sub, &updated);

		if (updated) {
			/* position changed */
			orb_copy(ORB_ID(vehicle_local_position), local_position_sub, &local_position);
		}

		/* update attitude estimate */
		orb_check(attitude_sub, &updated);

		if (updated) {
			/* position changed */
			orb_copy(ORB_ID(vehicle_attitude), attitude_sub, &attitude);
		}

		//update condition_global_position_valid
		//Global positions are only published by the estimators if they are valid
		if (hrt_absolute_time() - global_position.timestamp > POSITION_TIMEOUT) {
			//We have had no good fix for POSITION_TIMEOUT amount of time
			if (status_flags.condition_global_position_valid) {
				set_tune_override(TONE_GPS_WARNING_TUNE);
				status_changed = true;
				status_flags.condition_global_position_valid = false;
			}
		} else if (global_position.timestamp != 0) {
			// Got good global position estimate
			if (!status_flags.condition_global_position_valid) {
				status_changed = true;
				status_flags.condition_global_position_valid = true;
			}
		}

		/* update condition_local_position_valid and condition_local_altitude_valid */
		/* hysteresis for EPH */
		bool local_eph_good;

		if (status_flags.condition_local_position_valid) {
			if (local_position.eph > eph_threshold * 2.5f) {
				local_eph_good = false;

			} else {
				local_eph_good = true;
			}

		} else {
			if (local_position.eph < eph_threshold) {
				local_eph_good = true;

			} else {
				local_eph_good = false;
			}
		}

		check_valid(local_position.timestamp, POSITION_TIMEOUT, local_position.xy_valid
			    && local_eph_good, &(status_flags.condition_local_position_valid), &status_changed);
		check_valid(local_position.timestamp, POSITION_TIMEOUT, local_position.z_valid,
			    &(status_flags.condition_local_altitude_valid), &status_changed);

		/* Update land detector */
		orb_check(land_detector_sub, &updated);
		if (updated) {
			orb_copy(ORB_ID(vehicle_land_detected), land_detector_sub, &land_detector);

			if (was_landed != land_detector.landed) {
				if (land_detector.landed) {
					mavlink_and_console_log_info(&mavlink_log_pub, "Landing detected");
				} else {
					mavlink_and_console_log_info(&mavlink_log_pub, "Takeoff detected");
					have_taken_off_since_arming = true;
				}
			}

			if (was_falling != land_detector.freefall) {
				if (land_detector.freefall) {
					mavlink_and_console_log_info(&mavlink_log_pub, "Freefall detected");
				}
			}


			was_landed = land_detector.landed;
			was_falling = land_detector.freefall;
		}



		/* Update hysteresis time. Use a time of factor 5 longer if we have not taken off yet. */
		hrt_abstime timeout_time = disarm_when_landed * 1000000;

		if (!have_taken_off_since_arming) {
			timeout_time *= 5;
		}

		auto_disarm_hysteresis.set_hysteresis_time_from(false, timeout_time);

		// Check for auto-disarm
		if (armed.armed && land_detector.landed && disarm_when_landed > 0) {
			auto_disarm_hysteresis.set_state_and_update(true);
		} else {
			auto_disarm_hysteresis.set_state_and_update(false);
		}

		if (auto_disarm_hysteresis.get_state()) {
			arm_disarm(false, &mavlink_log_pub, "auto disarm on land");
		}

		if (!warning_action_on) {
			// store the last good main_state when not in an navigation
			// hold state
			main_state_before_rtl = internal_state.main_state;

		} else if (internal_state.main_state != commander_state_s::MAIN_STATE_AUTO_RTL
			&& internal_state.main_state != commander_state_s::MAIN_STATE_AUTO_LOITER
			&& internal_state.main_state != commander_state_s::MAIN_STATE_AUTO_LAND) {
			// reset flag again when we switched out of it
			warning_action_on = false;
		}

		orb_check(cpuload_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(cpuload), cpuload_sub, &cpuload);
		}

		/* update battery status */
		orb_check(battery_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(battery_status), battery_sub, &battery);

			/* only consider battery voltage if system has been running 6s (usb most likely detected) and battery voltage is valid */
			if (hrt_absolute_time() > commander_boot_timestamp + 6000000
			    && battery.voltage_filtered_v > 2.0f * FLT_EPSILON) {

				/* if battery voltage is getting lower, warn using buzzer, etc. */
				if (battery.warning == battery_status_s::BATTERY_WARNING_LOW &&
				   !low_battery_voltage_actions_done) {
					low_battery_voltage_actions_done = true;
					if (armed.armed) {
						mavlink_log_critical(&mavlink_log_pub, "LOW BATTERY, RETURN TO LAND ADVISED");
					} else {
						mavlink_log_critical(&mavlink_log_pub, "LOW BATTERY, TAKEOFF DISCOURAGED");
					}

				} else if (!status_flags.usb_connected &&
					   battery.warning == battery_status_s::BATTERY_WARNING_CRITICAL &&
					   !critical_battery_voltage_actions_done) {
					critical_battery_voltage_actions_done = true;

					if (!armed.armed) {
						mavlink_log_critical(&mavlink_log_pub, "CRITICAL BATTERY, SHUT SYSTEM DOWN");

					} else {
						if (low_bat_action == 1) {
							// let us send the critical message even if already in RTL
							if (warning_action_on || TRANSITION_CHANGED == main_state_transition(&status, commander_state_s::MAIN_STATE_AUTO_RTL, main_state_prev, &status_flags, &internal_state)) {
								warning_action_on = true;
								mavlink_log_emergency(&mavlink_log_pub, "CRITICAL BATTERY, RETURNING TO LAND");

							} else {
								mavlink_log_emergency(&mavlink_log_pub, "CRITICAL BATTERY, RTL FAILED");
							}

						} else if (low_bat_action == 2) {
							if (warning_action_on || TRANSITION_CHANGED == main_state_transition(&status, commander_state_s::MAIN_STATE_AUTO_LAND, main_state_prev, &status_flags, &internal_state)) {
								warning_action_on = true;
								mavlink_log_emergency(&mavlink_log_pub, "CRITICAL BATTERY, LANDING AT CURRENT POSITION");

							} else {
								mavlink_log_emergency(&mavlink_log_pub, "CRITICAL BATTERY, LANDING FAILED");
							}

						} else {
							mavlink_log_emergency(&mavlink_log_pub, "CRITICAL BATTERY, LANDING ADVISED!");
						}
					}

					status_changed = true;
				}

				/* End battery voltage check */
			}
		}

		/* update subsystem */
		orb_check(subsys_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(subsystem_info), subsys_sub, &info);

			//warnx("subsystem changed: %d\n", (int)info.subsystem_type);

			/* mark / unmark as present */
			if (info.present) {
				status.onboard_control_sensors_present |= info.subsystem_type;

			} else {
				status.onboard_control_sensors_present &= ~info.subsystem_type;
			}

			/* mark / unmark as enabled */
			if (info.enabled) {
				status.onboard_control_sensors_enabled |= info.subsystem_type;

			} else {
				status.onboard_control_sensors_enabled &= ~info.subsystem_type;
			}

			/* mark / unmark as ok */
			if (info.ok) {
				status.onboard_control_sensors_health |= info.subsystem_type;

			} else {
				status.onboard_control_sensors_health &= ~info.subsystem_type;
			}

			status_changed = true;
		}

		/* update position setpoint triplet */
		orb_check(pos_sp_triplet_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(position_setpoint_triplet), pos_sp_triplet_sub, &pos_sp_triplet);
		}

		/* If in INIT state, try to proceed to STANDBY state */
		if (!status_flags.condition_calibration_enabled && status.arming_state == vehicle_status_s::ARMING_STATE_INIT) {
			arming_ret = arming_state_transition(&status,
							     &battery,
							     &safety,
							     vehicle_status_s::ARMING_STATE_STANDBY,
							     &armed,
							     true /* fRunPreArmChecks */,
							     &mavlink_log_pub,
							     &status_flags,
							     avionics_power_rail_voltage,
							     can_arm_without_gps,
							     hrt_elapsed_time(&commander_boot_timestamp));

			if (arming_ret == TRANSITION_CHANGED) {
				arming_state_changed = true;
			} else if (arming_ret == TRANSITION_DENIED) {
				/* do not complain if not allowed into standby */
				arming_ret = TRANSITION_NOT_CHANGED;
			}

		}


		/*
		 * Check for valid position information.
		 *
		 * If the system has a valid position source from an onboard
		 * position estimator, it is safe to operate it autonomously.
		 * The flag_vector_flight_mode_ok flag indicates that a minimum
		 * set of position measurements is available.
		 */

		orb_check(gps_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(vehicle_gps_position), gps_sub, &gps_position);
		}

		/* Initialize map projection if gps is valid */
		if (!map_projection_global_initialized()
		    && (gps_position.eph < eph_threshold)
		    && (gps_position.epv < epv_threshold)
		    && hrt_elapsed_time((hrt_abstime *)&gps_position.timestamp) < 1e6) {
			/* set reference for global coordinates <--> local coordiantes conversion and map_projection */
			globallocalconverter_init((double)gps_position.lat * 1.0e-7, (double)gps_position.lon * 1.0e-7,
						  (float)gps_position.alt * 1.0e-3f, hrt_absolute_time());
		}

		/* check if GPS is ok */
		if (!status_flags.circuit_breaker_engaged_gpsfailure_check) {
			bool gpsIsNoisy = gps_position.noise_per_ms > 0 && gps_position.noise_per_ms < COMMANDER_MAX_GPS_NOISE;

			//Check if GPS receiver is too noisy while we are disarmed
			if (!armed.armed && gpsIsNoisy) {
				if (!status_flags.gps_failure) {
					mavlink_log_critical(&mavlink_log_pub, "GPS signal noisy");
					set_tune_override(TONE_GPS_WARNING_TUNE);

					//GPS suffers from signal jamming or excessive noise, disable GPS-aided flight
					status_flags.gps_failure = true;
					status_changed = true;
				}
			}
       // PX4_INFO("gps_position.timestamp: %llu",gps_position.timestamp);
			if (gps_position.fix_type >= 3 && hrt_elapsed_time(&gps_position.timestamp) < FAILSAFE_DEFAULT_TIMEOUT) {
				/* handle the case where gps was regained */
				if (status_flags.gps_failure && !gpsIsNoisy) {
					status_flags.gps_failure = false;
					status_changed = true;
					if (status_flags.condition_home_position_valid) {
						mavlink_log_critical(&mavlink_log_pub, "GPS fix regained");
                        PX4_INFO("GPS fix regained");
					}
				}

			} else if (!status_flags.gps_failure) {
				status_flags.gps_failure = true;
				status_changed = true;
				mavlink_log_critical(&mavlink_log_pub, "GPS fix lost");
                PX4_INFO("GPS fix lost");
			}
		}

		/* start mission result check */
		orb_check(mission_result_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(mission_result), mission_result_sub, &_mission_result);

			if (status.mission_failure != _mission_result.mission_failure) {
				status.mission_failure = _mission_result.mission_failure;
				status_changed = true;

				if (status.mission_failure) {
					mavlink_log_critical(&mavlink_log_pub, "mission cannot be completed");
				}
			}
		}

		/* start geofence result check */
		orb_check(geofence_result_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(geofence_result), geofence_result_sub, &geofence_result);
		}

		// Geofence actions
		if (armed.armed && (geofence_result.geofence_action != geofence_result_s::GF_ACTION_NONE)) {

			static bool geofence_loiter_on = false;
			static bool geofence_rtl_on = false;

			// check for geofence violation
			if (geofence_result.geofence_violated) {
				static hrt_abstime last_geofence_violation = 0;
				const hrt_abstime geofence_violation_action_interval = 10000000; // 10 seconds
				if (hrt_elapsed_time(&last_geofence_violation) > geofence_violation_action_interval) {

					last_geofence_violation = hrt_absolute_time();

					switch (geofence_result.geofence_action) {
						case (geofence_result_s::GF_ACTION_NONE) : {
							// do nothing
							break;
						}
						case (geofence_result_s::GF_ACTION_WARN) : {
							// do nothing, mavlink critical messages are sent by navigator
							break;
						}
						case (geofence_result_s::GF_ACTION_LOITER) : {
							if (TRANSITION_CHANGED == main_state_transition(&status, commander_state_s::MAIN_STATE_AUTO_LOITER, main_state_prev, &status_flags, &internal_state)) {
								geofence_loiter_on = true;
							}
							break;
						}
						case (geofence_result_s::GF_ACTION_RTL) : {
							if (TRANSITION_CHANGED == main_state_transition(&status, commander_state_s::MAIN_STATE_AUTO_RTL, main_state_prev, &status_flags, &internal_state)) {
								geofence_rtl_on = true;
							}
							break;
						}
						case (geofence_result_s::GF_ACTION_TERMINATE) : {
							warnx("Flight termination because of geofence");
							mavlink_log_critical(&mavlink_log_pub, "Geofence violation: flight termination");
							armed.force_failsafe = true;
							status_changed = true;
							break;
						}
					}
				}
			}

			// reset if no longer in LOITER or if manually switched to LOITER
			geofence_loiter_on = geofence_loiter_on
									&& (internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LOITER)
									&& (sp_man.loiter_switch == manual_control_setpoint_s::SWITCH_POS_OFF
										|| sp_man.loiter_switch == manual_control_setpoint_s::SWITCH_POS_NONE);

			// reset if no longer in RTL or if manually switched to RTL
			geofence_rtl_on = geofence_rtl_on
								&& (internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_RTL)
								&& (sp_man.return_switch == manual_control_setpoint_s::SWITCH_POS_OFF
									|| sp_man.return_switch == manual_control_setpoint_s::SWITCH_POS_NONE);

			warning_action_on = warning_action_on || (geofence_loiter_on || geofence_rtl_on);
		}

		// revert geofence failsafe transition if sticks are moved and we were previously in MANUAL or ASSIST
		if (warning_action_on &&
		   (main_state_before_rtl == commander_state_s::MAIN_STATE_MANUAL ||
			main_state_before_rtl == commander_state_s::MAIN_STATE_ALTCTL ||
			main_state_before_rtl == commander_state_s::MAIN_STATE_POSCTL ||
			main_state_before_rtl == commander_state_s::MAIN_STATE_ACRO ||
			main_state_before_rtl == commander_state_s::MAIN_STATE_RATTITUDE ||
			main_state_before_rtl == commander_state_s::MAIN_STATE_STAB)) {

			// transition to previous state if sticks are increased
			const float min_stick_change = 0.2f;
			if ((_last_sp_man.timestamp != sp_man.timestamp) &&
				((fabsf(sp_man.x) - fabsf(_last_sp_man.x) > min_stick_change) ||
				 (fabsf(sp_man.y) - fabsf(_last_sp_man.y) > min_stick_change) ||
				 (fabsf(sp_man.z) - fabsf(_last_sp_man.z) > min_stick_change) ||
				 (fabsf(sp_man.r) - fabsf(_last_sp_man.r) > min_stick_change))) {

				main_state_transition(&status, main_state_before_rtl, main_state_prev, &status_flags, &internal_state);
			}
		}


		/* Check for mission flight termination */
		if (armed.armed && _mission_result.flight_termination &&
		    !status_flags.circuit_breaker_flight_termination_disabled) {

			armed.force_failsafe = true;
			status_changed = true;
			static bool flight_termination_printed = false;

			if (!flight_termination_printed) {
				mavlink_log_critical(&mavlink_log_pub, "Geofence violation: flight termination");
				flight_termination_printed = true;
			}

			if (counter % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0) {
				mavlink_log_critical(&mavlink_log_pub, "Flight termination active");
			}
		}

		/* Only evaluate mission state if home is set,
		 * this prevents false positives for the mission
		 * rejection. Back off 2 seconds to not overlay
		 * home tune.
		 */
		if (status_flags.condition_home_position_valid &&
			(hrt_elapsed_time(&_home.timestamp) > 2000000) &&
			_last_mission_instance != _mission_result.instance_count) {
			if (!_mission_result.valid) {
				/* the mission is invalid */
				tune_mission_fail(true);
				warnx("mission fail");
			} else if (_mission_result.warning) {
				/* the mission has a warning */
				tune_mission_fail(true);
				warnx("mission warning");
			} else {
				/* the mission is valid */
				tune_mission_ok(true);
			}

			/* prevent further feedback until the mission changes */
			_last_mission_instance = _mission_result.instance_count;
		}

		/* RC input check */
		if (!status_flags.rc_input_blocked && sp_man.timestamp != 0 &&
		    (hrt_absolute_time() < sp_man.timestamp + (uint64_t)(rc_loss_timeout * 1e6f))) {
			/* handle the case where RC signal was regained */
			if (!status_flags.rc_signal_found_once) {
				status_flags.rc_signal_found_once = true;
				status_changed = true;

			} else {
				if (status.rc_signal_lost) {
					mavlink_log_info(&mavlink_log_pub, "MANUAL CONTROL REGAINED after %llums",
							     (hrt_absolute_time() - rc_signal_lost_timestamp) / 1000);
					status_changed = true;
				}
			}

			status.rc_signal_lost = false;

			/* check if left stick is in lower left position and we are in MANUAL, Rattitude, or AUTO_READY mode or (ASSIST mode and landed) -> disarm
			 * do it only for rotary wings in manual mode or fixed wing if landed */
			if ((status.is_rotary_wing || (!status.is_rotary_wing && land_detector.landed)) && status.rc_input_mode != vehicle_status_s::RC_IN_MODE_OFF &&
			    (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED || status.arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR) &&
			    (internal_state.main_state == commander_state_s::MAIN_STATE_MANUAL ||
			    	internal_state.main_state == commander_state_s::MAIN_STATE_ACRO ||
			    	internal_state.main_state == commander_state_s::MAIN_STATE_STAB ||
			    	internal_state.main_state == commander_state_s::MAIN_STATE_RATTITUDE ||
			    	land_detector.landed) &&
			    sp_man.r < -STICK_ON_OFF_LIMIT && sp_man.z < 0.1f) {

				if (stick_off_counter > rc_arm_hyst) {
					/* disarm to STANDBY if ARMED or to STANDBY_ERROR if ARMED_ERROR */
					arming_state_t new_arming_state = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED ? vehicle_status_s::ARMING_STATE_STANDBY :
									   vehicle_status_s::ARMING_STATE_STANDBY_ERROR);
					arming_ret = arming_state_transition(&status,
									     &battery,
									     &safety,
									     new_arming_state,
									     &armed,
									     true /* fRunPreArmChecks */,
									     &mavlink_log_pub,
									     &status_flags,
									     avionics_power_rail_voltage,
									     can_arm_without_gps,
									     hrt_elapsed_time(&commander_boot_timestamp));

					if (arming_ret == TRANSITION_CHANGED) {
						arming_state_changed = true;
					}

					stick_off_counter = 0;

				} else {
					stick_off_counter++;
				}

			} else {
				stick_off_counter = 0;
			}

			/* check if left stick is in lower right position and we're in MANUAL mode -> arm */
			if (sp_man.r > STICK_ON_OFF_LIMIT && sp_man.z < 0.1f && status.rc_input_mode != vehicle_status_s::RC_IN_MODE_OFF ) {
				if (stick_on_counter > rc_arm_hyst) {

					/* we check outside of the transition function here because the requirement
					 * for being in manual mode only applies to manual arming actions.
					 * the system can be armed in auto if armed via the GCS.
					 */

					if ((internal_state.main_state != commander_state_s::MAIN_STATE_MANUAL)
						&& (internal_state.main_state != commander_state_s::MAIN_STATE_ACRO)
						&& (internal_state.main_state != commander_state_s::MAIN_STATE_STAB)
						&& (internal_state.main_state != commander_state_s::MAIN_STATE_ALTCTL)
						&& (internal_state.main_state != commander_state_s::MAIN_STATE_POSCTL)
						&& (internal_state.main_state != commander_state_s::MAIN_STATE_RATTITUDE)
						) {
						print_reject_arm("NOT ARMING: Switch to a manual mode first.");

					} else if (!status_flags.condition_home_position_valid &&
								geofence_action == geofence_result_s::GF_ACTION_RTL) {
						print_reject_arm("NOT ARMING: Geofence RTL requires valid home");

					} else if (status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
						arming_ret = arming_state_transition(&status,
										     &battery,
										     &safety,
										     vehicle_status_s::ARMING_STATE_ARMED,
										     &armed,
										     true /* fRunPreArmChecks */,
										     &mavlink_log_pub,
										     &status_flags,
										     avionics_power_rail_voltage,
										     can_arm_without_gps,
										     hrt_elapsed_time(&commander_boot_timestamp));

						if (arming_ret == TRANSITION_CHANGED) {
							arming_state_changed = true;
						} else {
							usleep(100000);
							print_reject_arm("NOT ARMING: Preflight checks failed");
						}
					}
					stick_on_counter = 0;

				} else {
					stick_on_counter++;
				}

			} else {
				stick_on_counter = 0;
			}

			if (arming_ret == TRANSITION_CHANGED) {
				if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
					mavlink_log_info(&mavlink_log_pub, "ARMED by RC");

				} else {
					mavlink_log_info(&mavlink_log_pub, "DISARMED by RC");
				}

				arming_state_changed = true;

			} else if (arming_ret == TRANSITION_DENIED) {
				/*
				 * the arming transition can be denied to a number of reasons:
				 *  - pre-flight check failed (sensors not ok or not calibrated)
				 *  - safety not disabled
				 *  - system not in manual mode
				 */
				tune_negative(true);
			}

			/* evaluate the main state machine according to mode switches */
			bool first_rc_eval = (_last_sp_man.timestamp == 0) && (sp_man.timestamp > 0);
			transition_result_t main_res = set_main_state_rc(&status);

			/* play tune on mode change only if armed, blink LED always */
			if (main_res == TRANSITION_CHANGED || first_rc_eval) {
				tune_positive(armed.armed);
				main_state_changed = true;

			} else if (main_res == TRANSITION_DENIED) {
				/* DENIED here indicates bug in the commander */
				mavlink_log_critical(&mavlink_log_pub, "main state transition denied");
			}

			/* check throttle kill switch */
#ifdef emergency_stop
            if (sp_man.kill_switch == manual_control_setpoint_s::SWITCH_POS_ON ||(mavlink_system.sysid != 1 && formation1.kill == 1)) {
#else
            if (sp_man.kill_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
#endif

				/* set lockdown flag */
				if (!armed.lockdown) {
					mavlink_log_emergency(&mavlink_log_pub, "MANUAL KILL SWITCH ENGAGED");
				}
				armed.lockdown = true;
#ifdef emergency_stop
            } else if (sp_man.kill_switch == manual_control_setpoint_s::SWITCH_POS_OFF && (mavlink_system.sysid == 1 || formation1.kill == 0)) {
#else
            } else if (sp_man.kill_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
#endif
                if (armed.lockdown) {
					mavlink_log_emergency(&mavlink_log_pub, "MANUAL KILL SWITCH OFF");
				}
				armed.lockdown = false;
			}
			/* no else case: do not change lockdown flag in unconfigured case */

		} else {
			if (!status_flags.rc_input_blocked && !status.rc_signal_lost) {
				mavlink_log_critical(&mavlink_log_pub, "MANUAL CONTROL LOST (at t=%llums)", hrt_absolute_time() / 1000);
				status.rc_signal_lost = true;
				rc_signal_lost_timestamp = sp_man.timestamp;
				status_changed = true;
			}
		}

		/* data links check */
		bool have_link = false;

		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			if (telemetry_last_heartbeat[i] != 0 &&
			    hrt_elapsed_time(&telemetry_last_heartbeat[i]) < datalink_loss_timeout * 1e6) {
				/* handle the case where data link was gained first time or regained,
				 * accept datalink as healthy only after datalink_regain_timeout seconds
				 * */
				if (telemetry_lost[i] &&
				    hrt_elapsed_time(&telemetry_last_dl_loss[i]) > datalink_regain_timeout * 1e6) {

					/* report a regain */
					if (telemetry_last_dl_loss[i] > 0) {
						mavlink_and_console_log_info(&mavlink_log_pub, "data link #%i regained", i);
					} else if (telemetry_last_dl_loss[i] == 0) {
						/* new link */
					}

					/* got link again or new */
					status_flags.condition_system_prearm_error_reported = false;
					status_changed = true;

					telemetry_lost[i] = false;
					have_link = true;

				} else if (!telemetry_lost[i]) {
					/* telemetry was healthy also in last iteration
					 * we don't have to check a timeout */
					have_link = true;
				}

			} else {

				if (!telemetry_lost[i]) {
					/* only reset the timestamp to a different time on state change */
					telemetry_last_dl_loss[i]  = hrt_absolute_time();

					mavlink_and_console_log_info(&mavlink_log_pub, "data link #%i lost", i);
					telemetry_lost[i] = true;
				}
			}
		}

		if (have_link) {
			/* handle the case where data link was regained */
			if (status.data_link_lost) {
				status.data_link_lost = false;
				status_changed = true;
			}

		} else {
			if (!status.data_link_lost) {
				if (armed.armed) {
					mavlink_log_critical(&mavlink_log_pub, "ALL DATA LINKS LOST");
				}
				status.data_link_lost = true;
				status.data_link_lost_counter++;
				status_changed = true;
			}
		}

		/* handle commands last, as the system needs to be updated to handle them */
		orb_check(actuator_controls_sub, &updated);

		if (updated) {
			/* got command */
			orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_controls_sub, &actuator_controls);

			/* Check engine failure
			 * only for fixed wing for now
			 */
			if (!status_flags.circuit_breaker_engaged_enginefailure_check &&
			    status.is_rotary_wing == false &&
			    armed.armed &&
			    ((actuator_controls.control[3] > ef_throttle_thres &&
			      battery.current_a / actuator_controls.control[3] <
			      ef_current2throttle_thres) ||
			     (status.engine_failure))) {
				/* potential failure, measure time */
				if (timestamp_engine_healthy > 0 &&
				    hrt_elapsed_time(&timestamp_engine_healthy) >
				    ef_time_thres * 1e6 &&
				    !status.engine_failure) {
					status.engine_failure = true;
					status_changed = true;
					mavlink_log_critical(&mavlink_log_pub, "Engine Failure");
				}

			} else {
				/* no failure reset flag */
				timestamp_engine_healthy = hrt_absolute_time();

				if (status.engine_failure) {
					status.engine_failure = false;
					status_changed = true;
				}
			}
		}

		/* reset main state after takeoff has completed */
		/* only switch back to posctl */
		if (main_state_prev == commander_state_s::MAIN_STATE_POSCTL) {

			if (internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_TAKEOFF
					&& _mission_result.finished) {

				main_state_transition(&status, main_state_prev, main_state_prev, &status_flags, &internal_state);
			}
		}

		/* handle commands last, as the system needs to be updated to handle them */
		orb_check(cmd_sub, &updated);

		if (updated) {
			/* got command */
			orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);

			/* handle it */
			if (handle_command(&status, &safety, &cmd, &armed, &_home, &global_position, &local_position,
					&attitude, &home_pub, &command_ack_pub, &command_ack, &_roi, &roi_pub)) {
				status_changed = true;
			}
		}

		/* Check for failure combinations which lead to flight termination */
		if (armed.armed &&
		    !status_flags.circuit_breaker_flight_termination_disabled) {
			/* At this point the data link and the gps system have been checked
			 * If we are not in a manual (RC stick controlled mode)
			 * and both failed we want to terminate the flight */
			if (internal_state.main_state != commander_state_s::MAIN_STATE_MANUAL &&
			    internal_state.main_state != commander_state_s::MAIN_STATE_ACRO &&
			    internal_state.main_state != commander_state_s::MAIN_STATE_RATTITUDE &&
			    internal_state.main_state != commander_state_s::MAIN_STATE_STAB &&
			    internal_state.main_state != commander_state_s::MAIN_STATE_ALTCTL &&
			    internal_state.main_state != commander_state_s::MAIN_STATE_POSCTL &&
			    ((status.data_link_lost && status_flags.gps_failure) ||
			     (status_flags.data_link_lost_cmd && status_flags.gps_failure_cmd))) {
				armed.force_failsafe = true;
				status_changed = true;
				static bool flight_termination_printed = false;

				if (!flight_termination_printed) {
					mavlink_log_critical(&mavlink_log_pub, "DL and GPS lost: flight termination");
					flight_termination_printed = true;
				}

				if (counter % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0) {
					mavlink_log_critical(&mavlink_log_pub, "DL and GPS lost: flight termination");
				}
			}

			/* At this point the rc signal and the gps system have been checked
			 * If we are in manual (controlled with RC):
			 * if both failed we want to terminate the flight */
			if ((internal_state.main_state == commander_state_s::MAIN_STATE_ACRO ||
			     internal_state.main_state == commander_state_s::MAIN_STATE_RATTITUDE ||
			     internal_state.main_state == commander_state_s::MAIN_STATE_MANUAL ||
			     internal_state.main_state == commander_state_s::MAIN_STATE_STAB ||
			     internal_state.main_state == commander_state_s::MAIN_STATE_ALTCTL ||
			     internal_state.main_state == commander_state_s::MAIN_STATE_POSCTL) &&
			    ((status.rc_signal_lost && status_flags.gps_failure) ||
			     (status_flags.rc_signal_lost_cmd && status_flags.gps_failure_cmd))) {
				armed.force_failsafe = true;
				status_changed = true;
				static bool flight_termination_printed = false;

				if (!flight_termination_printed) {
					warnx("Flight termination because of RC signal loss and GPS failure");
					flight_termination_printed = true;
				}

				if (counter % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0) {
					mavlink_log_critical(&mavlink_log_pub, "RC and GPS lost: flight termination");
				}
			}
		}

		/* Get current timestamp */
		const hrt_abstime now = hrt_absolute_time();

		/* First time home position update - but only if disarmed */
		if (!status_flags.condition_home_position_valid && !armed.armed) {
            PX4_INFO("First time home position update ");
			commander_set_home_position(home_pub, _home, local_position, global_position, attitude);
		}

		/* update home position on arming if at least 500 ms from commander start spent to avoid setting home on in-air restart */
		else if (((!was_armed && armed.armed) || (was_landed && !land_detector.landed)) &&
			(now > commander_boot_timestamp + INAIR_RESTART_HOLDOFF_INTERVAL)) {
            PX4_INFO("update home position on arming  ");
			commander_set_home_position(home_pub, _home, local_position, global_position, attitude);
		}

		was_armed = armed.armed;

		/* print new state */
		if (arming_state_changed) {
			status_changed = true;
			arming_state_changed = false;
		}

		/* now set navigation state according to failsafe and main state */
        //use this fuction to change the state     yuli

#ifdef RECEIVE_STATUS  //订阅这个消息－－来自于其他线程，接收本飞机的当前状态
        orb_check(return_flag_sub, &updated);
        if(updated)
        {
            orb_copy(ORB_ID(follow_to_commander), return_flag_sub, &return_flag);
         //   PX4_INFO("foramtion1.lat:%.7f", formation1.lat);
        }

#endif
#ifdef CHANGE_STATE
        PX4_INFO("formation1.status :%d",formation1.status);
        followtarget_time = 10e6 * (hrt_abstime)mavlink_system.sysid;  //每个系统不同的ＩＤ值乘以时间－－代表不同飞机的ｆｏｌｌｏｗｔａｒｇｅｔ时间
        if( (mavlink_system.sysid != 1 || sp_man.aux1 > 0.5f) && (mavlink_system.sysid == 1 || formation1.status == 5 ||formation1.status == 12 ||formation1.status == 10))
        {
           // PX4_INFO("sp_man.aux1 > 0.5f");
            if(plan_time_flag == false)
            {
               task_start_time = hrt_absolute_time();
               plan_time_flag = true;
            }
            task_planning_time = hrt_absolute_time() - task_start_time;
            task_plan();
        }
        else
        {
            plan_time_flag = false;
        }

#endif
        bool nav_state_changed = set_nav_state(&status,
						       &internal_state,
						       &mavlink_log_pub,
						       (datalink_loss_enabled > 0),
						       _mission_result.finished,
						       _mission_result.stay_in_failsafe,
						       &status_flags,
						       land_detector.landed,
						       (rc_loss_enabled > 0),
						       offboard_loss_act,
						       offboard_loss_rc_act);

		if (status.failsafe != failsafe_old) {
			status_changed = true;

			if (status.failsafe) {
				mavlink_log_critical(&mavlink_log_pub, "failsafe mode on");

			} else {
				mavlink_log_critical(&mavlink_log_pub, "failsafe mode off");
			}

			failsafe_old = status.failsafe;
		}

		// TODO handle mode changes by commands
		if (main_state_changed || nav_state_changed) {
			status_changed = true;
			main_state_changed = false;
		}

		/* publish states (armed, control mode, vehicle status) at least with 5 Hz */
		if (counter % (200000 / COMMANDER_MONITORING_INTERVAL) == 0 || status_changed) {
			set_control_mode();
			control_mode.timestamp = now;
			orb_publish(ORB_ID(vehicle_control_mode), control_mode_pub, &control_mode);

			status.timestamp = now;
			orb_publish(ORB_ID(vehicle_status), status_pub, &status);
            PX4_INFO("nav.state:%d",status.nav_state);

			armed.timestamp = now;

			/* set prearmed state if safety is off, or safety is not present and 5 seconds passed */
			if (safety.safety_switch_available) {

				/* safety is off, go into prearmed */
				armed.prearmed = safety.safety_off;
			} else {
				/* safety is not present, go into prearmed
				 * (all output drivers should be started / unlocked last in the boot process
				 * when the rest of the system is fully initialized)
				 */
				armed.prearmed = (hrt_elapsed_time(&commander_boot_timestamp) > 5 * 1000 * 1000);
			}
			orb_publish(ORB_ID(actuator_armed), armed_pub, &armed);
		}

		/* play arming and battery warning tunes */
		if (!arm_tune_played && armed.armed && (!safety.safety_switch_available || (safety.safety_switch_available
							&& safety.safety_off))) {
			/* play tune when armed */
			set_tune(TONE_ARMING_WARNING_TUNE);
			arm_tune_played = true;

		} else if (!status_flags.usb_connected &&
			   (status.hil_state != vehicle_status_s::HIL_STATE_ON) &&
			   (battery.warning == battery_status_s::BATTERY_WARNING_CRITICAL)) {
			/* play tune on battery critical */
			set_tune(TONE_BATTERY_WARNING_FAST_TUNE);

		} else if ((status.hil_state != vehicle_status_s::HIL_STATE_ON) &&
			   (battery.warning == battery_status_s::BATTERY_WARNING_LOW)) {
			/* play tune on battery warning or failsafe */
			set_tune(TONE_BATTERY_WARNING_SLOW_TUNE);

		} else {
			set_tune(TONE_STOP_TUNE);
		}

		/* reset arm_tune_played when disarmed */
		if (!armed.armed || (safety.safety_switch_available && !safety.safety_off)) {

			//Notify the user that it is safe to approach the vehicle
			if (arm_tune_played) {
				tune_neutral(true);
			}

			arm_tune_played = false;
		}

		/* play sensor failure tunes if we already waited for hotplug sensors to come up and failed */
		hotplug_timeout = hrt_elapsed_time(&commander_boot_timestamp) > HOTPLUG_SENS_TIMEOUT;

		if (!sensor_fail_tune_played && (!status_flags.condition_system_sensors_initialized && hotplug_timeout)) {
			set_tune_override(TONE_GPS_WARNING_TUNE);
			sensor_fail_tune_played = true;
			status_changed = true;
		}

		/* update timeout flag */
		if(!(hotplug_timeout == status_flags.condition_system_hotplug_timeout)) {
			status_flags.condition_system_hotplug_timeout = hotplug_timeout;
			status_changed = true;
		}

		counter++;

		int blink_state = blink_msg_state();

		if (blink_state > 0) {
			/* blinking LED message, don't touch LEDs */
			if (blink_state == 2) {
				/* blinking LED message completed, restore normal state */
				control_status_leds(&status, &armed, true, &battery, &cpuload);
			}

		} else {
			/* normal state */
			control_status_leds(&status, &armed, status_changed, &battery, &cpuload);
		}

		status_changed = false;

		if (!armed.armed) {
			/* Reset the flag if disarmed. */
			have_taken_off_since_arming = false;
		}


		/* publish internal state for logging purposes */
		if (commander_state_pub != nullptr) {
			orb_publish(ORB_ID(commander_state), commander_state_pub, &internal_state);

		} else {
			commander_state_pub = orb_advertise(ORB_ID(commander_state), &internal_state);
		}

		usleep(COMMANDER_MONITORING_INTERVAL);
	}

	/* wait for threads to complete */
	ret = pthread_join(commander_low_prio_thread, NULL);

	if (ret) {
		warn("join failed: %d", ret);
	}

	rgbled_set_mode(RGBLED_MODE_OFF);

	/* close fds */
	led_deinit();
	buzzer_deinit();
	px4_close(sp_man_sub);
	px4_close(offboard_control_mode_sub);
	px4_close(local_position_sub);
	px4_close(global_position_sub);
	px4_close(gps_sub);
	px4_close(sensor_sub);
	px4_close(safety_sub);
	px4_close(cmd_sub);
	px4_close(subsys_sub);
	px4_close(diff_pres_sub);
	px4_close(param_changed_sub);
	px4_close(battery_sub);
	px4_close(land_detector_sub);

	thread_running = false;

	return 0;
}

void
get_circuit_breaker_params()
{
	status_flags.circuit_breaker_engaged_power_check = circuit_breaker_enabled("CBRK_SUPPLY_CHK", CBRK_SUPPLY_CHK_KEY);
	status_flags.circuit_breaker_engaged_usb_check = circuit_breaker_enabled("CBRK_USB_CHK", CBRK_USB_CHK_KEY);
	status_flags.circuit_breaker_engaged_airspd_check = circuit_breaker_enabled("CBRK_AIRSPD_CHK", CBRK_AIRSPD_CHK_KEY);
	status_flags.circuit_breaker_engaged_enginefailure_check = circuit_breaker_enabled("CBRK_ENGINEFAIL", CBRK_ENGINEFAIL_KEY);
	status_flags.circuit_breaker_engaged_gpsfailure_check = circuit_breaker_enabled("CBRK_GPSFAIL", CBRK_GPSFAIL_KEY);
	status_flags.circuit_breaker_flight_termination_disabled = circuit_breaker_enabled("CBRK_FLIGHTTERM", CBRK_FLIGHTTERM_KEY);
}

void
check_valid(hrt_abstime timestamp, hrt_abstime timeout, bool valid_in, bool *valid_out, bool *changed)
{
	hrt_abstime t = hrt_absolute_time();
	bool valid_new = (t < timestamp + timeout && t > timeout && valid_in);

	if (*valid_out != valid_new) {
		*valid_out = valid_new;
		*changed = true;
	}
}

void
control_status_leds(vehicle_status_s *status_local, const actuator_armed_s *actuator_armed, bool changed, battery_status_s *battery_local, const cpuload_s *cpuload_local)
{
	bool overload = (cpuload_local->load > 0.75f) || (cpuload_local->ram_usage > 0.98f);

	/* driving rgbled */
	if (changed) {
		bool set_normal_color = false;
		bool hotplug_timeout = hrt_elapsed_time(&commander_boot_timestamp) > HOTPLUG_SENS_TIMEOUT;

		/* set mode */
		if (overload) {
			rgbled_set_mode(RGBLED_MODE_BLINK_FAST);
			rgbled_set_color(RGBLED_COLOR_PURPLE);
			set_normal_color = false;

		} else if (status_local->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			rgbled_set_mode(RGBLED_MODE_ON);
			set_normal_color = true;

		} else if (status_local->arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR || (!status_flags.condition_system_sensors_initialized && hotplug_timeout)) {
			rgbled_set_mode(RGBLED_MODE_BLINK_FAST);
			rgbled_set_color(RGBLED_COLOR_RED);

		} else if (status_local->arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
			rgbled_set_mode(RGBLED_MODE_BREATHE);
			set_normal_color = true;

		} else if (!status_flags.condition_system_sensors_initialized && !hotplug_timeout) {
			rgbled_set_mode(RGBLED_MODE_BREATHE);
			set_normal_color = true;

		} else {	// STANDBY_ERROR and other states
			rgbled_set_mode(RGBLED_MODE_BLINK_NORMAL);
			rgbled_set_color(RGBLED_COLOR_RED);
		}

		if (set_normal_color) {
			/* set color */
			if (status.failsafe) {
				rgbled_set_color(RGBLED_COLOR_PURPLE);
			} else if (battery_local->warning == battery_status_s::BATTERY_WARNING_LOW) {
				rgbled_set_color(RGBLED_COLOR_AMBER);
			} else if (battery_local->warning == battery_status_s::BATTERY_WARNING_CRITICAL) {
				rgbled_set_color(RGBLED_COLOR_RED);
			} else {
				if (status_flags.condition_home_position_valid && status_flags.condition_global_position_valid) {
					rgbled_set_color(RGBLED_COLOR_GREEN);

				} else {
					rgbled_set_color(RGBLED_COLOR_BLUE);
				}
			}
		}
	}

#if defined (CONFIG_ARCH_BOARD_PX4FMU_V1) || defined (CONFIG_ARCH_BOARD_PX4FMU_V4) || defined (CONFIG_ARCH_BOARD_CRAZYFLIE)

	/* this runs at around 20Hz, full cycle is 16 ticks = 10/16Hz */
	if (actuator_armed->armed) {
		/* armed, solid */
		led_on(LED_BLUE);

	} else if (actuator_armed->ready_to_arm) {
		/* ready to arm, blink at 1Hz */
		if (leds_counter % 20 == 0) {
			led_toggle(LED_BLUE);
		}

	} else {
		/* not ready to arm, blink at 10Hz */
		if (leds_counter % 2 == 0) {
			led_toggle(LED_BLUE);
		}
	}

#endif

	/* give system warnings on error LED */
	if (overload) {
		if (leds_counter % 2 == 0) {
			led_toggle(LED_AMBER);
		}

	} else {
		led_off(LED_AMBER);
	}

	leds_counter++;
}

transition_result_t
set_main_state_rc(struct vehicle_status_s *status_local)
{
	/* set main state according to RC switches */
	transition_result_t res = TRANSITION_DENIED;

	// XXX this should not be necessary any more, we should be able to
	// just delete this and respond to mode switches
	/* if offboard is set already by a mavlink command, abort */
	if (status_flags.offboard_control_set_by_command) {
		return main_state_transition(status_local, commander_state_s::MAIN_STATE_OFFBOARD, main_state_prev, &status_flags, &internal_state);
	}

	/* manual setpoint has not updated, do not re-evaluate it */
	if (((_last_sp_man.timestamp != 0) && (_last_sp_man.timestamp == sp_man.timestamp)) ||
		((_last_sp_man.offboard_switch == sp_man.offboard_switch) &&
		 (_last_sp_man.return_switch == sp_man.return_switch) &&
		 (_last_sp_man.mode_switch == sp_man.mode_switch) &&
		 (_last_sp_man.acro_switch == sp_man.acro_switch) &&
		 (_last_sp_man.rattitude_switch == sp_man.rattitude_switch) &&
		 (_last_sp_man.posctl_switch == sp_man.posctl_switch) &&
		 (_last_sp_man.loiter_switch == sp_man.loiter_switch) &&
		 (_last_sp_man.mode_slot == sp_man.mode_slot))) {

		// update these fields for the geofence system

		if (!warning_action_on) {
			_last_sp_man.timestamp = sp_man.timestamp;
			_last_sp_man.x = sp_man.x;
			_last_sp_man.y = sp_man.y;
			_last_sp_man.z = sp_man.z;
			_last_sp_man.r = sp_man.r;
		}

		/* no timestamp change or no switch change -> nothing changed */
		return TRANSITION_NOT_CHANGED;
	}

	_last_sp_man = sp_man;

	/* offboard switch overrides main switch */
	if (sp_man.offboard_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_OFFBOARD, main_state_prev, &status_flags, &internal_state);

		if (res == TRANSITION_DENIED) {
			print_reject_mode(status_local, "OFFBOARD");
			/* mode rejected, continue to evaluate the main system mode */

		} else {
			/* changed successfully or already in this state */
			return res;
		}
	}

	/* RTL switch overrides main switch */
	if (sp_man.return_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
		warnx("RTL switch changed and ON!");
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_RTL, main_state_prev, &status_flags, &internal_state);

		if (res == TRANSITION_DENIED) {
			print_reject_mode(status_local, "AUTO RTL");

			/* fallback to LOITER if home position not set */
			res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_LOITER, main_state_prev, &status_flags, &internal_state);
		}

		if (res != TRANSITION_DENIED) {
			/* changed successfully or already in this state */
			return res;
		}

		/* if we get here mode was rejected, continue to evaluate the main system mode */
	}

	/* we know something has changed - check if we are in mode slot operation */
	if (sp_man.mode_slot != manual_control_setpoint_s::MODE_SLOT_NONE) {

		if (sp_man.mode_slot >= sizeof(_flight_mode_slots) / sizeof(_flight_mode_slots[0])) {
			warnx("m slot overflow");
			return TRANSITION_DENIED;
		}

		int new_mode = _flight_mode_slots[sp_man.mode_slot];

		if (new_mode < 0) {
			/* slot is unused */
			res = TRANSITION_NOT_CHANGED;

		} else {
			res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

			/* ensure that the mode selection does not get stuck here */
			int maxcount = 5;

			/* enable the use of break */
			/* fallback strategies, give the user the closest mode to what he wanted */
			while (res == TRANSITION_DENIED && maxcount > 0) {

				maxcount--;

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_MISSION) {

					/* fall back to loiter */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode(status_local, "AUTO MISSION");
					res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_RTL) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode(status_local, "AUTO RTL");
					res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_LAND) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode(status_local, "AUTO LAND");
					res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_TAKEOFF) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode(status_local, "AUTO TAKEOFF");
					res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode(status_local, "AUTO FOLLOW");
					res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_LOITER) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_POSCTL;
					print_reject_mode(status_local, "AUTO HOLD");
					res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_POSCTL) {

					/* fall back to altitude control */
					new_mode = commander_state_s::MAIN_STATE_ALTCTL;
					print_reject_mode(status_local, "POSITION CONTROL");
					res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_ALTCTL) {

					/* fall back to stabilized */
					new_mode = commander_state_s::MAIN_STATE_STAB;
					print_reject_mode(status_local, "ALTITUDE CONTROL");
					res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_STAB) {

					/* fall back to manual */
					new_mode = commander_state_s::MAIN_STATE_MANUAL;
					print_reject_mode(status_local, "STABILIZED");
					res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}
			}
		}

		return res;
	}

	/* offboard and RTL switches off or denied, check main mode switch */
	switch (sp_man.mode_switch) {
	case manual_control_setpoint_s::SWITCH_POS_NONE:
		res = TRANSITION_NOT_CHANGED;
		break;

	case manual_control_setpoint_s::SWITCH_POS_OFF:		// MANUAL
		if (sp_man.acro_switch == manual_control_setpoint_s::SWITCH_POS_ON) {

			/* manual mode is stabilized already for multirotors, so switch to acro
			 * for any non-manual mode
			 */
			// XXX: put ACRO and STAB on separate switches
			if (status.is_rotary_wing && !status.is_vtol) {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_ACRO, main_state_prev, &status_flags, &internal_state);
			} else if (!status.is_rotary_wing) {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_STAB, main_state_prev, &status_flags, &internal_state);
			} else {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, main_state_prev, &status_flags, &internal_state);
			}

		}
		else if(sp_man.rattitude_switch == manual_control_setpoint_s::SWITCH_POS_ON){
			/* Similar to acro transitions for multirotors.  FW aircraft don't need a
			 * rattitude mode.*/
			if (status.is_rotary_wing) {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_RATTITUDE, main_state_prev, &status_flags, &internal_state);
			} else {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_STAB, main_state_prev, &status_flags, &internal_state);
			}
		}else {
			res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, main_state_prev, &status_flags, &internal_state);
		}

		// TRANSITION_DENIED is not possible here
		break;

	case manual_control_setpoint_s::SWITCH_POS_MIDDLE:		// ASSIST
		if (sp_man.posctl_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
			res = main_state_transition(status_local, commander_state_s::MAIN_STATE_POSCTL, main_state_prev, &status_flags, &internal_state);

			if (res != TRANSITION_DENIED) {
				break;	// changed successfully or already in this state
			}

			print_reject_mode(status_local, "POSITION CONTROL");
		}

		// fallback to ALTCTL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_ALTCTL, main_state_prev, &status_flags, &internal_state);

		if (res != TRANSITION_DENIED) {
			break;	// changed successfully or already in this mode
		}

		if (sp_man.posctl_switch != manual_control_setpoint_s::SWITCH_POS_ON) {
			print_reject_mode(status_local, "ALTITUDE CONTROL");
		}

		// fallback to MANUAL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, main_state_prev, &status_flags, &internal_state);
		// TRANSITION_DENIED is not possible here
		break;

	case manual_control_setpoint_s::SWITCH_POS_ON:			// AUTO
		if (sp_man.loiter_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
			res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_LOITER, main_state_prev, &status_flags, &internal_state);

			if (res != TRANSITION_DENIED) {
				break;	// changed successfully or already in this state
			}

			print_reject_mode(status_local, "AUTO PAUSE");

		} else {
			res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_MISSION, main_state_prev, &status_flags, &internal_state);

			if (res != TRANSITION_DENIED) {
				break;	// changed successfully or already in this state
			}

			print_reject_mode(status_local, "AUTO MISSION");

			// fallback to LOITER if home position not set
			res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_LOITER, main_state_prev, &status_flags, &internal_state);

			if (res != TRANSITION_DENIED) {
				break;  // changed successfully or already in this state
			}
		}

		// fallback to POSCTL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_POSCTL, main_state_prev, &status_flags, &internal_state);

		if (res != TRANSITION_DENIED) {
			break;  // changed successfully or already in this state
		}

		// fallback to ALTCTL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_ALTCTL, main_state_prev, &status_flags, &internal_state);

		if (res != TRANSITION_DENIED) {
			break;	// changed successfully or already in this state
		}

		// fallback to MANUAL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, main_state_prev, &status_flags, &internal_state);
		// TRANSITION_DENIED is not possible here
		break;

	default:
		break;
	}

	return res;
}

void
set_control_mode()
{
	/* set vehicle_control_mode according to set_navigation_state */
	control_mode.flag_armed = armed.armed;
	control_mode.flag_external_manual_override_ok = (!status.is_rotary_wing && !status.is_vtol);
	control_mode.flag_system_hil_enabled = status.hil_state == vehicle_status_s::HIL_STATE_ON;
	control_mode.flag_control_offboard_enabled = false;

	switch (status.nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = stabilization_required();
		control_mode.flag_control_attitude_enabled = stabilization_required();
		control_mode.flag_control_rattitude_enabled = false;
		control_mode.flag_control_altitude_enabled = false;
		control_mode.flag_control_climb_rate_enabled = false;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_acceleration_enabled = false;
		control_mode.flag_control_termination_enabled = false;
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_rattitude_enabled = true;
		control_mode.flag_control_altitude_enabled = false;
		control_mode.flag_control_climb_rate_enabled = false;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_acceleration_enabled = false;
		control_mode.flag_control_termination_enabled = false;
		/* override is not ok in stabilized mode */
		control_mode.flag_external_manual_override_ok = false;
		break;

	case vehicle_status_s::NAVIGATION_STATE_RATTITUDE:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_rattitude_enabled = true;
		control_mode.flag_control_altitude_enabled = false;
		control_mode.flag_control_climb_rate_enabled = false;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_acceleration_enabled = false;
		control_mode.flag_control_termination_enabled = false;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_rattitude_enabled = false;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_acceleration_enabled = false;
		control_mode.flag_control_termination_enabled = false;
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_rattitude_enabled = false;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_position_enabled = !status.in_transition_mode;
		control_mode.flag_control_velocity_enabled = !status.in_transition_mode;
		control_mode.flag_control_acceleration_enabled = false;
		control_mode.flag_control_termination_enabled = false;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER:
		/* override is not ok for the RTL and recovery mode */
		control_mode.flag_external_manual_override_ok = false;
		/* fallthrough */
	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_rattitude_enabled = false;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_position_enabled = !status.in_transition_mode;
		control_mode.flag_control_velocity_enabled = !status.in_transition_mode;
		control_mode.flag_control_acceleration_enabled = false;
		control_mode.flag_control_termination_enabled = false;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_rattitude_enabled = false;
		control_mode.flag_control_altitude_enabled = false;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_acceleration_enabled = false;
		control_mode.flag_control_termination_enabled = false;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = false;
		control_mode.flag_control_rattitude_enabled = false;
		control_mode.flag_control_altitude_enabled = false;
		control_mode.flag_control_climb_rate_enabled = false;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_acceleration_enabled = false;
		control_mode.flag_control_termination_enabled = false;
		break;

	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		/* TODO: check if this makes sense */
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_rattitude_enabled = false;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_acceleration_enabled = false;
		control_mode.flag_control_altitude_enabled = false;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_termination_enabled = false;
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		/* disable all controllers on termination */
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = false;
		control_mode.flag_control_attitude_enabled = false;
		control_mode.flag_control_rattitude_enabled = false;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_acceleration_enabled = false;
		control_mode.flag_control_altitude_enabled = false;
		control_mode.flag_control_climb_rate_enabled = false;
		control_mode.flag_control_termination_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_offboard_enabled = true;

		/*
		 * The control flags depend on what is ignored according to the offboard control mode topic
		 * Inner loop flags (e.g. attitude) also depend on outer loop ignore flags (e.g. position)
		 */
		control_mode.flag_control_rates_enabled = !offboard_control_mode.ignore_bodyrate ||
			!offboard_control_mode.ignore_attitude ||
			!offboard_control_mode.ignore_position ||
			!offboard_control_mode.ignore_velocity ||
			!offboard_control_mode.ignore_acceleration_force;

		control_mode.flag_control_attitude_enabled = !offboard_control_mode.ignore_attitude ||
			!offboard_control_mode.ignore_position ||
			!offboard_control_mode.ignore_velocity ||
			!offboard_control_mode.ignore_acceleration_force;

		control_mode.flag_control_rattitude_enabled = false;

		control_mode.flag_control_acceleration_enabled = !offboard_control_mode.ignore_acceleration_force &&
		  !status.in_transition_mode;

		control_mode.flag_control_velocity_enabled = (!offboard_control_mode.ignore_velocity ||
			!offboard_control_mode.ignore_position) && !status.in_transition_mode &&
			!control_mode.flag_control_acceleration_enabled;

		control_mode.flag_control_climb_rate_enabled = (!offboard_control_mode.ignore_velocity ||
			!offboard_control_mode.ignore_position) && !control_mode.flag_control_acceleration_enabled;

		control_mode.flag_control_position_enabled = !offboard_control_mode.ignore_position && !status.in_transition_mode &&
		  !control_mode.flag_control_acceleration_enabled;

		control_mode.flag_control_altitude_enabled = (!offboard_control_mode.ignore_velocity ||
			!offboard_control_mode.ignore_position) && !control_mode.flag_control_acceleration_enabled;

		break;

	default:
		break;
	}
}

bool
stabilization_required()
{
	return (status.is_rotary_wing ||		// is a rotary wing, or
		status.vtol_fw_permanent_stab || 	// is a VTOL in fixed wing mode and stabilisation is on, or
		(vtol_status.vtol_in_trans_mode && 	// is currently a VTOL transitioning AND
			!status.is_rotary_wing));	// is a fixed wing, ie: transitioning back to rotary wing mode
}

void
print_reject_mode(struct vehicle_status_s *status_local, const char *msg)
{
	hrt_abstime t = hrt_absolute_time();

	if (t - last_print_mode_reject_time > PRINT_MODE_REJECT_INTERVAL) {
		last_print_mode_reject_time = t;
		mavlink_log_critical(&mavlink_log_pub, "REJECT %s", msg);

		/* only buzz if armed, because else we're driving people nuts indoors
		they really need to look at the leds as well. */
		tune_negative(armed.armed);
	}
}

void
print_reject_arm(const char *msg)
{
	hrt_abstime t = hrt_absolute_time();

	if (t - last_print_mode_reject_time > PRINT_MODE_REJECT_INTERVAL) {
		last_print_mode_reject_time = t;
		mavlink_log_critical(&mavlink_log_pub, msg);
		tune_negative(true);
	}
}

void answer_command(struct vehicle_command_s &cmd, unsigned result,
					orb_advert_t &command_ack_pub, vehicle_command_ack_s &command_ack)
{
	switch (result) {
	case vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED:
		tune_positive(true);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_DENIED:
		tune_negative(true);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_FAILED:
		tune_negative(true);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		tune_negative(true);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED:
		tune_negative(true);
		break;

	default:
		break;
	}

	/* publish ACK */
	command_ack.command = cmd.command;
	command_ack.result = result;

	if (command_ack_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_command_ack), command_ack_pub, &command_ack);

	} else {
		command_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &command_ack, vehicle_command_ack_s::ORB_QUEUE_LENGTH);
	}
}

void *commander_low_prio_loop(void *arg)
{
	/* Set thread name */
	px4_prctl(PR_SET_NAME, "commander_low_prio", px4_getpid());

	/* Subscribe to command topic */
	int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
	struct vehicle_command_s cmd;
	memset(&cmd, 0, sizeof(cmd));

	/* command ack */
	orb_advert_t command_ack_pub = nullptr;
	struct vehicle_command_ack_s command_ack;
	memset(&command_ack, 0, sizeof(command_ack));

	/* timeout for param autosave */
	hrt_abstime need_param_autosave_timeout = 0;

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[1];

	/* use the gyro to pace output - XXX BROKEN if we are using the L3GD20 */
	fds[0].fd = cmd_sub;
	fds[0].events = POLLIN;

	while (!thread_should_exit) {
		/* wait for up to 1000ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 1000);

		/* timed out - periodic check for thread_should_exit, etc. */
		if (pret == 0) {
			/* trigger a param autosave if required */
			if (need_param_autosave) {
				if (need_param_autosave_timeout > 0 && hrt_elapsed_time(&need_param_autosave_timeout) > 200000ULL) {
					int ret = param_save_default();

					if (ret != OK) {
						mavlink_log_critical(&mavlink_log_pub, "settings auto save error");
					} else {
						PX4_DEBUG("commander: settings saved.");
					}

					need_param_autosave = false;
					need_param_autosave_timeout = 0;
				} else {
					need_param_autosave_timeout = hrt_absolute_time();
				}
			}
		} else if (pret < 0) {
		/* this is undesirable but not much we can do - might want to flag unhappy status */
			warn("commander: poll error %d, %d", pret, errno);
			continue;
		} else {

			/* if we reach here, we have a valid command */
			orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);

			/* ignore commands the high-prio loop or the navigator handles */
			if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_MODE ||
			    cmd.command == vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM ||
			    cmd.command == vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF ||
			    cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_SERVO ||
			    cmd.command == vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED) {

				continue;
			}

			/* only handle low-priority commands here */
			switch (cmd.command) {

			case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
				if (is_safe(&status, &safety, &armed)) {

					if (((int)(cmd.param1)) == 1) {
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub, command_ack);
						usleep(100000);
						/* reboot */
						px4_systemreset(false);

					} else if (((int)(cmd.param1)) == 3) {
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub, command_ack);
						usleep(100000);
						/* reboot to bootloader */
						px4_systemreset(true);

					} else {
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED, command_ack_pub, command_ack);
					}

				} else {
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED, command_ack_pub, command_ack);
				}

				break;

			case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION: {

					int calib_ret = PX4_ERROR;

					/* try to go to INIT/PREFLIGHT arming state */
					if (TRANSITION_DENIED == arming_state_transition(&status,
											 &battery,
											 &safety,
											 vehicle_status_s::ARMING_STATE_INIT,
											 &armed,
											 false /* fRunPreArmChecks */,
											 &mavlink_log_pub,
											 &status_flags,
											 avionics_power_rail_voltage,
											 can_arm_without_gps,
											 hrt_elapsed_time(&commander_boot_timestamp))) {
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED, command_ack_pub, command_ack);
						break;
					} else {
						status_flags.condition_calibration_enabled = true;
					}

					if ((int)(cmd.param1) == 1) {
						/* gyro calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub, command_ack);
						calib_ret = do_gyro_calibration(&mavlink_log_pub);

					} else if ((int)(cmd.param2) == 1) {
						/* magnetometer calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub, command_ack);
						calib_ret = do_mag_calibration(&mavlink_log_pub);

					} else if ((int)(cmd.param3) == 1) {
						/* zero-altitude pressure calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED, command_ack_pub, command_ack);

					} else if ((int)(cmd.param4) == 1) {
						/* RC calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub, command_ack);
						/* disable RC control input completely */
						status_flags.rc_input_blocked = true;
						calib_ret = OK;
						mavlink_log_info(&mavlink_log_pub, "CAL: Disabling RC IN");

					} else if ((int)(cmd.param4) == 2) {
						/* RC trim calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub, command_ack);
						calib_ret = do_trim_calibration(&mavlink_log_pub);

					} else if ((int)(cmd.param5) == 1) {
						/* accelerometer calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub, command_ack);
						calib_ret = do_accel_calibration(&mavlink_log_pub);
					} else if ((int)(cmd.param5) == 2) {
						// board offset calibration
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub, command_ack);
						calib_ret = do_level_calibration(&mavlink_log_pub);
					} else if ((int)(cmd.param6) == 1) {
						/* airspeed calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub, command_ack);
						calib_ret = do_airspeed_calibration(&mavlink_log_pub);

					} else if ((int)(cmd.param7) == 1) {
						/* do esc calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub, command_ack);
						calib_ret = do_esc_calibration(&mavlink_log_pub, &armed);

					} else if ((int)(cmd.param4) == 0) {
						/* RC calibration ended - have we been in one worth confirming? */
						if (status_flags.rc_input_blocked) {
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub, command_ack);
							/* enable RC control input */
							status_flags.rc_input_blocked = false;
							mavlink_log_info(&mavlink_log_pub, "CAL: Re-enabling RC IN");
							calib_ret = OK;
						}
						/* this always succeeds */
						calib_ret = OK;
					}

					status_flags.condition_calibration_enabled = false;

					if (calib_ret == OK) {
						tune_positive(true);

						// Update preflight check status
						// we do not set the calibration return value based on it because the calibration
						// might have worked just fine, but the preflight check fails for a different reason,
						// so this would be prone to false negatives.

						bool checkAirspeed = false;
						bool hotplug_timeout = hrt_elapsed_time(&commander_boot_timestamp) > HOTPLUG_SENS_TIMEOUT;
						/* Perform airspeed check only if circuit breaker is not
						 * engaged and it's not a rotary wing */
						if (!status_flags.circuit_breaker_engaged_airspd_check &&
						    (!status.is_rotary_wing || status.is_vtol)) {
							checkAirspeed = true;
						}

						status_flags.condition_system_sensors_initialized = Commander::preflightCheck(&mavlink_log_pub, true, true, true, true, checkAirspeed,
							!(status.rc_input_mode >= vehicle_status_s::RC_IN_MODE_OFF), !status_flags.circuit_breaker_engaged_gpsfailure_check,
							/* checkDynamic */ true, is_vtol(&status), /* reportFailures */ hotplug_timeout, /* prearm */ false, hrt_elapsed_time(&commander_boot_timestamp));

						arming_state_transition(&status,
									&battery,
									&safety,
									vehicle_status_s::ARMING_STATE_STANDBY,
									&armed,
									false /* fRunPreArmChecks */,
									&mavlink_log_pub,
									&status_flags,
								        avionics_power_rail_voltage,
									can_arm_without_gps,
									hrt_elapsed_time(&commander_boot_timestamp));

					} else {
						tune_negative(true);
					}

					break;
				}

			case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_STORAGE: {

					if (((int)(cmd.param1)) == 0) {
						int ret = param_load_default();

						if (ret == OK) {
							mavlink_log_info(&mavlink_log_pub, "settings loaded");
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub, command_ack);

						} else {
							mavlink_log_critical(&mavlink_log_pub, "settings load ERROR");

							/* convenience as many parts of NuttX use negative errno */
							if (ret < 0) {
								ret = -ret;
							}

							if (ret < 1000) {
								mavlink_log_critical(&mavlink_log_pub, "ERROR: %s", strerror(ret));
							}

							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED, command_ack_pub, command_ack);
						}

					} else if (((int)(cmd.param1)) == 1) {

#ifdef __PX4_QURT
						// TODO FIXME: on snapdragon the save happens too early when the params
						// are not set yet. We therefore need to wait some time first.
						usleep(1000000);
#endif

						int ret = param_save_default();

						if (ret == OK) {
							if (need_param_autosave) {
								need_param_autosave = false;
								need_param_autosave_timeout = 0;
							}

							/* do not spam MAVLink, but provide the answer / green led mechanism */
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub, command_ack);

						} else {
							mavlink_log_critical(&mavlink_log_pub, "settings save error");

							/* convenience as many parts of NuttX use negative errno */
							if (ret < 0) {
								ret = -ret;
							}

							if (ret < 1000) {
								mavlink_log_critical(&mavlink_log_pub, "ERROR: %s", strerror(ret));
							}

							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED, command_ack_pub, command_ack);
						}
					} else if (((int)(cmd.param1)) == 2) {

						/* reset parameters and save empty file */
						param_reset_all();
						int ret = param_save_default();

						if (ret == OK) {
							/* do not spam MAVLink, but provide the answer / green led mechanism */
							mavlink_log_critical(&mavlink_log_pub, "onboard parameters reset");
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub, command_ack);

						} else {
							mavlink_log_critical(&mavlink_log_pub, "param reset error");
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED, command_ack_pub, command_ack);
						}
					}

					break;
				}

			case vehicle_command_s::VEHICLE_CMD_START_RX_PAIR:
				/* handled in the IO driver */
				break;

			default:
				/* don't answer on unsupported commands, it will be done in main loop */
				break;
			}
		}
	}

	px4_close(cmd_sub);

	return NULL;
}