#include "Copter.h"
#include <iostream>
#include <utility>
#include <time.h>
#include <iostream>
#include <AP_HAL_Linux/GPIO_BBB.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_Menu/AP_Menu.h>

using namespace std;

pair<int, int> echo_val(0, 0);                    // Values for the echo pins <left, right>
pair<int, int> echo_start(1, 1);                  // Flag to determine if you should start or end a timer
pair<uint32_t, uint32_t> timer_ns(0, 0);          // The raw output of each timer in ns
struct timespec l_start, l_stop, r_start, r_stop; // Used to get time from timers

// Trigger timers
struct timespec trigger_start, trigger_end;
int trigger_start_flag = 0;
int left_timer_start_flag = 0;
int right_timer_start_flag = 0; 

// Helper functions
void start_left_timer() {

    // Get start of timer
    clock_gettime(CLOCK_MONOTONIC, &l_start);
    

}

void end_left_timer() {

    // Get end of timer
    clock_gettime(CLOCK_MONOTONIC, &l_stop);

    // Write the timer value into a global variable
    timer_ns.first = (uint32_t)(l_stop.tv_nsec - l_start.tv_nsec);
}

void start_right_timer() {


    // Get start of timer
    clock_gettime(CLOCK_REALTIME, &r_start);

}

void end_right_timer() {

    // Get end of timer
    clock_gettime(CLOCK_REALTIME, &r_stop);

    // Write the timer value into a global variable
    timer_ns.second = (uint32_t)(r_stop.tv_nsec - r_start.tv_nsec);
}

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Copter::ultrasonic_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);

    // Initialize GPIO pins
    hal.gpio->init();

    hal.gpio->pinMode(LEFT_ECHO, HAL_GPIO_INPUT); 
    hal.gpio->pinMode(RIGHT_ECHO, HAL_GPIO_INPUT);
    hal.gpio->pinMode(TRIGGER, HAL_GPIO_OUTPUT);

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::ultrasonic_run()
{

    if (hal.gpio->read(LEFT_ECHO) && left_timer_start_flag == 0) {
        left_timer_start_flag = 1;
        start_left_timer();
    }
    else if (!hal.gpio->read(LEFT_ECHO) && left_timer_start_flag == 1) {
        left_timer_start_flag = 0;
        end_left_timer();
    }

    if (hal.gpio->read(RIGHT_ECHO) && right_timer_start_flag == 0) {
        right_timer_start_flag = 1;
        start_right_timer();
    }
    else if (!hal.gpio->read(RIGHT_ECHO) && right_timer_start_flag == 1) {
        right_timer_start_flag = 0;
        end_right_timer();
    }

    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // BEGINNING OF ADDED CODE
    if (trigger_start_flag == 0) {
        clock_gettime(CLOCK_REALTIME, &trigger_start);
        if ((uint32_t)(trigger_start.tv_nsec - trigger_end.tv_nsec) > 48000000) { // Suggests over a 50ms measurement cycle.
            hal.gpio->write(TRIGGER, 1);
	    start_left_timer();
	    start_right_timer();
            trigger_start_flag = 1;
            clock_gettime(CLOCK_REALTIME, &trigger_start);
        }
        
    }
    else {
        clock_gettime(CLOCK_REALTIME, &trigger_end);
        if ((uint32_t)(trigger_end.tv_nsec - trigger_start.tv_nsec) > 2000000) {
            hal.gpio->write(TRIGGER, 0);
	    end_left_timer();
	    end_right_timer();
            trigger_start_flag = 0;
            clock_gettime(CLOCK_REALTIME, &trigger_end);
        }
    }

    // If any of the timers' echo values are >= 45ms, then the beacon has been disconnected.
    // Drop throttle and exit.
    //if (timer_ns.first >= 45000000 || timer_ns.second >= 45000000) {
    //    motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
    //    // multicopters do not stabilize roll/pitch/yaw when disarmed
    //    attitude_control->set_throttle_out_unstabilized(100,true,g.throttle_filt); // TODO: Ensure that this will not drop the dron

    //    // disarm when the landing detector says we've landed
    //    if (ap.land_complete) {
    //        init_disarm_motors();
    //    }
    //    return;
    //}

    // Calculate what direction the drone should be moving based on timer values.
    // As of now, just move at a constant rate until it faces the US sensor

    //cout << timer_ns.first << " " << timer_ns.second << endl;
    if (timer_ns.first > timer_ns.second) {
        //cout << "TARGET YAW RATE POSITIVE" << endl;
        target_yaw_rate = POS_DEG; // If left echo > right echo, needs to bring left sensor closer, i.e. cw
    }
    else if (timer_ns.first < timer_ns.second) {
        //cout << "TARGET YAW RATE NEGATIVE" << endl;
        target_yaw_rate = NEG_DEG; // If right echo > left echo, needs to bring right sensor closer, i.e. ccw
    }

    else {
        target_yaw_rate = 0;  

        // If the echos are within a reasonable distance from each other, 
        // then attempt to keep the drone at a distance of about 1.5 meters from the beacon

        // Convert timer from ns to us.
        // Don't care which timer, as they should be close to the same.
        uint32_t timer_us = timer_ns.first / 1000;
        
        // distance(cm) = 0.0347cm/us * echo_time(us)
        // Subtract 4ms (4000us) for the consistent delay
        uint16_t distance = ((timer_us - 4000) * 340) / 10000;

        cout << distance << endl;
        if (distance < (DISTANCE - DIST_VAR)) {
            target_pitch = NEG_DEG;
        }
        else if (distance > (DISTANCE + DIST_VAR)) {
            target_pitch = POS_DEG;
        } 
    }
    // END OF ADDED CODE

 

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());
	std::cout << channel_roll->get_control_in() << " " << channel_pitch->get_control_in() << " " << channel_throttle->get_control_in() << std::endl;
    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}
