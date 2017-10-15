#include "Copter.h"
#include <utility>
#include <time.h>
#include <iostream>
#include <AP_HAL_Linux/GPIO_BBB.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_Menu/AP_Menu.h>

#define LEFT_ECHO 57  // (32 * 1) + 25 for GPIO 1_25
#define RIGHT_ECHO 49 // (32 * 1) + 17 for GPIO 1_17
#define TRIGGER 98    // (32 * 3) + 2 for GPIO 3_2
#define POS_DEG 300 // Centidegrees, so this is 3 degrees
#define NEG_DEG -300
#define DISTANCE 150 // Distance to keep drone from beacon in cm 
#define DIST_VAR 20  // Distance variation

using namespace std;

pair<int, int> echo_val(0, 0);                    // Values for the echo pins <left, right>
pair<int, int> echo_start(1, 1);                  // Flag to determine if you should start or end a timer
pair<uint32_t, uint32_t> timer_ns(0, 0);          // The raw output of each timer in ns
struct timespec l_start, l_stop, r_start, r_stop; // Used to get time from timers

// Trigger timers
struct timespec trigger_start, trigger_end;
int trigger_start_flag = 0;

// Helper functions
void run_left_timers() {

    // FIXME: Remove me when you're done debugging
    cout << "RUN LEFT TIMER: " << echo_val.first << endl;

    // LEFT ECHO VALUE WRITE
    if (echo_val.first && echo_start.first) {        
        echo_start.first = 0;

        // Get start of timer
        clock_gettime(CLOCK_REALTIME, &l_start);
    }
    else if (!echo_val.first && !echo_start.first) {
        echo_start.first = 1;

        // Get end of timer
        clock_gettime(CLOCK_REALTIME, &l_stop);

        // Write the timer value into a global variable
        timer_ns.first = (uint32_t)(l_stop.tv_nsec - l_start.tv_nsec);
    }


}

void run_right_timers() {

    cout << "RUN RIGHT TIMER: " << echo_val.second << endl;

    // RIGHT ECHO VALUE WRITE
    if (echo_val.second && echo_start.second) {        
        echo_start.second = 0;

        // Get start of timer
        clock_gettime(CLOCK_REALTIME, &r_start);
    }
    else if (!echo_val.second && !echo_start.second) {
        echo_start.second = 1;

        // Get end of timer
        clock_gettime(CLOCK_REALTIME, &r_stop);

        // Write the timer value into a global variable
        timer_ns.second = (uint32_t)(r_stop.tv_nsec - r_start.tv_nsec);
    }
}


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Copter::ultrasonic_init(bool ignore_checks)
{
    
    // Initialize GPIO pins
    hal.gpio->init();

    hal.gpio->pinMode(LEFT_ECHO, HAL_GPIO_INPUT); 
    hal.gpio->pinMode(RIGHT_ECHO, HAL_GPIO_INPUT);
    hal.gpio->pinMode(TRIGGER, HAL_GPIO_OUTPUT);

    hal.gpio->attach_interrupt(LEFT_ECHO, run_left_timers, HAL_GPIO_INTERRUPT_HIGH);
    hal.gpio->attach_interrupt(RIGHT_ECHO, run_right_timers, HAL_GPIO_INTERRUPT_HIGH);


#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter Alt Hold if the Rotor Runup is not complete
    if (!ignore_checks && !motors->rotor_runup_complete()){
        return false;
    }
#endif

    // initialize vertical speeds and leash lengths
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // stop takeoff if running
    takeoff_stop();

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Copter::ultrasonic_run()
{
    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->get_althold_lean_angle_max());



    // BEGINNING OF ADDED CODE
    if (trigger_start_flag == 0) {
        clock_gettime(CLOCK_REALTIME, &trigger_start);
        if ((uint32_t)(trigger_start.tv_nsec - trigger_end.tv_nsec) > 50500) { // Suggests over a 60ms measurement cycle.
            hal.gpio->write(TRIGGER, 1);
            trigger_start_flag = 1;
        }
        
    }
    else {
        clock_gettime(CLOCK_REALTIME, &trigger_end);
        if ((uint32_t)(trigger_start.tv_nsec - trigger_end.tv_nsec) > 10000) {
            hal.gpio->write(TRIGGER, 0);
            trigger_start_flag = 0;
        }
    }


    // Calculate what direction the drone should be moving based on timer values.
    // As of now, just move at a constant rate until it faces the US sensor
    float target_yaw_rate;

    if (timer_ns.first > timer_ns.second)
        target_yaw_rate = POS_DEG; // If left echo > right echo, needs to bring left sensor closer, i.e. cw
    else if (timer_ns.first < timer_ns.second)
        target_yaw_rate = NEG_DEG; // If right echo > left echo, needs to bring right sensor closer, i.e. ccw

    else {
        target_yaw_rate = 0;  

        // If the echos are within a reasonable distance from each other, 
        // then attempt to keep the drone at a distance of about 1.5 meters from the beacon

        // Convert timer from ns to us.
        // Don't care which timer, as they should be close to the same.
        uint32_t timer_us = timer_ns.first / 1000;
        
        // distance(cm) = 0.0347cm/us * echo_time(us)
        uint16_t distance = (timer_us * 340) / 10000;

        if (distance < (DISTANCE - DIST_VAR)) {
            target_pitch = NEG_DEG;
        }
        else if (distance > (DISTANCE + DIST_VAR)) {
            target_pitch = POS_DEG;
        } 
    }
    // END OF ADDED CODE
    



    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (target_climb_rate > 0.0f) && motors->rotor_runup_complete());
#else
    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);
#endif

    // Alt Hold State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        althold_state = AltHold_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        althold_state = AltHold_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
#if FRAME_CONFIG == HELI_FRAME    
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
#else
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        pos_control->update_z_controller();
        break;

    case AltHold_Takeoff:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case AltHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        avoid.adjust_roll_pitch(target_roll, target_pitch, aparm.angle_max);
#endif

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
        break;
    }
}
