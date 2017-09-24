#pragma once

#include "AP_Beacon_Backend.h"
#include <AP_HAL/AP_HAL.h>

#define BUFFER_SIZE 5
#define TRIGGER_CHANNEL 5
#define ECHO_LEFT 6
#define ECHO_RIGHT 7

// Keep the drone a fixed distance +- 5cm from the beacon
#define DRONE_DISTANCE_MAX 5  // cm
#define DRONE_DISTANCE_MIN -5 // cm

#define DRONE_START_HEIGHT 75 // cm. Change this to whatever, but keep it low for safety

extern const AP_HAL::HAL& hal;

class AP_Beacon_Ultrasound : public AP_Beacon_Backend
{
public:
			//Constructor for AP_Beacon_Ultrasound
			AP_Beacon_Ultrasound(AP_Beacon &frontend);
													
			//Check whether device is healthy
			bool healthy();
	
			//Update data from the ultrasonic sensor
			void update();
	
private:
			AP_HAL::UARTDriver *_xbee=hal.uartF;
			AP_HAL::RCOutput *_trigger=hal.rcout;
			AP_HAL::RCInput *_echo=hal.rcin;
			
			uint32_t last_update_ms = 0;
			int avg_left_data=0;
			int avg_right_data=0;
			
			//This function translate the echo signal into a distance number;
			uint16_t _dis_calculation(uint32_t echo_time);
			
			//Ring Buffer for storing the data read from ultrasonic sensor;
			uint16_t left_ring[BUFFER_SIZE];
			uint16_t right_ring[BUFFER_SIZE];
			uint8_t size_of_ring=0;
			uint8_t index=0; //Index of the next reading
			
			uint16_t left_echo=0;
			uint16_t right_echo=0;

            int32_t vehicle_x = 0;
            int32_t vehicle_y = 0; // As of now this should NEVER CHANGE
            int32_t vehicle_z = DRONE_START_HEIGHT;
};
