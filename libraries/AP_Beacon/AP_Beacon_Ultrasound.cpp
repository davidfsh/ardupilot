#include "AP_Beacon_Ultrasound.h"

AP_Beacon_Ultrasound::AP_Beacon_Ultrasound(AP_Beacon &frontend):
	AP_Beacon_Backend(frontend)
{
	_trigger->enable_ch(TRIGGER_CHANNEL);
	// The period shall be 25ms, which equals to 4m maximum
	_trigger->set_freq(1U << (TRIGGER_CHANNEL - 1) ,40);
	_xbee->begin(9600);
}

// return true if sensor is bascially healthy (we are receiving data)
bool AP_Beacon_Ultrasound::healthy()
{
	// healthy if we have get a data in the past 300ms
    // I am not sure whether we need to shrink this time check threshold,
    // since the period of ultraonic sensor update is 25ms.
	return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

// update the state of the sensor
void AP_Beacon_Ultrasound::update()
{
	//Read the extern ultrasound of last cycle
	left_echo = _dis_calculation(_echo->read(ECHO_LEFT));
	right_echo = _dis_calculation(_echo->read(ECHO_RIGHT));
	_xbee->printf("1");  // Synchronization the ultrasonic transmitter and the receiver clock
	_trigger->write(TRIGGER_CHANNEL, 10);
	
    // Add echos to ring buffers
	left_ring[index]=left_echo;
	right_ring[index]=right_echo;
	index = (index+1)%BUFFER_SIZE;

    // Increase size of buffer as initial elements are added
	if (size_of_ring<BUFFER_SIZE) size_of_ring++;
   
    // Read ring buffers to average values to get a distance calculation that is
    // less susceptible ot change
	int temp_sum_left=0; int temp_sum_right=0; 
	for (int i=0; i<size_of_ring; i++)
	{
		temp_sum_left+=left_ring[i];
		temp_sum_right+=right_ring[i];
	} 
	avg_left_data = temp_sum_left / size_of_ring;
	avg_right_data = temp_sum_right / size_of_ring;

    // Vehicles X Y Z Coordinates in Meters
    // O O  x    
    //  X   + y
    // O O  (z coming out of the screen)
    if (DRONE_DISTANCE_MIN > avg_left_data || avg_left_data > DRONE_DISTANCE_MAX) {
        vehicle_x = 0;
    }
    else {
        vehicle_x = avg_left_data;
    }

    // Vehicle position vector, converting cm to m
    Vector3f veh_pos(Vector3f(vehicle_x / 100.0f, vehicle_y / 100.0f, -vehicle_z / 100.0f));

    // Set position in meters, position accuracy is 1.
    // Note that yaw (twist) is corrected for automatically
    set_vehicle_position(veh_pos, 1);

	last_update_ms = AP_HAL::millis();
}

// Convert PWM echo signal into distance
// returns distance in cm
uint16_t AP_Beacon_Ultrasound::_dis_calculation(uint32_t echo_time)
{
	// distance(cm) = .0347cm/us * echo_time(us)
    return round(echo_time*340.0/10000);	
}

