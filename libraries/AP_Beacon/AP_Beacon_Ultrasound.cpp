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
	// healthy if we have get a data in the past 300ms, I am not sure whether we need to shrink this time check threshold, since the period of ultraonic sensor update is 25ms.
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
    
    //This is a ring buffer to filter the reading of ultrasonic sensor a little bit. It seems that I do some silly processing here. My original idea is to sum up the valid numbers in the ring buffer from the oldest value. However, it turns out to be both wrong and unnecassary. 
	int temp_sum_left=0; int temp_sum_right=0; //int shift_index=0;
	for (int i=0; i<size_of_ring; i++)
	{
		/*if (index>=i) 
		{
			shift_index=index-i;
		}
		else 
		{
			shift_index=index+BUFFER_SIZE-i;
		}*/
		temp_sum_left+=left_ring[i];
		temp_sum_right+=right_ring[i];
	} 
	last_left_data = temp_sum_left / size_of_ring;
	last_right_data = temp_sum_right / size_of_ring;

    // TODO: Need to use the set_vehicle_position function (AP_Beacon_Backend)
    // to set the vehicle position based on the data received.
    // Check AP_Beacon_Pozyx.cpp for an example
    
    // Vehicles X Y Z Coordinates in Meters
    // O O  x    
    //  X   + y
    // O O  (z coming out of the screen)
    // TODO
    int32_t vehicle_x = 0;
    int32_t vehicle_y = 0;
    int32_t vehicle_z = 0;
    Vector3f veh_pos(Vector3f(vehicle_x / 1000.0f, vehicle_y / 1000.0f, -vehicle_z / 1000.0f));

    // Set position in meters, position accuracy is 1.
    // NOTE: This might be as simple as changing the y position slightly
    // as the yaw (twist) is changed automatically w/ this function
    set_vehicle_distance(veh_pos, 1);

	last_update_ms = AP_HAL::millis();
}

// Convert PWM echo signal into distance
// returns distance in m?
uint16_t AP_Beacon_Ultrasound::_dis_calculation(uint32_t echo_time)
{
	// distance(cm) = .0347cm/us * echo_time(us)
    return round(echo_time*340.0/10000);	
}

