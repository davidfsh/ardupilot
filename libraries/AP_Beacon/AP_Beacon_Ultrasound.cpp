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
	// healthy if we have get a data in the past 25 ms(change the define in AP_Beacon.h)
	return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT-MS);
}

// update the state of the sensor
void AP_Beacon_Ultrasound::update()
{
	//Read the ultrasound of last cycle
	left_echo = _dis_calculation(_echo->read(ECHO_LEFT));
	right_echo = _dsi_calculation(_echo->read(ECHO_RIGHT));
	_xbee->printf("1");  // Synchronization the ultrasonic transmitter and the receiver clock
	_trigger->write(TRIGGER_CHANNEL, 10);
	
	left_ring[index]=left_echo;
	right_ring[index]=right_echo;
	index = (index+1)%BUFFER_SIZE;
	if (size_of_ring<BUFFER_SIZE) size_of_ring++;
		
	int temp_sum_left=0; int temp_sum_right=0; int shift_index=0;
	for (int i=0; i<size_of_ring; i++)
	{
		if (index>=i) 
		{
			shift_index=index-i;
		}
		else 
		{
			shift_index=index+BUFFER_SIZE-i;
		}
		temp_sum_left+=left_ring[shift_index];
		temp_sum_right+=right_ring[shift_index];
	} 
	last_left_data = temp_sum_left / size_of_ring;
	last_right_data = temp_sum_right / size_of_ring;
		
	last_update_ms = AP_HAL::millis();
}

// Convert PWM echo signal into distance
uint16_t AP_Beacon_Ultrasound::_dis_calculation(uint32_t echo_time)
{
	return round(echo_time*340.0/10000);	
}