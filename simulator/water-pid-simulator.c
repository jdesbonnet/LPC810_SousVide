/**
 * Simulate controlling water temperature of a slow cooking pot (eg rice
 * cooker). 
 *
 * Copyright Joe Desbonnet, jdesbonnet@gmail.com, 14 Aug 2013.
 */

#include <stdio.h>
#include <math.h>

// Specific heat capacity of water (J/kg/K)
#define SHC_H2O (4186.0)


int main (int argc, char ** argv) {

	double element_power = 400;
	double dt = 0.1;
	double Kp = 1;
	double Ki = 0.001;
	double Kd = 0.001;

	double time;
	double water_quantity = 1.0;
	double ambient_temperature = 20;
	double sensor_temperature = 20;
	double water_temperature = 20;
	double set_point_temperature = 60;

	double sensor_time_constant = 1.0/300.0;
	double cooling_constant = 0.001;

	double integral = 0;
	double derivative = 0;
	double error, prev_error;
	double output;

	int element_on = 0;

	for (time = 0; time < 3600.0; time+=dt) {

		// Update the model
		if (element_on) {
			water_temperature += element_power * water_quantity * dt / SHC_H2O;
		}
		water_temperature -= (water_temperature - ambient_temperature) * dt * cooling_constant;
		if (water_temperature > 100.0) {
			water_temperature = 100.0;
		} 
		sensor_temperature += (water_temperature - sensor_temperature) * dt * sensor_time_constant; // ?


		// Update PID calculations
		error = set_point_temperature - sensor_temperature;
		integral += error*dt;
		derivative = (error-prev_error)/dt;
		output = Kp * error + Ki * integral + Kd * derivative;
		prev_error = error;

		// Heater element PWM
		if (output <= 0) {
			element_on = 0;
		} else {
			element_on = ( (output/10.0) > fmod(time,100.0) ) ? 0 : 1;
		}

		fprintf (stdout, "%f %f %f   %f %f  %d\n", time, water_temperature, sensor_temperature, error, output, element_on);
	}

}


