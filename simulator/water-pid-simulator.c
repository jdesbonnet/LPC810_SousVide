/**
 * Simulate controlling water temperature of a slow cooking pot (eg rice
 * cooker). 
 *
 * Copyright Joe Desbonnet, jdesbonnet@gmail.com, 14 Aug 2013.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

// Specific heat capacity of water (J/kg/K)
#define SHC_H2O (4186.0)

typedef struct {
	double t_start;
	double t_end;
	double dt;
	double set_point_temperature;
	double Kp;
	double Ki;
	double Kd;
	double water_mass; // Mass of water in kg
	double ambient_temperature; // Ambient temperature in °C
	double sensor_time_constant; // Sensor temperature equalization tc
	double temperature_tolerance; // Max +/- °C allowed
	double heater_pwm_period;
} simulation_parameters_t;

int main (int argc, char ** argv) {

	int i;
	simulation_parameters_t param;

	// defaults
	param.t_start=0.0;
	param.t_end=7200.0;
	param.dt = 0.1;
	param.set_point_temperature=60.0;
	param.water_mass = 1.0;
	param.ambient_temperature = 20.0;
	param.sensor_time_constant = 1.0/300.0;
	param.temperature_tolerance = 0.25;
	param.heater_pwm_period = 60;

	for (i = 1; i < argc; i++) {
		if (strcmp("-ts",argv[i])==0) {
			param.t_start = atof(argv[i+1]);
			i++;
		}
		if (strcmp("-te",argv[i])==0) {
			param.t_end = atof(argv[i+1]);
			i++;
		}

		if (strcmp("-dt",argv[i])==0) {
			param.dt = atof(argv[i+1]);
			i++;
		}

		if (strcmp("-setpoint",argv[i])==0) {
			param.set_point_temperature = atof(argv[i+1]);
			i++;
		}

		if (strcmp("-Kp",argv[i])==0) {
			param.Kp = atof(argv[i+1]);
			i++;

		}
		if (strcmp("-Ki",argv[i])==0) {
			param.Ki = atof(argv[i+1]);
			i++;
		}
		if (strcmp("-Kd",argv[i])==0) {
			param.Kd = atof(argv[i+1]);
			i++;
		}
		if (strcmp("-l",argv[i])==0) {
			param.water_mass = atof(argv[i+1]);
			i++;
		}
		if (strcmp("-tol",argv[i])==0) {
			param.temperature_tolerance = atof(argv[i+1]);
			i++;
		}
		if (strcmp("-pwmp",argv[i])==0) {
			param.heater_pwm_period = atof(argv[i+1]);
			i++;
		}
	}

	run_simulation(param);
}

int time_equals(double ta, double tb) {
	double d = tb-ta;
	return (d < 1e-6 && d > -1e-6);
}

int run_simulation (simulation_parameters_t param) {

	int i = 0;
	double heater_power = 400;
	double dt = 0.1;

	double time;
	double sensor_temperature = param.ambient_temperature;
	double water_temperature = param.ambient_temperature;
	
	double cooling_constant = 0.0001;

	double integral = 0;
	double derivative = 0;
	double error;
	double prev_error = param.set_point_temperature - sensor_temperature;
	double output;

	int heater_on = 0;
	double heater_pwm_dutycycle = 0;

	// Time at which temperature has settled within tolerated range
	double settle_time;


	int t0,t1,t2,t3,delta,expfrac,j;



	for (time = param.t_start; time < param.t_end; time+=dt) {

		//
		// Update the model
		//

		// Energy in from heating element
		if (heater_on) {
			water_temperature += heater_power * param.water_mass * dt / SHC_H2O;
		}

		// Energy out due to heat loss
		water_temperature -= (water_temperature - param.ambient_temperature) * dt * cooling_constant;
		if (water_temperature > 100.0) {
			water_temperature = 100.0;
		}

		// Temperature sensor
		sensor_temperature += (water_temperature - sensor_temperature) * dt * param.sensor_time_constant; // ?

		if ( (water_temperature >= param.set_point_temperature-param.temperature_tolerance) 
			&& (water_temperature < param.set_point_temperature+param.temperature_tolerance) ) {
			if (settle_time==0) {
				settle_time=time;
			}
		} else {
			settle_time=0;
		}


		// Simulation iteration is every dt, but PID is evaluated less often.
		if (i++ % 10 == 0) {
			// Update PID calculations.
			error = param.set_point_temperature - sensor_temperature;
		
			integral += error*dt*10.0;
			derivative = (error-prev_error)/(dt*10);
			output = param.Kp * error + param.Ki * integral + param.Kd * derivative;
			prev_error = error;

			// Heater element PWM
			if (output <= 0) {
			//if (output <= 0 || time > 3600) {
				//heater_on = 0;
				heater_pwm_dutycycle=0;
			} else {
				heater_pwm_dutycycle = output/100.0;
				if (heater_pwm_dutycycle>1.0) {
					heater_pwm_dutycycle = 1.0;
				}

				
			}

			// Experimental test burn idea
			if (time == 0) {
				t0 = sensor_temperature;
			}
			if (time < 900.0) {
				heater_pwm_dutycycle=0.0;
			}
			if (time < 60.0) {
				heater_pwm_dutycycle=1.0;
			}
			if (time_equals(time,60.0)) {
fprintf (stderr,"T60\n");
				t1 = sensor_temperature*1000;
			}
			if (time>=60.0 && time < 120.0) {
				heater_pwm_dutycycle=0.0;
			}
			if (time_equals(time,120.0)) {
fprintf (stderr,"T120\n");
				t2 = sensor_temperature*1000;
			}
			if (time_equals(time,180.0)) {
fprintf (stderr,"T180\n");
				t3 = sensor_temperature*1000;
				expfrac = ((t3-t2)*256)/(t2-t1);
fprintf (stderr,"t3=%d, t2=%d t1=%d t0=%d expfrac=%d\n",t3,t2,t1,t0,expfrac);
				j = 0;
				do {
					delta = (t3-t2) * expfrac;
					t2 = t3;
					t3 += delta/256;
					j++;
				} while (delta > 16);
				fprintf (stderr,"estimated final temp=%d j=%d delta=%d\n", t3,j,delta);
				// reset PID
				integral = 0;
			}	

		}

		heater_on = (fmod(time,param.heater_pwm_period) >= heater_pwm_dutycycle * param.heater_pwm_period )  ?  0 : 1;

		fprintf (stdout, "%f %f %f   %f %f %f %d   %f %f %f\n", time, water_temperature, sensor_temperature, error, output, heater_pwm_dutycycle, heater_on , 
				param.Kp*error,
				param.Ki*integral,
				param.Kd*derivative
		);
	}

	fprintf (stderr,"settle_time=%f\n", settle_time);

}


