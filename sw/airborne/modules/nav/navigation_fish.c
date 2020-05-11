/*
 * Copyright (C) Adjiri Adam <adam.adjiri@etu.isae-ensma.fr>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/nav/navigation_fish.c"
 * @author Adjiri Adam <adam.adjiri@etu.isae-ensma.fr>
 * Navigate Bebops like fish
 */

#include "modules/nav/navigation_fish.h"
#include "modules/multi/traffic_info.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "state.h"
#include "autopilot.h"
#include "subsystems/navigation/waypoints.h"
#include "math/pprz_geodetic_float.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define BODY_LENGTH 0.2
#define DIR_FLUCT 0.1
double sign(double x);
double normal_random_gen();
double distance_to_wall();
double angle_to_wall();
double calculate_new_heading();
bool calculate_next_destination();
static float CurrentX=1.0;
static float CurrentY=1.0;
double headingo=0.0;
double heading=0.0;
int i=0;
int j=0;

//  mathematical sign function
double sign(double x){
	if(x>=0){
		return 1.0;
	}else{
		return -1.0;
	}

}


/* Gaussian random number generator 
 * mean =0 invariance =1
 * using Box-Muller method
 */
double normal_random_gen(){
	double random1=((double)rand())/((double)(RAND_MAX));
	double random2=((double)rand())/((double)(RAND_MAX));
	return cos(2*3.14*random1)*sqrt(-2.*log(random2));
}


// Calculates distance between the uav and wall
double distance_to_wall(){
	if (10.0-sqrt(CurrentX*CurrentX + CurrentY*CurrentY) <0){
		return 0.000001;
	}else {
		return 10.0-sqrt(CurrentX*CurrentX + CurrentY*CurrentY);}
}


// calculates the relative orientation too the wall
double angle_to_wall(){
	if(CurrentX==0 && CurrentY==0){
		return (2*3.14 - heading);
	}else{
		return 2*3.14 - heading + sign(CurrentX)*acos(CurrentY/(sqrt(CurrentX*CurrentX + CurrentY*CurrentY))) ;
	}
}


//function that calculates new heading for the uav
double calculate_new_heading(){
	double rw=distance_to_wall();
	double tetaw=angle_to_wall();
	double fw=exp(pow(rw/(2*BODY_LENGTH),2)*(-1.0));
	double ow=(sin(tetaw))*(1+ 0.7*cos(2*tetaw));

	double new_heading=0.0;
	new_heading=heading+(DIR_FLUCT*(1-(2/3)*fw)*normal_random_gen())+fw*ow*4;			
	return new_heading;
}


//main function using velocity control
bool nav_fish_run_velocity(void){

	autopilot_set_mode(19);
	CurrentX = stateGetPositionEnu_f()->x;
	CurrentY = stateGetPositionEnu_f()->y;
	if (i==0){
	heading=calculate_new_heading();
	double estimated_distance=(sqrt(pow(CurrentX+(2*cos(heading)),2)+pow(CurrentY+(2*sin(heading)),2)));
		if(estimated_distance >10.0){
			j++;
			if(j==5){
				heading=heading+0.8;
				j=0;
			}
		}else{
        		guidance_h_set_guided_heading(heading);
			i=1;
		}


	}else if(i==1){
		guidance_h_set_guided_body_vel(1.0,0.0);
		i=2;
	}else{
		i=0;
		guidance_h_set_guided_body_vel(0.0,0.0);
	}
	return true;
}





// function that determines the next destination 
//mutually exclusive with nav_fish_run_velocity
bool calculate_next_destination(){
	if(j==0){
		float headingo=calculate_new_heading();
		i++;
		if(i==5){headingo=headingo+(3.14/3);}
		if(i==6){headingo=headingo-(3.14/3);i=0;}
		double estimated_distance=(sqrt(pow(CurrentX+(cos(headingo)),2)+pow(CurrentY+(sin(headingo)),2)));
		if(estimated_distance < 10.0){
			heading=headingo;
			CurrentX=CurrentX + (cos(heading));
			CurrentY=CurrentY + (sin(heading));
			i=0;
			struct EnuCoor_f x={CurrentX,CurrentY,5.0};
			waypoint_set_enu(4,&x);
			j=1;
		}
	}
	return true;
}

//main function for position control
//mutually exclusive with nav_fish_run_velocity
bool nav_fish_run_position(){
	if(j==1){
		NavGotoWaypoint(4);
		nav_set_heading_towards_waypoint(4);
		j=0;
	}
	return true;
}































	//acInfoGetPositionEnu_f(id);
//CurrentY=acInfoGetPositionEnu_f(id)->y;


