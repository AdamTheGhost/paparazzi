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
#include "generated/flight_plan.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define BODY_LENGTH 0.6
float sign(float x);
float normal_random_gen();
float distance_to_wall(float x ,float y);
float angle_to_wall();
float calculate_new_heading();
bool calculate_next_destination();
void navigation_fish_setFishParameter(float param);
void navigation_fish_setAngleParameter(float param);
float yw=0.8;
float DIR_FLUCT=0.3;
float ang=0.0;
float step_size=0.5;
static float CurrentX=1.0;
static float CurrentY=1.0;
float headingo=0.0;
float heading=0.0;
float new_heading=0.0;
float tetaw=0.0;
float rw=0.0;
int big_angles=0;
int stuck=0;
int standby=0;
int i=0;
int j=0;
int controlleur_frequence=4;
float heading200=0.0;
float x200=0.0;
float y200=0.0;
//  mathematical sign function
float sign(float x){
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
float normal_random_gen(){
	float random1=((float)rand())/((float)(RAND_MAX));
	float random2=((float)rand())/((float)(RAND_MAX));
	return cos(2*3.14*random1)*sqrt(-2.*log(random2));
}


// Calculates distance between the uav and wall
float distance_to_wall(float x, float y){
	if (10.0-sqrt(x*x + y*y) <0){
		return 0.000001;
	}else {
		return 10.0-sqrt(x*x + y*y);}
}


// calculates the relative orientation too the wall
float angle_to_wall(){
	if(CurrentX==0 && CurrentY==0){
		return ( - heading);
	}else{
		return  - heading + sign(CurrentX)*acos(CurrentY/(sqrt(CurrentX*CurrentX + CurrentY*CurrentY))) ;
	}
}


//function that calculates new heading for the uav
float calculate_new_heading(){
	rw=distance_to_wall(CurrentX,CurrentY);
	tetaw=angle_to_wall();
	float fw=exp(pow(rw/(2 *BODY_LENGTH),2)*(-1.0));
	float ow=(sin(tetaw))*(1+ 0.7*cos(2*tetaw));
	step_size=1.1-(fw);

	new_heading=(DIR_FLUCT*(1-(2/3)*fw)*normal_random_gen())+fw*ow*yw;		
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
		float diff_heading=calculate_new_heading();
	//	printf("  diff heading =%f   \n",diff_heading*180/3.14);
		float headingo_1=heading + diff_heading;
		float headingo_2=heading - diff_heading;
		float estimated_distance_1=distance_to_wall((CurrentX+(step_size*sin(headingo_1))),(CurrentY+(step_size*cos(headingo_1))));
		float estimated_distance_2=distance_to_wall((CurrentX+(step_size*sin(headingo_2))),(CurrentY+(step_size*cos(headingo_2))));;	
		if(estimated_distance_1 > 0.6){
			if(abs(heading-headingo_1) >1.57){
				big_angles++;
			}
			heading=headingo_1;
			
			CurrentX=CurrentX + (step_size*sin(heading));
			CurrentY=CurrentY + (step_size*cos(heading));
			i=0;
			struct EnuCoor_i x={POS_BFP_OF_REAL(CurrentX),POS_BFP_OF_REAL(CurrentY),POS_BFP_OF_REAL(5.0)};
			//struct EnuCoor_f x={CurrentX,CurrentY,5.0};
			//waypoint_set_enu(4,&x);
			waypoint_move_enu_i(WP_p1,&x);
			j=1;
			printf("%f,%f,%f\n",rw,tetaw,diff_heading);
		}else {
			if(estimated_distance_2 > 0.6){
				if(abs(heading-headingo_2) >1.57){
					big_angles++;
				}
			heading=headingo_2;
			
			CurrentX=CurrentX + (step_size*sin(heading));
			CurrentY=CurrentY + (step_size*cos(heading));
			i=0;
			struct EnuCoor_i x={POS_BFP_OF_REAL(CurrentX),POS_BFP_OF_REAL(CurrentY),POS_BFP_OF_REAL(5.0)};
			//struct EnuCoor_f x={CurrentX,CurrentY,5.0};
			//waypoint_set_enu(4,&x);
			waypoint_move_enu_i(WP_p1,&x);
			j=1;
			printf("%f,%f,%f\n",rw,tetaw,diff_heading);
			}
		}
	}
	return true;
}






//main function for position control
//mutually exclusive with nav_fish_run_velocity
bool nav_fish_run_position(){
	if(controlleur_frequence==0 || distance_to_wall(CurrentX,CurrentY) < 1.2){
		controlleur_frequence=4;
		if(standby>0){
			standby--;
		}else{
			if(j==1){
				nav_set_heading_towards_waypoint(4);
				NavGotoWaypoint(4);
			
				j=0;
			}else{
				stuck++;
			}
		}
	}else{
		controlleur_frequence--;
	}
	return true;
}










bool parameter_data(){
	if(DIR_FLUCT <=1.0){
		printf("for yw=%f   and DIR_FLUCT=%f   big angles = %d  and stuck times =%d \n",yw,DIR_FLUCT,big_angles,stuck);


	}
	yw=yw+0.5;
	if(yw >10){
		DIR_FLUCT=DIR_FLUCT+0.1;
		yw=0.5;
	}

	standby=4;
	CurrentX=1.0;
	CurrentY=1.0;
	heading=0.0;
	struct EnuCoor_i x={POS_BFP_OF_REAL(CurrentX),POS_BFP_OF_REAL(CurrentY),POS_BFP_OF_REAL(5.0)};
	waypoint_move_enu_i(WP_p1,&x);
	NavGotoWaypoint(4);
	nav_set_heading_deg(0);
	big_angles=0;
	stuck=0;
return true;


}































bool axis_test(){
//autopilot_set_mode(19);
nav_set_manual(0.4, 0.4, 0.4);
printf("ang=%f \n",ang);
return true;
}

bool speed_test(){
autopilot_set_mode(19);
guidance_h_set_guided_body_vel(1.0,0.0);
return true;
}
bool comm_test(){
	float xt=(acInfoGetPositionEnu_f(200)->x)*2.57;
	float yt=(acInfoGetPositionEnu_f(200)->y)*2.57;
	float course =acInfoGetCourse(200);
	float deltaX=xt-x200;
	float deltaY=yt-y200;
	if(sqrt(deltaX*deltaX+deltaY*deltaY) >0.2){
		heading200=sign(deltaX)*sign(deltaY)*acos(abs(deltaY)/(sqrt(deltaX*deltaX + deltaY*deltaY)));
		if(sign(deltaY)<0){heading200=heading200+3.14;}
	}
	printf("x= %f     y=%f   course1= %f   course2=%f \n",xt,yt,heading200*180/3.14,course*180/3.14);
	x200=xt;
	y200=yt;

	return true;
}


void navigation_fish_setFishParameter(float param){
printf("param =%f  \n",param);
printf("yw=%f \n",yw);
//nav_set_heading_deg(param);
}


void navigation_fish_setAngleParameter(float param){
nav_set_heading_deg(param);
/*heading=param*3.14/180;
CurrentX = stateGetPositionEnu_f()->x;
	CurrentY = stateGetPositionEnu_f()->y;
printf("angle =%f \n",angle_to_wall()*180/3.14);*/
}


