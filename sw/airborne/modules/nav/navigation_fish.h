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

/** @file "modules/nav/navigation_fish.h"
 * @author Adjiri Adam <adam.adjiri@etu.isae-ensma.fr>
 * Navigate Bebops like fish
 */

#ifndef NAVIGATION_FISH_H
#define NAVIGATION_FISH_H 
#include "std.h"
#include "modules/multi/traffic_info.h"
//#define YW 0.0
extern float YW,YATT,LATT,D0ATT,YALI,LALI,D0ALI;
extern float ang;
extern float DIR_FLUCT;
extern int myneighbor;
extern bool nav_fish_run_velocity(void);
extern bool calculate_next_destination();
extern bool nav_fish_run_position();
extern bool axis_test();
extern bool comm_test();
extern bool speed_test();
extern bool parameter_data();
void navigation_fish_setFishParameter(float param);
void navigation_fish_setAngleParameter(float param);
#endif  // NAVIGATION_FISH_H
