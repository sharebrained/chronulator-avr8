/*
 **********************************************************
 *
 * Copyright 2008-2013 ShareBrained Technology, Inc.
 *
 * This file is part of chronulator-avr8.
 *
 * chronulator-avr8 is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * chronulator-avr8 is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General
 * Public License along with chronulator-avr8. If not, see
 * <http://www.gnu.org/licenses/>.
 *
 **********************************************************
 */
 
#ifndef __CHRONULATOR_TYPES_H__
#define __CHRONULATOR_TYPES_H__

typedef enum meter_mode {
    METER_MODE_SHOW_TIME = 0,
    METER_MODE_CALIBRATE_ZERO_SCALE = 1,
    METER_MODE_CALIBRATE_FULL_SCALE = 2,
} meter_mode_t;

typedef enum button_mode {
    BUTTON_MODE_NONE = 0,
    BUTTON_MODE_HOURS = 1,
    BUTTON_MODE_MINUTES = 2,
    BUTTON_MODE_BOTH = 3,
} button_mode_t;

typedef enum _PowerMode {
    POWER_MODE_LOW_POWER,
    POWER_MODE_HIGH_POWER,
} PowerMode;

#endif//__CHRONULATOR_TYPES_H__
