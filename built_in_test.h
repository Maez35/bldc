/*
	Copyright 2020 Martin Paz mpaz@paltatech.com

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef BUILT_IN_TEST_H_
#define BUILT_IN_TEST_H_

typedef struct {

	float m_motor_V_L1;
	float m_motor_V_L2;
	float m_motor_V_L3;
	float m_motor_I_L1;
	float m_motor_I_L2;
	float m_motor_I_L3;
	int m_motor_I_offset_L1;
	int m_motor_I_offset_L2;
	int m_motor_I_offset_L3;
} built_in_test_t;


// Channel 1,2 and 3 TOP pins
#define HW_TOP_CH1_GPIO			GPIOA
#define HW_TOP_CH1_PIN			8
#define HW_TOP_CH2_GPIO			GPIOA
#define HW_TOP_CH2_PIN			9
#define HW_TOP_CH3_GPIO			GPIOA
#define HW_TOP_CH3_PIN			10

// Channel 1,2 and 3 BOTTOM pins
#define HW_BOTT_CH1_GPIO		GPIOB
#define HW_BOTT_CH1_PIN			13
#define HW_BOTT_CH2_GPIO		GPIOB
#define HW_BOTT_CH2_PIN			14
#define HW_BOTT_CH3_GPIO		GPIOB
#define HW_BOTT_CH3_PIN			15


// Private variables
static volatile built_in_test_t built_in_test_1;

// Functions
void built_in_test_init(void);


#endif /* BUILT_IN_TEST_H_ */
