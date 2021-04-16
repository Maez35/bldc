/*
 * built_in_test.c
 *
 *  Created on: Mar 15, 2021
 *      Author: motorcontrol
 */

#include "stm32f4xx_conf.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "hw.h"
#include "app.h"
#include "ch.h"
#include "hal.h"

#include "conf_general.h"
#include "commands.h"
#include "datatypes.h"
#include "mempools.h"
#include "terminal.h"
#include "timeout.h"
#include "utils.h"

#include "mc_interface.h"
#include "mcpwm_foc.h"

#include "built_in_test.h"

// Threads
static THD_WORKING_AREA(built_in_test_thread_wa, 1024);
static THD_FUNCTION(built_in_test_thread, arg);

// Private functions
static void terminal_cmd_BIT_check_offset(int argc, const char **argv);
static void terminal_cmd_BIT_single_pulse_test(int argc, const char **argv);
static void terminal_cmd_BIT_double_pulse_test(int argc, const char **argv);
static void terminal_cmd_BIT_check_all_leg(int argc, const char **argv);

void single_pulse_test (
		float first_pulse_dwell_us,
		float first_pulse_off_us,
		char leg[32],
		char side[32],
		char supply_leg[32]);

static void timer_reinit(int f_sw);
static void stop_pwm_hw(void);

#define TIMER_UPDATE_DUTY(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty2; \
		TIM1->CCR3 = duty3; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;

#define TIMER_UPDATE_SAMP(samp) \
		TIM2->CCR2 = (samp / 2);


bool BIT_status = false;


void built_in_test_init(void) {

	// Start threads
	chThdCreateStatic(built_in_test_thread_wa, sizeof(built_in_test_thread_wa), NORMALPRIO, built_in_test_thread, NULL);

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"BIT_check_offset",
			"Check sensor current and offset",
			0,
			terminal_cmd_BIT_check_offset);

	terminal_register_command_callback(
			"BIT_single_pulse_test",
			"Trigger a single pulse test - For example: BIT_single_pulse_test 10.2 2.3 A Bottom --supply-with C",
			"[pulse_dwell_us] [pulse_off_us] [leg: A, B or C] [side: T or B] [argument(optional) --supply-with] [supply_leg(optional): A, B or C]",
			terminal_cmd_BIT_single_pulse_test);

	terminal_register_command_callback(
			"BIT_double_pulse_test",
			"Trigger a double pulse test -  For example: BIT_double_pulse_test 10.2 2.3 3.0 A Bottom --supply-with C",
			"[first_pulse_dwell_us] [first_pulse_off_us] [second_pulse_dwell_us] [leg: A, B or C] [side: T or B] [argument(optional) --supply-with] [supply_leg(optional): A, B or C]",
			terminal_cmd_BIT_double_pulse_test);

	terminal_register_command_callback(
			"BIT_check_all_leg",
			"Check all legs with a simple pulse test for each leg combination - For example: BIT_check_all_leg 10.2 2.3",
			"[pulse_dwell_us] [pulse_off_us]",
			terminal_cmd_BIT_check_all_leg);
}


static THD_FUNCTION(built_in_test_thread, arg) {
	(void)arg;

	chRegSetThreadName("built in test");

	for(;;) {

		int curr0_offset;
		int curr1_offset;
		int curr2_offset;

		mcpwm_foc_get_current_offsets(&curr0_offset, &curr1_offset, &curr2_offset, false);

		built_in_test_1.m_motor_I_offset_L1 = curr0_offset;
		built_in_test_1.m_motor_I_offset_L2 = curr1_offset;
		built_in_test_1.m_motor_I_offset_L3 = curr2_offset;

		built_in_test_1.m_motor_V_L1 = ((float)ADC_Value[ADC_IND_SENS1] * V_REG / 4096.0) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
		built_in_test_1.m_motor_V_L2 = ((float)ADC_Value[ADC_IND_SENS2] * V_REG / 4096.0) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
		built_in_test_1.m_motor_V_L3 = ((float)ADC_Value[ADC_IND_SENS3] * V_REG / 4096.0) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;

		built_in_test_1.m_motor_I_L1 = (((float)(ADC_Value[ADC_IND_CURR1] - built_in_test_1.m_motor_I_offset_L1)) * V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);
		built_in_test_1.m_motor_I_L2 = (((float)(ADC_Value[ADC_IND_CURR2] - built_in_test_1.m_motor_I_offset_L2)) * V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);
#ifdef HW_HAS_3_SHUNTS
		built_in_test_1.m_motor_I_L3 = (((float)(ADC_Value[ADC_IND_CURR3] - built_in_test_1.m_motor_I_offset_L3)) * V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);
#endif

		chThdSleepMilliseconds(1);

		if (BIT_status == true){
			// Reset the watchdog
			timeout_feed_WDT(THREAD_MCPWM);
		}
	}
}


// Callback function for the terminal command with arguments.
static void terminal_cmd_BIT_check_offset(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	BIT_status = true;
	/**************************************************************************************/
	utils_sys_lock_cnt();

	mc_configuration *mcconf 	 = mempools_alloc_mcconf();
	mc_configuration *mcconf_old = mempools_alloc_mcconf();

	*mcconf 	= *mc_interface_get_configuration();
	*mcconf_old = *mc_interface_get_configuration();

	// Disable timeout
	systime_t tout = timeout_get_timeout_msec();
	float tout_c = timeout_get_brake_current();
	timeout_reset();
	timeout_configure(65000, 0.0);

	int motor_old = mc_interface_get_motor_thread();

	mc_interface_select_motor_thread(1);
	mc_interface_unlock();
	mc_interface_release_motor();
	mc_interface_lock();

	mc_interface_select_motor_thread(2);
	mc_interface_unlock();
	mc_interface_release_motor();
	mc_interface_lock();

	// Reset the watchdog
	timeout_feed_WDT(THREAD_MCPWM);

	timeout_configure_IWDT_slowest();

	/**************************************************************************************/

	if (mcconf_old->motor_type == MOTOR_TYPE_FOC)
	{
		mc_interface_select_motor_thread(1);
		mc_state m_state = mc_interface_get_state();
		commands_printf("State Motor Control %d \n", m_state);

		commands_printf("V L1: %.2f V \tV L2: %.2f V \tV L3: %.2f V", (double)built_in_test_1.m_motor_V_L1, (double)built_in_test_1.m_motor_V_L2, (double)built_in_test_1.m_motor_V_L3);
		commands_printf("I L1: %.2f A \tI L2: %.2f A \tI L3: %.2f A", (double)built_in_test_1.m_motor_I_L1, (double)built_in_test_1.m_motor_I_L2, (double)built_in_test_1.m_motor_I_L3);
		commands_printf("I-offset L1: %.2f A \tI-offset L2: %.2f A \tI-offset L3: %.2f A", (double)built_in_test_1.m_motor_I_offset_L1, (double)built_in_test_1.m_motor_I_offset_L2, (double)built_in_test_1.m_motor_I_offset_L3);
		commands_printf(" ");

		TIMER_UPDATE_DUTY(0, 0, 0);

		// Enable TIM1
		TIM_Cmd(TIM1, DISABLE);

		// Main Output Enable
		TIM_CtrlPWMOutputs(TIM1, DISABLE);

		TIM_ClearFlag(TIM1, TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3);

		TIM_DeInit(TIM1);

		timer_reinit(mcconf_old->foc_f_sw);

		chThdSleepMilliseconds(100);
	}


	/**************************************************************************************/
	timeout_configure_IWDT();

	timeout_feed_WDT(THREAD_MCPWM);

	timeout_configure(tout, tout_c);

	mc_interface_set_configuration(mcconf_old);

	mempools_free_mcconf(mcconf);
	mempools_free_mcconf(mcconf_old);

	mc_interface_select_motor_thread(1);
	mc_interface_release_motor();
	mc_interface_unlock();
	mc_interface_select_motor_thread(2);
	mc_interface_release_motor();
	mc_interface_unlock();

	mc_interface_select_motor_thread(motor_old);
	mc_interface_release_motor();

	utils_sys_unlock_cnt();
	/**************************************************************************************/

	commands_printf("");

	BIT_status = false;

}

static void terminal_cmd_BIT_single_pulse_test(int argc, const char **argv) {
//	utils_sys_lock_cnt();
//	mc_interface_lock();

	BIT_status = true;

	(void)argc;
	(void)argv;

	float first_pulse_dwell_us = 0;
	float first_pulse_off_us = 0;
	char leg[10];
	char supply_leg[10];
	char side[10];
	char argument[32];

	mc_interface_select_motor_thread(1);

	mc_state m_state = mc_interface_get_state();

	if (m_state == MC_STATE_RUNNING){
		commands_printf("Motor Control is Running \n");
		return;
	}

	commands_printf("State Motor Control %d \n", m_state);

	if( (argc == 5) || (argc == 7) ) {
		sscanf(argv[1], "%f", &first_pulse_dwell_us);
		sscanf(argv[2], "%f", &first_pulse_off_us);
		sscanf(argv[3], "%s", leg);
		sscanf(argv[4], "%s", side);

		// validate arguments
		if ( !((leg[0] == 'A') || (leg[0] == 'B') || (leg[0] == 'C'))){
			commands_printf("Invalid leg. Use A, B or C \n");
			return;
		}
		if ( !((side[0] == 'T') || (side[0] == 'B'))){
			commands_printf("Invalid side. Use T, TOP, B or BOTTOM \n");
			return;
		}

		if(argc == 7) {
			sscanf(argv[5], "%s", argument);
			sscanf(argv[6], "%s", supply_leg);

			if ( !((supply_leg[0] == 'A') || (supply_leg[0] == 'B') || (supply_leg[0] == 'C'))){
				commands_printf("Invalid supply leg. Use A, B or C \n");
				return;
			}
			if(leg[0] == supply_leg[0]) {
				commands_printf("Supply with a different leg. Aborting.");
				return;
			}
			if( strcmp(argument, "--supply-with") != 0 ){
				commands_printf("Invalid argument. Use ""--supply-with"" \n");
				return;
			}
		}
		else{
			argument[0] = 0;
			supply_leg[0] = 0;
		}

		/**************************************************************************************/
		mc_configuration *mcconf 	 = mempools_alloc_mcconf();
		mc_configuration *mcconf_old = mempools_alloc_mcconf();

		*mcconf 	= *mc_interface_get_configuration();
		*mcconf_old = *mc_interface_get_configuration();

		int motor_old = mc_interface_get_motor_thread();

		mc_interface_select_motor_thread(1);
		mc_interface_unlock();
		mc_interface_release_motor();
		mc_interface_lock();

		mc_interface_select_motor_thread(2);
		mc_interface_unlock();
		mc_interface_release_motor();
		mc_interface_lock();

		utils_sys_lock_cnt();

		// Disable timeout
		systime_t tout = timeout_get_timeout_msec();
		float tout_c = timeout_get_brake_current();
		timeout_reset();
		timeout_configure(65000, 0.0);

		timeout_configure_IWDT_slowest();

		// Reset the watchdog
		timeout_feed_WDT(THREAD_MCPWM);

		/**************************************************************************************/

		if (mcconf_old->motor_type == MOTOR_TYPE_FOC)
		{
			float period = first_pulse_dwell_us/1000000.0 + first_pulse_off_us/1000000.0;
			uint32_t foc_f_sw_old = mcconf->foc_f_sw;
			float old_period = 1.0/(float)foc_f_sw_old;

			if( period > old_period) {
				commands_printf("Period too long, must be less than %.2f usec. Aborting.", (double)(old_period * 1000000.0));
			}
			else{

				float m_I_max_set = mcconf->l_current_max;
				float m_V_max_set = mcconf->l_max_vin;
				float m_V_in = GET_INPUT_VOLTAGE();

				commands_printf("I Max set: %.2f A \tV Max set: %.2f V \tV in: %.2f V \n", (double)m_I_max_set,(double)m_V_max_set,(double)m_V_in);

				if(argc == 7) {
					commands_printf("%s %s %s %s %s %s %s",argv[0], argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]);
				}
				else {
					commands_printf("%s %s %s %s %s",argv[0], argv[1], argv[2], argv[3], argv[4]);
				}

				single_pulse_test(first_pulse_dwell_us, first_pulse_off_us, leg, side, supply_leg);

				timer_reinit(mcconf_old->foc_f_sw);

			}
		}

		/**************************************************************************************/
		timeout_configure_IWDT();

		// Reset the watchdog
		timeout_feed_WDT(THREAD_MCPWM);

		timeout_configure(tout, tout_c);

		mc_interface_set_configuration(mcconf_old);

		mempools_free_mcconf(mcconf);
		mempools_free_mcconf(mcconf_old);

		mc_interface_release_motor();

		mc_interface_select_motor_thread(1);
		mc_interface_unlock();
		mc_interface_select_motor_thread(2);
		mc_interface_unlock();

		utils_sys_unlock_cnt();

		mc_interface_select_motor_thread(motor_old);

		/**************************************************************************************/
		//		// Resume system operation
		//		__enable_irq();
//
//		commands_printf("Fired!");
	}
	else {
		commands_printf("4 or 6 arguments required. For example: BIT_single_pulse_test 10.2 2.3 A Bottom --supply-with C");
	}
	commands_printf(" ");

//	mc_interface_unlock();
//	utils_sys_unlock_cnt();

	BIT_status = false;
	// Reset the watchdog
	timeout_feed_WDT(THREAD_MCPWM);
}


static void terminal_cmd_BIT_check_all_leg(int argc, const char **argv){
	utils_sys_lock_cnt();
	(void)argc;
	(void)argv;

	float first_pulse_dwell_us = 0;
	float first_pulse_off_us = 0;

	mc_state m_state = mc_interface_get_state();

	if (m_state == MC_STATE_RUNNING){
		commands_printf("Motor Control is Running \n");
		return;
	}

	if (argc == 3) {
		sscanf(argv[1], "%f", &first_pulse_dwell_us);
		sscanf(argv[2], "%f", &first_pulse_off_us);

		/**************************************************************************************/
		mc_configuration *mcconf 	 = mempools_alloc_mcconf();
		mc_configuration *mcconf_old = mempools_alloc_mcconf();

		*mcconf 	= *mc_interface_get_configuration();
		*mcconf_old = *mc_interface_get_configuration();

//		int motor_old = mc_interface_get_motor_thread();
//
//		mc_interface_select_motor_thread(1);
//		mc_interface_unlock();
//		mc_interface_release_motor();
//		mc_interface_lock();
//
//		mc_interface_select_motor_thread(2);
//		mc_interface_unlock();
//		mc_interface_release_motor();
//		mc_interface_lock();

//		utils_sys_lock_cnt();

		timeout_configure_IWDT_slowest();

		// Disable timeout
		systime_t tout = timeout_get_timeout_msec();
		float tout_c = timeout_get_brake_current();
		timeout_reset();
		timeout_configure(65000, 0.0);

		// Reset the watchdog
		timeout_feed_WDT(THREAD_MCPWM);

		/**************************************************************************************/
		float period = first_pulse_dwell_us/1000000.0 + first_pulse_off_us/1000000.0;

		uint32_t foc_f_sw_old = mcconf->foc_f_sw;

		float old_period = 1.0/(float)foc_f_sw_old;

		if( period > old_period) {
			commands_printf("Period too long, must be less than %.2f usec. Aborting.", (double)(old_period * 1000000.0));
		}
		else{

			float m_I_max_set = mcconf->l_current_max;
			float m_V_max_set = mcconf->l_max_vin;
			float m_V_in = GET_INPUT_VOLTAGE();

			commands_printf("I Max set: %.2f A \tV Max set: %.2f V \tV in: %.2f V \n", (double)m_I_max_set,(double)m_V_max_set,(double)m_V_in);


			commands_printf("%s %s %s %s %s supply with  %s", argv[0], argv[1], argv[2], "A", "TOP", "B");
			single_pulse_test(first_pulse_dwell_us, first_pulse_off_us, "A","T","B");

			commands_printf("%s %s %s %s %s supply with  %s", argv[0], argv[1], argv[2], "A", "TOP", "C");
			single_pulse_test(first_pulse_dwell_us, first_pulse_off_us, "A","T","C");

			commands_printf("%s %s %s %s %s supply with  %s", argv[0], argv[1], argv[2], "B", "TOP", "A");
			single_pulse_test(first_pulse_dwell_us, first_pulse_off_us, "B","T","A");

			commands_printf("%s %s %s %s %s supply with  %s", argv[0], argv[1], argv[2], "B", "TOP", "C");
			single_pulse_test(first_pulse_dwell_us, first_pulse_off_us, "B","T","C");

			commands_printf("%s %s %s %s %s supply with  %s", argv[0], argv[1], argv[2], "C", "TOP", "A");
			single_pulse_test(first_pulse_dwell_us, first_pulse_off_us, "C","T","A");

			commands_printf("%s %s %s %s %s supply with  %s", argv[0], argv[1], argv[2], "C", "TOP", "B");
			single_pulse_test(first_pulse_dwell_us, first_pulse_off_us, "C","T","B");


			commands_printf("%s %s %s %s %s supply with  %s", argv[0], argv[1], argv[2], "A", "BOTTOM", "B");
			single_pulse_test(first_pulse_dwell_us, first_pulse_off_us, "A","B","B");

			commands_printf("%s %s %s %s %s supply with  %s", argv[0], argv[1], argv[2], "A", "BOTTOM", "C");
			single_pulse_test(first_pulse_dwell_us, first_pulse_off_us, "A","B","C");

			commands_printf("%s %s %s %s %s supply with  %s", argv[0], argv[1], argv[2], "B", "BOTTOM", "A");
			single_pulse_test(first_pulse_dwell_us, first_pulse_off_us, "B","B","A");

			commands_printf("%s %s %s %s %s supply with  %s", argv[0], argv[1], argv[2], "B", "BOTTOM", "C");
			single_pulse_test(first_pulse_dwell_us, first_pulse_off_us, "B","B","C");

			commands_printf("%s %s %s %s %s supply with  %s", argv[0], argv[1], argv[2], "C", "BOTTOM", "A");
			single_pulse_test(first_pulse_dwell_us, first_pulse_off_us, "C","B","A");

			commands_printf("%s %s %s %s %s supply with  %s", argv[0], argv[1], argv[2], "C", "BOTTOM", "B");
			single_pulse_test(first_pulse_dwell_us, first_pulse_off_us, "C","B","B");
		}

		/**************************************************************************************/
		timeout_feed_WDT(THREAD_MCPWM);

		timeout_configure(tout, tout_c);

		timeout_configure_IWDT();

		mcconf_old->foc_f_sw = foc_f_sw_old;

		mc_interface_set_configuration(mcconf_old);

		mempools_free_mcconf(mcconf);
		mempools_free_mcconf(mcconf_old);

//		mc_interface_release_motor();
//
//		mc_interface_select_motor_thread(1);
//		mc_interface_unlock();
//		mc_interface_select_motor_thread(2);
//		mc_interface_unlock();
//
//		utils_sys_unlock_cnt();
//
//		mc_interface_select_motor_thread(motor_old);

		/**************************************************************************************/
	}
	else {
		commands_printf("2 arguments required. For example: BIT_check_all_leg 10.2 2.3");
	}

	commands_printf(" ");

	utils_sys_unlock_cnt();
	timeout_feed_WDT(THREAD_MCPWM);
}

void single_pulse_test (
		float first_pulse_dwell_us,
		float first_pulse_off_us,
		char leg[10],
		char side[10],
		char supply_leg[10])
{

	/**************************************************************************************/
	mc_configuration *mcconf 	 = mempools_alloc_mcconf();
	mc_configuration *mcconf_old = mempools_alloc_mcconf();

	*mcconf 	= *mc_interface_get_configuration();
	*mcconf_old = *mc_interface_get_configuration();
//
//	// Reset the watchdog
//	timeout_feed_WDT(THREAD_TIMER);

	/**************************************************************************************/

	if(leg[0] == supply_leg[0]) {
		commands_printf("Supply with a different leg. Aborting.");
		return;
	}

	first_pulse_dwell_us /= 1000000.0;
	first_pulse_off_us /= 1000000.0;

	float period = first_pulse_dwell_us + first_pulse_off_us;
	uint32_t foc_f_sw_old = mcconf->foc_f_sw;
	float old_period = 1.0/(float)foc_f_sw_old;

	if( period > old_period) {
		commands_printf("Period too long, must be less than %.2f usec. Aborting.", (double)(old_period * 1000000.0));
		return;
	}

	uint32_t foc_f_sw_new= 1.0 / period;

//	chThdSleepMilliseconds(10);

	float top = SYSTEM_CORE_CLOCK / (int)foc_f_sw_new;
	float duty_first_f =  first_pulse_dwell_us / period;
	uint32_t compare_first = (uint32_t)(top * duty_first_f);

	//pulse test Bottom mosfets
	if(side[0] == 'B') {
		// clear pin top mosfets
		palClearPad(HW_TOP_CH1_GPIO, HW_TOP_CH1_PIN);
		palClearPad(HW_TOP_CH2_GPIO, HW_TOP_CH2_PIN);
		palClearPad(HW_TOP_CH3_GPIO, HW_TOP_CH3_PIN);
		// TOP  Re-Configuration: Channel 1 to 3 as gpio function push-pull
		palSetPadMode(HW_TOP_CH1_GPIO, HW_TOP_CH1_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
		palSetPadMode(HW_TOP_CH2_GPIO, HW_TOP_CH2_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
		palSetPadMode(HW_TOP_CH3_GPIO, HW_TOP_CH3_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
	}

	//pulse test Top mosfets
	else if(side[0] == 'T') {
		// clear bottom mosfets
		palClearPad(HW_BOTT_CH1_GPIO, HW_BOTT_CH1_PIN);
		palClearPad(HW_BOTT_CH2_GPIO, HW_BOTT_CH2_PIN);
		palClearPad(HW_BOTT_CH3_GPIO, HW_BOTT_CH3_PIN);
		// BOTTOM Re-Configuration: Channel 1 to 3 as gpio function push-pull
		palSetPadMode(HW_BOTT_CH1_GPIO, HW_BOTT_CH1_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
		palSetPadMode(HW_BOTT_CH2_GPIO, HW_BOTT_CH2_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
		palSetPadMode(HW_BOTT_CH3_GPIO, HW_BOTT_CH3_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);

		// when powerstage is bootstrapped, we need to pre-charge the top gate driver by driving the leg output
		// to zero volts.
		switch (leg[0]) {
		case 'A':
			palSetPad(HW_BOTT_CH1_GPIO, HW_BOTT_CH1_PIN);
			chThdSleepMicroseconds(100);
			palClearPad(HW_BOTT_CH1_GPIO, HW_BOTT_CH1_PIN);
			break;
		case 'B':
			palSetPad(HW_BOTT_CH2_GPIO, HW_BOTT_CH2_PIN);
			chThdSleepMicroseconds(100);
			palClearPad(HW_BOTT_CH2_GPIO, HW_BOTT_CH2_PIN);
			break;
		case 'C':
			palSetPad(HW_BOTT_CH3_GPIO, HW_BOTT_CH3_PIN);
			chThdSleepMicroseconds(100);
			palClearPad(HW_BOTT_CH3_GPIO, HW_BOTT_CH3_PIN);
			break;
		}
	}
	else
		return;

	switch (supply_leg[0]) {
	case 'A':
		if(side[0] == 'B')
			palSetPad(HW_TOP_CH1_GPIO, HW_TOP_CH1_PIN);		// PA8
		else if(side[0] == 'T')
			palSetPad(HW_BOTT_CH1_GPIO, HW_BOTT_CH1_PIN);	// PB13
		break;

	case 'B':
		if(side[0] == 'B')
			palSetPad(HW_TOP_CH2_GPIO, HW_TOP_CH2_PIN);		// PA9
		else if(side[0] == 'T')
			palSetPad(HW_BOTT_CH2_GPIO, HW_BOTT_CH2_PIN);	// PB14
		break;

	case 'C':
		if(side[0] == 'B')
			palSetPad(HW_TOP_CH3_GPIO, HW_TOP_CH3_PIN);		// PA10
		else if(side[0] == 'T')
			palSetPad(HW_BOTT_CH3_GPIO, HW_BOTT_CH3_PIN);	// PB15
		break;

	}

	//	chThdSleepMilliseconds(1);	// wait for mosfet to fully turn ON
	//	chThdSleepMicroseconds(100);

	TIM_DeInit(TIM1);

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	TIM1->CNT = 0;

	// TIM1 clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK / (int)foc_f_sw_new;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// Channel 1, 2 and 3 Configuration in PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = compare_first; 					//TIM1->ARR / 2;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	if(side[0] == 'B') {
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	}

	//pulse test top mosfet
	else if(side[0] == 'T') {
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	}

	switch (leg[0]) {
	case 'A':
		TIM_OC1Init(TIM1, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
		break;
	case 'B':
		TIM_OC2Init(TIM1, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
		break;
	case 'C':
		TIM_OC3Init(TIM1, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
		break;
	}

	TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM_ClearFlag(TIM1, TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3);

	// Enable TIM1
	TIM_Cmd(TIM1, ENABLE);

	// Main Output Enable
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	float m_V_in = GET_INPUT_VOLTAGE();

	float V_L1 = 0, V_L2 = 0, V_L3 = 0;
	float I_L1 = 0, I_L2 = 0, I_L3 = 0;

	int curr1_offset;
	int curr2_offset;
	int curr3_offset;

	mcpwm_foc_get_current_offsets(&curr1_offset, &curr2_offset, &curr3_offset, false);

	switch (leg[0]) {
	case 'A':
		while( !(TIM1->SR & (TIM_SR_CC1IF)) );
		TIM_ClearFlag(TIM1, TIM_FLAG_CC1);
		break;

	case 'B':
		while( !(TIM1->SR & (TIM_SR_CC2IF)) );
		TIM_ClearFlag(TIM1, TIM_FLAG_CC2);
		break;

	case 'C':
		while( !(TIM1->SR & (TIM_SR_CC3IF)) );
		TIM_ClearFlag(TIM1, TIM_FLAG_CC3);
		break;
	}

	TIMER_UPDATE_DUTY(0, 0, 0);

	TIM_Cmd(TIM1, DISABLE);
	TIM_CtrlPWMOutputs(TIM1, DISABLE);
	TIM_ClearFlag(TIM1, TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3);
	TIM_DeInit(TIM1);

//	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
//	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
//	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
//
//	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
//	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
//	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
//
//	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
//	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
//	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
//
//	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

	V_L1 = ((float)ADC_Value[ADC_IND_SENS1] * V_REG / 4096.0) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	I_L1 = (((float)(ADC_Value[ADC_IND_CURR1] - curr1_offset)) * V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);

	V_L2 = ((float)ADC_Value[ADC_IND_SENS2] * V_REG / 4096.0) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	I_L2 = (((float)(ADC_Value[ADC_IND_CURR2] - curr2_offset)) * V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);

	V_L3 = ((float)ADC_Value[ADC_IND_SENS3] * V_REG / 4096.0) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
#ifdef HW_HAS_3_SHUNTS
	I_L3 = (((float)(ADC_Value[ADC_IND_CURR3] - curr3_offset)) * V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);
#endif

	commands_printf("V L1: %.2f V \tV L2: %.2f V \tV L3: %.2f V", (double)V_L1,(double)V_L2,(double)V_L3);
	commands_printf("I L1: %.2f A \tI L2: %.2f A \tI L3: %.2f A", (double)I_L1,(double)I_L2,(double)I_L3);

	switch (leg[0]) {
	case 'A':
		if(side[0] == 'T')	{
			if ((m_V_in - V_L1) > 1.0)	commands_printf("Fault in V sensor of leg A");
		}
		if (fabsf(I_L1) < 1.0)			commands_printf("Fault in I sensor of leg A");
		break;
	case 'B':
		if(side[0] == 'T')	{
			if ((m_V_in - V_L2) > 1.0)	commands_printf("Fault in V sensor of leg B");
		}
		if (fabsf(I_L2) < 1.0)			commands_printf("Fault in I sensor of leg B");
		break;
	case 'C':
		if(side[0] == 'T')	{
			if ((m_V_in - V_L3) > 1.0)	commands_printf("Fault in V sensor of leg C");
		}
		if (fabsf(I_L3) < 1.0)			commands_printf("Fault in I sensor of leg C");
		break;
	}

	// turn off all mosfets
	palClearPad(HW_TOP_CH1_GPIO, HW_TOP_CH1_PIN);
	palClearPad(HW_TOP_CH2_GPIO, HW_TOP_CH2_PIN);
	palClearPad(HW_TOP_CH3_GPIO, HW_TOP_CH3_PIN);

	palClearPad(HW_BOTT_CH1_GPIO, HW_BOTT_CH1_PIN);
	palClearPad(HW_BOTT_CH2_GPIO, HW_BOTT_CH2_PIN);
	palClearPad(HW_BOTT_CH3_GPIO, HW_BOTT_CH3_PIN);

	// GPIO Configuration: Channel 1 to 6 as alternate function push-pull
	palSetPadMode(HW_TOP_CH1_GPIO, HW_TOP_CH1_PIN, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
	palSetPadMode(HW_TOP_CH2_GPIO, HW_TOP_CH2_PIN, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
	palSetPadMode(HW_TOP_CH3_GPIO, HW_TOP_CH3_PIN, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);

	palSetPadMode(HW_BOTT_CH1_GPIO, HW_BOTT_CH1_PIN, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
	palSetPadMode(HW_BOTT_CH2_GPIO, HW_BOTT_CH2_PIN, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
	palSetPadMode(HW_BOTT_CH3_GPIO, HW_BOTT_CH3_PIN, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);

	/**************************************************************************************/
//	// Reset the watchdog
//	timeout_feed_WDT(THREAD_TIMER);

//	mc_interface_set_configuration(mcconf_old);

	mempools_free_mcconf(mcconf);
	mempools_free_mcconf(mcconf_old);

	/**************************************************************************************/

	commands_printf(" ");
}


static void terminal_cmd_BIT_double_pulse_test(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	float first_pulse_dwell_us = 0;
	float first_pulse_off_us = 0;
	float second_pulse_dwell_us = 0;

	char leg[32];
	char supply_leg[32];
	char side[32];
	char argument[32];

	mc_state m_state = mc_interface_get_state();

	if (m_state == MC_STATE_RUNNING){
		commands_printf("Motor Control is Running \n");
		return;
	}


	if( (argc == 6) || (argc == 8) ) {
		sscanf(argv[1], "%f", &first_pulse_dwell_us);
		sscanf(argv[2], "%f", &first_pulse_off_us);
		sscanf(argv[3], "%f", &second_pulse_dwell_us);
		sscanf(argv[4], "%s", leg);
		sscanf(argv[5], "%s", side);

		// validate arguments
		if ( !((leg[0] == 'A') || (leg[0] == 'B') || (leg[0] == 'C')))
			return;
		if ( !((side[0] == 'T') || (side[0] == 'B')))
			return;

		if(argc == 8) {
			sscanf(argv[6], "%s", argument);
			sscanf(argv[7], "%s", supply_leg);

			// validate arguments
			if ( !((supply_leg[0] == 'A') || (supply_leg[0] == 'B') || (supply_leg[0] == 'C')))
				return;
		}
		else{
			argument[0] = 0;
			supply_leg[0] = 0;
		}


		if(argc == 8) {
			commands_printf("%s %s %s %s %s %s %s %s",argv[0], argv[1], argv[2], argv[3], argv[4], argv[5], argv[6], argv[7]);
		}
		else {
			commands_printf("%s %s %s %s %s %s",argv[0], argv[1], argv[2], argv[3], argv[4], argv[5]);
		}


		/**************************************************************************************/
		mc_configuration *mcconf 	 = mempools_alloc_mcconf();
		mc_configuration *mcconf_old = mempools_alloc_mcconf();

		*mcconf 	= *mc_interface_get_configuration();
		*mcconf_old = *mc_interface_get_configuration();

		// Disable timeout
		systime_t tout = timeout_get_timeout_msec();
		float tout_c = timeout_get_brake_current();

		int motor_old = mc_interface_get_motor_thread();

		mc_interface_select_motor_thread(1);
		mc_interface_unlock();
		mc_interface_release_motor();
		mc_interface_lock();

		mc_interface_select_motor_thread(2);
		mc_interface_unlock();
		mc_interface_release_motor();
		mc_interface_lock();

		utils_sys_lock_cnt();

		// Reset the watchdog
		timeout_feed_WDT(THREAD_MCPWM);

		timeout_configure_IWDT_slowest();

		//		commands_printf("Before t 100ms");
		//		chThdSleepMilliseconds(1000);
		//		commands_printf("After  t 100ms");

		/**************************************************************************************/

		//		float m_res = mcconf->foc_motor_r;
		//		float m_ind = mcconf->foc_motor_l;
		float m_I_max_set = mcconf->l_current_max;
		float m_V_max_set = mcconf->l_max_vin;
		float m_V_in = GET_INPUT_VOLTAGE();
		//		float m_T_charger_to_I_max_set;

		//		commands_printf("Motor Resistance %.2f mOhm", (double)m_res * 1000);
		//		commands_printf("Motor Inductance %.2f uHy ", (double)m_ind * 1000000);
		//
		commands_printf("I Max set: %.2f A \tV Max set: %.2f V \tV in: %.2f V", (double)m_I_max_set,(double)m_V_max_set,(double)m_V_in);
		//		commands_printf("I Max over motor: %.2f A ", (double)(m_V_in / m_res));

		//		m_T_charger_to_I_max_set = (double)((m_I_max_set / (m_V_in / m_res)) * (m_ind/m_res));

		//		commands_printf("Time to charge (60%%): %.2f useg", (double)((m_ind/m_res) * 1000000));
		//		commands_printf("Time to charge I set max: %.2f useg", (double)(m_T_charger_to_I_max_set * 1000000));


		first_pulse_dwell_us /= 1000000.0;
		first_pulse_off_us /= 1000000.0;
		second_pulse_dwell_us /= 1000000.0;

		float period = first_pulse_dwell_us + first_pulse_off_us;

		uint32_t foc_f_sw_old = mcconf->foc_f_sw;
		float old_period = 1.0/(float)foc_f_sw_old;

		if( period > old_period) {
			commands_printf("Period too long, must be less than %.2f usec. Aborting.", (double)(old_period * 1000000.0));
			return;
		}

		mcconf->foc_f_sw = 1.0 / period;

		chThdSleepMilliseconds(10);

		mc_interface_set_configuration(mcconf);

		float top = SYSTEM_CORE_CLOCK / (int)mcconf->foc_f_sw;

		float duty_first_f =  first_pulse_dwell_us / period;
		float duty_second_f =  second_pulse_dwell_us / period;

		uint32_t compare_first = (uint32_t)(top * duty_first_f);
		uint32_t compare_second = (uint32_t)(top * duty_second_f);
		//
		//		commands_printf("First pulse dwell %.2f usec", (double)(first_pulse_dwell_us * 1000000.0));
		//		commands_printf("First pulse off time %.2f usec", (double)(first_pulse_off_us * 1000000.0));
		//		commands_printf("Second pulse dwell %.2f usec", (double)(second_pulse_dwell_us * 1000000.0));
		//		commands_printf("Old Period time %.2f usec \nNew Period time %.2f usec", (double)(old_period * 1000000.0), (double)(period * 1000000.0));


		//pulse test Bottom mosfets
		if(side[0] == 'B') {

			// clear pin top mosfets
			palClearPad(HW_TOP_CH1_GPIO, HW_TOP_CH1_PIN);
			palClearPad(HW_TOP_CH2_GPIO, HW_TOP_CH2_PIN);
			palClearPad(HW_TOP_CH3_GPIO, HW_TOP_CH3_PIN);

			// TOP  Re-Configuration: Channel 1 to 3 as gpio function push-pull
			palSetPadMode(HW_TOP_CH1_GPIO, HW_TOP_CH1_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
			palSetPadMode(HW_TOP_CH2_GPIO, HW_TOP_CH2_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
			palSetPadMode(HW_TOP_CH3_GPIO, HW_TOP_CH3_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);

		}

		//pulse test Top mosfets
		if(side[0] == 'T') {

			// clear bottom mosfets
			palClearPad(HW_BOTT_CH1_GPIO, HW_BOTT_CH1_PIN);
			palClearPad(HW_BOTT_CH2_GPIO, HW_BOTT_CH2_PIN);
			palClearPad(HW_BOTT_CH3_GPIO, HW_BOTT_CH3_PIN);

			// BOTTOM Re-Configuration: Channel 1 to 3 as gpio function push-pull
			palSetPadMode(HW_BOTT_CH1_GPIO, HW_BOTT_CH1_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
			palSetPadMode(HW_BOTT_CH2_GPIO, HW_BOTT_CH2_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
			palSetPadMode(HW_BOTT_CH3_GPIO, HW_BOTT_CH3_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);

			// when powerstage is bootstrapped, we need to pre-charge the top gate driver by driving the leg output
			// to zero volts.
			switch (leg[0]) {
			case 'A':
				palSetPad(HW_BOTT_CH1_GPIO, HW_BOTT_CH1_PIN);
				chThdSleepMicroseconds(50);
				palClearPad(HW_BOTT_CH1_GPIO, HW_BOTT_CH1_PIN);
				break;
			case 'B':
				palSetPad(HW_BOTT_CH2_GPIO, HW_BOTT_CH2_PIN);
				chThdSleepMicroseconds(50);
				palClearPad(HW_BOTT_CH2_GPIO, HW_BOTT_CH2_PIN);
				break;
			case 'C':
				palSetPad(HW_BOTT_CH3_GPIO, HW_BOTT_CH3_PIN);
				chThdSleepMicroseconds(50);
				palClearPad(HW_BOTT_CH3_GPIO, HW_BOTT_CH3_PIN);
				break;
			}
		}

		if( strcmp(argument, "--supply-with") == 0 ) {
			if(leg[0] == supply_leg[0]) {
				commands_printf("Supply with a different leg. Aborting.");
				return;
			}
			//			commands_printf("Activating leg %s to supply the test.", supply_leg);

			switch (supply_leg[0]) {
			case 'A':
				if(side[0] == 'B')
					palSetPad(HW_TOP_CH1_GPIO, HW_TOP_CH1_PIN);		// PA8
				if(side[0] == 'T')
					palSetPad(HW_BOTT_CH1_GPIO, HW_BOTT_CH1_PIN);	// PB13

				break;
			case 'B':
				if(side[0] == 'B')
					palSetPad(HW_TOP_CH2_GPIO, HW_TOP_CH2_PIN);		// PA9
				if(side[0] == 'T')
					palSetPad(HW_BOTT_CH2_GPIO, HW_BOTT_CH2_PIN);	// PB14

				break;
			case 'C':
				if(side[0] == 'B')
					palSetPad(HW_TOP_CH3_GPIO, HW_TOP_CH3_PIN);		// PA10
				if(side[0] == 'T')
					palSetPad(HW_BOTT_CH3_GPIO, HW_BOTT_CH3_PIN);	// PB15

				break;
			}
		}


		chThdSleepMilliseconds(1);	// wait for mosfet to fully turn ON
		//		chThdSleepMicroseconds(100);


		//		// disable interrupts
		//		__disable_irq();
		//

		TIM_DeInit(TIM1);

		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  TIM_OCInitStructure;

		TIM1->CNT = 0;

		// TIM1 clock enable
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

		// Time Base configuration
		TIM_TimeBaseStructure.TIM_Prescaler = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK / (int)mcconf->foc_f_sw;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

		// Channel 1, 2 and 3 Configuration in PWM mode
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
		TIM_OCInitStructure.TIM_Pulse = compare_first; 					//TIM1->ARR / 2;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

		if(side[0] == 'B') {
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
			TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
		}

		//pulse test top mosfet
		if(side[0] == 'T') {
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
			TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
		}

		switch (leg[0]) {
		case 'A':
			TIM_OC1Init(TIM1, &TIM_OCInitStructure);
			TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
			break;
		case 'B':
			TIM_OC2Init(TIM1, &TIM_OCInitStructure);
			TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
			break;
		case 'C':
			TIM_OC3Init(TIM1, &TIM_OCInitStructure);
			TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
			break;
		}

		TIM_CCPreloadControl(TIM1, ENABLE);
		TIM_ARRPreloadConfig(TIM1, ENABLE);

		TIM_ClearFlag(TIM1, TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3);

		// Enable TIM1
		TIM_Cmd(TIM1, ENABLE);

		// Main Output Enable
		TIM_CtrlPWMOutputs(TIM1, ENABLE);

		float V_L1 = 0, V_L2 = 0, V_L3 = 0;
		float I_L1 = 0, I_L2 = 0, I_L3 = 0;

		int curr1_offset;
		int curr2_offset;
		int curr3_offset;

		mcpwm_foc_get_current_offsets(&curr1_offset, &curr2_offset, &curr3_offset, false);

		switch (leg[0]) {
		case 'A':
			while( !(TIM1->SR & (TIM_SR_CC1IF)) );
			TIM_ClearFlag(TIM1, TIM_FLAG_CC1);
			TIMER_UPDATE_DUTY(compare_second, 0, 0);
			while( !(TIM1->SR & (TIM_SR_CC1IF)) );
			TIM_ClearFlag(TIM1, TIM_FLAG_CC1);
			break;

		case 'B':
			while( !(TIM1->SR & (TIM_SR_CC2IF)) );
			TIM_ClearFlag(TIM1, TIM_FLAG_CC2);
			TIMER_UPDATE_DUTY(0, compare_second, 0);
			while( !(TIM1->SR & (TIM_SR_CC2IF)) );
			TIM_ClearFlag(TIM1, TIM_FLAG_CC2);
			break;

		case 'C':
			while( !(TIM1->SR & (TIM_SR_CC3IF)) );
			TIM_ClearFlag(TIM1, TIM_FLAG_CC3);
			TIMER_UPDATE_DUTY(0, 0, compare_second);
			while( !(TIM1->SR & (TIM_SR_CC3IF)) );
			TIM_ClearFlag(TIM1, TIM_FLAG_CC3);
			break;
		}

		TIMER_UPDATE_DUTY(0, 0, 0);

		// Enable TIM1
		TIM_Cmd(TIM1, DISABLE);

		// Main Output Enable
		TIM_CtrlPWMOutputs(TIM1, DISABLE);

		TIM_ClearFlag(TIM1, TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3);

		TIM_DeInit(TIM1);


		V_L1 = ((float)ADC_Value[ADC_IND_SENS1] * V_REG / 4096.0) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
		I_L1 = (((float)(ADC_Value[ADC_IND_CURR1] - curr1_offset)) * V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);

		V_L2 = ((float)ADC_Value[ADC_IND_SENS2] * V_REG / 4096.0) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
		I_L2 = (((float)(ADC_Value[ADC_IND_CURR2] - curr2_offset)) * V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);

		V_L3 = ((float)ADC_Value[ADC_IND_SENS3] * V_REG / 4096.0) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
#ifdef HW_HAS_3_SHUNTS
		I_L3 = (((float)(ADC_Value[ADC_IND_CURR3] - curr3_offset)) * V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);
#endif

		commands_printf("V L1: %.2f V \tV L2: %.2f V \tV L3: %.2f V", (double)V_L1,(double)V_L2,(double)V_L3);
		commands_printf("I L1: %.2f A \tI L2: %.2f A \tI L3: %.2f A", (double)I_L1,(double)I_L2,(double)I_L3);

		switch (leg[0]) {
		case 'A':
			//			if(side[0] == 'B')
			//			{
			////				palSetPad(HW_TOP_CH1_GPIO, HW_TOP_CH1_PIN);		// PA8
			//			}
			if(side[0] == 'T')
			{
				if ((m_V_in - V_L1) > 1.0)	commands_printf("Fault in V sensor of leg A");
			}

			if (fabsf(I_L1) < 1.0)		commands_printf("Fault in I sensor of leg A");
			break;
		case 'B':
			//			if(side[0] == 'B')
			//			{
			////				palSetPad(HW_TOP_CH2_GPIO, HW_TOP_CH2_PIN);		// PA9
			//			}
			if(side[0] == 'T')
			{
				if ((m_V_in - V_L2) > 1.0)	commands_printf("Fault in V sensor of leg B");
			}

			if (fabsf(I_L2) < 1.0)		commands_printf("Fault in I sensor of leg B");
			break;
		case 'C':
			//			if(side[0] == 'B')
			//			{
			////				palSetPad(HW_TOP_CH3_GPIO, HW_TOP_CH3_PIN);		// PA10
			//			}
			if(side[0] == 'T')
			{
				if ((m_V_in - V_L3) > 1.0)	commands_printf("Fault in V sensor of leg C");
			}
			if (fabsf(I_L3) < 1.0)		commands_printf("Fault in I sensor of leg C");
			break;
		}

		//		//delay supply mosfets turn OFF 0.4ms to prevent disturbing the diode recovery
		//		volatile int timeout = 10000;
		//		while (timeout--);



		// turn off all mosfets
		palClearPad(HW_TOP_CH1_GPIO, HW_TOP_CH1_PIN);
		palClearPad(HW_TOP_CH2_GPIO, HW_TOP_CH2_PIN);
		palClearPad(HW_TOP_CH3_GPIO, HW_TOP_CH3_PIN);

		palClearPad(HW_BOTT_CH1_GPIO, HW_BOTT_CH1_PIN);
		palClearPad(HW_BOTT_CH2_GPIO, HW_BOTT_CH2_PIN);
		palClearPad(HW_BOTT_CH3_GPIO, HW_BOTT_CH3_PIN);

		// GPIO Configuration: Channel 1 to 6 as alternate function push-pull
		palSetPadMode(HW_TOP_CH1_GPIO, HW_TOP_CH1_PIN, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
		palSetPadMode(HW_TOP_CH2_GPIO, HW_TOP_CH2_PIN, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
		palSetPadMode(HW_TOP_CH3_GPIO, HW_TOP_CH3_PIN, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);

		palSetPadMode(HW_BOTT_CH1_GPIO, HW_BOTT_CH1_PIN, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
		palSetPadMode(HW_BOTT_CH2_GPIO, HW_BOTT_CH2_PIN, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
		palSetPadMode(HW_BOTT_CH3_GPIO, HW_BOTT_CH3_PIN, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);

		/**************************************************************************************/

		timeout_configure_IWDT();

		// Reset the watchdog
		timeout_feed_WDT(THREAD_MCPWM);

		//		chThdSleepMilliseconds(100);

		timeout_configure(tout, tout_c);

		mc_interface_set_configuration(mcconf_old);

		mempools_free_mcconf(mcconf);
		mempools_free_mcconf(mcconf_old);

		mc_interface_release_motor();

		mc_interface_select_motor_thread(1);
		mc_interface_unlock();
		mc_interface_select_motor_thread(2);
		mc_interface_unlock();

		utils_sys_unlock_cnt();

		mc_interface_select_motor_thread(motor_old);

		/**************************************************************************************/

		//		// Resume system operation
		//		__enable_irq();

		commands_printf("Fired!");

	}
	else {
		commands_printf("4 or 6 arguments required. For example: double_pulse_test 10.2 2.3 A Bottom --supply-with C");
		commands_printf(" ");
	}
	commands_printf(" ");
	//	return;
}

//
//
///**
// * Try to apply the new timer settings. This is really not an elegant solution, but for now it is
// * the best I can come up with.
// */
//static void update_timer_attempt(void) {
//	utils_sys_lock_cnt();
//
//	// Set the next timer settings if an update is far enough away
//	if (!timer_struct.updated && TIM1->CNT > 10 && TIM1->CNT < (TIM1->ARR - 500)) {
//		// Disable preload register updates
//		TIM1->CR1 |= TIM_CR1_UDIS;
//		TIM8->CR1 |= TIM_CR1_UDIS;
//
//		// Set the new configuration
//		TIM1->ARR = timer_struct.top;
//		TIM1->CCR1 = timer_struct.duty;
//		TIM1->CCR2 = timer_struct.duty;
//		TIM1->CCR3 = timer_struct.duty;
//		TIM8->CCR1 = timer_struct.val_sample;
//		TIM1->CCR4 = timer_struct.curr1_sample;
//		TIM8->CCR2 = timer_struct.curr2_sample;
//#ifdef HW_HAS_3_SHUNTS
//		TIM8->CCR3 = timer_struct.curr3_sample;
//#endif
//
//		// Enables preload register updates
//		TIM1->CR1 &= ~TIM_CR1_UDIS;
//		TIM8->CR1 &= ~TIM_CR1_UDIS;
//		timer_struct.updated = true;
//	}
//
//	utils_sys_unlock_cnt();
//}

static void timer_reinit(int f_sw) {
	utils_sys_lock_cnt();

	TIM_DeInit(TIM1);
	TIM_DeInit(TIM8);
	TIM_DeInit(TIM2);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	TIM1->CNT = 0;
	TIM2->CNT = 0;
	TIM8->CNT = 0;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseStructure.TIM_Period = (SYSTEM_CORE_CLOCK / f_sw);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	// Channel 1, 2 and 3 Configuration in PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM1->ARR / 2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

	// Automatic Output enable, Break, dead time and lock configuration
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime =  conf_general_calculate_deadtime(HW_DEAD_TIME_NSEC, SYSTEM_CORE_CLOCK);
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

#ifdef HW_USE_BRK
	// Enable BRK function. Hardware will asynchronously stop any PWM activity upon an
	// external fault signal. PWM outputs remain disabled until MCU is reset.
	// software will catch the BRK flag to report the fault code
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
#else
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
#endif

	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);
	TIM_CCPreloadControl(TIM8, ENABLE);
	TIM_ARRPreloadConfig(TIM8, ENABLE);

	// ------------- Timer2 for ADC sampling ------------- //
	// Time Base configuration
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 250;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_CCPreloadControl(TIM2, ENABLE);

	// PWM outputs have to be enabled in order to trigger ADC on CCx
	TIM_CtrlPWMOutputs(TIM2, ENABLE);

	// TIM1 Master and TIM8 slave
#if defined HW_HAS_DUAL_MOTORS || defined HW_HAS_DUAL_PARALLEL
	// TODO: Explain. See: https://www.cnblogs.com/shangdawei/p/4758988.html
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Enable);
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
	TIM_SelectInputTrigger(TIM8, TIM_TS_ITR0);
	TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Trigger);
	TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Enable);
	TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Update);
	TIM_SelectInputTrigger(TIM2, TIM_TS_ITR1);
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
#else
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
	TIM_SelectInputTrigger(TIM2, TIM_TS_ITR0);
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
#endif

	// Enable TIM1 and TIM2
#ifdef HW_HAS_DUAL_MOTORS
	TIM8->CNT = TIM1->ARR;
#else
	TIM8->CNT = 0;
#endif
	TIM1->CNT = 0;
	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

//	// Prevent all low side FETs from switching on
//	stop_pwm_hw();

	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

	// Main Output Enable
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM8, ENABLE);

	// Sample intervals
	TIMER_UPDATE_SAMP(MCPWM_FOC_CURRENT_SAMP_OFFSET);

	// Enable CC2 interrupt, which will be fired in V0 and V7
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	utils_sys_unlock_cnt();

	nvicEnableVector(TIM2_IRQn, 6);
}

//static void stop_pwm_hw(void) {
//	motor->m_id_set = 0.0;
//	motor->m_iq_set = 0.0;
//
////		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
////		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
////		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
////
////		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
////		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
////		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
////
////		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
////		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
////		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
////
////		TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
//
//		motor->m_output_on = false;
//
//}
