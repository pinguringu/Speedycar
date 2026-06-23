/*
 * main.h
 *
 *  Created on: 14.03.2023
 *      Author: eloseguigarc
 */

#include <LoRa_base.h>
#include <stdio.h>
#include "cybsp.h"
#include "cy_utils.h"
#include "cy_retarget_io.h"

#ifndef MAIN_H_
#define MAIN_H_

/* Declarations for systick counters */

#define TICKS_PER_SECOND       		(10000u)	//Rate of systick trigger per second
#define TICKS_WAIT_LIGHT       		(10000u)
#define TICKS_WAIT_CONTROL_READING	(4u)

//various variables
uint16_t counter = 0;
uint16_t min_PWM = 64;
uint16_t max_PWM = 132;
uint16_t delta_PWM = 64;
uint16_t current_delta = 0;

/* Variables to keep the PWM statuses */
uint16_t current_PWM_THROTTLE = 0;
uint16_t current_PWM_STEERING = 0;
uint16_t current_PWM_BUZZER = 0;

//packet structure
#define SOP (0b11100010)
#define EOP (0b00011101)

/* Flags */
uint8_t flag_receive = 0;
uint8_t flag_control_reading = 0;
uint8_t flag_flaps = 0;
uint8_t flag_abort_button = 0;
uint8_t flag_abort = 0;
uint8_t flag_disconnect = 0;
uint8_t flag_buzzer = 0;

uint16_t cnt_abort_button = 0;
uint16_t max_abort_button = 20;
uint16_t max_disconnect = 10;
uint16_t cont_disconnect = 0;

uint8_t delta_elevator = 0; //elevator up tilt for landing sequence

uint8_t flag_headlights = 0;

//variables to read the response of the LoRa in UART
uint8_t SPI_buffer_received[16];
uint8_t SPI_buffer_received_cnt;

//variables to store the message from transmitter
uint8_t length = 0;
uint8_t result_arr[7];

/** Definitions **/
/*Period value of CCUs for servomotor, to achieve 50Hz for servomotors */
#define PR_50HZ			(1563)
/*Match value which gives 1ms pulse for servomotors (minimum value) */
#define PR_ZERO 		(78)
/*Match value which gives 1.5ms pulse for servomotors (medium value) */
#define PR_MID 			(118)
/*Match value which gives 2ms pulse for servomotors (maximum value) */
#define PR_FULL 		(156)
/*Match value for  PWM CCU period*/
#define PR_PERIOD_PWM	(1900)
/*Half of period*/
#define PR_DC_MID		(PR_PERIOD_PWM >> 1)
/*Max PWM allowed*/
#define PR_DC_MAX		(PR_PERIOD_PWM - 50)
/*Min PWM allowed*/
#define PR_DC_MIN		(50)
/*Offset for the throttle joystick*/
#define OFFSET_JS_THROTTLE	(31)
/*Offset for the steering joystick*/
#define OFFSET_JS_STEER	(34)
/* Frequency of the loop which decodes message and assigns PWMs */
#define MAIN_LOOP_FREQ	(10)
/* Frequency for the transmission of the wireless packet */
#define WIRELESS_FREQ 	(868E6)

/* Buzzer period */
#define BUZZER_PERIOD				(1000)
/* Comparator for buzzer to beep */
#define BEEP						(500)
/* Comparator for buzzer to not beep */
#define NOBEEP						(1)

/* Bands to judge if motor should move forward or backward */
#define BAND_FORWARD				(OFFSET_JS_THROTTLE - 3)
#define BAND_BACKWARD				(OFFSET_JS_THROTTLE + 3)
#define BAND_STEER_LEFT				(OFFSET_JS_STEER + 5)
#define BAND_STEER_RIGHT			(OFFSET_JS_STEER - 5)

/*When ERU is not working because I am inept, manually call interrupt */
#define ERU_NOT_WORKING (0)



#define INTERRUPT_PRIORITY_NODE_ID          eru_1_ogu_0_IRQN
#define INTERRUPT_EVENT_PRIORITY            eru_1_ogu_0_NUM
#define ERU_EXTERNAL_EVENT_HANDLER          eru_1_ogu_0_INTERRUPT_HANDLER



//function definitions
uint8_t messageInterpreter(uint8_t message[], uint8_t length);
void ERU_EXTERNAL_EVENT_HANDLER(void);
void abortProcedure(void);
void activateFlaps(uint8_t level);
void startReport(void);

/** Configure PWM outputs as GPIO. */
#define PWM_OFF()   {PORT0->IOCR0 &= ~(uint32_t)(0x7F7F7F7F);\
        			PORT0->IOCR4 &= ~(uint32_t)(0x00007F7F);}
/** Configure PWM outputs as CCU80 output. */
#define PWM_ON()    {PORT0->IOCR0 |= 0x98989898; /* ALT3 Push-pull. */ \
                    PORT0->IOCR4 |= 0x00009898; /* ALT3 Push-pull. */ ;}

/* Throttle, Ailerons, Rudder, Elevator */
#define UPDATE_DUTY(comp0, comp1, comp2, comp3)	{CCU80_CC80->CR1S = comp0; /* PWM duty cycle. */ \
									CCU80_CC81->CR1S = comp1; /* PWM duty cycle. */ \
									CCU80_CC82->CR1S = comp2; /* PWM duty cycle. */ \
                                	CCU80->GCSS = (CCU8_GCSS_S2SE_Msk |CCU8_GCSS_S1SE_Msk | CCU8_GCSS_S0SE_Msk); /* Shadow transfer allowed. */ \
}
#define UPDATE_STEERING_DUTY(comp) {PWM_STEERING_HW->CRS = comp;\
                                	CCU41->GCSS = CCU4_GCSS_S3SE_Msk;\
}

#endif /* MAIN_H_ */
