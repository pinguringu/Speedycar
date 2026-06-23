/*
 * main.h
 *
 *  Created on: 14.03.2023
 *      Author: eloseguigarc
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "LoRa_base.h"
#include <stdio.h>
#include "cybsp.h"
#include "cy_utils.h"
#include "cy_retarget_io.h"

/* ── Timing ────────────────────────────────────────────────────────────────── */
#define TICKS_PER_SECOND            (10000u)
#define TICKS_WAIT_CONTROL_READING  (4u)

/* ── Packet framing ────────────────────────────────────────────────────────── */
#define SOP  (0b11100010)
#define EOP  (0b00011101)

/* ── PWM periods and compare values ───────────────────────────────────────── */
/* Period for motor PWM at the chosen CCU frequency */
#define PR_PERIOD_PWM   (1900)
/* Mid-point: motors stopped */
#define PR_DC_MID       (PR_PERIOD_PWM >> 1)
/* Maximum compare value allowed */
#define PR_DC_MAX       (PR_PERIOD_PWM - 50)
/* Minimum compare value allowed */
#define PR_DC_MIN       (50)

/* ── Joystick offsets ──────────────────────────────────────────────────────── */
/* Centre value of the throttle joystick axis */
#define OFFSET_JS_THROTTLE  (31)
/* Centre value of the steering joystick axis */
#define OFFSET_JS_STEER     (34)

/* ── Deadbands ─────────────────────────────────────────────────────────────── */
#define BAND_FORWARD        (OFFSET_JS_THROTTLE - 3)
#define BAND_BACKWARD       (OFFSET_JS_THROTTLE + 3)
#define BAND_STEER_LEFT     (OFFSET_JS_STEER + 5)
#define BAND_STEER_RIGHT    (OFFSET_JS_STEER - 5)

/* ── Buzzer ────────────────────────────────────────────────────────────────── */
#define BUZZER_PERIOD   (1000)
#define BEEP            (500)
#define NOBEEP          (1)

/* ── Main loop frequency (Hz) ──────────────────────────────────────────────── */
#define MAIN_LOOP_FREQ  (10)

/* ── LoRa frequency ────────────────────────────────────────────────────────── */
#define WIRELESS_FREQ   (868E6)

/* ── ERU interrupt mapping ─────────────────────────────────────────────────── */
#define INTERRUPT_PRIORITY_NODE_ID  eru_1_ogu_0_IRQN
#define INTERRUPT_EVENT_PRIORITY    eru_1_ogu_0_NUM
#define ERU_EXTERNAL_EVENT_HANDLER  eru_1_ogu_0_INTERRUPT_HANDLER

/* ── Debug workaround flag ─────────────────────────────────────────────────── */
#define ERU_NOT_WORKING (0)

/* ── Global variables (defined in main.c) ──────────────────────────────────── */
extern uint16_t current_PWM_THROTTLE;
extern uint16_t current_PWM_STEERING;

extern uint8_t  flag_receive;
extern uint8_t  flag_control_reading;
extern uint8_t  flag_abort_button;
extern uint8_t  flag_abort;
extern uint8_t  flag_disconnect;
extern uint8_t  flag_headlights;

extern uint16_t cnt_abort_button;
extern uint16_t max_abort_button;
extern uint16_t max_disconnect;
extern uint16_t cont_disconnect;

extern uint8_t  length;
extern uint8_t  result_arr[7];

#if ENABLE_XMC_DEBUG_PRINT
extern uint8_t  SPI_buffer_received[16];
extern uint8_t  SPI_buffer_received_cnt;
#endif

/* ── PWM output control macros ─────────────────────────────────────────────── */
/** Set PWM output pins as GPIO (motors off, silent). */
#define PWM_OFF()  { PORT0->IOCR0 &= ~(uint32_t)(0x7F7F7F7F); \
                     PORT0->IOCR4 &= ~(uint32_t)(0x00007F7F); }
/** Set PWM output pins as CCU80 alternate function (motors driven). */
#define PWM_ON()   { PORT0->IOCR0 |= 0x98989898; \
                     PORT0->IOCR4 |= 0x00009898; }

/** Update CCU80 compare registers for all three PWM channels. */
#define UPDATE_DUTY(comp0, comp1, comp2, comp3) \
    { CCU80_CC80->CR1S = (comp0); \
      CCU80_CC81->CR1S = (comp1); \
      CCU80_CC82->CR1S = (comp2); \
      CCU80->GCSS = (CCU8_GCSS_S2SE_Msk | CCU8_GCSS_S1SE_Msk | CCU8_GCSS_S0SE_Msk); }

/* ── Function declarations ─────────────────────────────────────────────────── */
uint8_t messageInterpreter(uint8_t message[], uint8_t length);
void    ERU_EXTERNAL_EVENT_HANDLER(void);
void    abortProcedure(void);
void    activateFlaps(uint8_t level);
void    startReport(void);

#endif /* MAIN_H_ */
