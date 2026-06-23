/******************************************************************************
 * File Name: main.c
 *
 * SpeedyCar — LoRa receiver
 * Receives a 7-byte LoRa packet at 868 MHz and drives two tank-track motors
 * via CCU80 PWM outputs on an XMC4200 / Custom4200 board.
 *****************************************************************************/
#include "main.h"
#include "xmc_spi.h"

/* Define macro to enable/disable printing of debug messages */
#define ENABLE_XMC_DEBUG_PRINT (0)

/* ── Global variable definitions (declared extern in main.h) ──────────────── */
uint16_t current_PWM_THROTTLE = 0;
uint16_t current_PWM_STEERING = 0;

uint8_t  flag_receive         = 0;
uint8_t  flag_control_reading = 0;
uint8_t  flag_abort_button    = 0;
uint8_t  flag_abort           = 0;
uint8_t  flag_disconnect      = 0;
uint8_t  flag_headlights      = 0;

uint16_t cnt_abort_button     = 0;
uint16_t max_abort_button     = 20;
uint16_t max_disconnect       = 10;
uint16_t cont_disconnect      = 0;

uint8_t  length               = 0;
uint8_t  result_arr[7];

#if ENABLE_XMC_DEBUG_PRINT
uint8_t  SPI_buffer_received[16];
uint8_t  SPI_buffer_received_cnt = 0;
#endif

/* ── File-local variables ─────────────────────────────────────────────────── */
/* Systick counter for control-reading period */
static uint16_t ticks_control  = 0;
/* Set to 1 only when the main loop is ready to accept the next message */
static uint8_t  flag_loop_done = 1;

/* ── Helpers ──────────────────────────────────────────────────────────────── */

/** Busy-wait for approximately 'cycles' NOP cycles. */
static inline void delay_cycles(uint32_t cycles)
{
    for (uint32_t k = 0; k < cycles; k++) { __NOP(); }
}

/**
 * Apply a PWM modification to PR_DC_MID and clamp the result.
 *
 * forward == 1 : compare = PR_DC_MID + mod  (saturates at PR_DC_MIN when mod is large negative)
 * forward == 0 : compare = PR_DC_MID - mod  (saturates at PR_DC_MIN when mod is large positive)
 */
static inline uint16_t clamp_pwm(int16_t mod, int forward)
{
    int32_t val = forward
        ? ((int32_t)PR_DC_MID + mod)
        : ((int32_t)PR_DC_MID - mod);

    if (val <= PR_DC_MIN) { return PR_DC_MIN; }
    if (val >= PR_DC_MAX) { return PR_DC_MAX; }
    return (uint16_t)val;
}

/* ── Platform SPI transfer (LoRa driver back-end) ─────────────────────────── */
uint8_t LoRa_singleTransfer(uint8_t address, uint8_t value)
{
    volatile uint16_t readData;
    volatile uint16_t readData1;

    XMC_SPI_CH_EnableSlaveSelect(SPI0_HW, XMC_SPI_CH_SLAVE_SELECT_0);

    XMC_SPI_CH_Transmit(SPI0_HW, address, XMC_SPI_CH_MODE_STANDARD);
    while ((XMC_SPI_CH_GetStatusFlag(SPI0_HW) & XMC_SPI_CH_STATUS_FLAG_TRANSMIT_SHIFT_INDICATION) == 0U);
    XMC_SPI_CH_ClearStatusFlag(SPI0_HW, XMC_SPI_CH_STATUS_FLAG_TRANSMIT_SHIFT_INDICATION);
    while ((XMC_SPI_CH_GetStatusFlag(SPI0_HW) & (XMC_SPI_CH_STATUS_FLAG_RECEIVE_INDICATION | XMC_SPI_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION)) == 0);
    XMC_SPI_CH_ClearStatusFlag(SPI0_HW, XMC_SPI_CH_STATUS_FLAG_RECEIVE_INDICATION & XMC_SPI_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION);

    XMC_SPI_CH_Transmit(SPI0_HW, value, XMC_SPI_CH_MODE_STANDARD);
    while ((XMC_SPI_CH_GetStatusFlag(SPI0_HW) & XMC_SPI_CH_STATUS_FLAG_TRANSMIT_SHIFT_INDICATION) == 0U);
    XMC_SPI_CH_ClearStatusFlag(SPI0_HW, XMC_SPI_CH_STATUS_FLAG_TRANSMIT_SHIFT_INDICATION);
    while ((XMC_SPI_CH_GetStatusFlag(SPI0_HW) & (XMC_SPI_CH_STATUS_FLAG_RECEIVE_INDICATION | XMC_SPI_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION)) == 0);
    XMC_SPI_CH_ClearStatusFlag(SPI0_HW, XMC_SPI_CH_STATUS_FLAG_RECEIVE_INDICATION & XMC_SPI_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION);

    XMC_SPI_CH_DisableSlaveSelect(SPI0_HW);

    delay_cycles(SystemCoreClock / 400000);
    readData  = XMC_SPI_CH_GetReceivedData(SPI0_HW);
    delay_cycles(SystemCoreClock / 40000);
    readData1 = XMC_SPI_CH_GetReceivedData(SPI0_HW);

    (void)readData; /* first byte (address echo) not used */

#if ENABLE_XMC_DEBUG_PRINT
    if (SPI_buffer_received_cnt > 15) { SPI_buffer_received_cnt = 15; }
    SPI_buffer_received[SPI_buffer_received_cnt] = readData1;
    SPI_buffer_received_cnt++;
#endif

    return readData1;
}

/* ── Interrupt handlers ───────────────────────────────────────────────────── */

/**
 * ERU external-event ISR — fires when the LoRa DIO0 pin goes high.
 * Reads the incoming packet into result_arr[] and sets flag_receive.
 */
void ERU_EXTERNAL_EVENT_HANDLER(void)
{
    if (flag_loop_done == 1) {
        flag_loop_done = 0;

        length = LoRa_handleDio0Rise();

        if (length > 0) {
            for (uint8_t i = 0; i < 7; i++) {
                result_arr[i] = (uint8_t)LoRa_read();
            }
#if ENABLE_XMC_DEBUG_PRINT
            printf("LoRa received %hu %hu %hu %hu %hu %hu %hu\r\n",
                result_arr[0], result_arr[1], result_arr[2], result_arr[3],
                result_arr[4], result_arr[5], result_arr[6]);
#endif
            flag_receive = 1;
        }

        /* Reset FIFO address pointer (register 0x0d = REG_FIFO_ADDR_PTR) */
        LoRa_writeRegister(0x0d, 0);
        /* Clear LoRa IRQ flags after all reading is done */
        LoRa_clearDIOrise();
    }
}

/**
 * SysTick ISR — rate set by MAIN_LOOP_FREQ.
 * Handles abort-button debounce and the control-reading tick counter.
 */
void SysTick_Handler(void)
{
    if (flag_abort_button == 1) {
        cnt_abort_button++;
        if (cnt_abort_button >= (max_abort_button << 1)) {
            flag_abort        = 0;
            cnt_abort_button  = 0;
        } else if (cnt_abort_button >= max_abort_button) {
            flag_abort = 1;
        }
    }

    if (ticks_control >= TICKS_WAIT_CONTROL_READING) {
        flag_control_reading = 1;
        ticks_control        = 0;
    } else {
        ticks_control++;
    }
}

/* ── Main ─────────────────────────────────────────────────────────────────── */
int main(void)
{
    cy_rslt_t result = cybsp_init();
    if (result != CY_RSLT_SUCCESS) { CY_ASSERT(0); }

    /* Stabilise power rails */
    delay_cycles(SystemCoreClock / 8);

    /* Keep PWM pins as GPIO until everything is configured */
    PWM_OFF();

#if ENABLE_XMC_DEBUG_PRINT
    cy_retarget_io_init(CYBSP_DEBUG_UART_HW);
    printf("Initialization done\r\n");
    for (uint8_t n = 0; n <= 15; n++) { SPI_buffer_received[n] = 0; }
    SPI_buffer_received_cnt = 0;
#endif

    /* ── CCU80 PWM initialisation ──────────────────────────────────────────── */
    uint16_t PWM_LEFT_TRACK_compare  = 0;
    uint16_t PWM_RIGHT_TRACK_compare = 0;
    uint16_t PWM_BUZZER_compare      = NOBEEP;

    XMC_CCU8_EnableClock(CCU80, PWM_BUZZER_NUM);
    XMC_CCU8_EnableClock(CCU80, PWM_LEFT_TRACK_NUM);
    XMC_CCU8_EnableClock(CCU80, PWM_RIGHT_TRACK_NUM);

    XMC_CCU8_SLICE_SetTimerPeriodMatch(PWM_BUZZER_HW, BUZZER_PERIOD);
    XMC_CCU8_SLICE_SetTimerValue(PWM_BUZZER_HW, 0);
    XMC_CCU8_SLICE_SetTimerCompareMatch(PWM_BUZZER_HW, PWM_BUZZER_compare, PWM_BUZZER_compare);

    XMC_CCU8_SLICE_SetTimerPeriodMatch(PWM_LEFT_TRACK_HW, PR_PERIOD_PWM);
    XMC_CCU8_SLICE_SetTimerValue(PWM_LEFT_TRACK_HW, PR_DC_MID);
    XMC_CCU8_SLICE_SetTimerCompareMatch(PWM_LEFT_TRACK_HW, PWM_LEFT_TRACK_compare, PWM_LEFT_TRACK_compare);

    XMC_CCU8_SLICE_SetTimerPeriodMatch(PWM_RIGHT_TRACK_HW, PR_PERIOD_PWM);
    XMC_CCU8_SLICE_SetTimerValue(PWM_RIGHT_TRACK_HW, PR_DC_MID);
    XMC_CCU8_SLICE_SetTimerCompareMatch(PWM_RIGHT_TRACK_HW, PWM_RIGHT_TRACK_compare, PWM_RIGHT_TRACK_compare);

    XMC_CCU8_EnableShadowTransfer(CCU80, (CCU8_GCSS_S2SE_Msk | CCU8_GCSS_S1SE_Msk | CCU8_GCSS_S0SE_Msk));

    /* Synchronous start via SCU trigger */
    XMC_CCU8_SLICE_EVENT_CONFIG_t synchronous_input = {
        .mapped_input = 7,
        .edge         = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,
        .level        = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
        .duration     = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED
    };
    XMC_CCU8_SLICE_StartConfig(PWM_BUZZER_HW,     XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START);
    XMC_CCU8_SLICE_StartConfig(PWM_LEFT_TRACK_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START);
    XMC_CCU8_SLICE_StartConfig(PWM_RIGHT_TRACK_HW,XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START);
    XMC_CCU8_SLICE_ConfigureEvent(PWM_BUZZER_HW,      XMC_CCU8_SLICE_EVENT_0, &synchronous_input);
    XMC_CCU8_SLICE_ConfigureEvent(PWM_LEFT_TRACK_HW,  XMC_CCU8_SLICE_EVENT_0, &synchronous_input);
    XMC_CCU8_SLICE_ConfigureEvent(PWM_RIGHT_TRACK_HW, XMC_CCU8_SLICE_EVENT_0, &synchronous_input);
    XMC_SCU_SetCcuTriggerHigh(XMC_SCU_CCU_TRIGGER_CCU80);
    XMC_SCU_SetCcuTriggerLow(XMC_SCU_CCU_TRIGGER_CCU80);

    /* ── SPI + LoRa initialisation ─────────────────────────────────────────── */
    XMC_SPI_CH_Start(SPI0_HW);

    /* Hardware reset sequence */
    delay_cycles(SystemCoreClock / 8);
    XMC_GPIO_SetOutputLow(Lora_RESET_PORT, Lora_RESET_PIN);
    delay_cycles(SystemCoreClock / 8);
    XMC_GPIO_SetOutputHigh(Lora_RESET_PORT, Lora_RESET_PIN);
    delay_cycles(SystemCoreClock / 8);

    uint8_t status = LoRa_begin(WIRELESS_FREQ);
    if (status == 0) {
#if ENABLE_XMC_DEBUG_PRINT
        printf("Starting LoRa failed!\r\n");
#endif
        XMC_GPIO_SetOutputHigh(LED_RED_PORT, LED_RED_PIN);
        while (1) {}
    }

    delay_cycles(SystemCoreClock / 8);
    LoRa_receive(0); /* explicit header mode */
    delay_cycles(SystemCoreClock / 8);

    /* ── NVIC configuration ────────────────────────────────────────────────── */
    SysTick_Config(SystemCoreClock / MAIN_LOOP_FREQ);
    NVIC_SetPriority(SysTick_IRQn, 1);

    NVIC_SetPriority(INTERRUPT_PRIORITY_NODE_ID,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), INTERRUPT_EVENT_PRIORITY, 0));
    NVIC_EnableIRQ(INTERRUPT_PRIORITY_NODE_ID);

    NVIC_SetPriority(CCU40_0_IRQn, 4);
    NVIC_EnableIRQ(CCU40_0_IRQn);

    /* ── Local loop variables ──────────────────────────────────────────────── */
    uint8_t  error             = 0;
    int16_t  angle             = 0;
    int16_t  pwm_modification1 = 0;
    int16_t  pwm_modification2 = 0;
    uint16_t speed_gain        = 5;

    /* ── Main loop ─────────────────────────────────────────────────────────── */
    while (1) {

#if ERU_NOT_WORKING
        /* Fallback polling path used when the ERU interrupt is not available.
         * Reads D0 and manually invokes the ISR logic. */
        if (flag_control_reading == 1) {
            uint8_t D0_value = XMC_GPIO_GetInput(XMC_GPIO_PORT2, 2);
            XMC_GPIO_ToggleOutput(XMC_GPIO_PORT0, 6);
            if (D0_value == 1) {
                ERU_EXTERNAL_EVENT_HANDLER();
            } else {
                cont_disconnect++;
                if (cont_disconnect >= max_disconnect) {
                    flag_disconnect = 1;
                    flag_abort      = 1;
                }
            }
            flag_control_reading = 0;
        }
#endif /* ERU_NOT_WORKING */

        /* ── Process received LoRa packet ──────────────────────────────────── */
        if (flag_receive == 1) {
            error         = messageInterpreter(result_arr, length);
            flag_receive  = 0;

#if ENABLE_XMC_DEBUG_PRINT
            printf("error code is %hu\r\n", error);
#endif

            if (error == 10) {
                cont_disconnect = 0;
                if (flag_disconnect == 1) {
                    flag_disconnect = 0;
                    flag_abort      = 0;
                }

                if (flag_abort == 0) {
                    /* ── Deadband: no throttle, tank-turn only ──────────────── */
                    if ((current_PWM_THROTTLE > BAND_FORWARD) &&
                        (current_PWM_THROTTLE < BAND_BACKWARD)) {

                        if (current_PWM_STEERING > BAND_STEER_LEFT) {
                            uint16_t diff = (current_PWM_STEERING - OFFSET_JS_STEER) << (speed_gain - 1);
                            PWM_LEFT_TRACK_compare  = PR_DC_MID - diff;
                            PWM_RIGHT_TRACK_compare = PR_DC_MID + diff;
                        } else if (current_PWM_STEERING < BAND_STEER_RIGHT) {
                            uint16_t diff = (OFFSET_JS_STEER - current_PWM_STEERING) << (speed_gain - 1);
                            PWM_LEFT_TRACK_compare  = PR_DC_MID + diff;
                            PWM_RIGHT_TRACK_compare = PR_DC_MID - diff;
                        } else {
                            PWM_LEFT_TRACK_compare  = PR_DC_MID;
                            PWM_RIGHT_TRACK_compare = PR_DC_MID;
                        }

                    /* ── Forward ────────────────────────────────────────────── */
                    } else if (current_PWM_THROTTLE > BAND_BACKWARD) {
                        angle            = (int16_t)((OFFSET_JS_STEER - current_PWM_STEERING) >> 1);
                        pwm_modification1 = (int16_t)(((current_PWM_THROTTLE - OFFSET_JS_THROTTLE) - angle) << speed_gain);
                        pwm_modification2 = (int16_t)(((current_PWM_THROTTLE - OFFSET_JS_THROTTLE) + angle) << speed_gain);
                        PWM_LEFT_TRACK_compare  = clamp_pwm(pwm_modification1, 1);
                        PWM_RIGHT_TRACK_compare = clamp_pwm(pwm_modification2, 1);

                    /* ── Backward ───────────────────────────────────────────── */
                    } else if (current_PWM_THROTTLE < BAND_FORWARD) {
                        angle            = (int16_t)((OFFSET_JS_STEER - current_PWM_STEERING) >> 1);
                        pwm_modification1 = (int16_t)(((OFFSET_JS_THROTTLE - current_PWM_THROTTLE) - angle) << speed_gain);
                        pwm_modification2 = (int16_t)(((OFFSET_JS_THROTTLE - current_PWM_THROTTLE) + angle) << speed_gain);
                        PWM_LEFT_TRACK_compare  = clamp_pwm(pwm_modification1, 0);
                        PWM_RIGHT_TRACK_compare = clamp_pwm(pwm_modification2, 0);
                    }
                } else {
                    /* Abort active — stop both tracks */
                    PWM_LEFT_TRACK_compare  = PR_DC_MID;
                    PWM_RIGHT_TRACK_compare = PR_DC_MID;
                }

                /* Headlights */
                if (flag_headlights) {
                    XMC_GPIO_SetOutputHigh(HEADLIGHTS_PORT, HEADLIGHTS_PIN);
                } else {
                    XMC_GPIO_SetOutputLow(HEADLIGHTS_PORT, HEADLIGHTS_PIN);
                }
            } else {
                cont_disconnect++;
            }
        } else {
            /* No packet this iteration — count as missing tick */
            if (flag_control_reading == 1) {
                cont_disconnect++;
                flag_control_reading = 0;
            }
        }

        /* ── Disconnect / abort watchdog ───────────────────────────────────── */
        if (cont_disconnect >= max_disconnect) {
            flag_disconnect         = 1;
            flag_abort              = 1;
            PWM_LEFT_TRACK_compare  = PR_DC_MID;
            PWM_RIGHT_TRACK_compare = PR_DC_MID;
        }

        /* ── PWM output gate ───────────────────────────────────────────────── */
        /* Disable CCU80 output when stopped to suppress motor noise */
        if (PWM_LEFT_TRACK_compare == PR_DC_MID) {
            PWM_OFF();
        } else {
            PWM_ON();
        }

        /* Hard clamp before writing to hardware */
        if (PWM_LEFT_TRACK_compare  > PR_DC_MAX) { PWM_LEFT_TRACK_compare  = PR_DC_MAX; }
        if (PWM_RIGHT_TRACK_compare > PR_DC_MAX) { PWM_RIGHT_TRACK_compare = PR_DC_MAX; }

        UPDATE_DUTY(PWM_LEFT_TRACK_compare, PWM_RIGHT_TRACK_compare, PWM_BUZZER_compare, 0);

        flag_loop_done = 1;
    }
}

/* ── LED blink ISR (CCU40 slice 0, ~0.25 s period) ───────────────────────── */
static uint8_t blink_sequence = 0;

void CCU40_0_IRQHandler(void)
{
    switch (blink_sequence) {
        case 0:
            if (flag_disconnect) {
                XMC_GPIO_SetOutputHigh(LED_RED_PORT,  LED_RED_PIN);
            } else {
                XMC_GPIO_SetOutputHigh(LED_BLUE_PORT, LED_BLUE_PIN);
            }
            if (flag_abort) {
                XMC_GPIO_SetOutputHigh(LED_YELLOW_PORT, LED_YELLOW_PIN);
            }
            XMC_GPIO_SetOutputHigh(LED_GREEN_PORT, LED_GREEN_PIN);
            break;

        case 1:
            XMC_GPIO_SetOutputLow(LED_RED_PORT,    LED_RED_PIN);
            XMC_GPIO_SetOutputLow(LED_YELLOW_PORT, LED_YELLOW_PIN);
            XMC_GPIO_SetOutputLow(LED_GREEN_PORT,  LED_GREEN_PIN);
            XMC_GPIO_SetOutputLow(LED_BLUE_PORT,   LED_BLUE_PIN);
            break;

        default:
            break;
    }

    blink_sequence = (blink_sequence >= 1) ? 0 : blink_sequence + 1;
}

/* ── Message interpreter ──────────────────────────────────────────────────── */

/**
 * Decode a 7-byte LoRa packet.
 *
 * Returns 10 on success, or an error code 1–8:
 *   1 = wrong length
 *   2 = bad SOP
 *   3 = bad EOP
 *   4–8 = MSB guard bit missing in bytes 1–5
 */
uint8_t messageInterpreter(uint8_t message[], uint8_t L)
{
    if (L != 7)                                       { return 1; }
    if (message[0] != SOP)                            { return 2; }
    if (message[6] != EOP)                            { return 3; }
    if ((message[1] & 0x80) != 0x80)                  { return 4; }
    if ((message[2] & 0x80) != 0x80)                  { return 5; }
    if ((message[3] & 0x80) != 0x80)                  { return 6; }
    if ((message[4] & 0x80) != 0x80)                  { return 7; }
    if ((message[5] & 0x80) != 0x80)                  { return 8; }

    /* Byte 1: button / flag field */
    flag_abort_button = ((message[1] & 0x60) == 0x60) ? 1u : 0u;
    flag_headlights   = ((message[1] & 0x01) == 0x01) ? 1u : 0u;

    /* Bytes 2–3: right stick axes (not used) */

    /* Byte 4: throttle (left stick Y) */
    current_PWM_THROTTLE = (message[4] & 0x7f);

    /* Byte 5: steering (left stick X) */
    current_PWM_STEERING = (message[5] & 0x7f);

    return 10;
}

/* ── Stub functions ───────────────────────────────────────────────────────── */

void abortProcedure(void)
{
    /* Abort logic is handled inline in the main loop */
#if ENABLE_XMC_DEBUG_PRINT
    printf("abort procedure\r\n");
#endif
}

void activateFlaps(uint8_t level)
{
    (void)level; /* not used on SpeedyCar */
#if ENABLE_XMC_DEBUG_PRINT
    printf("activate flaps\r\n");
#endif
}

void startReport(void)
{
    /* Placeholder: switch to TX mode and send telemetry */
#if ENABLE_XMC_DEBUG_PRINT
    printf("start report\r\n");
#endif
}

/* [] END OF FILE */
