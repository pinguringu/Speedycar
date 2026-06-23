/******************************************************************************
* File Name:   main.c
*
******************************************************************************
*This is the receiver program for the FoamPlane
*It receives the LoRa transmission at 868mHz
*It decodes the messages and actuates the servos and ESC
*****************************************************************************/
#include "main.h"

/** Initializations **/
/* Systick ticks for message read */
uint16_t ticks_control = 0;
/* Systick ticks for LED */
uint16_t ticks_light = 0;
/* Flag for message read loop */
uint8_t flag_loop_done = 1; //Only if this flag is set, when a message comes it is read

/* Define macro to enable/disable printing of debug messages */
#define ENABLE_XMC_DEBUG_PRINT (0)

/*External interrupt function call which happens when LoRa reads something and D0 goes high */
void ERU_EXTERNAL_EVENT_HANDLER(void)
{
	if(flag_loop_done == 1)
	{
		flag_loop_done = 0;

		/* check length and reset FIFO */
		length = LoRa_handleDio0Rise();
		/* Variable will hold received words */
		volatile uint8_t reading = 0;
		if (length > 0){
			/* Read all packets based on length */
			for (uint8_t i = 0; i < 7; i++) {
				reading = LoRa_read();
				result_arr[i] = reading;
				/* For debug */
				//char character = reading+'0';
				//result[1] = reading;
				//sprintf(result, "%d", (int)reading);
			  }
			#if ENABLE_XMC_DEBUG_PRINT
			  printf("LoRa received %hu %hu %hu %hu %hu %hu %hu\r\n",result_arr[0],result_arr[1],result_arr[2],result_arr[3],result_arr[4],result_arr[5],result_arr[6]);
			  __NOP();
			#endif
			/* Flag which signals that a message was received and stored */
			flag_receive = 1;
		}
		/* reset FIFO address */
		LoRa_writeRegister(0x0d, 0);
		/* Clear LoRa interrupts, important to do it after all reading is finished */
		LoRa_clearDIOrise();

	}
}


/** Systick executes every 1/X ms based on MAIN_LOOP_FREQ **/
/* It has a counter for signal lost, abort */
/* Counter for executing the main loop */
/* Counter for LEDs */
void SysTick_Handler(void)
{
	/* If the button for abort is being pushed, the counter increases
	 * if limit reach, abort is activated
	 * if pressed again for longer, abort flag is put low
	 */
	if(flag_abort_button == 1)
	{
		cnt_abort_button++;
		if(cnt_abort_button >= (max_abort_button << 1))
		{
			flag_abort = 0;
			cnt_abort_button = 0;
		}
		else if(cnt_abort_button >= max_abort_button)
		{
			flag_abort = 1;
		}
	}

	if(ticks_control>= TICKS_WAIT_CONTROL_READING)
	{
		flag_control_reading = 1;
		ticks_control = 0;
	}
	else
	{
		ticks_control++;
	}
}


uint8_t LoRa_singleTransfer(uint8_t address, uint8_t value)
{
  volatile uint16_t readAddress;
  volatile uint16_t readData;
  volatile uint16_t readData1;

  XMC_SPI_CH_EnableSlaveSelect(SPI0_HW, XMC_SPI_CH_SLAVE_SELECT_0);
  XMC_SPI_CH_Transmit(SPI0_HW, address, XMC_SPI_CH_MODE_STANDARD);
  while((XMC_SPI_CH_GetStatusFlag(SPI0_HW) & XMC_SPI_CH_STATUS_FLAG_TRANSMIT_SHIFT_INDICATION) == 0U);
  XMC_SPI_CH_ClearStatusFlag(SPI0_HW, XMC_SPI_CH_STATUS_FLAG_TRANSMIT_SHIFT_INDICATION);
  while((XMC_SPI_CH_GetStatusFlag(SPI0_HW) & (XMC_SPI_CH_STATUS_FLAG_RECEIVE_INDICATION | XMC_SPI_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION)) == 0);
  XMC_SPI_CH_ClearStatusFlag(SPI0_HW, XMC_SPI_CH_STATUS_FLAG_RECEIVE_INDICATION & XMC_SPI_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION);
  XMC_SPI_CH_Transmit(SPI0_HW, value, XMC_SPI_CH_MODE_STANDARD);
  while((XMC_SPI_CH_GetStatusFlag(SPI0_HW) & XMC_SPI_CH_STATUS_FLAG_TRANSMIT_SHIFT_INDICATION ) == 0U);
  XMC_SPI_CH_ClearStatusFlag(SPI0_HW, XMC_SPI_CH_STATUS_FLAG_TRANSMIT_SHIFT_INDICATION);
  while((XMC_SPI_CH_GetStatusFlag(SPI0_HW) & (XMC_SPI_CH_STATUS_FLAG_RECEIVE_INDICATION | XMC_SPI_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION)) == 0);
  XMC_SPI_CH_ClearStatusFlag(SPI0_HW, XMC_SPI_CH_STATUS_FLAG_RECEIVE_INDICATION & XMC_SPI_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION);
  XMC_SPI_CH_DisableSlaveSelect( SPI0_HW );
  for(uint32_t k=0;k<(SystemCoreClock/400000);k++){
      	 __NOP();
      	}
  readData = XMC_SPI_CH_GetReceivedData(SPI0_HW);
  for(uint32_t k=0;k<(SystemCoreClock/40000);k++){
        	 __NOP();
        	}
  readData1 = XMC_SPI_CH_GetReceivedData(SPI0_HW);

  #if ENABLE_XMC_DEBUG_PRINT
    if(SPI_buffer_received_cnt>15){
    	SPI_buffer_received_cnt = 15;
    }
  	  SPI_buffer_received[SPI_buffer_received_cnt] = readData1;
  	  SPI_buffer_received_cnt++;
  #endif

  return readData1;
}


int main(void)
{
    cy_rslt_t result;

    /*Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Delay to stabilize power in MCU */
    for(uint32_t k=0;k<SystemCoreClock/8;k++){
        __NOP();
    }
    /* Disable PWM outputs to avoid weird behaviour */
    PWM_OFF();

    #if ENABLE_XMC_DEBUG_PRINT
    cy_retarget_io_init(CYBSP_DEBUG_UART_HW);
    printf("Initialization done\r\n");
    uint8_t n;

    for(n=0;n<=15;n++){
    	SPI_buffer_received[n] = 0;
    }
    SPI_buffer_received_cnt = 0;
    #endif

    /* Initialise PWM compares */
    uint16_t PWM_LEFT_TRACK_compare = 0;
    uint16_t PWM_RIGHT_TRACK_compare = 0;
    uint16_t PWM_BUZZER_compare = NOBEEP;

	/* Synchronous start of timers to control ARCP pulse location. */
	XMC_CCU8_EnableClock(CCU80, PWM_BUZZER_NUM);
    XMC_CCU8_EnableClock(CCU80, PWM_LEFT_TRACK_NUM);
    XMC_CCU8_EnableClock(CCU80, PWM_RIGHT_TRACK_NUM);
    /* Start PWMs with initial values */
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
    /* All timers start with global flag */
    XMC_CCU8_SLICE_StartConfig(PWM_BUZZER_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START);
	XMC_CCU8_SLICE_StartConfig(PWM_LEFT_TRACK_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START);
	XMC_CCU8_SLICE_StartConfig(PWM_RIGHT_TRACK_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START);
	XMC_CCU8_SLICE_EVENT_CONFIG_t synchronous_input = {.mapped_input = 7, .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE, .level = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_NONE, .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED};
	XMC_CCU8_SLICE_ConfigureEvent(PWM_BUZZER_HW, XMC_CCU8_SLICE_EVENT_0, &synchronous_input);
	XMC_CCU8_SLICE_ConfigureEvent(PWM_LEFT_TRACK_HW, XMC_CCU8_SLICE_EVENT_0, &synchronous_input);
	XMC_CCU8_SLICE_ConfigureEvent(PWM_RIGHT_TRACK_HW, XMC_CCU8_SLICE_EVENT_0, &synchronous_input);
	XMC_SCU_SetCcuTriggerHigh(XMC_SCU_CCU_TRIGGER_CCU80);
	XMC_SCU_SetCcuTriggerLow(XMC_SCU_CCU_TRIGGER_CCU80);

	/* Enable PWM outputs */
	//PWM_ON();

    /* Start the SPI Channel */
    XMC_SPI_CH_Start( SPI0_HW );

    /* Reset LoRa externally and wait some time for POR */
    for(uint32_t k=0;k<SystemCoreClock/8;k++){
        __NOP();
        }
    XMC_GPIO_SetOutputLow(Lora_RESET_PORT, Lora_RESET_PIN);
    for(uint32_t k=0;k<SystemCoreClock/8;k++){
    	 __NOP();
    	}
    XMC_GPIO_SetOutputHigh(Lora_RESET_PORT, Lora_RESET_PIN);
    for(uint32_t k=0;k<SystemCoreClock/8;k++){
    	__NOP();
      }


    /* Begin LoRa setup */
    uint8_t status = 0;
    status = LoRa_begin(WIRELESS_FREQ);
    if (status == 0)
    {
		#if ENABLE_XMC_DEBUG_PRINT
        printf("Starting LoRa failed!");
		#endif
        XMC_GPIO_SetOutputHigh(LED_RED_PORT, LED_RED_PIN);
        while (1);
    }



    /* Put Lora on receive, 0 means explicit mode, otherwise input the message length */
    for(uint32_t k=0;k<SystemCoreClock/8;k++)
    {
        __NOP();
    }
    LoRa_receive(0);
    for(uint32_t k=0;k<SystemCoreClock/8;k++)
    {
        __NOP();
    }

    /* Main loop triggered by Systick at X Hz. */
    SysTick_Config(SystemCoreClock / MAIN_LOOP_FREQ);
    NVIC_SetPriority(SysTick_IRQn, 1);

    /*Set Priority for ERU IRQ*/
    NVIC_SetPriority(INTERRUPT_PRIORITY_NODE_ID,NVIC_EncodePriority(NVIC_GetPriorityGrouping(),INTERRUPT_EVENT_PRIORITY, 0));
    /*Enable the ERU Interrupt*/
    NVIC_EnableIRQ(INTERRUPT_PRIORITY_NODE_ID);

    /*Set Priority for TIMER CCU*/
    NVIC_SetPriority(CCU40_0_IRQn, 4);
    NVIC_EnableIRQ(CCU40_0_IRQn);

    /* Variable to know if the message is free or errors */
    uint8_t error = 0;

	/* Angle of steering to make direction decisions*/
	volatile int16_t angle = 0;
	volatile int16_t pwm_modification1 = 0;
	volatile int16_t pwm_modification2 = 0;
	/* Gain from the analog stick value to the PWM value applied*/
	uint16_t speed_gain = 5;

    while (1)
    {

#if ERU_NOT_WORKING
    	//check if something came in LoRa
    	//volatile status = LoRa_readRegister(0x01);
    	if(flag_control_reading == 1){
    		D0_value = XMC_GPIO_GetInput(XMC_GPIO_PORT2, 2);
    		//if(D0_value == 1){
    			//debug
    			XMC_GPIO_ToggleOutput(XMC_GPIO_PORT0, 6);
    			ERU_EXTERNAL_EVENT_HANDLER();

    		//}
    		//else{
    		    cont_disconnect++;
    		    if(cont_disconnect >= max_disconnect)
    		    {
    		    	flag_disconnect = 1;
    		    	flag_abort = 1;
    		    	PWM0_compare = 60;
    		    	PWM1_compare = 100;
    		    	PWM2_compare = 100 + 12;
    		    	PWM3_compare = 100 + 12;
    		    }
    		//}
    		flag_control_reading = 0;
    	}
#endif
		/* when a message from lora has arrived */
		if(flag_receive == 1){
			/* Interpret message and act on it */
			error = messageInterpreter(result_arr, length);
			#if ENABLE_XMC_DEBUG_PRINT
			printf("error code is %hu \r\n",error);
			#endif
			flag_receive = 0;

			/* If there are no errors in the packages */
			if(error == 10){
    			cont_disconnect = 0;
    			if(flag_disconnect == 1){
    				flag_disconnect = 0;
    				flag_abort = 0;
    			}
    			if(flag_abort == 0){
    				/* No movement within the deadband but can turn like a tank*/
					if((current_PWM_THROTTLE > BAND_FORWARD) && (current_PWM_THROTTLE < BAND_BACKWARD))
					{
						if(current_PWM_STEERING > BAND_STEER_LEFT)
						{
							PWM_LEFT_TRACK_compare = PR_DC_MID - ((current_PWM_STEERING - OFFSET_JS_STEER) << (speed_gain - 1));
							PWM_RIGHT_TRACK_compare = PR_DC_MID + ((current_PWM_STEERING - OFFSET_JS_STEER) << (speed_gain - 1));
						}
						else if(current_PWM_STEERING < BAND_STEER_RIGHT)
						{
							PWM_LEFT_TRACK_compare = PR_DC_MID + ((OFFSET_JS_STEER - current_PWM_STEERING) << (speed_gain - 1));
							PWM_RIGHT_TRACK_compare = PR_DC_MID - ((OFFSET_JS_STEER - current_PWM_STEERING) << (speed_gain - 1));
						}
						else
						{
    						PWM_LEFT_TRACK_compare = PR_DC_MID;
    						PWM_RIGHT_TRACK_compare = PR_DC_MID;
    					}
    				}
    				/* Forward or backward uses the same logic*/
    				else{
    						if(current_PWM_THROTTLE > BAND_BACKWARD){
    							angle = (OFFSET_JS_STEER - current_PWM_STEERING) >> 1;
    							pwm_modification1 = (((current_PWM_THROTTLE - OFFSET_JS_THROTTLE) - angle) << speed_gain);
    							if(- pwm_modification1 >= PR_DC_MID)
    							{
									PWM_LEFT_TRACK_compare = PR_DC_MIN;
								}
								else
								{
									PWM_LEFT_TRACK_compare = PR_DC_MID + pwm_modification1;
								}
    							pwm_modification2 = (((current_PWM_THROTTLE - OFFSET_JS_THROTTLE) + angle) << speed_gain);
    							if(- pwm_modification2 >= PR_DC_MID)
    							{
									PWM_RIGHT_TRACK_compare = PR_DC_MIN;
								}
								else
								{
									PWM_RIGHT_TRACK_compare = PR_DC_MID + pwm_modification2;
								}
    						}
    						else if(current_PWM_THROTTLE < BAND_FORWARD){
    							angle = (OFFSET_JS_STEER - current_PWM_STEERING) >> 1;
    							pwm_modification1 = (((OFFSET_JS_THROTTLE - current_PWM_THROTTLE) - angle) << speed_gain);
    							if(pwm_modification1 >= PR_DC_MID)
    							{
									PWM_LEFT_TRACK_compare = PR_DC_MIN;
								}
								else
								{
									PWM_LEFT_TRACK_compare = PR_DC_MID - pwm_modification1;
								}
								pwm_modification2 = (((OFFSET_JS_THROTTLE - current_PWM_THROTTLE) + angle) << speed_gain);
    							if(pwm_modification2 >= PR_DC_MID)
    							{
									PWM_RIGHT_TRACK_compare = PR_DC_MIN;
								}
								else
								{
									PWM_RIGHT_TRACK_compare = PR_DC_MID - pwm_modification2;
								}
    						}
    						else{
								/* This will never happen*/
    						}
    					
    				}

    			}
    			else
    			{
    				PWM_LEFT_TRACK_compare = PR_DC_MID;
    				PWM_RIGHT_TRACK_compare = PR_DC_MID;

    			}
    			if(flag_headlights)
    			{
					XMC_GPIO_SetOutputHigh(HEADLIGHTS_PORT, HEADLIGHTS_PIN);
				}
				else
				{
					XMC_GPIO_SetOutputLow(HEADLIGHTS_PORT, HEADLIGHTS_PIN);
				}
    		}
			else
			{
				cont_disconnect++;
			}
		}
		else
		{
			if(flag_control_reading == 1)
			{
				cont_disconnect++;
				flag_control_reading = 0;
			}
		}
		/* Lost connection to transmitter for a while */
		if(cont_disconnect >= max_disconnect)
		{
			flag_disconnect = 1;
			flag_abort = 1;
			/* Keep both PWM signals low to stop motor */
			PWM_LEFT_TRACK_compare = PR_DC_MID;
    		PWM_RIGHT_TRACK_compare = PR_DC_MID;
    		
		}
		/* If we want to stop the motors we have to disable PWM, if not it will be noisy */
		if(PWM_LEFT_TRACK_compare == PR_DC_MID)
		{
			PWM_OFF();
		}
		else
		{
			PWM_ON();

		}
		/* Max limitation to avoid overflow of counter*/
		if(PWM_LEFT_TRACK_compare > PR_DC_MAX)
		{
			PWM_LEFT_TRACK_compare = PR_DC_MAX;
		}
		if(PWM_RIGHT_TRACK_compare > PR_DC_MAX)
		{
			PWM_RIGHT_TRACK_compare = PR_DC_MAX;
		}
		UPDATE_DUTY(PWM_LEFT_TRACK_compare, PWM_RIGHT_TRACK_compare, PWM_BUZZER_compare, 0);

    flag_loop_done = 1;
    }


}

/* 0.25s timer IRQ
 * for now used just for LED blinking*/
uint8_t blink_sequence = 0;
void CCU40_0_IRQHandler(void)
{
	switch (blink_sequence)
	{
		case 0:
			if(flag_disconnect == 1)
			{
				XMC_GPIO_SetOutputHigh(LED_RED_PORT, LED_RED_PIN);
			}
			else
			{
				XMC_GPIO_SetOutputHigh(LED_BLUE_PORT, LED_BLUE_PIN);
			}
			if(flag_abort == 1)
			{
				XMC_GPIO_SetOutputHigh(LED_YELLOW_PORT, LED_YELLOW_PIN);
			}
			XMC_GPIO_SetOutputHigh(LED_GREEN_PORT, LED_GREEN_PIN);
			break;
		case 1:
			XMC_GPIO_SetOutputLow(LED_RED_PORT, LED_RED_PIN);
			XMC_GPIO_SetOutputLow(LED_YELLOW_PORT, LED_YELLOW_PIN);
			XMC_GPIO_SetOutputLow(LED_GREEN_PORT, LED_GREEN_PIN);
			XMC_GPIO_SetOutputLow(LED_BLUE_PORT, LED_BLUE_PIN);
			break;
	}
	/* Reset sequence */
	if(blink_sequence >= 1)
	{
		blink_sequence = 0;
	}
	else
	{
		blink_sequence++;
	}
}




//This function applies the information contained in the package received
uint8_t messageInterpreter(uint8_t message[], uint8_t L){
	//check for correctness of structure

	if (L !=7 )
	{
		return 1;
	}
	else if (message[0] != SOP)
	{
		return 2;
	}
	else if (message[6] != EOP)
	{
		return 3;
	}
	else if ((message[1] & 0b10000000) != 0b10000000)
	{
		return 4;
	}
	if ((message[2] & 0b10000000) != 0b10000000)
	{
		return 5;
	}
	if ((message[3] & 0b10000000) != 0b10000000)
	{
		return 6;
	}
	if ((message[4] & 0b10000000) != 0b10000000)
	{
		return 7;
	}
	if ((message[5] & 0b10000000) != 0b10000000)
	{
		return 8;
	}
	else
	{
		if((message[1] & 0b01100000) == 0b01100000)
		{
			flag_abort_button = 1;
		}
		else
		{
			flag_abort_button = 0;
		}
		if((message[1] & 0b00000010) == 0b00000010)
		{
//			startReport();
		}
		else
		{
		}
		if((message[1] & 0b00000001) == 0b00000001)
		{
			flag_headlights = 1;
		}
		else
		{
			flag_headlights = 0;
		}
		/* Check message 2 (Right stick X) NOT USED */
		/* Check message 3 (Right stick Y) NOT USED*/
		/* Check message 4 (Left stick Y) */
		current_PWM_THROTTLE = (message[4] & 0b01111111);
		/* Check message 5 (Left stick X) */
		current_PWM_STEERING = (message[5] & 0b01111111);
		return 10;
	}
}



void abortProcedure(void){
	//it is handled in the main function
	#if ENABLE_XMC_DEBUG_PRINT
	printf("abort procedure");
	#endif
}

void activateFlaps(uint8_t level){

	flag_flaps = level;
	#if ENABLE_XMC_DEBUG_PRINT
	printf("activate flaps");
	#endif
}

void startReport(void){
	//write function for the plane to change to transmitter and send telemetry data to user
	#if ENABLE_XMC_DEBUG_PRINT
	printf("start report");
	#endif
}




/*******************************************************************************
* Function Name: PWM_0_PERIOD_MATCH_EVENT_HANDLER
********************************************************************************
* Summary:
* This is the interrupt handler function for the CCU4 compare match interrupt.
* It clears the event flag and restarts the timer. 
*
* Parameters:
*  none
*
* Return:
*  void
*
*******************************************************************************/

//void PWM_0_PERIOD_MATCH_EVENT_HANDLER(void)
//{
//    /* Clear pending interrupt */
//    XMC_CCU8_SLICE_ClearEvent(PWM_0_HW, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
//
//    /* Restart the Timer using SCU.GSC40 signal*/
//    //XMC_SCU_SetCcuTriggerLow(XMC_SCU_CCU_TRIGGER_CCU40);
//    XMC_CCU8_SLICE_SetTimerCompareMatch(PWM_0_HW, 0, PWM0_compare);
//    XMC_CCU8_EnableShadowTransfer(ccu8_0_HW, XMC_CCU8_SHADOW_TRANSFER_SLICE_0);
//
//    //XMC_SCU_SetCcuTriggerHigh(XMC_SCU_CCU_TRIGGER_CCU40);
//    counter ++;
//}
//
//void PWM_1_PERIOD_MATCH_EVENT_HANDLER(void)
//{
//    /* Clear pending interrupt */
//    XMC_CCU8_SLICE_ClearEvent(PWM_1_HW, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
//
//    /* Restart the Timer using SCU.GSC40 signal*/
//    //XMC_SCU_SetCcuTriggerLow(XMC_SCU_CCU_TRIGGER_CCU40);
//    if(PWM1_compare >= min_PWM && PWM1_compare <= max_PWM){
//    	XMC_CCU8_SLICE_SetTimerCompareMatch(PWM_1_HW, 0, PWM1_compare);
//    	XMC_CCU8_EnableShadowTransfer(ccu8_0_HW, XMC_CCU8_SHADOW_TRANSFER_SLICE_1);
//    }
//
//    //XMC_SCU_SetCcuTriggerHigh(XMC_SCU_CCU_TRIGGER_CCU40);
//    counter ++;
//}
//
//void PWM_2_PERIOD_MATCH_EVENT_HANDLER(void)
//{
//    /* Clear pending interrupt */
//    XMC_CCU8_SLICE_ClearEvent(PWM_2_HW, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
//
//    /* Restart the Timer using SCU.GSC40 signal*/
//    //XMC_SCU_SetCcuTriggerLow(XMC_SCU_CCU_TRIGGER_CCU40);
//    if(PWM1_compare >= min_PWM && PWM1_compare <= max_PWM){
//    	XMC_CCU8_SLICE_SetTimerCompareMatch(PWM_2_HW, 0, PWM2_compare);
//    	XMC_CCU8_EnableShadowTransfer(ccu8_0_HW, XMC_CCU8_SHADOW_TRANSFER_SLICE_2);
//    }
//
//    //XMC_SCU_SetCcuTriggerHigh(XMC_SCU_CCU_TRIGGER_CCU40);
//    counter ++;
//}
//
//void PWM_3_PERIOD_MATCH_EVENT_HANDLER(void)
//{
//    /* Clear pending interrupt */
//    XMC_CCU8_SLICE_ClearEvent(PWM_3_HW, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
//
//    /* Restart the Timer using SCU.GSC40 signal*/
//    //XMC_SCU_SetCcuTriggerLow(XMC_SCU_CCU_TRIGGER_CCU40);
//    if(PWM1_compare >= min_PWM && PWM1_compare <= max_PWM){
//    	XMC_CCU8_SLICE_SetTimerCompareMatch(PWM_3_HW, 0, PWM3_compare);
//    	XMC_CCU8_EnableShadowTransfer(ccu8_0_HW, XMC_CCU8_SHADOW_TRANSFER_SLICE_3);
//    }
//
//    //XMC_SCU_SetCcuTriggerHigh(XMC_SCU_CCU_TRIGGER_CCU40);
//    counter ++;
//}

/* [] END OF FILE */
