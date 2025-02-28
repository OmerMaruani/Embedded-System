/***************************************************************************//**
 * PWM signal  for EFM32PG28_PK2506A
 *******************************************************************************
 * Copyright (C) 2025 BRAUDE COLLEGE
 * Electronics & Electrical Engineering Department
 * All rights reserved.
 *******************************************************************************
 * Owner        :  Dr. Fadil Tareef
 * FILE NAME    :  EXRC4_3.c
 * DATE         :  25 SEP 2024
 * TARGET MCU   :  EFM32PG28_PK2506A
 * DESCRIPTION  :  PWM signal for EFM32PG28_PK2506A using timer0.
 *                 The code demonstrates how to generate a PWM signal
 *                 with timer0/compare mode using emlib functions .
 * NOTES        :  This program created by simplicity studio 5
 *
 ******************************************************************************/
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"

#include "em_gpio.h"
#include "em_timer.h"
#include "sl_segmentlcd.h"


#include "gpiointerrupt.h"
#include "sl_assert.h"
#include "sl_common.h"
#include "sl_interrupt_manager.h"

//#include "em_int.h"
#include <stddef.h>
#include <string.h>  // Include for strcmp
#include "em_eusart.h"



// Desired PWM frequency and initial duty cycle
#define PWM_FREQ            50      //32bits Timer0
#define PWM2_FREQ           800     //16bits Timer1

float INITIAL_DUTY_CYCLE =  5 ;    // Servo motor
int INITIAL2_DUTY_CYCLE = 25;        // DC Motor
bool PWM_Grow = true;                // Flag to servo move





// Duty cycle global variable for IRQ handler use
static volatile float dutyCycle;
static volatile float dutyCycle2;

static volatile bool start = true;   // Flag for start, and stop
static volatile bool right =  false;  // Flag for direction control
static volatile bool rotate = false;  // Flag for rotate control


uint32_t timerFreq, topValue, dutyCount;
uint32_t timerFreq2, topValue2, dutyCount2;


static volatile bool InterruptStartStopFlag = false;
//bool InterruptStartStopFlag = false;
//bool InterruptStartStopFlag


// Timer2 interrupt mode
#define sys_clock 39000000  // SYS clock in Hz
#define timer_prescaler 1   // Timer divider = 1
#define signal_freq 2       // Signal frequency in Hz (0.5-second interval)



#define BUFLEN  10                // Buffer size
volatile uint8_t buffer[BUFLEN];  // Buffer
volatile uint32_t inpos = 0;      // Buffer position

uint32_t outpos = 0;
volatile bool receive = true;     // True while receiving data


void gpioCallbackStartStop()
{
  InterruptStartStopFlag = ! InterruptStartStopFlag;
  GPIO_PinOutToggle(gpioPortC,11);
  start = !start;
  if( start == true)
    {
      TIMER_Enable(TIMER1, true);   // Motor on
      INITIAL2_DUTY_CYCLE = 50;      // Speed 50
      InitPWM2();                   // Speed to 75 to initial the motor
      sl_segment_lcd_write("ON");

    }
  else
    {
      TIMER_Enable(TIMER1, false);  // Motor off
      sl_segment_lcd_write("OFF");
    }

}

void gpioCallbackSpeedControl()
{
  GPIO_PinOutSet(gpioPortC,10);
  if(start == true)
    {

      //InitPWM();
      InitPWM2(); // Increase speed
      //GPIO_PinOutSet(gpioPortC,10);
    }
  GPIO_PinOutClear(gpioPortC,10);

}

void gpioCallbackDirectionControl()
{



   //First stop the motor
  GPIO_PinOutClear(gpioPortD,7);
  GPIO_PinOutClear(gpioPortD,6);

  //Use that to wait the motor to stop, then change direction
  volatile int delayMs = 1000;
  volatile uint32_t i;
     for (i = 0; i < delayMs * 1000; i++) {
         // Empty loop to create a delay
     }


  right = !right ;
  if(right == true)
    {
      GPIO_PinOutClear(gpioPortD,7);
      GPIO_PinOutSet(gpioPortD,6);
    }
  else
    {
      GPIO_PinOutSet(gpioPortD,7);
      GPIO_PinOutClear(gpioPortD,6);
    }


}

void gpioCallbackRotateControl()
{

  rotate = !rotate;

  if( rotate )
    {
      GPIO_PinOutSet(gpioPortD,13);
      TIMER_Enable(TIMER2, true);

    }
  else
    {
      GPIO_PinOutClear(gpioPortD,13);
      TIMER_Enable(TIMER2, false);
    }

  /*
  GPIO_PinOutSet(gpioPortD,13);

  //Use that to wait the motor to stop, then change direction
    volatile int delayMs = 1000;
    volatile uint32_t i;
       for (i = 0; i < delayMs * 1000; i++) {
           // Empty loop to create a delay
       }

  InitPWM();



  GPIO_PinOutClear(gpioPortD,13);
*/
}




/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{
    //Enable GPIO & TIMER0 clock
    CMU_ClockEnable(cmuClock_GPIO, true);
    CMU_ClockEnable(cmuClock_TIMER0, true);
    CMU_ClockEnable(cmuClock_TIMER1, true);
    CMU_ClockEnable(cmuClock_TIMER2, true);
    CMU_ClockEnable(cmuClock_EUSART1, true);

    // Configure PD11 as output for PWM
    GPIO_PinModeSet(gpioPortD, 11 , gpioModePushPull, 0);
    // Configure PD10 as output for PWM ( 50hz )
    GPIO_PinModeSet(gpioPortD, 10 , gpioModePushPull, 0);

    // Configure the EUSART TX pin to the board controller as an output - PB0
    GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 1);
    // Configure the EUSART RX pin to the board controller as an input - PB1
    GPIO_PinModeSet(gpioPortA, 3, gpioModeInput, 0);


    // Configure PB0,1 LED0,1
    GPIO_PinModeSet(gpioPortB,1, gpioModeInput, 1);   //PB0
    GPIO_PinModeSet(gpioPortB,6, gpioModeInput, 1);   //PB1
    GPIO_PinModeSet(gpioPortC,10, gpioModePushPull, 0);   //LED0
    GPIO_PinModeSet(gpioPortC,11, gpioModePushPull, 1);   //LED1


    // Direction
    GPIO_PinModeSet(gpioPortD,8, gpioModeInputPull,1);  //Interrupt pushbutton
    GPIO_PinModeSet(gpioPortD,7, gpioModePushPull, 1);  //Right
    GPIO_PinModeSet(gpioPortD,6, gpioModePushPull, 0);  //Left

    // Rotate
    GPIO_PinModeSet(gpioPortD,14, gpioModeInputPull,1);  //Interrupt pushbutton
    GPIO_PinModeSet(gpioPortD,13, gpioModePushPull,0);    //Led for rotate



    // Initialize the LCD
    CMU_ClockEnable(cmuClock_LCD,true);    //  LCD Enable
    sl_segment_lcd_init(false);




    InitPWM();
    InitPWM2();









    //Initialize GPIO interrupts
    GPIOINT_Init();
    __enable_irq();


    //PB1 Interrupt
    GPIO_ExtIntConfig(gpioPortB,6,6,false,true,true);
    GPIOINT_CallbackRegister(6,gpioCallbackStartStop);

    //PB0 Interrupt
    GPIO_ExtIntConfig(gpioPortB,1,1,false,true,true);
    GPIOINT_CallbackRegister(1,gpioCallbackSpeedControl);

    //PB3 Interrupt direction
    GPIO_ExtIntConfig(gpioPortD,8,8,false,true,true);
    GPIOINT_CallbackRegister(8,gpioCallbackDirectionControl);


    //PB4 Interrupt rotate
    GPIO_ExtIntConfig(gpioPortD,14,14,false,true,true);
    GPIOINT_CallbackRegister(14,gpioCallbackRotateControl);



    InitTimer2();


    InitCommunication();

}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{








}



void InitPWM()
{

  // Servo motor ( 50hz )
  // 0deg = 5% duty cycle, 90deg = 7.5 duty cycle, 180deg = 10 duty cycle

  if(INITIAL_DUTY_CYCLE == 10)  PWM_Grow = false;
  else if(INITIAL_DUTY_CYCLE == 3)  PWM_Grow = true;

  if(PWM_Grow) INITIAL_DUTY_CYCLE += 0.5;
  else INITIAL_DUTY_CYCLE -= 0.5;


  //Define local variables
  //uint32_t timerFreq, topValue, dutyCount;
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;

  // Don't start counter on initialization
  timerInit.enable = false;

  // PWM mode sets/clears the output on compare/overflow events
  timerCCInit.mode = timerCCModePWM;

  TIMER_Init(TIMER0, &timerInit);

  // Route CC0 output to PA6
    GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN;
    GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
                      | (10 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);

    TIMER_InitCC(TIMER0, 0, &timerCCInit);

    // Set top value to overflow at the desired PWM_FREQ frequency
    timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timerInit.prescale + 1);
    topValue = (timerFreq / PWM_FREQ);
    TIMER_TopSet(TIMER0, topValue);

    // Set dutyCycle global variable and compare value for initial duty cycle
    dutyCycle = INITIAL_DUTY_CYCLE;
    dutyCount = (topValue * INITIAL_DUTY_CYCLE) / 100;
    TIMER_CompareSet(TIMER0, 0, dutyCount);

    // Now start the TIMER
    TIMER_Enable(TIMER0, true);

    //sl_segment_lcd_lower_number(INITIAL_DUTY_CYCLE);






}

void InitPWM2()
{
  if(INITIAL2_DUTY_CYCLE == 100)   INITIAL2_DUTY_CYCLE = 0;
    INITIAL2_DUTY_CYCLE += 25;


  //Define local variables
  //uint32_t timerFreq, topValue, dutyCount;
  TIMER_Init_TypeDef timerInit2 = TIMER_INIT_DEFAULT;
  TIMER_InitCC_TypeDef timerCCInit2 = TIMER_INITCC_DEFAULT;

  // Don't start counter on initialization
  timerInit2.enable = false;

  // PWM mode sets/clears the output on compare/overflow events
  timerCCInit2.mode = timerCCModePWM;

  TIMER_Init(TIMER1, &timerInit2);

  // Route CC0 output to PA6
    GPIO->TIMERROUTE[1].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN;
    GPIO->TIMERROUTE[1].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
                      | (11 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);

    TIMER_InitCC(TIMER1, 0, &timerCCInit2);

    // Set top value to overflow at the desired PWM_FREQ frequency
    timerFreq2 = CMU_ClockFreqGet(cmuClock_TIMER1) / (timerInit2.prescale + 1);
    topValue2 = (timerFreq2 / PWM2_FREQ);
    TIMER_TopSet(TIMER1, topValue2);

    // Set dutyCycle global variable and compare value for initial duty cycle
    dutyCycle2 = INITIAL2_DUTY_CYCLE;
    dutyCount2 = (topValue2 * INITIAL2_DUTY_CYCLE) / 100;
    TIMER_CompareSet(TIMER1, 0, dutyCount2);

    // Now start the TIMER
    TIMER_Enable(TIMER1, true);

    sl_segment_lcd_lower_number(INITIAL2_DUTY_CYCLE);




}

void InitTimer2()
{

  // Set Timer2 to generate an interrupt every 1 second
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  timerInit.prescale = timerPrescale1024;  // Set the prescaler to 1024
  TIMER_Init(TIMER2, &timerInit);

  // Set the compare value for a 1-second interval (with a 39 MHz clock)
  TIMER_TopSet(TIMER2, 38087/4);  // 39 MHz / 1024 ≈ 38087 timer counts for 1 second

  // Set up the TIMER2 overflow interrupt
  TIMER_IntClear(TIMER2, TIMER_IF_OF);  // Clear any previous interrupts
  TIMER_IntEnable(TIMER2, TIMER_IF_OF);  // Enable overflow interrupt
  NVIC_EnableIRQ(TIMER2_IRQn);  // Enable the interrupt in the NVIC

  // Wait untill first pushbutton3 interrupt
  TIMER_Enable(TIMER2, false);
}

void InitCommunication()
{
  // Default asynchronous initializer (9600bps, 8N1, no flow control)
  EUSART_UartInit_TypeDef init = EUSART_UART_INIT_DEFAULT_HF;
  init.baudrate = 9600;
  init.oversampling = eusartOVS4;
  // Route EUSART1 TX and RX to the board controller TX=PB0 and RX=PB1 pins
  GPIO->EUSARTROUTE[1].TXROUTE = (gpioPortA << _GPIO_EUSART_TXROUTE_PORT_SHIFT)
                 | (2 << _GPIO_EUSART_TXROUTE_PIN_SHIFT);
  GPIO->EUSARTROUTE[1].RXROUTE = (gpioPortA << _GPIO_EUSART_RXROUTE_PORT_SHIFT)
                 | (3 << _GPIO_EUSART_RXROUTE_PIN_SHIFT);
  // Enable RX and TX signals now that they have been routed
  GPIO->EUSARTROUTE[1].ROUTEEN = GPIO_EUSART_ROUTEEN_RXPEN | GPIO_EUSART_ROUTEEN_TXPEN;
  // Configure and enable EUSART0 for high-frequency (EM0/1) operation
  EUSART_UartInitHf(EUSART1, &init);
  // Enable NVIC USART sources
  NVIC_ClearPendingIRQ(EUSART1_RX_IRQn);
  NVIC_EnableIRQ(EUSART1_RX_IRQn);
  NVIC_ClearPendingIRQ(EUSART1_TX_IRQn);
  NVIC_EnableIRQ(EUSART1_TX_IRQn);

  //NVIC_SetPriority(EUSART1_RX_IRQn, 0);  // 0 is the highest priority (lower number = higher priority)


  // Enable receive FIFO level interrupt (defaults to one frame)
  EUSART_IntEnable(EUSART1, EUSART_IEN_RXFL);

  //GPIO_PinModeSet(gpioPortD,6, gpioModePushPull, 0);  //

}


/*

void StartStop()
{

}

void StopRotate()
{
  TIMER_Enable(TIMER0, false);
  sl_segment_lcd_write("Off");
}

void LeftSide()
{
  GPIO_PinOutSet(gpioPortD,9);
  GPIO_PinOutClear(gpioPortD,8);
}

void RightSide()
{
  GPIO_PinOutClear(gpioPortD,9);
  GPIO_PinOutSet(gpioPortD,8);
}

void StartRotate()
{
  GPIO_PinOutClear(gpioPortD,9);
  GPIO_PinOutSet(gpioPortD,8);
}
*/


void TIMER2_IRQHandler(void)
{
  // Clear the interrupt flag
  TIMER_IntClear(TIMER2, TIMER_IF_OF);
  InitPWM();

}


void EUSART1_RX_IRQHandler(void)
{
  // Get the character just received
  buffer[inpos] = EUSART1->RXDATA;
  // Exit loop on new line or buffer full
  if ((buffer[inpos] != '\r') && (inpos < BUFLEN))
    inpos++;
  else
     receive = false;   // Stop receiving on CR

  //gpioCallbackStartStop, gpioCallbackSpeedControl,
  //gpioCallbackDirectionControl,gpioCallbackRotateControl

      if(strncmp(buffer, "O", 1) == 0) GPIO_PinOutSet(gpioPortD,6);
      else if(strncmp(buffer, "N", 1) == 0) GPIO_PinOutClear(gpioPortD,6);
      else if(strncmp(buffer, "P", 1) == 0) gpioCallbackStartStop();
      else if(strncmp(buffer, "S", 1) == 0) gpioCallbackSpeedControl();
      else if(strncmp(buffer, "D", 1) == 0) gpioCallbackDirectionControl();
      else if(strncmp(buffer, "R", 1) == 0) gpioCallbackRotateControl();



      for (int i = 0; i < BUFLEN; i++)
            buffer[i] = 0;
      inpos = 0;

  /*
   * The EUSART differs from the USART in that explicit clearing of
   * RX interrupt flags is required even after emptying the RX FIFO.
   */
  EUSART_IntClear(EUSART1, EUSART_IF_RXFL);

}

/*
void EUSART1_TX_IRQHandler(void)
{
  // Send a previously received character
  if (outpos < inpos)
  {
    EUSART1->TXDATA = buffer[outpos++];

    EUSART_IntClear(EUSART1, EUSART_IF_TXFL);
  }
  else

  {
    receive = true;   // Go back into receive when all is sent
    EUSART_IntDisable(EUSART1, EUSART_IEN_TXFL);
  }
}
*/


