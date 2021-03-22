#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"

// Extracted From timer.c
#include "inc/hw_types.h"   //!
#include "driverlib/timer.h" //!

#define freq_m1 1500 //!
uint32_t g_ui32SysClock;
uint32_t g_ui32PWMIncrement;//!
uint32_t vel_pwm=0; //!
uint32_t g_ui32Flags;
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

char data[30] = ""; //Define an array for receiving data

void //all this function was copied from timers.c
//It is used for working with interruptions
Timer0IntHandler(void)
{
    char cOne, cTwo;
    //
    // Clear the timer interrupt.
    //
    MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //
    HWREGBITW(&g_ui32Flags, 0) ^= 1; //? ⁼=: XOR

    //
    // Use the flags to Toggle the LED for this timer
    //
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, g_ui32Flags);

    //
    // Update the interrupt status.
    //
    MAP_IntMasterDisable(); //Does not admit  interrupts at this instant

    //Aditional logic
    cOne = HWREGBITW(&g_ui32Flags, 0) ? '1' : '0';  //Obtain status
    cTwo = HWREGBITW(&g_ui32Flags, 1) ? '1' : '0';  //Obtain status
    //UARTprintf("\rT1: %c  T2: %c", cOne, cTwo);
    MAP_IntMasterEnable();
}


void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    ui32Status = MAP_UARTIntStatus(UART0_BASE, true);

    MAP_UARTIntClear(UART0_BASE, ui32Status);

    uint8_t ind = 0;
    uint8_t dig1, dig2, dig3, dig4; //!
    while(MAP_UARTCharsAvail(UART0_BASE)) //While UART channel is available
    {
        //MAP_UARTCharPutNonBlocking(UART0_BASE, MAP_UARTCharGetNonBlocking(UART0_BASE));
	    data[ind] = MAP_UARTCharGetNonBlocking(UART0_BASE); //Receive UART data
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0); //Write on PN0 - HIGH

        SysCtlDelay(g_ui32SysClock / (1000 * 3));

        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);   //Write on PN0 - LOW
	ind++;
    }
    
    if (data[0]=='P' && data[1] == 'W' && data[2] == 'M' && data[3] == '_') //!
    {
        dig1 = data[4] - 48; //!
        dig2 = data[5] - 48; //!
        dig3 = data[6] - 48; //!
        dig4 = data[7] - 48; //!
        vel_pwm = dig1 * 1000 + dig2*100 + dig3*10 + dig4; //!
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, vel_pwm); //! 
        // Third argument [0 - (clock_frequency/prescaler)/PWM_frequency-1] --> [0 - (120000000/64)/1500-1] --> 1249 //Module 0 PWM 5 (Output)
    }
}

void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    while(ui32Count--)
    {
        MAP_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++); //Send UART data
    }
}

int
main(void)
{
    uint32_t ui32PWMClockRate; //!
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240), 120000000);
                                             
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); //Enables to work with peripherals
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); //! Enables peripheral PWM0
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //! Enables peripheral F
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG); // Enables peripheral G

    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0); //Defines PN0 as an Output

    MAP_GPIOPinConfigure(GPIO_PG1_M0PWM5); //** Defines PG1
    MAP_GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1); //! Defines PG1 as PWM
    MAP_PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_64); //! Prescaler defined as 64
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); //! Enables peripheral timer 0
    ui32PWMClockRate = g_ui32SysClock / 64; //! Prescaler 64

    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); //** // PWM Generator 0

    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, (ui32PWMClockRate / freq_m1)); //! //1500 == freq 

    g_ui32PWMIncrement = ((ui32PWMClockRate / freq_m1) / 1000); //! 

    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, g_ui32PWMIncrement); //! Module 0 PWM 5 (Output)
    // Third argument [0 - (clock_frequency/prescaler)/PWM_frequency-1] --> [0 - (120000000/64)/1500-1] --> 1249 
    
    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); //! Configure timers
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock); //!

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); //Enables UART
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //Enablaes GPIOA

    MAP_IntMasterEnable(); //! Enable Interrupts

    MAP_IntEnable(INT_TIMER0A);//! Enable interrupts for timer 0A
    MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);//! Enable interrupts for timer 0A
    
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0IntHandler); //! //Register interupts

    MAP_TimerEnable(TIMER0_BASE, TIMER_A);//! Enable interrupts for timer 0A

    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    MAP_UARTConfigSetExpClk(UART0_BASE, g_ui32SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    MAP_IntEnable(INT_UART0);
    MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    //UARTSend((uint8_t *)"\033[2JEnter text: ", 16);

    while(1)
    {
    }
}
