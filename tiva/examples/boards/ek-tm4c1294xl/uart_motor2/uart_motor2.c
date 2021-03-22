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

#include "inc/hw_types.h"   //!
#include "driverlib/timer.h" //!

#define freq_m1 500 //!

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

char data[30] = "";

void 
Timer1IntHandler(void)
{
    char cOne, cTwo;

    //
    // Clear the timer interrupt.
    //
    MAP_TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT); //changed for exercise

    //
    // Toggle the flag for the first timer.
    //
    HWREGBITW(&g_ui32Flags, 0) ^= 1; //? ⁼=: XOR

    //
    // Use the flags to Toggle the LED for this timer
    //
    //GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, g_ui32Flags);//!

    //
    // Update the interrupt status.
    //
    MAP_IntMasterDisable(); //Doe not admit  interrupts at this instant
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
    uint8_t dig1, dig2, dig3, dig4, dig5; //!
    while(MAP_UARTCharsAvail(UART0_BASE))
    {
        //MAP_UARTCharPutNonBlocking(UART0_BASE, MAP_UARTCharGetNonBlocking(UART0_BASE));
        data[ind] = MAP_UARTCharGetNonBlocking(UART0_BASE);
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, GPIO_PIN_5);

        SysCtlDelay(g_ui32SysClock / (1000 * 3));

        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, 0);
        ind++;
    }
    if (data[0]=='S' && data[1] == 'P' && data[2] == 'E' && data[3] == 'E' && data[4] == 'D' && data[5] == '_' )  //!
    {
        dig1 = data[6] - 48; //!
        dig2 = data[7] - 48; //!
        dig3 = data[8] - 48; //!
        dig4 = data[9] - 48; //!
        dig5 = data[10] - 48; //!
        vel_pwm = dig1 * 10000 + dig2*1000 + dig3*100 + dig4*10 + dig5*1; //!
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, vel_pwm); //! 
    }
}

void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    while(ui32Count--)
    {
        MAP_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
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
                                             
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); //!
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //!
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG); //!
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK); // 

    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);

    MAP_GPIOPinConfigure(GPIO_PK5_M0PWM7); //** 
    MAP_GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_5); //! 
    MAP_PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_8); //!   
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); //! 
    ui32PWMClockRate = g_ui32SysClock / 8; //! 

    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); //** 

    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, (ui32PWMClockRate / freq_m1)); //! 500 == freq //** 

    g_ui32PWMIncrement = ((ui32PWMClockRate / freq_m1) / 1000); //! * //changed for exercise

    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, g_ui32PWMIncrement); //! 
    
    MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); //! 
    MAP_TimerLoadSet(TIMER1_BASE, TIMER_B, g_ui32SysClock); //! 

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    MAP_IntMasterEnable(); //!

    MAP_IntEnable(INT_TIMER1B);//! 
    
    MAP_TimerIntEnable(TIMER1_BASE, TIMER_TIMB_TIMEOUT);//! 
    

    TimerIntRegister(TIMER1_BASE, TIMER_B, Timer1IntHandler); //! 

    MAP_TimerEnable(TIMER1_BASE, TIMER_B);//!

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
