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

//From timer.c
#include "inc/hw_types.h"   //!
#include "driverlib/timer.h" //!

#define freq_m1 1000 //chaged for exercise
uint32_t g_ui32SysClock;
uint32_t g_ui32PWMIncrement;//!
uint32_t vel_pwm=0; //!
uint32_t g_ui32Flags;
bool flag;
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

char data[30] = "";

void //all this function was copied from timers.c
Timer1IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    MAP_TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT); //?TIMEOUT: Tiempo de espera cumplido

    //
    // Toggle the flag for the first timer.
    //
    HWREGBITW(&g_ui32Flags, 0) ^= 1; //? ^=: XOR W/R on register

    //
    // Update the interrupt status.
    //
    
    // Aditional logic
    HWREGBITW(&g_ui32Flags, 0) ? '1' : '0'; //? Obtener status
    HWREGBITW(&g_ui32Flags, 1) ? '1' : '0'; //? Obtener status
    

    MAP_IntMasterDisable(); 
    
    if (flag) 
    {
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0xFF);
    }else{
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0x00);

    }
    flag = !flag; //Negates flag

    MAP_IntMasterEnable();
}


void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    ui32Status = MAP_UARTIntStatus(UART0_BASE, true);

    MAP_UARTIntClear(UART0_BASE, ui32Status);
    uint8_t ind = 0;
    uint8_t dig1,dig2,dig3,dig4;

    while(MAP_UARTCharsAvail(UART0_BASE))
    {
        // MAP_UARTCharPutNonBlocking(UART0_BASE,
        //                            MAP_UARTCharGetNonBlocking(UART0_BASE)); //enviar
	
	data[ind] = MAP_UARTCharGetNonBlocking(UART0_BASE);
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);

        SysCtlDelay(g_ui32SysClock / (1000 * 3));

        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
	ind++;
    }
    if (data[0] == 'P' && data[1] == 'W' && data[2] == 'M' && data[3] == '_')
    {
        dig1 = data[4] - 48; 
        dig2 = data[5] - 48; 
        dig3 = data[6] - 48; 
        dig4 = data[7] - 48; 
        vel_pwm = dig1 * 1000 + dig2 *100 + dig3 *10 + dig4 * 1;
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, vel_pwm); ////Third argument  {0 - (clock_frequency/prescaler)/PWM_frequency -1} ---> (120000000/8)/250 -1
    }
    
    if (data[0] == 'L' && data[1] == 'D' && data[2] == '_' && data[3] == 'O' && data[4] == 'N')
    {
        MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0xFF);
    }
    
    if (data[0] == 'L' && data[1] == 'D' && data[2] == '_' && data[3] == 'O' && data[4] == 'F' && data[5] == 'F')
    {
        MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x00);    
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
    
    uint32_t ui32PWMClockRate;////!!!
    
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240), 120000000);
                                             
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //*
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); //*

    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4); //*
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1); //*
    

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); //Enable timer 1
    //
    // Enable the GPIO port that is used for the PWM output.
    //
    MAP_GPIOPinConfigure(GPIO_PF0_M0PWM0);//** Only for PWM
    
    MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0); //*
    
    MAP_PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_64); //*
    ui32PWMClockRate = g_ui32SysClock / 64; //*

    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); //*

    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, (ui32PWMClockRate / freq_m1)); //1000 = freq

    g_ui32PWMIncrement = ((ui32PWMClockRate / freq_m1) / 1000);
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1200); ////Third argument  {0 - (clock_frequency/prescaler)/PWM_frequency -1} ---> (120000000/8)/250 -1


    MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(TIMER1_BASE, TIMER_B, g_ui32SysClock/2);

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);

    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_0);

    MAP_IntMasterEnable();

    

    // Set GPIO A0 and A1 as UART pins.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    MAP_UARTConfigSetExpClk(UART0_BASE, g_ui32SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
                             
    MAP_IntEnable(INT_TIMER1B);
    MAP_TimerIntEnable(TIMER1_BASE, TIMER_TIMB_TIMEOUT);
    //TimerIntRegister(TIMER1_BASE, TIMER_B, Timer1IntHandler); //? Registrar interrupción (op 2)
    MAP_TimerEnable(TIMER1_BASE, TIMER_B);

    //
    // Enable the UART interrupt.
    //
    MAP_IntEnable(INT_UART0);
    MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    
    UARTSend((uint8_t *)"\033[2JEnter text: ", 16);

    while(1)
    {
    }
}