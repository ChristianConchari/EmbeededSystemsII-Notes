#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif
uint8_t step[4]={0x01,0x02,0x04,0x08};

int
main(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); //Enable peripheral N
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL); //Enable peripheral L

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))	//While peripheral N is enabled
    {
    }

    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0); //PN0
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3); //PL0, PL1, PL2, PL3

    while(1)
    {
	for(int i=0; i<200;i++)
	{
		GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,i%4); //Write in PL0|1|2|3
		SysCtlDelay(4000000);
	}
	SysCtlDelay(120000000); 
	for(int i=200; i>0;i--)
	{
		GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,i%4);	//Write in PL0|1|2|3
		SysCtlDelay(4000000);
	}
	SysCtlDelay(120000000);
    }
}
