#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/adc.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

#define PWM_FREQUENCY 55

volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
float ui8Adjust;
volatile uint32_t ui32VoltAvg;
volatile uint32_t ui32Volt1;
volatile uint32_t ui32Volt2;
volatile  uint32_t voltage;
volatile  uint32_t volt1;



//#define CCP2_PERIPH             SYSCTL_PERIPH_GPIOD
unsigned long g_ulFlags;
//unsigned long count;
unsigned long timer;
//char r[20];
const float time = 0.1;
int freq;
int rpm;
int rpm_thousand;
int rpm_hundred;
int rpm_ten;
volatile uint32_t count;
/*
 *  Function prototype
 */
void calc_freq();
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void Timer0Handler(void)
{   //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Get the counter value
    //
    timer=TimerValueGet(TIMER1_BASE,TIMER_A);

    //
    // Reset the counter value to 10000
    //
    TimerLoadSet(TIMER1_BASE, TIMER_A,10000);

}

//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
//
//*****************************************************************************
void
Timer1IntHandler(void){
}


int main(void){
    // comparator
    ui8Adjust = 500;

    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    // enable peripheral PWM1 and GPIOD modules
    // for the PWM output on pin PD0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    // Display rpm thousand
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4);
    // Display rpm ten
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
    // Display rpm hundred
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);


    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_M1PWM0);

    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    ui32PWMClock = SysCtlClockGet() / 64;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);

    // Set system clock to 80MHz using a PLL (200MHz / 2.5 = 80MHz)
    //SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
    //SYSCTL_XTAL_16MHZ);

    // ====== ADC Code ====================================================== //

    /* Before you begin:
     * We are connecting the potentiometer to PB5, which corresponds to AIN11.
     * See page 801 in the datasheet for pin assignments. Note that the two
     * ADC modules, ADC0 and ADC1, share the same 12 input channels.
     */

    /* This array is used for storing the data read from the ADC FIFO. It
     * must be as large as the FIFO for the sequencer in use.  This example
     * uses sequence 3 which has a FIFO depth of 1.  If another sequence
     * was used with a deeper FIFO, then the array size must be changed.
     */
    uint32_t pui32ADC0Value[1];

    // Enable the ADC peripheral and wait for it to be ready.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)){}

    // Enable GPIO port B and wait for it to be ready.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)){}

    // Configure PB5 as an ADC input
    GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_5);

    /* Configure the ADC sample sequence.
     * Enable sample sequence 3 with a processor signal trigger.  Sequence 3
     * will do a single sample when the processor sends a signal to start the
     * conversion.
     */
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    /* Configure the ADC sample sequence steps
     * Configure step 0 on sequence 3.  Sample channel 11 (ADC_CTL_CH11) in
     * single-ended mode (default) and configure the interrupt flag
     * (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
     * that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
     * 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
     * sequence 0 has 8 programmable steps.  Since we are only doing a single
     * conversion using sequence 3 we will only configure step 0.  For more
     * information on the ADC sequences and steps, reference the datasheet.
     */
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH11 | ADC_CTL_IE |
                             ADC_CTL_END);

    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    /* Clear the interrupt status flag.  This is done to make sure the
     * interrupt flag is cleared before we sample.
     */
    ADCIntClear(ADC0_BASE, 3);

    // Enable the peripherals used by this example.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    // Enable the D peripheral used by the TIMER 1 pin CPP2.(!! The counter will not work if the peripheral is not enabled)

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // This function takes one of the valid names for a Timer pin and configures
    // the pin for its Timer functionality depending on the part that is defined.
    GPIOPinConfigure(GPIO_PF2_T1CCP0);
    GPIOPinTypeTimer(GPIO_PORTF_BASE, GPIO_PIN_2);

    // Configure the two 32-bit periodic timers.
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_A_CAP_COUNT);

    // This function configures the timer load value; if the timer is running
    // then the value is immediately loaded into the timer.
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());//8000000/10=80000=100 milliseconds
    TimerLoadSet(TIMER1_BASE, TIMER_A, 10000);


    //TimerLoadSet(TIMER1_BASE, TIMER_A,10000);
    // This function configures the signal edge(s) that triggers the
    // timer when in capture mode.
    TimerControlEvent(TIMER1_BASE,TIMER_A,TIMER_EVENT_POS_EDGE);

    // Setup the interrupt for the Timer0-TimerA timeouts.
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER0_BASE, TIMER_A, *Timer0Handler);

    // Enable the timers.
    TimerEnable(TIMER0_BASE, TIMER_A);// 100ms period interrupt
    TimerEnable(TIMER1_BASE, TIMER_A);// rising edge counter

    // Enable processor interrupts.
    IntMasterEnable();



    while(1){
        // TIMER INTERRUPTS PART
        calc_freq();

        // Trigger the ADC conversion.
        ADCProcessorTrigger(ADC0_BASE, 3);

        // Wait for conversion to be completed.
        while(!ADCIntStatus(ADC0_BASE, 3, false)){}

        // Clear the ADC interrupt flag.
        ADCIntClear(ADC0_BASE, 3);

        /* Read ADC Value. ADC has 12-bit precision so the output ranges from
         * 0 to 4095
         */
        ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);

        /* Set the PWM duty cycle to based on the ADC reading. The ADC is
         * 12-bit, so its max reading is 4095.
         *
         * Warning! The PWM module has hard time hitting 100%, so when you turn
         * the potentiometer all the way to the max, the LED may shut off or
         * flicker. This is a known issue with the PWM module, not an ADC issue:
         * https://e2e.ti.com/support/microcontrollers/tiva_arm/f/908/t/448664
         *
         * The LED may blink a little when you turn the potentiometer to 0.
         * I think this is just noise that's more visible when the LED
         * brightness is low. The custom Tiva board should have a better
         * analog power supply, so hopefully the issue will be resolved with
         * that. We'll see...
         */
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3) * pui32ADC0Value[0] / 4095);
        volt1 = pui32ADC0Value[0];

        voltage = (3.3 - ((3.3 * volt1) / 4095));

        ui8Adjust = volt1/4.0;
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
        SysCtlDelay(100000);
        if (rpm<1000){
            rpm_thousand=0;
            rpm_hundred=rpm/100;
            rpm_ten=(rpm-(rpm_hundred*100))/10;
        }
        else{
        rpm_thousand=rpm/1000;
        rpm_hundred=(rpm-(rpm/1000)*1000)/100;
        rpm_ten=(rpm-(rpm/100)*100)/10;
        }
        // Using PORT E PIN 1 2
        if(rpm_thousand==0){
            // D C B A
            // 0 0 0 0
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4,0);
        }else if(rpm_thousand==1){
            // D C B A
            // 0 0 0 1
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4,2);
        }else if(rpm_thousand==2){
            // D C B A
            // 0 0 1 0
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4,6);
        }

        // Using PORT B PIN 0 1 2 3
        if(rpm_hundred==0){
            // D C B A
            // 0 0 0 0
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0);
        }else if(rpm_hundred==1){
            // D C B A
            // 0 0 0 1
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,1);
        }else if(rpm_hundred==2){
            // D C B A
            // 0 0 1 0
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,2);
        }else if(rpm_hundred==3){
            // 0 0 1 1
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,3);
        }else if(rpm_hundred==4){
            // 0 1 0 0
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,4);
        }else if(rpm_hundred==5){
            // 0 1 0 1
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,5);
        }else if(rpm_hundred==6){
            // 0 1 1 0
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,6);
        }else if(rpm_hundred==7){
            // 0 1 1 1
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,7);
        }else if(rpm_hundred==8){
            // 1 0 0 0
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,8);
        }else if(rpm_hundred==9){
            // 1 0 0 1
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,9);
        }
        int x=10;
        // Using PORT A PIN 4 5 6 7
        if(rpm_ten==0){
            GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0);
        }else if(rpm_ten==1){
            GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,1*x);
        }else if(rpm_ten==2){
            GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,2*x);
        }else if(rpm_ten==3){
            GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,3*x);
        }else if(rpm_ten==4){
            GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,4*x);
        }else if(rpm_ten==5){
            GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,5*x);
        }else if(rpm_ten==6){
            GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,6*x);
        }else if(rpm_ten==7){
            GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,7*x);
        }else if(rpm_ten==8){
            GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,8*x);
        }else if(rpm_ten==9){
            GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,9*x);

        }
    }
}

void calc_freq(){
    //char r[8];
    float time = 1; // width of one period
    count = (10000 - timer);  // *100 converts kHz to Hz for easy displaying
    freq = time * count;            // Time is the time period the first timer is measuring, in this case it is 100mS
    rpm = freq/2*60;
}






