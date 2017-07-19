/* Title: SASB Single Axis FLoator Test Code
 * Author: Tyler Larson
 * Purpose: Through use of a PID controller a current sensor and a displacement sensor, the code manipulates a motor driver to hover a plate.
 */

/*
 * Includes
 */

#include "F28x_Project.h"   //Seems like a nice header file, includes a lot of things so I don't have to

/*
 * Defines
 */

//Sets the size of the array for the Integral and Derivative previous values
#define prev_size   64  //Size of the Memory array in the PID control

//Sets the number of Sysclk cycles that the ADC samples for
#define sample_cycles (20 - 1)  //Note* only change the first number, the 1 accounts for the minimum

/*
 * Structures
 */

/*! Position sensor structure */
typedef struct position_sensor_struct{
    int *sample_loc;        /*!< Sample value pointer */
    float sample;           /*!< Sample value, *(mm) */
    float target;           /*!< Target Displacement value *(mm) */
    float error;            /*!< Error for PID controls *(mm) */
    float dt;               /*!< Time period for PID control *(ms) */
    float kp;               /*!< Proportional Constant */
    float p;                /*!< Proportional calculated value */
    float ki;               /*!< Integral Constant */
    float i;                /*!< Integral calculated value */
    float kd;               /*!< Derivative Constant */
    float d;                /*!< Derivative calculated value */
    float pid_out;          /*!< Output of the PID control */
    int prev_place;         /*!< Current location in the array of previous values */
    int prev_last;          /*!< Last location in the array, used for Derivative term */
    float prev[prev_size];  /*!< Array of previous values, used for calculating the Derivative and Integral terms */
}position;

/*! Current Sensor and Coil Control Structure */
typedef struct current_coil_struct{
    int *sample_loc;        /*!< Sample value pointer */
    float sample;           /*!< Sample value, (*Amps) */
    float target;           /*!< Target Displacement value, *(Amps) */
    float error;            /*!< Error to determine more or less current, *(Amps)   **May change later for PWM and/or a PID loop */
    float scale;            /*!< Scaling constant to convert sensor reading to current, *(Amps/Volt) */
    float offset;           /*!< Offset constant to convert sensor reading to current, *(Amps) */
    float x_influence;      /*!< X-axis influence of the coil */
    float y_influence;      /*!< Y-axis influence of the coil */
    float bias;              /*!< Individual bias current of the coil **May use in Gauss/De-Gauss process in the future */
}current;

/*
 * Function Prototypes
 */
    //*Note, at some point this should be moved to a header file

void SetupPosition(position *sensor);
void SetupCoil(current *coil);
void Position_PID_Cntrl(position *sensor);
void current_target(current *coil,position *disp_sensor);
void Bang_Bang_Cntrl(current *coil);
void InitADCPart1();
void InitADCPart2();
void InitEPwm();
void PIEMap();
void EPwmStart();
interrupt void adca1_isr();
interrupt void adcb2_isr();
interrupt void epwm1_isr();
interrupt void epwm2_isr();

/*
 * Globals
 */

//Definition of the X1 instance of the position sensor structure
position X1;    //Variable used to store the PID math variables

//Definition of the current coil structure
current C1;     //Variable used to store the current control variables for coil 1

//Acuasition time calibration
    //*Used for Debug
int AQSTime = 499;
int AQUSFlag = 0;

//X displacement sensor calibration
    //*Used for Debug
float xmax;
float xmin;
float xdiff;

//Displacement sensor reading
int x1_sample;  //Global for x1 displacement sensor reading *(Digitized)

//Cutoff value for X displacement
    //*Used for Debug
int x1_cutoff = 0;  //Debuging purposes, clean out later

//Displacement sensor update flag
int x1_update;  //Flag for new displacement sensor readings

//Displacement sensor conversion scale
    //*Make this a define
float disp_scale = 1.079757 * 3.3 / 4096;   //Conversion scale for displacement sensor *(Millimeters/Volt)

//Displacement sensor conversion offset
    //*Make this a define
float disp_offset = 0.0004411644;  //Conversion offset for displacement sensor *(Millieters)

//Displacement sensor target
float x1_target = 1.2;    //Target float level *(Millimeters)

//Displacement sensor sample period
    //*Make this a define
float x1_dt = 0.1;   //*(MilliSeconds)

//Displacement to current ratio
    //*Make this a define
float disp_to_current = 2.2;  //Scales the PID output to current *(Meters/Amp)

//Current sensor reading
int i1_sample;  //Global for i1 current sensor reading *(Digitized)

//Current sensor reading in current
float i1_current;   //*(Amps)

//Current sensor update flag
int i1_update;  //Flag for new current sensor readings

//Current target
float i1_target;    //*(Amps)

//Current sensor conversion scale
    //*Make this a define
float current_scale = 10.168070782 * 3.3 / 4096;    //Conversion scale for current sensor *(Amps/Volt)

//Current sensor conversion offset      *Edited to zero
    //*Make this a define
float current_offset = -13.45198481;   //Conversion offset for current sensor *(Amps)

//Current Bias current for SASB floator
    //*Make this a define
float current_bias = 2.2;   //Calculated from the desired gap and the force on the floator due to gravity

//Maximum Current
float current_max = 15;  //Max current for operation *(Amps)

//Minimum Current
float current_min = -15; //Min current for operation *(Amps)

//PID constants
float   kp = .4,    //Default to 1.1 for Initial tuning
        ki = .3,    //Default to 0 for initial tuning
        kd = 15;    //Default to 0 for initial tuning

//Current control variables
float   i1_error,
        i1_pid_out;

//PWM control variables
int pwm_scale,
    pwm_duty_cycle,
    pwm_period;

/*
 * Main
 */
void main(void){
    //Main Initialization function for the 77D
    InitSysCtrl();

    //Setup the GPIO pins
    InitGpio();

    //Setup output pins
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO40 = 0;    
    GpioDataRegs.GPBSET.bit.GPIO40 = 1;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO40 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO40 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO40 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO39 = 0;
    GpioDataRegs.GPBSET.bit.GPIO39 = 1;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO39 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;
    EDIS;

    //Setup X1 variable
    SetupPosition(&X1);
    X1.sample_loc = &x1_sample;

    //Setup C1 variable
    SetupCoil(&C1);
    C1.sample_loc = &i1_sample;

    //Disable interrupts? Followed the example of process from the control suite example code
    DINT;

    //Initialize PIE control registers
    InitPieCtrl();

    //Disable CPU interrupts
    IER=0x0000;

    //Clear CPU Flags
    IFR=0x0000;

    //Initialize the PIE vector Table
    InitPieVectTable();

    //Setup PIE Table Mappings
    PIEMap();

    //ADC setup Part 1
    InitADCPart1();

    //ePWM setup function
    InitEPwm();

    //ADC setup Part 2
    InitADCPart2();

    //Enable output of ePWM signals to Pins
    InitEPwm1Gpio();
    InitEPwm2Gpio();

    //Enable The Interrupts
    IER |=(M_INT1 | M_INT3 | M_INT10); //Enable Groups 1,3 and 10

    //Enable Interrupts in PIE
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;  //Enable ADCA1 interrupt
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  //Enable ePWM1 interrupt
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;  //Enable ePWM2 interrupt
    PieCtrlRegs.PIEIER10.bit.INTx6 = 1; //Enable ADCB2 interrupt

    //Enable interrupts
    EINT;

    //Enable Real Time Management?
    ERTM;

    //Calibration Time
    //x1_target =

    //Start ePWM
    EPwmStart();
    //Loop
    for(;;){

        //Current control if statement
        if(i1_update){
            i1_update = 0;
            //Bang_Bang_Cntrl();

        }

        //If statement to facilitate the PID calculations,
            //*Figure out if the current control should still be here
        if(x1_update){
            x1_update = 0 ;
            GpioDataRegs.GPBTOGGLE.bit.GPIO40 = 1;
            Position_PID_Cntrl(&X1);        //*OLD* Takes 1.0226 micro seconds
            current_target(&C1, &X1);      //Displacement to Current target
            Bang_Bang_Cntrl(&C1);      //Current control function
            GpioDataRegs.GPBTOGGLE.bit.GPIO40 = 1;

        }

        //If statement to facilitate the changing of the acqusition time of ADCB and SOC1
        if(AQUSFlag){
            EALLOW;
            AdcbRegs.ADCSOC1CTL.bit.ACQPS = AQSTime;     //Set the sample window size for SOC1, 19=20 SysClkCycle
            EDIS;
            AQUSFlag=0;
        }

        //Simple if statement to capture the peak values of the x sensor, used durring initial debuging
        if(x1_sample>xmax||x1_sample<xmin){
            if(x1_sample>xmax){
                xmax=x1_sample;
            }
            else{
                xmin=x1_sample;
            }
            xdiff=xmax-xmin;
        }
    }
}

/*
 * Setup function for displacement structures
 */

void SetupPosition(position *sensor){
    int i;      //Used as a simple loop counter
    sensor->target = x1_target;         //Feeds the position structure the location of sample global
    sensor->dt = x1_dt;                 //Feeds in the specific sample period for the sensor
    sensor->kp = kp;                    //Sets default proportional constant
    sensor->ki = ki;                    //Sets default integral constant
    sensor->kd = kd;                    //Sets default derivative constant
    sensor->prev_place = 0;             //Starts the PID progression at 0
    sensor->prev_last = prev_size - 1;  //Starts the last pointer to the end of the array
    for(i=0;i<prev_size;i++){   
        sensor->prev[i]=0;              //Step by step through the PID memory array and zero it
    }
}

/*
 * Setup function for coil structure
 */

void SetupCoil(current *coil){
    coil->scale = current_scale;    //Sets the current conversion scale
    coil->offset = current_offset;  //Sets the current conversion offset
    coil->x_influence = 1;          //Sets the current x value scaling constant
    coil->y_influence = 1;          //Sets the current y value scaling constant
    coil->bias = current_bias;      //Sets the current bias point
}

/*
 * Position PID Control Function
 */

void Position_PID_Cntrl(position *sensor){
    //Limit sensor jitter
    if(*(sensor->sample_loc) < x1_cutoff){
        *(sensor->sample_loc) = 0;  //Sets to zero if jitter is close
    }

    //Read sample into structure
    sensor->sample = (*(sensor->sample_loc) * disp_scale) + disp_offset;    //*(Meters)

    //Initial error calculation between target and sample values
    sensor->error = sensor->sample - sensor->target;

    //Proportional term calculation
    sensor->p = sensor->error * sensor->kp;

    //Integral term calculation
        //This was fun to code, pretty much I subtract out the last value of the array at the same time I add in the new one!
    sensor->i = sensor->i + ((sensor->error - sensor->prev[sensor->prev_place]) * sensor->dt * sensor->ki);

    //Derivative term calculation
    sensor->d = (sensor->error - sensor->prev[sensor->prev_last]) * sensor->kd / sensor->dt;    

    //PID output
    sensor->pid_out = sensor->p + sensor->i + sensor->d;    //*(Meters?)

    //clean up operations to set the array for the next round of PID calculations
    /***May be better to have this in main or another function to
        easily scale for additional stabilization bearings      ***/
    sensor->prev[sensor->prev_place] = sensor->error;   //Places the current rounds error into the memory array
    sensor->prev_last = sensor->prev_place;     //Progresses the placement variable for the "last" location to the current place
    sensor->prev_place = (prev_size - 1) & (sensor->prev_place + 1);    //Progresses the current location variable while using the bitwise math to limit and loop the variable
}

/*
 * Current Target Function
 */

void current_target(current *coil, position *disp_sensor){
    float temp;
    /* In the next line is where the future addition of the X Y influence will be accounted for */
    temp = coil->bias + (disp_sensor->pid_out * disp_to_current); //*(Amps)
    if(temp > current_max || temp < current_min){
        if(temp > current_max){
            temp = current_max;     //Limits the target current to the max value
        }
        else{
            temp = current_min;     //Limits the target current to the min value
        }
    }
    coil->target = temp; //*(Amps)
}

/*
 * Current Control Function
 */

void Bang_Bang_Cntrl(current *coil){
    coil->sample = (*(coil->sample_loc) * coil->scale) + coil->offset;  //*(Amps)

    //Initial error calculation between target and sampled current
    coil->error = coil->sample - coil->target;

    //Moves the error to the PID output variable, allows for future integration of a PID control for current to PWM
    //i1_pid_out = i1_error;

    //Run P_H pin high or low depending on error
    if(coil->error < 0){
        GpioDataRegs.GPBSET.bit.GPIO39 = 1;     //Pin 88, This is Tied to the P_H pin for one of the Pololu's
    }
    else{
        GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;   //Pin 88, This is Tied to the P_H pin for one of the Pololu's
    }
    /*  Include when ready for PWM
    //Scales the current demand to a PWM duty cycle
    pwm_duty_cycle = i_pid_out * pwm_scale;

    //Logic flow to limit PWM min and max values
    if(pwm_duty_cycle < 0){
        pwm_duty_cycle = 0;
    }
    else{
        if(pwm_duty_cycle > pwm_period){
            pwm_duty_cycle = pwm_period;
        }
    }
     */
}

/*
 * ADC Initialization Functions
 */

void InitADCPart1(){
    //Enable editing of EALLOW registers
    EALLOW;

    //Enable The CLk signal for ADC A and B
    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
    CpuSysRegs.PCLKCR13.bit.ADC_B = 1;

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //Initialization Settings for ADC A
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;   //Sets Interrupt pulse to trigger after conversion
    AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0;    //Single Ended, not Differential
    AdcaRegs.ADCCTL2.bit.RESOLUTION = 0;    //Sets resolution to 12 bits, 1=16 bits
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;      //Sets the Prescale of the ADC, 6=1/4   *Be careful with value, if it is to low, issues with sample value will occur

    //Initialization Settings for ADC B
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;   //Sets Interrupt pulse to trigger after conversion
    AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0;    //Single Ended, not Differential
    AdcbRegs.ADCCTL2.bit.RESOLUTION = 0;    //Sets resolution to 12 bits, 1=16 bits
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;      //Sets the Prescale of the ADC, 6=1/4   *Be careful with value, if it is to low, issues with sample value will occur

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;      //Powers up all of the analog circuitry
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;      //Powers up all of the analog circuitry


    //Delay for 1 ms to allow ADCs to start up
    DELAY_US(1000);     //Required warmup time

    //Disable editing of EALLOW registers
    EDIS;
}

void InitADCPart2(){
    EALLOW;

    //Configuration Settings for ADCA
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC0, 5=ePWM1
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;      //Set the channel for SOC0 to convert, 0=ADCAIN0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 19;     //Set the sample window size for SOC0, 19=20 SysClkCycles
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;  //Set EOCx to trigger ADCINT1, 0=EOC0
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    //Enable/Disable ADCINT1
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 0; //Enable/Disable Continuous Mode
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //Clears ADCINT1 Flag

    //Configuration Settings for ADCB
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 8;    //Set the Trigger for SOC1, 8=ePWM2
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 0;      //Set the channel for SOC1 to convert, 0=ADCBIN0
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 19;     //Set the sample window size for SOC0, 19=20 SysClkCycles
    AdcbRegs.ADCINTSEL1N2.bit.INT2SEL = 1;  //Set EOCx to trigger ADCINT2, 1=EOC1
    AdcbRegs.ADCINTSEL1N2.bit.INT2E = 1;    //Enable/Disable ADCINT2
    AdcbRegs.ADCINTSEL1N2.bit.INT2CONT = 0; //Enable/Disable Continuous Mode
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;  //Clears ADCINT2 Flag

    EDIS;
}

/*
 * ePWM Initialization Function
 */

void InitEPwm(){
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;   //Disable the Time Base Clk
    //EDIS;

    //Enable Clk for PWM
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;   //Enable ePWM1 CLK
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;   //Enable ePWM2 CLK

    //Time Base Controls for ePWM1
    EPwm1Regs.TBCTL.bit.PHSEN = 0;      //Disable loading Counter register from Phase register
    EPwm1Regs.TBCTL.bit.PRDLD = 0;      //A shadow register is used to write to the Period register
    //EPwm1Regs.TBCTL.bit.SYNCOSEL      //Leave at default value
    //EPwm1Regs.TBCTL.bit.SWFSYNC       //Leave at default value
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;  //High speed Time Base Prescale, 0="/1"
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;     //Time Base Clock Prescale, 0="/1"
    //EPwm1Regs.TBCTL.bit.PHSDIR        //Leave at default value
    //EPwm1Regs.TBCTL.bit.FREE_SOFT     //Leave at default value
    EPwm1Regs.TBCTL.bit.CTRMODE = 3;    //Set counter mode, 3=freeze counter

    //Set ePWM1 counter to 0
    EPwm1Regs.TBCTR = 0;    //Resets ePWM1 counter register

    //Set ePWM1 period
    EPwm1Regs.TBPRD = 1249;     //Results in a period of 1/80 kHz

    //Disable ePWM1 phase sync
    EPwm1Regs.TBPHS.bit.TBPHS = 0;

    //Enable ePWM1 event Triggering selection
    EPwm1Regs.ETSEL.bit.SOCAEN = 0;     //Disable the SOCA conversion until ready
    EPwm1Regs.ETSEL.bit.SOCASEL = 1;    //Select SOCA to be triggered by ePWM1

    //Enable pulse on first event
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;

    //Set the Compare A register
    EPwm1Regs.CMPA.bit.CMPA = 625;      //Sets Compare A to a 50% duty cycle, arbitrary for now

    //Action qualifier control
    EPwm1Regs.AQCTLA.bit.CAU = 2;       //Action at A going up, 2=set_high
    EPwm1Regs.AQCTLA.bit.ZRO = 1;       //Action at zero, 1=clear


    //Time Base Controls for ePWM2
    EPwm2Regs.TBCTL.bit.PHSEN = 0;      //Disable loading Counter register from Phase register
    EPwm2Regs.TBCTL.bit.PRDLD = 0;      //A shadow register is used to write to the Period register
    //EPwm2Regs.TBCTL.bit.SYNCOSEL      //Leave at default value
    //EPwm2Regs.TBCTL.bit.SWFSYNC       //Leave at default value
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;  //High speed Time Base Prescale, 0="/1"
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;     //Time Base Clock Prescale, 0="/1"
    //EPwm2Regs.TBCTL.bit.PHSDIR        //Leave at default value
    //EPwm2Regs.TBCTL.bit.FREE_SOFT     //Leave at default value
    EPwm2Regs.TBCTL.bit.CTRMODE = 3;    //Set counter mode, 3=freeze counter

    //Set ePWM2 counter to 0
    EPwm2Regs.TBCTR = 0;    //Resets ePWM2 counter register

    //Set ePWM2 period
    EPwm2Regs.TBPRD = 9999;     //Results in a period of 1/10 kHz 

    //Disable ePWM2 phase sync
    EPwm2Regs.TBPHS.bit.TBPHS = 0;

    //Enable ePWM2 event Triggering selection
    EPwm2Regs.ETSEL.bit.SOCBEN = 0;     //Disable the SOCB conversion until ready
    EPwm2Regs.ETSEL.bit.SOCBSEL = 1;    //Select SOCB to be triggered by ePWM2

    //Enable pulse on first event
    EPwm2Regs.ETPS.bit.SOCBPRD = 1;

    //Set the Compare A register
    EPwm2Regs.CMPA.bit.CMPA = 5000;     //Sets Compare A to a 50% duty cycle, arbitrary for now

    //Action qualifier control
    EPwm2Regs.AQCTLA.bit.CAU = 2;       //Action at A going up, 2=set_high
    EPwm2Regs.AQCTLA.bit.ZRO = 1;       //Action at zero, 1=clear

    //CPU clock enable
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;   //Renable the Time Base Clk
    EDIS;
}

/*
 * PIE Mapping Function
 */

void PIEMap(){
    EALLOW;

    //PIE mappings for ADC interrupts
    PieVectTable.ADCA1_INT=&adca1_isr;
    PieVectTable.ADCB2_INT=&adcb2_isr;

    //PIE mappings for ePWM interrupts
    PieVectTable.EPWM1_INT=&epwm1_isr;
    PieVectTable.EPWM2_INT=&epwm2_isr;

    EDIS;
}

/*
 * ePWM Startup Function
 */

void EPwmStart(){
    EALLOW;

    //Enable SOCs
    EPwm1Regs.ETSEL.bit.SOCAEN = 1; 
    EPwm2Regs.ETSEL.bit.SOCBEN = 1;

    //Unfreeze ePWM to up count mode
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;

    EDIS;
}

/*
 * ADC1 ISR
 */

interrupt void adca1_isr(){


    //Write the sample to the global variable
    i1_sample = AdcaResultRegs.ADCRESULT0;      //Reads the result register of SOC0

    //Set the update flag
    i1_update = 1;  //Triggers the if statement in the main loop for Current Control

    //Clear the interrupt flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1=1;    //Clears ADCA interrupt 1

    //Acknowledge the interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;     //Acknowledges the interrupt in the PIE table for ADCA interrupt 1
}

/*
 * ADC2 ISR
 */

interrupt void adcb2_isr(){
    //Write the sample to the global variable
    x1_sample = AdcbResultRegs.ADCRESULT1;  //Reads the result register of SOC1

    //Set the update flag
    x1_update = 1;  //Triggers the if statement in the main loop for PID operations

    //Clear the interrupt flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2=1;    //Clears ADCB interrupt 2

    //Acknowledge the interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;    //Acknowledges the interrupt in the PIE table for ADCB interrupt 2
}

/*
 * ePWM1 ISR
 */
    //*Not enabled, doesn't do anything
    //*Kept in case we want to
interrupt void epwm1_isr(){
    //Clear the interrupt flag
    EPwm1Regs.ETCLR.bit.INT=1;

    //Acknowledge the interrupt
    PieCtrlRegs.PIEACK.all=PIEACK_GROUP3;
}

/*
 * ePWM2 ISR
 */
    //*Not enabled, doesn't do anything
    //*Kept in case we want to
interrupt void epwm2_isr(){
    //Clear the interrupt flag
    EPwm2Regs.ETCLR.bit.INT=1;

    //Acknowledge the interrupt
    PieCtrlRegs.PIEACK.all=PIEACK_GROUP3;
}

