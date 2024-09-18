/*  2DX3 Final Project uC code
                Code written to utilize I2C by grabbing distance data from VL53L1X using it's API
								and then communicate it to the PC using UART
                
                The VL53L1X is run with default firmware settings.

            Written by Hunter Boyd

*/
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"





#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait1ms(100);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

void PortL_Init(void){ //PL0 for button to start measurements, PL1 to untangle wires
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;				// activate clock for Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};	// allow time for clock to stabilize
	GPIO_PORTL_DIR_R &= ~0x03;        								// configure Port L pins (PL0-PL1) as input
  GPIO_PORTL_AFSEL_R &= ~0x03;     								// disable alt funct on Port L pins (PL0-PL1)
  GPIO_PORTL_DEN_R |= 0x03;        								// enable digital I/O on Port L pins (PL0-PL1)
																									// configure Port M as GPIO
  GPIO_PORTL_AMSEL_R &= ~0x03;     								// disable analog functionality on Port M	pins (PM0-PM3)	
	return;
}

void PortH_Init(void){ //PH0-PH3 as output pins to drive stepper motorr
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;	// activate clock for Port H
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){}	// allow time for clock to stabilize
    GPIO_PORTH_DIR_R |= 0x0F;	// configure Port H pins (PL0-PL1) as output
    GPIO_PORTH_AFSEL_R &= ~0x0F;	// disable alt funct on Port H pins
    GPIO_PORTH_DEN_R |= 0x0F;	// enable digital I/O on Port H pins 
    GPIO_PORTH_AMSEL_R &= ~0x0F;	// disable analog functionality on Port H	pins 
    return;
}


void rotate_motor(int steps){	//rotates motor clockwise																		
	int delay = 5;										
	for(int i=0; i < steps; i++){			// total steps performed in this function = steps*4								
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait1ms(delay);					//delay of 5ms per step			
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait1ms(delay);
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait1ms(delay);
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait1ms(delay);
	}
}

void rotate_ccw(int steps){	//rotates motor counter clockwise
	uint32_t delay = 5;															
	
	for(int i=0; i < steps; i++){			// total steps performed in this function = steps*4										
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait1ms(delay);				  //delay of 5ms per step							
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait1ms(delay);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait1ms(delay);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait1ms(delay);
	}
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
	uint8_t ID;
	uint8_t Model;
	int input = 0;
	int start_stop = 0;
	int angle = 0;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortL_Init();
	PortH_Init();

	status = VL53L1X_GetSensorId(dev, &wordData);

	// Wait for ToF device to boot
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait1ms(100);
  }
	FlashAllLEDs(); // LEDS flash to signal ToF sensor has booted up
	
	status = VL53L1X_ClearInterrupt(dev); // clear interrupt has to be called to enable next interrupt
	
  // Initialize the sensor with the default setting
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	while(1){ // Microcontroller and PC won't run their programs until they are synched
		input = UART_InChar();
		if (input == 's')
			break;
		}

    status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging
		
	// main program loop
	while(1){
		if(GPIO_PORTL_DATA_R & 0x01){ //toggle start/stop with the button at PL0
			start_stop ^= 1; //toggled
			angle = 0; 			//reference angle at 0	
			while(GPIO_PORTL_DATA_R & 0x01){ //debounce period
				SysTick_Wait1ms(100);
			}
		}
		
		if(start_stop){
			rotate_motor(1);
			GPIO_PORTF_DATA_R |= 0x10; // LED 3 turned on to signal motor rotation is taking place			
			angle++;
		}
		
		if((start_stop == 0) && (GPIO_PORTL_DATA_R & 0x02)){ // a full scan and a stopped motor, as well as a push of thhe PL1 button to untangle wires
			rotate_ccw(512);
			angle = 0;
			while(GPIO_PORTL_DATA_R & 0x02){ //debounce period
				SysTick_Wait1ms(100);
			}
		}
		
		if(angle % 8 == 0 && start_stop){ //every 5.125 degrees of rotation, a distance measurement is taken
			SysTick_Wait1ms(250); 					// 250 ms delay for added stability of the scan
			while (dataReady == 0){
					status = VL53L1X_CheckForDataReady(dev, &dataReady);
          VL53L1_WaitMs(dev, 5);
			}
			FlashLED1(1); // flash LED 1 to signal a distance measurement has been taken
			
			dataReady = 0;
			
			status = VL53L1X_GetDistance(dev, &Distance);						//The Measured Distance value
		
			status = VL53L1X_ClearInterrupt(dev); /* 8 clear interrupt has to be called to enable next interrupt*/
			

			sprintf(printf_buffer,"%u\r\n", Distance);
			UART_printf(printf_buffer);	//Distance measurement sent to PC via UART
		}
		
		if(angle % 512 == 0 && start_stop){	//Once the motor has spun 360 degrees, stop motor.
			start_stop ^= 1; 
			GPIO_PORTF_DATA_R &= 0xEF; // Turn off LED 3 to signal the motor isn't spinning anymore
		}
	}
	
VL53L1X_StopRanging(dev);
}

