# SASB_Demonstrator_77D
Brief:
  This is a port of the SASB Demonstrator to the TI TMS320F28377D Control Card

Basic Operation:
  When the floater falls below the target point, a magnetic field is generated to pull it back up. If the Floater is above the target the magnetic field is turned off allowing the floater to fall.

Pinouts:
  Current Sensors
  	-> Pin 09 (ADCA CH0) For H-Bridge 1
  	-> Pin 11 (ADCA CH1) For H-Bridge 2
  	-> Pin 15 (ADCA CH2) For H-Bridge 3
  	-> Pin 17 (ADCA CH3) For H-Bridge 4
  Displacement Sensor
  	-> Pin 12 (ADCB CH0)
  PWM High
  	-> Pin 86 (GPIO 34) For H-Bridge 1
  	-> Pin 88 (GPIO 39) For H-Bridge 2
  	-> Pin 90 (GPIO 44) For H-Bridge 3
  	-> Pin 92 (GPIO 45) For H-Bridge 4
  PWM Low and Dir should be pluged into one of the 3.3V pins on the board to keep them at 	logic high. This garentees that when the PWM High pin is toggled the 

Reference Pins:
  ADCA Clock Signal -> Pin 49 (ePWM1 Compair A)
  ADCB Clock Signal -> Pin 53 (ePWM2 Compair A)
  Length of Displacement calculations -> Pin 89

Ussage:
  After the board is plugged in one should be able to debug the code. This testbed setup allows the user to test any one of the H-Bridges for functionality. The H-Bridges are only operated in two states. When PWM_H is ran logic low, both ends of the coil are shorted to ground. When PWM_H is ran logic high, one end of the coil is ran to the input voltage while the other remains at ground.

Considerations:
  I do not believe that the code is the best it can be but I can promise it is better than it was. The commenting on the other hand may need more work.
  
  
