
#include "firebird_avr.h"				// Header file included that contains macro definitions essential for Firebird V robot
#include <util/delay.h>					// Standard AVR Delay Library
#include "lcd.h"
#include "agv_ctrl.h"

#define min_thresh  50 					//to accept angle in degrees for turning
unsigned char  Flag;
unsigned char node_detected;
int Node_count = -2;



void path_follow(unsigned char L_sen, unsigned char C_sen, unsigned char R_sen)
{
	Flag = 0;
	
	if ( (L_sen > min_thresh) && (C_sen > min_thresh) && (R_sen > min_thresh) )													//Node
	{
		velocity(0,0);
		Flag = 1;
		node_detected = 1;
	}
	
	if( (L_sen >= min_thresh) && (node_detected == 0) && ( Flag == 0 ) )
	{
		forward();
		velocity(80,180);
		Flag = 1;
	}

	if( (R_sen >= min_thresh) && (node_detected == 0) && ( Flag == 0 ) )
	{
		forward();
		velocity(180,80);																									// increase Left motor speed
		Flag = 1;
		//lcd_wr_char(2,3,'R');
	}
	

	if ( (C_sen >= min_thresh) && (node_detected == 0) && ( Flag == 0 ) )		// Forward
	{
		forward();
		velocity(200,212);																									// keep same speed
		Flag =1;
		//lcd_wr_char(2,3,'F');
	}
	
	if (node_detected)
	{
		//lcd_wr_char(2,1,'1');
		node_detected = 0;
		buzzer_on();
		_delay_ms(1000);
		buzzer_off();
		bar_graph_led_port_reg = 0x80;
		Node_count +=1;
				if (Node_count < 8)	
				 {	
					 lcd_string(1,1,"Node Count:");
					 lcd_numeric_value(2,1,(Node_count+1),2);
					 switch(Node_count)
					{
						case 0:
						forward_mm(65);
						right_degrees(90);
						break;
						
						case 3: 
						forward_mm(65);
						left_degrees(90);
						break;
						
						case 4: 
						forward_mm(65);
						left_degrees(90);
						break;
						
						case 7: 
						forward_mm(65);
						left_degrees(90);
						break;
						
						default:
						forward_mm(65);
						break;
					}
				 }
	}
	else
	{
		//lcd_wr_char(2,1,'0');
		bar_graph_led_port_reg = 0x00;
	}
	
	
}



int main(void)
{
	unsigned char R_sense,C_sense,L_sense;
	R_sense = 0;
	C_sense = 0;
	L_sense = 0;
	lcd_port_config();
	lcd_init();
	motors_pin_config();
	pwm_pin_config();
	position_encoder_pin_config();
	position_encoder_interrupt_config();
	adc_port_config();					// Initialize the ADC port
	adc_init();							// Initialize the ADC
	timer5_init();
	velocity(200,200);
	buzzer_init();
	while(1)
	{
		L_sense = ADC_Conversion(left_wl_sensor_channel);
		C_sense = ADC_Conversion(center_wl_sensor_channel);
		R_sense = ADC_Conversion(right_wl_sensor_channel);
		path_follow(L_sense, C_sense, R_sense);
	}
}
//---------------------------------- END ------------------------------------------------------------------
