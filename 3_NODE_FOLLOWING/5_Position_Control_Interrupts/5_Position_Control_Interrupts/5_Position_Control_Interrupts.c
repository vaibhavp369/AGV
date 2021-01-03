
#include "firebird_avr.h"				// Header file included that contains macro definitions essential for Firebird V robot
#include <util/delay.h>					// Standard AVR Delay Library
#include "lcd.h"
#include "agv_ctrl.h"

#define min_thresh  50 					//to accept angle in degrees for turning
unsigned char  Flag;
unsigned char node_detected;
int Node_count = -2;


void adc_init(){
	
	// enable ADC and pre-scalar = 64 (ADEN = 1, ADPS2 = 1, ADPS1 = 1, ADPS0 = 0)
	// and clear ADC start conversion bit, auto trigger enable bit, interrupt flag bit and interrupt enable bit
	ADCSRA_reg	|= ( (1 << ADEN_bit) | (1 << ADPS2_bit) | (1 << ADPS1_bit) );
	ADCSRA_reg	&= ~( (1 << ADSC_bit) | (1 << ADATE_bit) | (1 << ADIF_bit) | (1 << ADIE_bit) | (1 << ADPS0_bit) );
	
	// In ADCSRB, disable Analog Comparator Multiplexer, MUX5 bit and ADC Auto Trigger Source bits
	ADCSRB_reg	&= ~( (1 << ACME_bit) | (1 << MUX5_bit) | (1 << ADTS2_bit) | (1 << ADTS1_bit) | (1 << ADTS0_bit) );
	
	// In ADMUX, set the Reference Selection bits to use the AVCC as reference, and disable the channel selection bits MUX[4:0]
	ADMUX_reg	&= ~( (1 << REFS1_bit) | (1 << MUX4_bit) | (1 << MUX3_bit) | (1 << MUX2_bit) | (1 << MUX1_bit) | (1 << MUX0_bit) );
	ADMUX_reg	|= (1 << REFS0_bit);
	
	// In ADMUX, enable the ADLAR bit for 8-bit ADC result
	ADMUX_reg	|= (1 << ADLAR_bit);
	
	// In ACSR, disable the Analog Comparator by writing 1 to ACD_bit
	ACSR_reg	|= ( 1 << ACD_bit );
}




unsigned char ADC_Conversion(unsigned char channel_num)
{
	unsigned char adc_8bit_data;
	
	// MUX[5:0] bits to select the ADC channel number
	if ( channel_num > 7 )
	{
		ADCSRB_reg |= ( 1 << MUX5_bit );
	}
	channel_num	= channel_num & 0x07;
	ADMUX_reg	= ( ADMUX_reg | channel_num );
	
	// set the ADSC bit in ADCSRA register
	ADCSRA_reg		|= ( 1 << ADSC_bit );
	
	//Wait for ADC conversion to complete
	while( ( ADCSRA_reg & ( 1 << ADIF_bit ) ) == 0x00 );
	
	adc_8bit_data = ADCH_reg;
	
	// clear ADIF bit by writing 1 to it
	ADCSRA_reg		|= ( 1 << ADIF_bit );
	
	// clear the MUX5 bit
	ADCSRB_reg		&= ~( 1 << MUX5_bit );
	
	// clear the MUX[4:0] bits
	ADMUX_reg		&= ~( (1 << MUX4_bit) | (1 << MUX3_bit) | (1 << MUX2_bit) | (1 << MUX1_bit) | (1 << MUX0_bit) );
	
	return adc_8bit_data;
}



void adc_port_config (void)
{
	adc_sensor_low_ddr_reg		= 0x00;				// set PORTF direction as input
	adc_sensor_low_port_reg		= 0x00;				// set PORTF pins floating
	adc_sensor_high_ddr_reg		= 0x00;				// set PORTK direction as input
	adc_sensor_high_port_reg	= 0x00;				// set PORTK pins floating
}




//----------------------------- INTERRUPT SERVICE ROUTINES ----------------------------------------------

/**
 * @brief      ISR for right position encoder
 */






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
