
#include "firebird_avr.h"				// Header file included that contains macro definitions essential for Firebird V robot
#include <util/delay.h>					// Standard AVR Delay Library
#include "lcd.h"
#include "agv_ctrl.h"

#define min_thresh  50 					//to accept angle in degrees for turning
unsigned char  Flag;
unsigned char node_detected;
int Node_count = -2;

char path[] = "31212010110100";

char current_position_x = '3';
char current_position_y = '0';
char current_orientation = 'N';
char next_orientation;
char next_move;

int size = sizeof(path)/sizeof(path[0]);



char *next_position_x = &path[0];
char *next_position_y = &path[1];


char getNextOrientation(char* current_position_x,char* current_position_y,char** next_position_x,char** next_position_y)
{
	if( **next_position_x == '\0' || **next_position_y == '\0' )
	{
		return 'x';
	}
	if(*current_position_x == **next_position_x && *current_position_y <= **next_position_y){
		*current_position_x = **next_position_x;
		*current_position_y = **next_position_y;

		*next_position_x = *next_position_x + 2;
		*next_position_y = *next_position_y + 2;

		return 'N';
	}
	else if(*current_position_x <= **next_position_x && *current_position_y == **next_position_y){
		*current_position_x = **next_position_x;
		*current_position_y = **next_position_y;

		*next_position_x = *next_position_x + 2;
		*next_position_y = *next_position_y + 2;

		return 'E';
	}
	else if(*current_position_x == **next_position_x && *current_position_y >= **next_position_y){
		*current_position_x = **next_position_x;
		*current_position_y = **next_position_y;

		*next_position_x = *next_position_x + 2;
		*next_position_y = *next_position_y + 2;

		return 'S';
	}
	else if(*current_position_x >= **next_position_x && *current_position_y == **next_position_y){
		*current_position_x = **next_position_x;
		*current_position_y = **next_position_y;

		*next_position_x = *next_position_x + 2;
		*next_position_y = *next_position_y + 2;

		return 'W';
	}
}

char getNextMove(char* current_orientation,char next_orientation)
{
	if((*current_orientation == 'N' && next_orientation == 'E') || (*current_orientation == 'E' && next_orientation == 'S') ||
	(*current_orientation=='S' && next_orientation == 'W') || (*current_orientation == 'W' && next_orientation == 'N')){
		*current_orientation = next_orientation;
		return 'R';
	}
	else if((*current_orientation == 'N' && next_orientation == 'W') || (*current_orientation == 'W' && next_orientation == 'S') ||
	(*current_orientation=='S' && next_orientation == 'E') || (*current_orientation == 'E' && next_orientation == 'N')){
		*current_orientation = next_orientation;
		return 'L';
	}
	else if((*current_orientation == 'N' && next_orientation == 'N') || (*current_orientation == 'E' && next_orientation == 'E') ||
	(*current_orientation=='S' && next_orientation == 'S') || (*current_orientation == 'W' && next_orientation == 'W')){
		*current_orientation = next_orientation;
		return 'F';
	}
	else
	return 'S';
}



char path_follow(unsigned char L_sen, unsigned char C_sen, unsigned char R_sen)
{
	Flag = 0;
	node_detected = 0;
	if ( (L_sen > min_thresh) && (C_sen > min_thresh) && (R_sen > min_thresh) )		// Node Detected									
	{
		velocity(0,0);
		Flag = 1;
		node_detected = 1;
	}
	
	if( (L_sen >= min_thresh) && (node_detected == 0) && ( Flag == 0 ) )		// Left
	{
		forward();
		velocity(80,180);
		Flag = 1;
	}

	if( (R_sen >= min_thresh) && (node_detected == 0) && ( Flag == 0 ) )       // right
	{
		forward();
		velocity(180,80);																									
		Flag = 1;
	}
	

	if ( (C_sen >= min_thresh) && (node_detected == 0) && ( Flag == 0 ) )		// Forward
	{
		forward();
		velocity(200,212);																									
		Flag =1;
	}
	
	if (node_detected)
	{	
		
		
		lcd_string(1,1,"CP:");
		lcd_wr_char(1,4,current_position_x);
		lcd_wr_char(1,5,',');
		lcd_wr_char(1,6,current_position_y);
		
		
		lcd_string(1,8,"NP:");
		lcd_wr_char(1,11,*next_position_x);
		lcd_wr_char(1,12,',');
		lcd_wr_char(1,13,*next_position_y);
				
		
		next_orientation = getNextOrientation(&current_position_x,&current_position_y,&next_position_x,&next_position_y);
		
		lcd_string(2,1,"CO:");
		lcd_wr_char(2,4,next_orientation);
		/*
		lcd_string(2,8,"NO:");
		lcd_wr_char(2,11,next_orientation);
		*/
		
		
		if (next_orientation == 'x')
		  return 'x';
		next_move = getNextMove(&current_orientation,next_orientation);
		current_orientation = next_orientation;
		_delay_ms(500);

			switch(next_move)
			 {
				 
					case 'F':
							velocity(200,212);
							forward_mm(85);
							break;
											 
					case 'R':
						forward_mm(65);
						right_degrees(90);
					break;
					
					
					case 'L':
						forward_mm(65);
						left_degrees(90);
					break;
					
				   case 'S':
						lcd_wr_char(1,6,'G');		
			 }
		 
	}
	

}



int main(void)
{
	unsigned char R_sense,C_sense,L_sense;
	char stop_flag;
	R_sense = 0;
	C_sense = 0;
	L_sense = 0;
	lcd_port_config();
	lcd_init();
	motors_pin_config();
	pwm_pin_config();
	position_encoder_pin_config();
	position_encoder_interrupt_config();
	adc_port_config();					
	adc_init();							
	timer5_init();
	velocity(200,200);
	buzzer_init();
	while(1)
	{
		L_sense = ADC_Conversion(left_wl_sensor_channel);
		C_sense = ADC_Conversion(center_wl_sensor_channel);
		R_sense = ADC_Conversion(right_wl_sensor_channel);
		stop_flag = path_follow(L_sense, C_sense, R_sense);
		if(stop_flag == 'x')
		  break;
	}
}
//---------------------------------- END ------------------------------------------------------------------
