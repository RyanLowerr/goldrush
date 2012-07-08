#include "controller.h"
#include <avr/io.h>

#define set_bit(reg,bit) reg |= (1<<bit)
#define clear_bit(reg,bit) reg &= ~(1<<bit)

void controller_param_init(){
	controller.paramaters[PARAM_TRAVEL_X]			      = 127;
	controller.paramaters[PARAM_TRAVEL_Y]			      = 127;
	controller.paramaters[PARAM_TRAVEL_Z]			      = 127;
	controller.paramaters[PARAM_GAIT_MODE]		      = 0;
	controller.paramaters[PARAM_TURRET_PAN]		      = 127;
	controller.paramaters[PARAM_TURRET_TILT]		    = 127;
	controller.paramaters[PARAM_TURRET_MODE]		    = 0;
	controller.paramaters[PARAM_GUN_STATUS]		      = 0;
	controller.paramaters[PARAM_GUN_MODE]			      = 0;
	controller.paramaters[PARAM_POSE]			          = 0;
	controller.paramaters[PARAM_INTERPOLATION_TIME] = 150;	
}

void controller_init(long baud){
	UBRR1H = ((F_CPU / 16 + baud / 2) / baud - 1) >> 8;
	UBRR1L = ((F_CPU / 16 + baud / 2) / baud - 1);
	set_bit(UCSR1B, TXEN1);
	set_bit(UCSR1B, RXEN1);
	set_bit(UCSR1B, RXCIE1);

	controller_param_init();
}

void controller_tx(unsigned char data){
	while (bit_is_clear(UCSR1A, UDRE1));
	UDR1 = data;
}

unsigned char controller_rx(){
	while (bit_is_clear(UCSR1A, RXC1));
	return UDR1;
}

void controller_buffer_write(char x){
	if ((controller.buffer_end + 1) % BUFFER_SIZE != controller.buffer_start){
		controller.buffer[controller.buffer_end] = x;
		controller.buffer_end = (controller.buffer_end + 1) % BUFFER_SIZE;
	}
}

int controller_buffer_read(){
	char temp = controller.buffer[controller.buffer_start];
	controller.buffer_start = (controller.buffer_start + 1) % BUFFER_SIZE;
	return temp;
}

int controller_buffer_data_avaliable(){
	return (BUFFER_SIZE + controller.buffer_start - controller.buffer_end) % BUFFER_SIZE;
}

void controller_buffer_flush(){
	controller.buffer_start = controller.buffer_end;
}

void controller_param_set(int id, int length, int params[]){
	int i = 0;
	while(i < length){
		controller.paramaters[params[i]] = params[i + 1];
		i += 2;
	}
}

void controller_param_get(int id, int length, int params[]){
	int i = 0;
	int checksum = 0;
	controller_tx(0xFF);
	controller_tx(0xFF);
	controller_tx(id);
	controller_tx(INST_READ);
	controller_tx(length);
	while(i < length){
		controller_tx(params[i]);
		controller_tx(controller.paramaters[params[i]]);
		checksum += params[i];
		checksum += controller.paramaters[params[i]];
	}
	controller_tx(checksum);
}

void controller_read(){

	int index = -1;
	int checksum = 0;
	int id = 0;
	int length = 0;
	int instruction = 0;

	while(controller_buffer_data_avaliable() > 0){

		// Searching for the first two 0xFF header bytes.
		if(index == -1){

			// First header byte.
			if(controller_buffer_read() == 0xff){

				// second header byte.
				if(controller_buffer_read() == 0xff){

					index = 0;
					checksum = 0;

					id = controller_buffer_read();
					checksum += id;

					instruction = controller_buffer_read();
					checksum += instruction;

					length = controller_buffer_read();
					checksum += length;

				}
			}
		}
		// Reading the packet following the header.
		else{

			if(index == length){

				if(255 - ((checksum) % 256) == controller_buffer_read()){

					if(instruction == INST_WRITE)
						controller_param_set(id, length, controller.temp_params);

					else if(instruction == INST_READ)
						controller_param_get(id, length, controller.temp_params);
					
					else if(instruction == INST_RESET)
						controller_param_init();

					index = -1;
				}
				else
					index = -1;
			}
			else{
				
				controller.temp_params[index] = (unsigned char) controller_buffer_read();
				
				if(controller.temp_params[index] != 0xff){
					checksum += (int) controller.temp_params[index];
					index++;
				}
				else
					index = -1;
			}
		}
	}

	controller_buffer_flush();
}

ISR(USART1_RX_vect){
	char received_char = UDR1;
	controller_buffer_write(received_char);
}
