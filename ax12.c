#include "ax12.h"
#include <avr/io.h>

#define set_bit(reg,bit) reg |= (1<<bit)
#define clear_bit(reg,bit) reg &= ~(1<<bit)

void ax12_init(long baud)
{
	UBRR0H = ((F_CPU / 16 + baud / 2) / baud - 1) >> 8;
	UBRR0L = ((F_CPU / 16 + baud / 2) / baud - 1);
}

void ax12_tx(unsigned char data)
{
	while (bit_is_clear(UCSR0A, UDRE0));
	UDR0 = data;
}

unsigned char ax12_rx()
{
	while (bit_is_clear(UCSR0A, RXC0));
	return UDR0;
}

void ax12_set_tx()
{
	clear_bit(UCSR0B, RXEN0);
	set_bit(UCSR0B, TXEN0);
}

void ax12_set_rx()
{
	clear_bit(UCSR0B, TXEN0);
	set_bit(UCSR0B, RXEN0);
}

void ax12_write(int id, int reg, int data)
{
	int checksum = ~((id + 4 + AX_INST_WRITE_DATA + reg + (data & 0xFF)) % 256);
	ax12_set_tx();
	ax12_tx(0xFF);
	ax12_tx(0xFF);
	ax12_tx(id);
	ax12_tx(4);
	ax12_tx(AX_INST_WRITE_DATA);
	ax12_tx(reg);
	ax12_tx(data & 0xFF);
	ax12_tx(checksum);
	ax12_set_rx();
}

void ax12_write2(int id, int reg, int data)
{
	int checksum = ~((id + 5 + AX_INST_WRITE_DATA + reg + (data & 0xFF) + ((data & 0xFF00) >> 8)) % 256);
	ax12_set_tx();
	ax12_tx(0xFF);
	ax12_tx(0xFF);
	ax12_tx(id);
	ax12_tx(5);
	ax12_tx(AX_INST_WRITE_DATA);
	ax12_tx(reg);
	ax12_tx(data & 0xFF);
	ax12_tx((data & 0xFF00) >> 8);
	ax12_tx(checksum);
	ax12_set_rx();
}

int ax12_read(int id, int reg, int length)
{
	int checksum = ~((id + 4 + AX_INST_READ_DATA + reg + length) % 256);
	ax12_set_tx();
	ax12_tx(0xFF);
	ax12_tx(0xFF);
	ax12_tx(id);
	ax12_tx(4);
	ax12_tx(AX_INST_READ_DATA);
	ax12_tx(reg);
	ax12_tx(length);
	ax12_tx(checksum);
	ax12_set_rx();
	/* TODO : Handle the return status packet. */
	return(0);
}

void ax12_reg_write(int id, int reg, int data)
{
	int checksum = ~((id + 4 + AX_INST_REG_WRITE + reg + (data & 0xFF)) % 256);
	ax12_set_tx();
	ax12_tx(0xFF);
	ax12_tx(0xFF);
	ax12_tx(id);
	ax12_tx(4);
	ax12_tx(AX_INST_REG_WRITE);
	ax12_tx(reg);
	ax12_tx(data & 0xFF);
	ax12_tx(checksum);
	ax12_set_rx();
}

void ax12_reg_write2(int id, int reg, int data)
{
	int checksum = ~((id + 4 + AX_INST_REG_WRITE + reg + (data & 0xFF) + ((data & 0xFF00) >> 8)) % 256);
	ax12_set_tx();
	ax12_tx(0xFF);
	ax12_tx(0xFF);
	ax12_tx(id);
	ax12_tx(5);
	ax12_tx(AX_INST_REG_WRITE);
	ax12_tx(reg);
	ax12_tx(data & 0xFF);
	ax12_tx((data & 0xFF00) >> 8);
	ax12_tx(checksum);
	ax12_set_rx();
}

void ax12_action(int id)
{
	int checksum = ~((id + 2 + AX_INST_ACTION) % 256);
	ax12_set_tx();
	ax12_tx(0xFF);
	ax12_tx(0xFF);
	ax12_tx(id);
	ax12_tx(2);
	ax12_tx(AX_INST_ACTION);
	ax12_tx(checksum);
	ax12_set_rx();
}

void ax12_ping(int id)
{
	int checksum = ~((id + 2 + AX_INST_PING) % 256);
	ax12_set_tx();
	ax12_tx(0xFF);
	ax12_tx(0xFF);
	ax12_tx(id);
	ax12_tx(2);
	ax12_tx(AX_INST_PING);
	ax12_tx(checksum);
	ax12_set_rx();
	/* TODO : Handle the return status packet. */
}

void ax12_reset(int id)
{
	int checksum = ~((id + 2 + AX_INST_RESET) % 256);
	ax12_set_tx();
	ax12_tx(0xFF);
	ax12_tx(0xFF);
	ax12_tx(id);
	ax12_tx(2);
	ax12_tx(AX_INST_RESET);
	ax12_tx(checksum);
	ax12_set_rx();
}

void ax12_sync_write(int reg, int L, int n, int value_list[])
{
	// Control many Dynamixel actuators at the same time. Limited to writing to a single
	// one or two byte register. 

	// reg = Starting address of the loctation where data is to be written
	//   L = Length of data to be written.
	//   n = Number of Dynamixel actuators.
  
  int index = n - 1;
	int length = (L + 1) * n + 4;
	int checksum = AX_BROADCAST_ID + length + AX_INST_SYNC_WRITE + reg + L;

	ax12_set_tx();
	ax12_tx(0xFF);
	ax12_tx(0xFF);
	ax12_tx(AX_BROADCAST_ID);
	ax12_tx(length);
	ax12_tx(AX_INST_SYNC_WRITE);
	ax12_tx(reg);
	ax12_tx(L);
	if(L > 1)
	{
		while(index >= 0)
		{
			ax12_tx(index + 1);
			ax12_tx(value_list[index] & 0xFF);
			ax12_tx((value_list[index] & 0xFF00) >> 8);
			checksum += (index + 1) + (value_list[index] & 0xFF) + ((value_list[index] & 0xFF00) >> 8);
			index--;
		}
	}
	else
	{
		while(index >= 0)
		{
			ax12_tx(n + 1);
			ax12_tx(value_list[n]);
			checksum += (n + 1) + value_list[n];
			index--;
		}
	}
	ax12_tx(~checksum % 256);
	ax12_set_rx();
}
