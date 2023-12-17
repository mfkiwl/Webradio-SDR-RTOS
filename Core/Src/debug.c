#include "debug.h"

uint8_t tx_buffer[TX_BUFF_SIZE];
uint16_t buff_head, buff_tail;

static void v_uart_puts(const char *s);
static void v_uart_putc(uint8_t c);


void debug_out(const char *s, ...)
{
#ifdef DEBUG
	unsigned int i, move;
	char c, str[16], *ptr;
	va_list ap;

	va_start(ap, s);

	for(;;)
	{
		c = *s++;

		if(c == 0)
		{
			break;
		}
		else if(c == '%')
		{
			c = *s++;
			if(isdigit(c) > 0)
			{
				move = c-'0';
				c = *s++;
			}
			else
			{
				move = 0;
			}

			switch(c)
			{
			case 's':
				ptr = va_arg(ap, char *);
				v_uart_puts(ptr);
				break;
			case 'b': //bin
				ltoa(va_arg(ap, long), str, 2);
				if(move)
				{
					for(i=0; str[i]; i++);
					for(; move>i; move--)
					{
						v_uart_putc('0');
					}
				}
				v_uart_puts(str);
				break;
			case 'i': //dec
				ltoa(va_arg(ap, long), str, 10);
				if(move)
				{
					for(i=0; str[i]; i++);
					for(; move>i; move--)
					{
						v_uart_putc('0');
					}
				}
				v_uart_puts(str);
				break;
			case 'u': //unsigned dec
				ultoa(va_arg(ap, unsigned long), str, 10);
				if(move)
				{
					for(i=0; str[i]; i++);
					for(; move>i; move--)
					{
						v_uart_putc('0');
					}
				}
				v_uart_puts(str);
				break;
			case 'x': //hex
				ltoa(va_arg(ap, long), str, 16);
				if(move)
				{
					for(i=0; str[i]; i++);
					for(; move>i; move--)
					{
						v_uart_putc('0');
					}
				}
				v_uart_puts(str);
				break;
			}
		}
		else
		{
			v_uart_putc(c);
		}
	}
	va_end(ap);
#endif
	return;
}

static void v_uart_puts(const char *s)
{
	//uint16_t len;
	while(*s)
	{
		v_uart_putc(*s++);
	}


	/*while(*s)
	{
		if(buff_tail < TX_BUFF_SIZE-1)
		{
			tx_buffer[buff_tail] = *s++;
			buff_tail++;
		}
		else
		{
			buff_tail = 0;
			tx_buffer[buff_tail] = *s++;
			buff_tail++;
		}
	}
	if(buff_head > buff_tail)
	{
		len = TX_BUFF_SIZE - buff_head + buff_tail + 1;
	}
	else
	{
		len = buff_tail - buff_head + 1;
	}

	HAL_UART_Transmit_DMA(uart_ptr, &tx_buffer[buff_head], len);
	buff_head = buff_tail;*/
	return;
}

static void v_uart_putc(uint8_t c)
{
	HAL_StatusTypeDef Status;
	Status = HAL_UART_Transmit(&huart4, &c, 1, HAL_MAX_DELAY);
	//Status = HAL_UART_Transmit_DMA(uart_ptr, &c, 1);

	if(Status != HAL_OK)
		DEBUGOUT("debug v_uart_putc error...\r\n");

	return;
}
