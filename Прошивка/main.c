#define Led_green PORTD6
#define Led_red PORTD5
#define CS_mcp41010 PORTC1
#define CS_max31855 PORTC0
#define SPI_sck PORTB5
#define SPI_miso PORTB4
#define SPI_mosi PORTB3
#define avaliable_spi 0
#define avaliable_usart 1
#define transmit_spi 2
#define transmit_usart 3
#define connect 4
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t data_mcp[2] = {0, 0};
volatile uint8_t data_max[4] = {0, 0, 0, 0};
volatile uint8_t data_recive[3] = {0, 0, 0};
volatile uint8_t data_transmit[3] = {0, 0, 0};
volatile uint8_t flags_avaliable = 0;
volatile uint8_t step_transmit = 0, counter_recive = 0, counter_usart = 0, counter_update_max = 0, counter_update_led = 0;
volatile uint8_t led_state = 0; // режим мигания светодиодами
volatile uint8_t mode = 0; // режим работы

void initialization()
{
	// Настройка порта B: PB0 - вход, PB1 - выход, (SCK, MOSI) - выход, MISO - вход
	DDRB |= (1 << PORTB1) | (1 << SPI_sck) | (1 << SPI_mosi);
	DDRB &= ~((1 << PORTB0) | (1 << SPI_miso));
	// Настройка порта D: PD7, PD0 - вход, PD6, PD5, PD1 - выход
	DDRD |= (1 << PORTD6) | (1 << PORTD5) | (1 << PORTD1);
	DDRD &= ~((1 << PORTD7) | (1 << PORTD0));
	// Настройка порта C: PC0 - выход, PC1 - выход
	DDRC |= (1 << CS_max31855) | (1 << CS_mcp41010);
	PORTB &= ~((1 << SPI_sck) | (1 << SPI_mosi));
	PORTC |= (1 << CS_mcp41010) | (1 << CS_max31855);
	// Настройка SPI: делитель на 128, режим master, прерывание включены
	SPCR |= (1 << SPIE) | (1 << SPE) | (1 << MSTR) | (1 << SPR0) | (1 << SPR1);
	SPCR &= ~((1 << CPOL) | (1 << CPHA) | (1 << DORD));
	// Настройка USART: асинхронный режим, 8 бит посылка, 1 стоп-бит, контроль четности отключен, скорость 9600 бод, прерывание по приему
	UCSR0A |= (1 << U2X0); //включаем ускоритель
	UBRR0 = 103;
	UCSR0B |= (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0) | (1 << TXCIE0);
	UCSR0C |= (1 << USBS0) | (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0C &= ~((1 << UMSEL00) | (1 << UMSEL01) | (1 << USBS0));
	// Настройка таймера 0: предделитель на 1024, прерывание по переполнению включен
	TCCR0B |= (1 << CS02) | (1 << CS00);
	TIMSK0 |= (1 << TOIE0);
	// Настройка таймера 2: предделитель на 1024, прерывание по переполнению включен
	TCCR2B |= (1 << CS00) | (1 << CS01) | (1 << CS02);
	TIMSK2 |= (1 << TOIE2);
	sei();
}

void spi_transmit_mcp(uint8_t command, uint8_t data)//отправка по spi на mcp41010
{
	flags_avaliable |= (1 << avaliable_spi);//флаг занятости шины spi
	data_mcp[0] = command;
	data_mcp[1] = data;
	PORTC &= ~(1 << CS_mcp41010); //прижимаем CS к земле
	flags_avaliable |= (1 << transmit_spi); // сообщаем, что занимаемся отправкой
	SPDR = data_mcp[0]; // отправляем 1 пакет данных
}

void spi_reception_max31855()
{
	flags_avaliable |= (1 << avaliable_spi);
	PORTC &= ~(1 << CS_max31855);
	SPDR = 0xFF;
}

void USART_Transmit(uint8_t command, uint16_t data) // передача команды и данных по uart
{
	//делим на пакеты
	data_transmit[0] = command;
	data_transmit[1] = data >> 8;
	data_transmit[2] = data & 0xFF;
	flags_avaliable |= (1 << avaliable_usart); // говорим что занят uart
	counter_usart = 0;
	UDR0 = data_transmit[0]; //записываем данные
}

volatile uint16_t temperature_avaliable()
{
	uint16_t temp = (data_max[0] << 4) | (data_max[1] >>4);
	return temp;
}

ISR(SPI_STC_vect)
{
	if (flags_avaliable & (1 << transmit_spi)) //проверяем отправляем ли мы данные
	{
		switch(step_transmit)
		{
			case 0:
				SPDR = data_mcp[1]; // отправляем 2 пакет данных
				step_transmit = 1; // следующий шаг в отправке
				break;
			case 1: //завершаем отправку
				PORTC |= (1 << CS_mcp41010); // отжимаем CS от земли
				step_transmit = 0;
				flags_avaliable &= ~(1 << avaliable_spi); // SPI свободен
				flags_avaliable &= ~(1 << transmit_spi); // больше отправлять не будем
				break;
		}	
	}
	else
	{
		data_max[step_transmit] = SPDR;
		step_transmit++;
		if (step_transmit > 3)
		{
			flags_avaliable &= (1 << avaliable_spi);
			PORTC |= (1 << CS_max31855);
			step_transmit = 0;
			if ((data_max[1] & 0b00000001) == 0b00000001)
			{
				mode = 3;
				USART_Transmit(12,0);
			}
		}
		else
		{
			SPDR = 0xFF;
		}
	}
}

ISR(TIMER0_OVF_vect)
{
	counter_update_max++;
	if (counter_update_max == 10)
	{
		counter_update_max = 0;
		if (!(flags_avaliable & (1 << transmit_spi)))
		{
			spi_reception_max31855();
		}
	}
}

ISR(TIMER2_OVF_vect)
{
	counter_update_led++;
	if (counter_update_led == 61)//режимы мигания светодиодов
	{
		counter_update_led = 0;
		switch (led_state)
		{
			case 1:
			if (PORTD & (1 << Led_green))
			{
				PORTD &= ~(1 << Led_green);
				PORTD |= (1 << Led_red);
			}
			else
			{
				PORTD |= (1 << Led_green);
				PORTD &= ~(1 << Led_red);
			}
			break;
			case 2:
			PORTD &= ~(1 << Led_red);
			if (PORTD & (1 << Led_green))
			{
				PORTD &= ~(1 << Led_green);
			}
			else
			{
				PORTD |= (1 << Led_green);
			}
			break;
			case 3:
			PORTD &= ~(1 << Led_green);
			if (PORTD & (1 << Led_red))
			{
				PORTD &= ~(1 << Led_red);
			}
			else
			{
				PORTD |= (1 << Led_red);
			}
			break;
		}
	}
}

ISR(USART_TX_vect)
{
	counter_usart++;
	if (counter_usart > 2)//последняя посылка
	{
		counter_usart = 0;
		flags_avaliable &= ~(1 << avaliable_usart);
		flags_avaliable &= ~(1 << transmit_usart);
	}
	else
	{
		UDR0 = data_transmit[counter_usart];// посылаем следующий пакет
	}
}

ISR(USART_RX_vect)
{
	data_recive[counter_recive] = UDR0;
	counter_recive ++;
	if (counter_recive > 2) // если приняли все пакеты
	{
		counter_recive = 0;
		switch(data_recive[0]) //смотрим, что за команда пришла
		{
			case 1:
				mode = data_recive[2];
				break;
			case 2:
				if (mode == 3)
				{
					USART_Transmit(12, 0);
				} 
				else
				{
					USART_Transmit(0xD, temperature_avaliable());
				}
				break;
			case 3:
				break;
			case 4:
				break;
			case 5:// компьютер принял пакет
				break;
			case 6:
				if ((data_recive[1] == 195) && (data_recive[2] == 204)) // запрос от "своей" программы
				{
					flags_avaliable |= (1 << connect);
					USART_Transmit(0x10,0x3C33);
				}
				break;
		}
	}
}

int main(void)
{
	initialization();
    while (1) 
    {
		switch(mode)
		{
			case 0: //режим простоя
				led_state = 0;
				PORTD |= (1 << Led_green);
				PORTD &= ~(1 << Led_red);
				// отключаем инвертор
				if (!(flags_avaliable & (1 << avaliable_spi)))
				{
					spi_transmit_mcp(0b00010001,0);
				}
				while (mode == 0) {}
				break;
			case 1: //режим подготовки к измерению
				led_state = 1;
				while (mode == 1){}
				break;
			case 2: //режим измерения
				led_state = 2;
				while (mode == 2){}
				break;
			case 3: //режим аварии
				led_state = 3;
				if (!(flags_avaliable & (1 << avaliable_spi)))
				{
					spi_transmit_mcp(0b00010001,0);
				}
				while(mode == 3){}
				break;
		}
    }
}
