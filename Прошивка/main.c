#define Led_green PORTD6
#define Led_red PORTD5
#define CS_mcp41010 PORTC1
#define CS_max31855 PORTC0
#define SPI_sck PORTB5
#define SPI_miso PORTB4
#define SPI_mosi PORTB3
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t data_mcp[2] = {0, 0};
volatile uint8_t data_max[4] = {0, 0, 0, 0};
volatile uint8_t avaliable_spi = 0, transmit_spi = 0, step_transmit = 0, counter_recive = 0;
volatile uint16_t counter_update_max = 0;
uint8_t mode = 1; // режим работы

void initialization()
{
	// Настройка порта B: PB0 - вход, PB1 - выход, (SCK, MOSI) - выход, MISO - вход
	DDRB |= (1 << PORTB1) | (1 << SPI_sck) | (1 << SPI_mosi);
	DDRB &= ~((1 << PORTB0) | (1 << SPI_miso));
	// Настройка порта D: PD7 - вход, PD6 - выход, PD5 - выход
	DDRD |= (1 << PORTD6) | (1 << PORTD5);
	DDRD &= ~(1 << PORTD7);
	// Настройка порта C: PC0 - выход, PC1 - выход
	DDRC |= (1 << CS_max31855) | (1 << CS_mcp41010);
	PORTB &= ~((1 << SPI_sck) | (1 << SPI_mosi));
	PORTC |= (1 << CS_mcp41010) | (1 << CS_max31855);
	// Настройка SPI: делитель на 128, режим master, прерывание включены
	SPCR |= (1 << SPIE) | (1 << SPE) | (1 << MSTR) | (1 << SPR0) | (1 << SPR1);
	SPCR &= ~((1 << CPOL) | (1 << CPHA) | (1 << DORD));
	// Настройка USART: асинхронный режим, 8 бит посылка, 1 стоп-бит, контроль четности отключен, скорость 9600 бод, прерывание по приему
	UCSR0A |= (1 << U2X0); //включаем ускоритель
	UBRR0 = 207;
	UCSR0B |= (1 << TXEN0);
	UCSR0C |= (1 << USBS0) | (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0C &= ~((1 << UMSEL00) | (1 << UMSEL01) | (1 << USBS0));
	// Настройка таймера 0: предделитель на 1024, прерывание по переполнению включен
	TCCR0B |= (1 << CS02) | (1 << CS00);
	TIMSK0 |= (1 << TOIE0);
	sei();
}

void spi_treansmit_mcp(uint8_t command, uint8_t data)//отправка по spi на mcp41010
{
	avaliable_spi = 1; //флаг занятости шины spi
	PORTC &= ~(1 << CS_mcp41010); //прижимаем CS к земле
	data_mcp[0] = command;
	data_mcp[1] = data;
	transmit_spi = 1; // сообщаем, что занимаемся отправкой
	SPDR = data_mcp[0]; // отправляем 1 пакет данных
}

void spi_reception_max31855()
{
	avaliable_spi = 1;
	PORTC &= ~(1 << CS_max31855);
	SPDR = 0xFF;
}

void USART_Transmit(uint8_t *data)
{
	while(*data != '\0')
	{
		while( !(UCSR0A & (1 << UDRE0))); //ожидаем опустошения буфера приема
		UDR0 = *data; //записываем данные
		data++;
	}
}

ISR(SPI_STC_vect)
{
	if (transmit_spi == 1) //проверяем отправляем ли мы данные
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
				avaliable_spi = 0; // SPI свободен
				transmit_spi = 0; // больше отправлять не будем
				break;
		}	
	}
	else
	{
		data_max[counter_recive] = SPDR;
		counter_recive++;
		if (counter_recive > 3)
		{
			avaliable_spi = 0;
			PORTC |= (1 << CS_max31855);
			counter_recive = 0;
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
	if (counter_update_max == 61)
	{
		if (avaliable_spi == 0)
		{
			counter_update_max = 0;
			spi_reception_max31855();
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
				PORTD |= (1 << Led_green);
				PORTD &= ~(1 << Led_red);
				// отключаем инвертор
				if (avaliable_spi == 0)
				{
					spi_treansmit_mcp(0b00010001,0);
					break;
				}
				break;
			case 1: //режим подготовки к измерению
				break;
			case 2: //режим измерения
				
				break;
		}
    }
}
