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
uint8_t mode = 1; // ����� ������

void initialization()
{
	// ��������� ����� B: PB0 - ����, PB1 - �����, (SCK, MOSI) - �����, MISO - ����
	DDRB |= (1 << PORTB1) | (1 << SPI_sck) | (1 << SPI_mosi);
	DDRB &= ~((1 << PORTB0) | (1 << SPI_miso));
	// ��������� ����� D: PD7 - ����, PD6 - �����, PD5 - �����
	DDRD |= (1 << PORTD6) | (1 << PORTD5);
	DDRD &= ~(1 << PORTD7);
	// ��������� ����� C: PC0 - �����, PC1 - �����
	DDRC |= (1 << CS_max31855) | (1 << CS_mcp41010);
	PORTB &= ~((1 << SPI_sck) | (1 << SPI_mosi));
	PORTC |= (1 << CS_mcp41010) | (1 << CS_max31855);
	// ��������� SPI: �������� �� 128, ����� master, ���������� ��������
	SPCR |= (1 << SPIE) | (1 << SPE) | (1 << MSTR) | (1 << SPR0) | (1 << SPR1);
	SPCR &= ~((1 << CPOL) | (1 << CPHA) | (1 << DORD));
	// ��������� USART: ����������� �����, 8 ��� �������, 1 ����-���, �������� �������� ��������, �������� 9600 ���, ���������� �� ������
	UCSR0A |= (1 << U2X0); //�������� ����������
	UBRR0 = 207;
	UCSR0B |= (1 << TXEN0);
	UCSR0C |= (1 << USBS0) | (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0C &= ~((1 << UMSEL00) | (1 << UMSEL01) | (1 << USBS0));
	// ��������� ������� 0: ������������ �� 1024, ���������� �� ������������ �������
	TCCR0B |= (1 << CS02) | (1 << CS00);
	TIMSK0 |= (1 << TOIE0);
	sei();
}

void spi_treansmit_mcp(uint8_t command, uint8_t data)//�������� �� spi �� mcp41010
{
	avaliable_spi = 1; //���� ��������� ���� spi
	PORTC &= ~(1 << CS_mcp41010); //��������� CS � �����
	data_mcp[0] = command;
	data_mcp[1] = data;
	transmit_spi = 1; // ��������, ��� ���������� ���������
	SPDR = data_mcp[0]; // ���������� 1 ����� ������
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
		while( !(UCSR0A & (1 << UDRE0))); //������� ����������� ������ ������
		UDR0 = *data; //���������� ������
		data++;
	}
}

ISR(SPI_STC_vect)
{
	if (transmit_spi == 1) //��������� ���������� �� �� ������
	{
		switch(step_transmit)
		{
			case 0:
				SPDR = data_mcp[1]; // ���������� 2 ����� ������
				step_transmit = 1; // ��������� ��� � ��������
				break;
			case 1: //��������� ��������
				PORTC |= (1 << CS_mcp41010); // �������� CS �� �����
				step_transmit = 0;
				avaliable_spi = 0; // SPI ��������
				transmit_spi = 0; // ������ ���������� �� �����
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
			case 0: //����� �������
				PORTD |= (1 << Led_green);
				PORTD &= ~(1 << Led_red);
				// ��������� ��������
				if (avaliable_spi == 0)
				{
					spi_treansmit_mcp(0b00010001,0);
					break;
				}
				break;
			case 1: //����� ���������� � ���������
				break;
			case 2: //����� ���������
				
				break;
		}
    }
}
