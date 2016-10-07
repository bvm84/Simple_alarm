/*
 * Simple Alarm
 * Ver 1.0
 * Created: 26.08.2016 12:37:18
 * Author : Brendel Vadim
 * e-mail: brendel.vadim@gmail.com
 
  ���� ����������:
  
  ���������� ������������ ����� ������� ������������. ������� ���������� - �����, �������� � �.�.
  ����������� 2 ������ (������������ ������� - ���������):
  1. ���������� � ������ � ������ ����� ������� � ������� ����� ������ �
  �������� ������� (�� �������� � ������� �����, ����-�����). ��� �������� � ������� ��������� (���� �� 
  ����� ���� ���������� ���������� ��� ������� ������), ������������ ���� �������� � ��������.
  �������� ����������� �������� �������, ���������, ����� ����� ���������, ����� ������ ����� ������ (� �������������). 
  2. ���������� � ������ � ������ ����� ������� � ������� ������ ���� � ����������. ��� �������� �������������,
  ��� ������� ������� * �� ����� ������ �������. ������� ��������� ����� ���� - ������� #.
  ����� ��������:
  ������������ �������� ������������ �������� ��� ������������ �������� (��� ����� ������ ������ � ������������)
  ������������ �������� ������������ ��� ���������� �� ������������ (����� ������, ��������� �����).
  ������������ ��������� ��������, ������������ ����� ��� ������������ ��������.
  ������������ ��������� �������� � ���� ��� ������������ ��������.
  ������������ �������� ������ �� ���������. ������������ �����.
  
  ������� �������:
  1. ���������� ������ �� ����� (������������ - ���������� ����)
  2. PIR ������ ��������  (������������ - ���������� ����).
  
  �������� ����������:
  1. ������ (������������� �����, ���� ����� - ����� ���� � ������� AC220� ��� DC12�).
  2. ����� (������������� ������, ���� 220� - ����� ���� � ������� AC220� ��� DC12�).
  3. ����������� - ��������� ������� Nokia, Siemens � ����. ������� ������ ������ ���������� ��������.
  
  ���������:
  1. ��������� "���������� �� ������" - �������. ����� - ���������� �� ������. ������ - ������ ������
   ���������� �� ������, �� ���� �������� ������������ �������� ������������ ��� ���������� �� ������������ 
  2. ��������� "����� � ������" - �������. ����� - ����� �� � ������. ������ - ������ ������ ������ � ������,
   �� ���� �������� ������������ �������� ������������ �������� ��� ������������ ��������
  3. ������������ ������������ - ��� ���������� �������.
   
   � ��������� ������ ���������� �������� ����������� �������� ���������� (AK-207-N-WWB ��� ������),
   ������� ��������� � ����� ������� � ������ ���� ��������� � �������. �������� 7������� ����� � 8�������.
   
   ���������� �����������:
   �����:
   PD0-PD7 - ����������. ����� � ��������� � �������. ���� �� ���������� �������, ��� ������ ������
   PC0 - ���� ������ ������ ������/�����
   P�1 - ������ ����� ������. ����� � ��������� � �������.
   PC2 - ������, ���� � ��������� � �������
   PC3 - PIR ������
   ��������� PC ��������������� ��� ����� ��� ������ �������
   
   ������
   PB0 - ����� �� ����� (��������� + ����+����� ����������)
   PB1 - ����� �� ����� (����������+ ����+����� ����������)
   PB2 - ����� �� ������ ������ �� ��������� ��������.
   PB3 - ��������� "���������� �� ������" - �������.
   PB4 - ��������� "����� � ������" - �������.
   ��������� PB ��������������� ��� ������ ������.
   
  ���� �������� �� ����������� ���������� �� ������� 8���.
  ������ �0- ��������� �������� �� ���������� ������ �����������, ������������ �����������.
   
  ����������1 (1����) ��������� ������ � ������������� ����� ������, ��������� �������� � ���������.
  ����������2 (10����) ������������� ������ T0 �������� ������ ������ � ��������� ���������
  ����������3 (100����) ������ ��������� ������ ������
  ����������4 (5����) �������� �� UART.  
 
 */ 

#include <avr/io.h>
#include <util/atomic.h>
#include <pt.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <pt-sem.h>


/*�������������*/
//������ �� ��������� ��� ������ � �����������
#define KBD_PASS {1, 2, 3, 4, #}
#define BUT_PASS 0b00000000
#define BUT_PASS_LEN 4
//���������������� ������-�������
//�����
#define REGIME_SEL PC0
#define REGIME_SEL_PORT PORTC
#define REGIME_SEL_DDR DDRC
#define REGIME_SEL_PIN PINC
#define PASS_BUT PC1 //���� �� ������ ����� ������
#define PASS_BUT_PORT PORTC
#define PASS_BUT_DDR DDRC
#define PASS_BUT_PIN PINC
#define KBD_PORT PORTD
#define KBD_DDR DDRD
#define KBD_PIN PIBD
#define GERKON PC2 //���� �� ���������� ������
#define GERKON_PORT PORTC
#define GERKON_DDR DDRC
#define GERKON_BUT_PIN PINC
#define PIR PC3 //���� �� PIR ������
#define PIR_PORT PORTC
#define PIR_DDR DDRC
#define PIR_BUT_PIN PINC

//������
#define LOUD PB0 //����� �� �����/�������� ������
#define LOUD_PORT PORTB
#define LOUD_PORT_DDR DDRB
#define LOUD_PORT_PIN PINB
#define LAMP PB1 //����� �� �����
#define LAMP_PORT PORTB
#define LAMP_PORT_DDR DDRB
#define LAMP_PORT_PIN PINB
#define MOBILE PB2 //����� �� ����� � ����������
#define MOBILE_PORT PORTB
#define MOBILE_PORT_DDR DDRB
#define MOBILE_PORT_PIN PINB
#define GREEN_LED PB3 //����� �� ��������� "���������� �� ������" - �������. 
#define GREEN_LED_PORT PORTB
#define GREEN_LED_PORT_DDR DDRB
#define GREEN_LED_PORT_PIN PINB
#define RED_LED PB4 // ����� �� ��������� "����� � ������" - �������. 
#define RED_LED_PORT PORTB
#define RED_LED_PORT_DDR DDRB
#define RED_LED_PORT_PIN PINB

//�������������� �������
#define ALARM_TYPE_BUTTON 0
#define ALARM_TYPE_KBD 1
#define LONG_P 2
#define SHORT_P 1
#define END_P 0
#define ALARM_OFF 0 //����� � ������������
#define ALARM_ON 1 // ���������� �� ������������
#define ALARM_UP 2 //���������� �� ��������, ���� ����� �� ����� � ����� ALARM_ON ��� ALARM_BREAK
#define ALARM_DOWN 3//���� ����� �� ����� � ����� ALARM_OFF ��� ALARM_BREAK
#define ALARM_BREAK 4 //���� � �����, ����� ������������ �����������, ����, ������, ������
#define LONG_PRESS 200 //200����
#define SHORT_PRESS 50 //50����
//#define PRESS_INTERVAL 500 //500����, ���� ��� ������� �� ��� ����� ������ �������� �����, �� �� PASS_TRY
#define PASS_TRY 3 // ���������� ������� ����� ������ ��� ������ � ��������. 3 ���� ���� ����������� - ALARM_BREAK
#define ALARM_UP_DELAY 60000//1��� �� �������� ������
#define ALARM_DOWN_DELAY 60000//1��� � PASS_TRY ���.-�� ������� ������ ������
#define LOUD_ON_TIME 1000 //1��� ����
#define LOUD_OFF_TIME 500 //0.5��� �����
#define LAMP_ON_TIME 500 //0.5��� ������
#define LAMP_OFF_TIME 500 //0.5��� �����
#define MOBILE_ON_TIME 1000 //���� �� ������ 1 ���.
#define MOBILE_OFF_TIME 60000 //��������� ����� ����� 1���
#define ST_CTC_HANDMADE 255-125

//��������������� ���������������
#define PIN_(port)  PIN  ## port
#define PIN(port)  PIN_(port)
#define B(bit_no)         (1 << (bit_no))
#define CB(reg, bit_no)   (reg) &= ~B(bit_no)
#define SB(reg, bit_no)   (reg) |= B(bit_no)
#define VB(reg, bit_no)   ( (reg) & B(bit_no) )
#define TB(reg, bit_no)   (reg) ^= B(bit_no)
//������� ��� ������ �� ������������ ���������
#define GREEN_LED_ON GREEN_LED_PORT|=_BV(GREEN_LED)
#define GREEN_LED_OFF GREEN_LED_PORT&=~_BV(GREEN_LED)
#define RED_LED_ON RED_LED_PORT|=_BV(RED_LED)
#define RED_LED_OFF RED_LED_PORT&=~_BV(RED_LED)
//������� ���������� ��������
#define LOUD_ON LOUD_PORT|=_BV(LOUD)
#define LOUD_OFF LOUD_PORT&=~_BV(LOUD)
#define LAMP_ON LAMP_PORT|=_BV(LAMP)
#define LAMP_OFF LAMP_PORT&=~_BV(LAMP)
#define MOBILE_ON MOBILE_PORT|=_BV(MOBILE)
#define MOBILE_OFF MOBILE_PORT&=~_BV(MOBILE)
//������ � �������
#define HI(x) ((x)>>8)
#define LO(x) ((x)& 0xFF)
#define ABS(x) ((x) < 0 ? -(x) : (x))

/*?�������������*/

/*���������� ���������� ����������*/
//��������� ������ �������� � ��������� �������
typedef struct {
	uint8_t alarm_type;
	uint8_t current_state;
	uint8_t previous_state;
	uint8_t KBD_pass[];
	uint8_t BUT_pass;
} alarm_struct;
static alarm_struct signalka={ALARM_TYPE_KBD, ALARM_OFF, ALARM_OFF, KBD_PASS, BUT_PASS}, *p_signalka=&signalka;

//��������� �� ��������� ������������
static struct pt Buttons_pt;
static struct pt Switch_pt;
static struct pt Leds_pt;
//����������� ���������� ���������� �������, ������ ������� �����
volatile static uint32_t st_timer0_millis=0;



/*���������� �������*/
//������ ������ �������� ���������� �������
uint32_t st_millis(void);
/*?���������� �������*/


/*�������*/
//������ ������ �������� ���������� �������
uint32_t st_millis(void)
{
	uint32_t m;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		m = st_timer0_millis;
	}

	return m;
}
/*?�������*/
void check_button()
{

}
uint8_t * get_password()
{

}


/*����������� ����������*/
//��������� ���������� �� ���������� �� �������0, ����������� ��������� ������
ISR(TIMER0_OVF_vect)
{
	st_timer0_millis++;
}

/*?����������� ����������*/

/* ����������� */

//���������� 1 - ��������� ������� ������, ��������� ������ ����������
PT_THREAD(Buttons(struct pt *pt))
{
	static uint32_t but_timer=0;
	static volatile uint8_t delay=0; //200���� �������� ����� ���������
	PT_BEGIN(pt);
	PT_WAIT_UNTIL(pt, (st_millis()-but_timer)>=10);
	but_timer=st_millis();
	/*if (delay>0) 
	{
		delay--;
	}
	else 
	{
		if (!(BUT1_PORT_PIN&(_BV(BUT1))))//((BUT1_PORT_PIN&(_BV(BUT1)))==0)
		{
			if (p_generator->regime==GEN_MANUAL)
			{
				p_generator->regime=GEN_PERIODIC;
				p_generator->period=PERIOD_HZ1;
			}
			else if ((p_generator->regime==GEN_PERIODIC)&&(p_generator->period==PERIOD_HZ1))
			{
				p_generator->period=PERIOD_HZ100;
			}
			else if ((p_generator->regime==GEN_PERIODIC)&&(p_generator->period==PERIOD_HZ100))
			{
				p_generator->period=PERIOD_HZ1000;
			}
			else if ((p_generator->regime==GEN_PERIODIC)&&(p_generator->period==PERIOD_HZ1000))
			{
				p_generator->regime=GEN_MANUAL;
			}
			p_generator->state=GEN_OFF;
			GENERATOR_OFF;
			DISCONECT_TIMER_FROM_PIN;
			OUT_OFF;
			delay=BUTTON_DELAY_200MSEC;
		}
		if (!(BUT0_PORT_PIN&(_BV(BUT0))))
		{
			if ((p_generator->regime==GEN_MANUAL))
			{
				p_generator->state=GEN_ONESHOT;
			}
			else if ((p_generator->regime==GEN_PERIODIC))
			{
				if (p_generator->duration==DURATION_US320)
				{
					p_generator->duration=DURATION_50;
				}
				else if (p_generator->duration==DURATION_50)
				{
					p_generator->duration=DURATION_90;
				}
				else
				{
					p_generator->duration=DURATION_US320;
				}
				p_generator->state=GEN_OFF;
			}
			GENERATOR_OFF;
			DISCONECT_TIMER_FROM_PIN;
			OUT_OFF;
			delay=BUTTON_DELAY_200MSEC;
		}
	}*/
	PT_END(pt);
}
//����������2 - �������� �������1 � ��������� ������ ������ ����������
PT_THREAD(Switch(struct pt *pt))
{
	static volatile uint32_t switch_timer=0;
	PT_BEGIN(pt);
	PT_WAIT_UNTIL(pt,(st_millis()-switch_timer)>=10);//������ ����������� ������ 10����
	switch_timer=st_millis();
	/*if (p_generator->state==GEN_OFF)
	{
		if(p_generator->regime==GEN_PERIODIC)
		{
			CLEAR_TCCR1B;
			//DISCONECT_TIMER_FROM_PIN;
			//OUT_OFF;
			if (p_generator->period==PERIOD_HZ1000)
			{
				TCCR1B|=PRESCALER_1_MASK;
				ICR1=PERIOD_HZ1000_TICKS;
			}
			else if (p_generator->period==PERIOD_HZ100)
			{
				TCCR1B|=PRESCALER_8_MASK;
				ICR1=PERIOD_HZ100_TICKS;
			}
			else
			{
				TCCR1B|=PRESCALER_256_MASK;
				ICR1=PERIOD_HZ1_TICKS;
			}
			if (p_generator->duration==DURATION_90)
			{
				OCR1=(uint16_t)((7*(uint32_t)ICR1)>>3);
			}
			else if (p_generator->duration==DURATION_50)
			{
				OCR1=(uint16_t)((uint32_t)ICR1>>1);
			}
			else
			{
				if (p_generator->period==PERIOD_HZ1000)
				{
					OCR1=(uint16_t)(5*(uint32_t)ICR1>>4);
				}
				else if (p_generator->period==PERIOD_HZ100)
				{
					OCR1=(uint16_t)((uint32_t)ICR1>>5);
				}
				else
				{
					OCR1=(uint16_t)(5*(uint32_t)ICR1>>14);
				}
			}
			p_generator->state=GEN_ON;
			CONNECT_TIMER_TO_PIN;//���������� ������ � ����
			GENERATOR_ON;
		}
		if (p_generator->regime==GEN_MANUAL)
		{
			//TIMSK&=~_BV(TOIE1);
			GENERATOR_OFF;
			DISCONECT_TIMER_FROM_PIN;
			OUT_OFF;
			LED0_OFF;
			LED1_OFF;
		}
	}
	else if (p_generator->regime==GEN_UART)
	{
		//��� ���-�� ����� :) ����� ����� ����� �������� ������ �� �����, � ����� � �������� �����������
	}
	else if ((p_generator->regime==GEN_MANUAL)&&(p_generator->state==GEN_ONESHOT))
	{
		CLEAR_TCCR1B;
		TCCR1B|=PRESCALER_1_MASK;
		DISCONECT_TIMER_FROM_PIN;
		TCNT1=0;
		ICR1=60000;
		OCR1=2500;
		CONNECT_TIMER_TO_PIN;
		GENERATOR_ON;
		LED0_ON;
		LED1_ON;
	}*/
	PT_END(pt);
}
PT_THREAD(Leds(struct pt *pt))
{
	static volatile uint32_t leds_timer=0;
	static volatile uint8_t counter1, counter2=0;
	PT_BEGIN(pt);
	PT_WAIT_UNTIL(pt,(st_millis()-leds_timer)>=100);//������ ����������� ������ 0.1����
	leds_timer=st_millis();
	/*if (p_generator->regime==GEN_PERIODIC)
	{
		switch(p_generator->period)
		{
			case PERIOD_HZ1:
			{
				if (counter1<10) //������� LED0 ��� � �������
				{
					LED0_ON;
					counter1++;
					counter2=0;
				}
				else if (counter2<10)
				{
					LED0_OFF;
					counter2++;
					if (counter2>=9)
					{
						counter1=0;
					}
				}
				LED1_OFF;
				break;
			}
			case PERIOD_HZ100: //������� LED0 - 10 ��� � �������
			{
				if (((LED0_PORT_PIN&(_BV(LED0)))==0))
				{
					LED0_ON;
				}
				else
				{
					LED0_OFF;
				}
				LED1_OFF;
				break;
			}
			case PERIOD_HZ1000: //LED0 ����� ����������
			{
				LED0_ON;
				LED1_OFF;
				break;
			}
		}
	}
	else if (p_generator->regime==GEN_UART) //������� ������ ������������
	{
		if ((LED0_PORT_PIN&(_BV(LED0)))==0)
		{
			LED0_OFF;
			LED1_OFF;
		}
		else 
		{
			LED0_ON;
			LED1_ON;
		}
	}
	else
	{
		LED0_OFF;
		LED1_OFF;	
	}*/
	PT_END(pt);
}
/*?�����������*/

int main(void)
{
	//��������� ������-�������
	DDRD=0b00000000; //All inputs (���� ������ ������ ���� �����)
	DDRB=0b11111111; //��� ���� PORTB - ������
	DDRC=0b11110000;//PC0 - ����� ���� �������� (������/�����), PC1 - ������, PC2- ������, PC3 - PIR
	PORTD=0b0000100;//�������� � ������� ����� 100k ��� ���� ����� ->������ �������� �� ����� 
	PORTB=0;//��� ������ PORTB �� �����.
	PORTC=0b0001111;//100k pull-up PC0-PC3
	
	//���������� ��� �������
	if (!(REGIME_SEL_PIN&(_BV(REGIME_SEL))))//((BUT1_PORT_PIN&(_BV(BUT1)))==0)
	{
		p_signalka->alarm_type=ALARM_TYPE_BUTTON;
	}
	else p_signalka->alarm_type=ALARM_TYPE_KBD;
	if ((p_signalka->alarm_type==ALARM_TYPE_KBD)&&(check_button()=='*'))
	{
		p_signalka->pass=get_password();
	}
	else 

	
	// ��������� ���������� �������
	TCCR0 |= (_BV(CS01) | _BV(CS00));
	// Enable interrupt
	//TIMSK |= _BV(TOIE0) | _BV(OCIE1A);
	// Set default value
	TCNT0 = ST_CTC_HANDMADE; //1ms tiks on 8mhz CPU clock
	
	//������������� ������������
	PT_INIT(&Buttons_pt);
	PT_INIT(&Switch_pt);
	PT_INIT(&Leds_pt);
	
	//��������� ������
	wdt_reset(); //���������� ������ �� ������ ��������
	wdt_enable(WDTO_2S); //��������� ������ � ������� 2�
	//��������� ���������
	sei();

    while(1)
    {
		//������ �����������
		PT_SCHEDULE(Buttons(&Buttons_pt));
		PT_SCHEDULE(Switch(&Switch_pt));
		PT_SCHEDULE(Leds(&Leds_pt));
		wdt_reset(); //������������ ���������� ������ ����� �� ��������� � �����
	 }
}


