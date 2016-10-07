/*
 * Simple Alarm
 * Ver 1.0
 * Created: 26.08.2016 12:37:18
 * Author : Brendel Vadim
 * e-mail: brendel.vadim@gmail.com
 
  Суть устройства:
  
  Устройство представляет собой простую сигнализацию. Область применения - гараж, кладовка и т.п.
  Реализовано 2 режима (переключение режимов - джампером):
  1. постановка и снятие с охраны одной кнопкой с помошью серии долгих и
  коротких нажатий (по аналогии с азбукой морзе, тире-точка). Код задается с помошью джамперов (туда же 
  может быть подключена клавиатура для второго режима), длительность кода задается в прошивке.
  Задается дительность длинного нажатия, короткого, паузы между нажатиями, время сброса ввода пароля (в миллисекундах). 
  2. постановка и снятие с охраны одной кнопкой с помошью набора кода с клавиатуры. Код задается пользователем,
  при нажатой клавище * во время ребута системы. Признак окончания ввода кода - клавиша #.
  Также задаются:
  Длительность задержки срабатывания сигналки при срабатывании датчиков (для ввода пароля снятия с сигнализации)
  Длительность задержки срабатывания при постановке на сигнализацию (ввели пароль, закрываем гараж).
  Длительность звукового импульса, длительность паузы при срабатывания сигналки.
  Длительность светового импульса и пазы при срабатывании сигналки.
  Длительность импульса звонка на мобильный. Длительность паузы.
  
  Входные датчики:
  1. герконовый датчик на двери (срабатывание - логический ноль)
  2. PIR датчик движения  (срабатывание - логический ноль).
  
  Выходные устройства:
  1. Сирена (автомобильный гудок, либо ревун - через реле с выбором AC220В или DC12В).
  2. Лампа (автомобильная фарная, либо 220В - через реле с выбором AC220В или DC12В).
  3. Опционально - мобильный телефон Nokia, Siemens и проч. Нажатие кнопки вызова последнего телефона.
  
  Индикация:
  1. Светодиод "Поставлено на охрану" - зеленый. Горит - поставлено на охрану. Мигает - введен пароль
   постановки на охрану, но идет задержка Длительность задержки срабатывания при постановке на сигнализацию 
  2. Светодиод "Снято с охраны" - красный. Горит - снято на с охраны. Мигает - введен пароль снятия с охраны,
   но идет задержка Длительность задержки срабатывания сигналки при срабатывании датчиков
  3. Срабатывание сигнализации - оба светодиода моргают.
   
   В следующей версии устройства возможно подключения цифровой клавиатуры (AK-207-N-WWB или аналог),
   поэтому программа и схема сделаны с учетом этой доработки в будущем. Возможны 7пиновые клавы и 8пиновые.
   
   Распиновка контроллера:
   Входы:
   PD0-PD7 - клавиатура. Входы с подтяжкой к питанию. сюда же подключена колодка, для пароля кнопки
   PC0 - ввод выбора режима кнопка/клава
   PС1 - кнопка ввода пароля. Входы с подтяжкой к питанию.
   PC2 - геркон, вход с подтяжкой к питанию
   PC3 - PIR датчик
   Остальные PC зарезервированы под входы под прочие датчики
   
   Выходы
   PB0 - выход на ревун (танзистор + реле+выбор напряжения)
   PB1 - выход на лампу (транзистор+ реле+выбор напряжения)
   PB2 - выход на кнопку вызова на мобильном телефоне.
   PB3 - Светодиод "Поставлено на охрану" - зеленый.
   PB4 - Светодиод "Снято с охраны" - красный.
   Остальные PB зарезервированы под прочие выходы.
   
  Ядро крутится от внутреннего генератора на частоте 8МГц.
  Таймер Т0- системный настроен на прерывание каждую милисекунду, прокручивает протопотоки.
   
  Протопоток1 (1мсек) проверяет кнопки и устанавливает режим работы, записывая значения в структуру.
  Протопоток2 (10мсек) конфигурирует таймер T0 согласно режиму работы и запускает генерацию
  Протопоток3 (100мсек) делает индикацию режима работы
  Протопоток4 (5мсек) общается по UART.  
 
 */ 

#include <avr/io.h>
#include <util/atomic.h>
#include <pt.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <pt-sem.h>


/*Макрооредения*/
//Пароль по умолчанию для режима с клавиатурой
#define KBD_PASS {1, 2, 3, 4, #}
#define BUT_PASS 0b00000000
#define BUT_PASS_LEN 4
//Макроопределения входов-выходов
//Входы
#define REGIME_SEL PC0
#define REGIME_SEL_PORT PORTC
#define REGIME_SEL_DDR DDRC
#define REGIME_SEL_PIN PINC
#define PASS_BUT PC1 //вход на кнопку ввода пароля
#define PASS_BUT_PORT PORTC
#define PASS_BUT_DDR DDRC
#define PASS_BUT_PIN PINC
#define KBD_PORT PORTD
#define KBD_DDR DDRD
#define KBD_PIN PIBD
#define GERKON PC2 //вход на герконовый датчик
#define GERKON_PORT PORTC
#define GERKON_DDR DDRC
#define GERKON_BUT_PIN PINC
#define PIR PC3 //вход на PIR датчик
#define PIR_PORT PORTC
#define PIR_DDR DDRC
#define PIR_BUT_PIN PINC

//Выходы
#define LOUD PB0 //выход на ревун/звуковой сигнал
#define LOUD_PORT PORTB
#define LOUD_PORT_DDR DDRB
#define LOUD_PORT_PIN PINB
#define LAMP PB1 //выход на лампы
#define LAMP_PORT PORTB
#define LAMP_PORT_DDR DDRB
#define LAMP_PORT_PIN PINB
#define MOBILE PB2 //выход на вызов с мобильника
#define MOBILE_PORT PORTB
#define MOBILE_PORT_DDR DDRB
#define MOBILE_PORT_PIN PINB
#define GREEN_LED PB3 //выход на Светодиод "Поставлено на охрану" - зеленый. 
#define GREEN_LED_PORT PORTB
#define GREEN_LED_PORT_DDR DDRB
#define GREEN_LED_PORT_PIN PINB
#define RED_LED PB4 // выход на Светодиод "Снято с охраны" - красный. 
#define RED_LED_PORT PORTB
#define RED_LED_PORT_DDR DDRB
#define RED_LED_PORT_PIN PINB

//Макроопредения режимов
#define ALARM_TYPE_BUTTON 0
#define ALARM_TYPE_KBD 1
#define LONG_P 2
#define SHORT_P 1
#define END_P 0
#define ALARM_OFF 0 //снято с сигнализации
#define ALARM_ON 1 // поставлено на сигнализацию
#define ALARM_UP 2 //Поставлено на сигналку, идет пауза до входа в режим ALARM_ON или ALARM_BREAK
#define ALARM_DOWN 3//Идет пауза до входа в режим ALARM_OFF или ALARM_BREAK
#define ALARM_BREAK 4 //Вход в режим, когда сигнализация срабатывает, орет, мигает, звонит
#define LONG_PRESS 200 //200мсек
#define SHORT_PRESS 50 //50мсек
//#define PRESS_INTERVAL 500 //500мсек, если нет нажатия за это время пароль вводится снова, но до PASS_TRY
#define PASS_TRY 3 // Количество попыток ввода пароля при снятии с сигналки. 3 раза ввел неправильно - ALARM_BREAK
#define ALARM_UP_DELAY 60000//1мин на закрытие дверей
#define ALARM_DOWN_DELAY 60000//1мин и PASS_TRY кол.-во попыток ввести пароль
#define LOUD_ON_TIME 1000 //1сек орет
#define LOUD_OFF_TIME 500 //0.5сек пауза
#define LAMP_ON_TIME 500 //0.5сек светит
#define LAMP_OFF_TIME 500 //0.5сек пауза
#define MOBILE_ON_TIME 1000 //жмет на кнопку 1 сек.
#define MOBILE_OFF_TIME 60000 //следующий вызов через 1мин
#define ST_CTC_HANDMADE 255-125

//Макроопредления вспомогательные
#define PIN_(port)  PIN  ## port
#define PIN(port)  PIN_(port)
#define B(bit_no)         (1 << (bit_no))
#define CB(reg, bit_no)   (reg) &= ~B(bit_no)
#define SB(reg, bit_no)   (reg) |= B(bit_no)
#define VB(reg, bit_no)   ( (reg) & B(bit_no) )
#define TB(reg, bit_no)   (reg) ^= B(bit_no)
//Макросы для работы со светодиодами индикации
#define GREEN_LED_ON GREEN_LED_PORT|=_BV(GREEN_LED)
#define GREEN_LED_OFF GREEN_LED_PORT&=~_BV(GREEN_LED)
#define RED_LED_ON RED_LED_PORT|=_BV(RED_LED)
#define RED_LED_OFF RED_LED_PORT&=~_BV(RED_LED)
//Макросы управления выходами
#define LOUD_ON LOUD_PORT|=_BV(LOUD)
#define LOUD_OFF LOUD_PORT&=~_BV(LOUD)
#define LAMP_ON LAMP_PORT|=_BV(LAMP)
#define LAMP_OFF LAMP_PORT&=~_BV(LAMP)
#define MOBILE_ON MOBILE_PORT|=_BV(MOBILE)
#define MOBILE_OFF MOBILE_PORT&=~_BV(MOBILE)
//Работа с числами
#define HI(x) ((x)>>8)
#define LO(x) ((x)& 0xFF)
#define ABS(x) ((x) < 0 ? -(x) : (x))

/*?Макрооредения*/

/*Объявление глобальных переменных*/
//Структура режима сигналки и начальные условия
typedef struct {
	uint8_t alarm_type;
	uint8_t current_state;
	uint8_t previous_state;
	uint8_t KBD_pass[];
	uint8_t BUT_pass;
} alarm_struct;
static alarm_struct signalka={ALARM_TYPE_KBD, ALARM_OFF, ALARM_OFF, KBD_PASS, BUT_PASS}, *p_signalka=&signalka;

//Указатели на структуры протопотоков
static struct pt Buttons_pt;
static struct pt Switch_pt;
static struct pt Leds_pt;
//Статическая переменная системного таймера, хранит текущее время
volatile static uint32_t st_timer0_millis=0;



/*Объявление функций*/
//Фукция выдачи текущего системного времени
uint32_t st_millis(void);
/*?Объявление функций*/


/*Функции*/
//Фукция выдачи текущего системного времени
uint32_t st_millis(void)
{
	uint32_t m;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		m = st_timer0_millis;
	}

	return m;
}
/*?Функции*/
void check_button()
{

}
uint8_t * get_password()
{

}


/*Обработчики прерываний*/
//Обработка прерывания по совпадению от таймера0, увеличивает системный таймер
ISR(TIMER0_OVF_vect)
{
	st_timer0_millis++;
}

/*?Обработчики прерываний*/

/* Протопотоки */

//Протопоток 1 - обработка нажатия кнопок, настройка режима генератора
PT_THREAD(Buttons(struct pt *pt))
{
	static uint32_t but_timer=0;
	static volatile uint8_t delay=0; //200мсек задержка между нажатиями
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
//Протопоток2 - настрока таймера1 и индикация режима работы генератора
PT_THREAD(Switch(struct pt *pt))
{
	static volatile uint32_t switch_timer=0;
	PT_BEGIN(pt);
	PT_WAIT_UNTIL(pt,(st_millis()-switch_timer)>=10);//запуск протопотока каждые 10мсек
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
			CONNECT_TIMER_TO_PIN;//подключаем таймер к пину
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
		//тут что-то будет :) можно прямо здесь написать работу от уарта, а можно в отдельно протопотоке
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
	PT_WAIT_UNTIL(pt,(st_millis()-leds_timer)>=100);//запуск протопотока каждые 0.1мсек
	leds_timer=st_millis();
	/*if (p_generator->regime==GEN_PERIODIC)
	{
		switch(p_generator->period)
		{
			case PERIOD_HZ1:
			{
				if (counter1<10) //моргает LED0 раз в секунду
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
			case PERIOD_HZ100: //моргает LED0 - 10 ращ в секунду
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
			case PERIOD_HZ1000: //LED0 горит непрерывно
			{
				LED0_ON;
				LED1_OFF;
				break;
			}
		}
	}
	else if (p_generator->regime==GEN_UART) //моргаем обоими светодиодами
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
/*?Протопотоки*/

int main(void)
{
	//Настройка входов-выходов
	DDRD=0b00000000; //All inputs (либо пароль кнопки либо клава)
	DDRB=0b11111111; //Все пины PORTB - выходы
	DDRC=0b11110000;//PC0 - выбор типа сигналки (кнопка/клава), PC1 - кнопка, PC2- геркон, PC3 - PIR
	PORTD=0b0000100;//Подтяжка к питанию через 100k для всех ножек ->кнопки замыкают на землю 
	PORTB=0;//Все выходы PORTB на земле.
	PORTC=0b0001111;//100k pull-up PC0-PC3
	
	//Определяем тип таймера
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

	
	// Настройка системного таймера
	TCCR0 |= (_BV(CS01) | _BV(CS00));
	// Enable interrupt
	//TIMSK |= _BV(TOIE0) | _BV(OCIE1A);
	// Set default value
	TCNT0 = ST_CTC_HANDMADE; //1ms tiks on 8mhz CPU clock
	
	//Инициализация протопотоков
	PT_INIT(&Buttons_pt);
	PT_INIT(&Switch_pt);
	PT_INIT(&Leds_pt);
	
	//Настройка собаки
	wdt_reset(); //сбрасываем собаку на всякий пожарный
	wdt_enable(WDTO_2S); //запускаем собаку с перидом 2с
	//Запускаем прерывани
	sei();

    while(1)
    {
		//Крутим протопотоки
		PT_SCHEDULE(Buttons(&Buttons_pt));
		PT_SCHEDULE(Switch(&Switch_pt));
		PT_SCHEDULE(Leds(&Leds_pt));
		wdt_reset(); //переодически сбрасываем собаку чтобы не улетететь в ресет
	 }
}


