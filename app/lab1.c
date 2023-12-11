#include "includes.h"

#define F_CPU 16000000UL // CPU frequency = 16 Mhz
#include <avr/io.h>
#include <util/delay.h>

#define N_TASKS 3
#define TASK_STK_SIZE OS_TASK_DEF_STK_SIZE
#define CDS_VALUE 871

OS_STK TaskStk[N_TASKS][TASK_STK_SIZE];

// Mailbox, readAdcTask에서 turnOnLEDTask로 A/D 컨버터 데이터 레지스터 값을 보낼 때 사용
OS_EVENT* Mbox;

unsigned char display[4];

void initAdc(); // A/D 컨버터 초기화

void readAdcTask(void* data); // A/D 컨버터 데이터 레지스터 값을 읽어오는 Task
void turnOnLEDTask(void* data); // 주변 밝기에 따라 LED를 켜는 Task
void turnOnFNDTask(void* data); // display 배열을 이용해 FND를 켜는 task

int main(void)
{
  OSInit();

  OS_ENTER_CRITICAL();
  TCCR0 = 0x07;
  TIMSK = _BV(TOIE0);
  TCNT0 = 256 - (CPU_CLOCK_HZ / OS_TICKS_PER_SEC / 1024);
  OS_EXIT_CRITICAL();

  Mbox = OSMboxCreate(0); // Mailbox 생성

  initAdc(); // A/D 컨버터 초기화
  OSTaskCreate(readAdcTask, (void*)0, (void*)&TaskStk[0][TASK_STK_SIZE - 1], 0);
  OSTaskCreate(turnOnLEDTask, (void*)0, (void*)&TaskStk[1][TASK_STK_SIZE - 1], 1);
  OSTaskCreate(turnOnFNDTask, (void*)0, (void*)&TaskStk[2][TASK_STK_SIZE - 1], 2);

  OSStart();
  return 0;
}

// A/D 컨버터 초기화 과정
void initAdc()
{
  ADMUX = 0x00;
  // AREF(+5V) 기준전압 사용
  // ADLAR = 0 --> A/D 컨버터 데이터 레지스터 오른쪽 정렬
  // ADC0 사용
  ADCSRA = 0x87; // 0b10000111
  // ADEN = 1 --> A/D 컨버터 허용
  // ADSC = 0 --> 맨 처음에는 A/D 변환을 하지 않음
  // ADFR = 0 --> 단일 변환 모드 사용
  // ADIF = 0 --> A/D 컨버터 인터럽트 플래그, 초기에는 0으로 설정
  // ADIE = 0 --> A/D 변환완료 인터럽트 사용하지 않음
  // ADPS = 0b111 --> A/D 컨버터 프리스케일러를 128분주로 설정
}

// A/D 컨버터 데이터 레지스터 값을 읽어오는 Task
void readAdcTask(void* data)
{
	unsigned short adc_low, adc_high;
	unsigned short value;

	ADCSRA |= 0x40; // ADSC = 1로 설정, A/D 변환 시작
	while ((ADCSRA & 0x10) != 0x10); // ADIF = 1이 될 때 까지 기다림
	
	// ADIF = 1이 됨, A/D 변환 완료
	adc_low = ADCL;
	adc_high = ADCH;
	
	value = (adc_high << 8) | adc_low;

  // A/D 컨버터 데이터 레지스터 값을 display 배열에 저장
  display[0] = value / 1000;
  display[1] = value / 100;
  display[2] = (value % 100) / 10;
  display[3] = value % 10;

	OSMboxPost(Mbox, (void*)&value); // readAdcTask에서 turnOnLEDTask로 A/D 컨버터 데이터 레지스터 값을 보냄
  OSTimeDlyHMSM(0, 0, 0, 500); // 500ms delay 발생
}

// 주변 밝기에 따라 LED를 켜는 Task
void turnOnLEDTask(void* data)
{
  DDRA = 0xFF;
  unsigned short read_value;
  unsigned short adc_data_reg_value;

  // A/D 컨버터의 데이터 레지스터 값이 CDS_VALUE보다 작으면 LED를 켜고 그렇지 않으면 LED를 끈다.
  while (1)
  {
    read_value = *(unsigned short*)OSMboxAccept(Mbox);

    if (read_value != (void*)0)
      adc_data_reg_value = read_value;

    if (adc_data_reg_value < CDS_VALUE)
      PORTA = 0xFF;
    else
      PORTA = 0x00;

    OSTimeDlyHMSM(0, 0, 0, 500); // 500ms delay 발생
  }
}

// display 배열을 이용해 FND를 켜는 task
void turnOnFNDTask(void* data)
{
  unsigned char FND_DATA[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7c, 0x07, 0x7f, 0x67, 0x40, 0x00};
  unsigned char fnd_sel[4] = {0x08, 0x04, 0x02, 0x01};
  unsigned char i;

  DDRC = 0xFF;
  DDRG = 0x0F;

  while (1)
	{
		for (i = 0; i < 4; i++)
		{
			PORTC = FND_DATA[display[i]];
			PORTG = fnd_sel[i];
			OSTimeDlyHMSM(0, 0, 0, 1);
		}
	}
}