#include "includes.h"

#define F_CPU 16000000UL // CPU frequency = 16 Mhz
#include <avr/io.h>
#include <util/delay.h>

#define N_TASKS 6
#define TASK_STK_SIZE OS_TASK_DEF_STK_SIZE

#define CDS_VALUE 871
#define ATS75_CONFIG_REG 1
#define ATS75_TEMP_REG 0
#define ATS75_ADDR 0x98

OS_STK TaskStk[N_TASKS][TASK_STK_SIZE];

// Mailbox, read_adc_task에서 turn_on_LED_task로 A/D 컨버터 데이터 레지스터 값을 보낼 때 사용
OS_EVENT* mbox_for_read_adc_task;

// Mailbox, temperature_task에서 calculate_FND_task로 온도 값을 보낼 때 사용
OS_EVENT* mbox_for_temperature_task;

// Message Queue, calculate_FND_task에서 turn_on_FND_task로 FND에 표시할 값에 대한 배열을 전달할 때 사용
OS_EVENT* msg_queue_for_calculate_FND_task;

// turn_on_siren_task에 대한 Event Flag (0x01일 경우 경보음이 울림)
OS_EVENT* event_flag_for_turn_on_siren_task;

void* msg_queue_arr[4];

void init_adc(); // A/D 컨버터 초기화
void init_I2C(); // I2C 초기 설정
void write_twi_1byte_nopreset(unsigned char reg, unsigned char data); // reg에 1byte의 data를 입력
void write_twi_0byte_nopreset(unsigned char reg);

void read_adc_task(void* data); // A/D 컨버터 데이터 레지스터 값을 읽어오는 Task
void turn_on_LED_task(void* data); // 주변 밝기에 따라 LED를 켜는 Task
void temperature_task(void* data); // 주변 온도를 측정하는 Task
void calculate_FND_task(void* data); // FND에 표시할 값을 계산하는 Task
void turn_on_FND_task(void* data); // FND를 켜는 task
void turn_on_siren_task(void* data); // 주변 온도가 일정 수준을 넘어섰을 때 경보음을 내는 Task

int main(void)
{
  OSInit();

  OS_ENTER_CRITICAL();
  TCCR0 = 0x07;       // 타이머/카운터0의 분주비를 1024로 설정
  TIMSK = _BV(TOIE0); // 타이머/카운터0의 오버플로우 인터럽트 Enable
  TCNT0 = 256 - (CPU_CLOCK_HZ / OS_TICKS_PER_SEC / 1024); // 타이머/카운터0의 초기 카운터 설정 
  OS_EXIT_CRITICAL();

  mbox_for_read_adc_task = OSMboxCreate(0);    // Mailbox 생성
  mbox_for_temperature_task = OSMboxCreate(0); // Mailbox 생성

  // 크기가 4인 Message Queue 생성
  msg_queue_for_calculate_FND_task = OSQCreate(&msg_queue_arr[0], 4);

  init_adc(); // A/D 컨버터 초기화

  // FND 출력 세팅
	DDRC = 0xFF;
	DDRG = 0x0F;

  OSTaskCreate(read_adc_task, (void*)0, (void*)&TaskStk[0][TASK_STK_SIZE - 1], 0);
  OSTaskCreate(turn_on_LED_task, (void*)0, (void*)&TaskStk[1][TASK_STK_SIZE - 1], 1);
  OSTaskCreate(temperature_task, (void*)0, (void*)&TaskStk[2][TASK_STK_SIZE - 1], 2);
  OSTaskCreate(calculate_FND_task, (void*)0, (void*)&TaskStk[3][TASK_STK_SIZE - 1], 3);
  OSTaskCreate(turn_on_FND_task, (void*)0, (void*)&TaskStk[4][TASK_STK_SIZE - 1], 4);
  OSTaskCreate(turn_on_siren_task, (void*)0, (void*)&TaskStk[5][TASK_STK_SIZE - 1], 5);

  OSStart();
  return 0;
}

// A/D 컨버터 초기화 과정
void init_adc()
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
void read_adc_task(void* data)
{
	unsigned short adc_low, adc_high;
	unsigned short value;

	ADCSRA |= 0x40; // ADSC = 1로 설정, A/D 변환 시작
	while ((ADCSRA & 0x10) != 0x10); // ADIF = 1이 될 때 까지 기다림
	
	// ADIF = 1이 됨, A/D 변환 완료
	adc_low = ADCL;
	adc_high = ADCH;
	
	value = (adc_high << 8) | adc_low;

  // read_adc_task에서 turn_on_LED_task로 A/D 컨버터 데이터 레지스터 값을 보냄
	OSMboxPost(mbox_for_read_adc_task, (void*)&value);
  OSTimeDlyHMSM(0, 0, 0, 500); // 500ms delay 발생
}

// 주변 밝기에 따라 LED를 켜는 Task
void turn_on_LED_task(void* data)
{
  DDRA = 0xFF;
  unsigned short read_value;
  unsigned short adc_data_reg_value;

  // A/D 컨버터의 데이터 레지스터 값이 CDS_VALUE보다 작으면 LED를 켜고 그렇지 않으면 LED를 끈다.
  while (1)
  {
    read_value = *(unsigned short*)OSMboxAccept(mbox_for_read_adc_task);

    if (read_value != (void*)0)
      adc_data_reg_value = read_value;

    if (adc_data_reg_value < CDS_VALUE)
      PORTA = 0xFF;
    else
      PORTA = 0x00;

    OSTimeDlyHMSM(0, 0, 0, 500); // 500ms delay 발생
  }
}

// I2C 초기 설정
void init_I2C()
{
	PORTD = 3; // SCL, SCK에 대한 내부 Pull Up 저항 연결
	SFIOR &= ~(1 << PUD); // PUD를 0으로 설정 --> Pull Up 비활성화
	TWBR = 32; // 100K Hz bus clock
	TWSR = 0; // TWI Clock 계산 시 분주를 사용하지 않음
  TWCR = _BV(TWEA) | _BV(TWEN); // TWI 버스 활성화, 1 byte 데이터 수신 시 ACK 신호를 1로 생성
}

// reg에 1byte의 data를 입력
void write_twi_1byte_nopreset(unsigned char reg, unsigned char data)
{
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); // I2C 통신 시작
  
  // TWINT가 1로 설정될 때 까지 기다리거나 ACK이 올 때 까지 기다림
  while (((TWCR & _BV(TWINT)) == 0x00) 
  || ((TWSR & 0xF8) != 0x08 && (TWSR & 0xF8) != 0x10));

  // Start 신호가 정상적으로 전송됨
  TWDR = ATS75_ADDR | 0; // ATS75 레지스터와 통신하며 Write 모드로 설정
  TWCR = _BV(TWINT) | _BV(TWEN); // 값을 전달

  while (((TWCR & _BV(TWINT)) == 0x00) 
  || (TWSR & 0xF8) != 0x18); // ACK이 올 때 까지 기다림
  
  // SLA + W 값이 정상적으로 전송됨
  TWDR = reg;
  TWCR = _BV(TWINT) | _BV(TWEN); // aTS75 Configuration Register에 해당하는 값을 전달

  while (((TWCR & _BV(TWINT)) == 0x00) 
  || (TWSR & 0xF8) != 0x28); // ACK이 올 때 까지 기다림
  
  // 데이터가 정상적으로 전송됨
  TWDR = data; // 0x00
  TWCR = _BV(TWINT) | _BV(TWEN); // 0x00을 전달

  while (((TWCR & _BV(TWINT)) == 0x00) 
  || (TWSR & 0xF8) != 0x28); // ACK이 올 때 까지 기다림

  // 데이터가 정상적으로 전송됨
  // aTS75 Configuration Register에는 0x00의 값이 저장되어 있음
  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); // Stop 신호 전달
}

void write_twi_0byte_nopreset(unsigned char reg)
{
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); // I2C 통신 시작

  // TWINT가 1로 설정될 때 까지 기다리거나 ACK이 올 때 까지 기다림
  while (((TWCR & _BV(TWINT)) == 0x00) 
  || ((TWSR & 0xF8) != 0x08 && (TWSR & 0xF8) != 0x10));

  // Start 신호가 정상적으로 전송됨
  TWDR = ATS75_ADDR | 0; // ATS75 레지스터와 통신하며 Write 모드로 설정
  TWCR = _BV(TWINT) | _BV(TWEN); // 값을 전달

  while (((TWCR & _BV(TWINT)) == 0x00) 
  || (TWSR & 0xF8) != 0x18); // ACK이 올 때 까지 기다림

  TWDR = reg; // ATS75_TEMP_REG
  TWCR = _BV(TWINT) | _BV(TWEN); // 값을 전달

  while (((TWCR & _BV(TWINT)) == 0x00) 
  || (TWSR & 0xF8) != 0x28); // ACK이 올 때 까지 기다림

  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); // Stop 신호 전달
}

int read_temperature(void)
{
  int value;

  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); // I2C 통신 시작
  while(!(TWCR & _BV(TWINT)));

  TWDR = ATS75_ADDR | 1; // ATS75 레지스터와 통신하며 Read 모드로 설정
  TWCR = _BV(TWINT) | _BV(TWEN);
  while(!(TWCR & _BV(TWINT)));

  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
  while(!(TWCR & _BV(TWINT)));

  value = TWDR; // 상위 8 bit 정보
  TWCR = _BV(TWINT) | _BV(TWEN);
  while(!(TWCR & _BV(TWINT)));

  value = ((value << 8) | TWDR);
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);

  // 33 = 0000_0000_0010_0001
  // value가 33 이상일 경우 TIMSK의 TOIE2(타이머/카운터2)의 오버플로우 인터럽트를 Enable함

  TIMSK = (value >= 33) ? TIMSK | _BV(TOIE2) : TIMSK & ~_BV(TOIE2);

  return value;
}

// 주변 온도를 측정하는 Task
void temperature_task(void* data)
{
  unsigned short temperature_value;
  init_I2C(); // I2C 초기 설정

  // aTS75 Configuration Register에 0x00을 입력함
  // 유효 비트는 9 bit이며 Normal 모드로 설정
  write_twi_1byte_nopreset(ATS75_CONFIG_REG, 0x00);

  // ATS75 Temperature Register에 접근
  write_twi_0byte_nopreset(ATS75_TEMP_REG);

  while (1)
  {
    temperature_value = read_temperature();
    OSMboxPost(mbox_for_temperature_task, (void*)&temperature_value);
    OSTimeDlyHMSM(0, 0, 0, 500); // 500ms delay 발생
  }
}

void calculate_FND_task(void* data)
{
  unsigned char value_int, value_deci, num[4];
  unsigned short temperature_value;
  INT8U	err;

  while (1)
	{
		temperature_value = *(unsigned short*)OSMboxPend(mbox_for_temperature_task, 0, &err);

    if ((temperature_value & 0x8000) != 0x8000) // Sign bit 체크
    {
      // Sign bit가 0이므로 양수 --> 부호를 표시하지 않음
      num[0] = 11;
    }
    else
    {
      // Sign bit가 1이므로 음수 --> 부호를 표시함
      num[0] = 10;
      temperature_value = (~temperature_value) - 1; // 2의 보수를 취함
    }

    value_int = (unsigned char)((temperature_value & 0x7F00) >> 8);
    value_deci = (unsigned char)(temperature_value & 0x00FF);

    num[1] = value_int / 10;
    num[2] = value_int % 10;

    if (value_deci & 0x80 == 0x80)
      num[3] = 5;
    else
      num[3] = 0;

    OSQPost(msg_queue_for_calculate_FND_task, (void*)num);
    OSTimeDlyHMSM(0, 0, 0, 500); // 500ms delay 발생
	}
}

// 주변 온도를 FND를 이용해 출력하는 task
void turn_on_FND_task(void* data)
{
  unsigned char FND_DATA[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7c, 0x07, 0x7f, 0x67, 0x40, 0x00};
  // FND_DATA[10] --> 음수 부호(-)
  // FND_DATA[11] --> 빈칸 (아무것도 표기하지 않음)
  unsigned char fnd_sel[4] = {0x08, 0x04, 0x02, 0x01};
  unsigned char* receive_data;
  unsigned char* display; // FND에 실제로 표시하는 데이터
  unsigned char i;

  DDRC = 0xFF;
  DDRG = 0x0F;

  while (1)
  {
    // Message Queue를 이용한 방법
    receive_data = (unsigned char*)OSQAccept(msg_queue_for_calculate_FND_task);

    if (receive_data != (void*)0)
    {
      // receive_data가 NULL이 아닌 경우에만 display 값을 갱신
      display = receive_data;
    }

    for (i = 0; i < 4; i++)
    {
      // display 배열을 이용하여 FND에 값을 출력
      PORTC = FND_DATA[display[i]];
      PORTG = fnd_sel[i];
      if (i == 2)
      {
        // 소수점 추가
        PORTC |= 0x80;
      }
      OSTimeDlyHMSM(0, 0, 0, 1); // 500ms delay 발생
    }
  }
}

volatile int siren_state = OFF;

// 타이머/카운터1에 대한 출력비교 인터럽트 ISR
ISR(TIMER1_COMPA_vect)
{
	if (siren_state == ON)
	{
		PORTB = 0x00;
		siren_state = OFF;
	}
	else
	{
		PORTB = 0x10;
		siren_state = ON;
	}
	TCNT1 = 0;
}

// 주변 온도가 일정 수준을 넘어섰을 때 경보음을 내는 Task
void turn_on_siren_task(void* data)
{
  // 경보음으로 라 음을 냄
  // 라 음의 주파수는 440hz
	// 주기: 0.0022초 = 2,272us --> 0, 1의 상태를 각각 1,136us를 유지함

  DDRB = 0x10;   // Buzzer를 사용함
	TCCR1A = 0x00;
	TCCR1B = 0x03; // 64분주
	TCCR1C = 0x00;
	TIMSK = 1 << OCIE1A;
	TCNT1 = 0;
	OCR1A = 284; // 카운터가 284가 되면 타이머/카운터1에 대한 출력비교 인터럽트 ISR이 실행됨
  sei();

  while (1)
  {

  }
}