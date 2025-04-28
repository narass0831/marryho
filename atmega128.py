#define F_CPU 16000000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include "LCD.h"

#define Kp  0.1
#define Ki  10.0
#define Kd   0.005
#define Ts  0.01

volatile unsigned char Right_m1, Right_count, Right_count0 = 0;
volatile unsigned int Left_m1, Left_count, Left_count0 = 0;
volatile float Wref, Left_Wr, Right_Wr, Left_Werr, Right_Werr, Left_Werr0, Right_Werr0, Left_Werr1, Right_Werr1 = 0;
volatile float Left_PID, Right_PID = 0;
volatile unsigned int Left_PWM, Right_PWM;
volatile unsigned char Ts_time = 0;
volatile char command = 'S';

void SPI_SlaveInit(void) {
    DDRB = (1 << PB4);   // MISO 출력, 나머지 핀 입력
    SPCR = (1 << SPE) | (1 << SPIE);  // SPI 활성화, Slave 모드, SPI 인터럽트 활성화
    sei();  // 전역 인터럽트 활성화
}

void Go_Straight(unsigned int speed) {
    PORTC = 0xA0;
    Wref = speed;
    OCR3A = Left_PID;
    OCR3B = Right_PID;
}

void Go_Back(unsigned int speed) {
    PORTC = 0x50;
    Wref = speed;
    OCR3A = Left_PID;
    OCR3B = Right_PID;
}

void Turn_Left(unsigned int speed) {

    PORTC = 0xE0;
   Wref = speed;
    OCR3B = Right_PID;
}

void Turn_Right(unsigned int speed) {

    PORTC = 0xB0;
   Wref = speed;
    OCR3A = Left_PID;

}

void Stop_Motor() {
    PORTC = 0x00;
    Wref = 0;
    Left_PID = 0;
    Right_PID = 0;
    OCR3A = 0;
    OCR3B = 0;
}

ISR(SPI_STC_vect) {
    char received_command = SPDR;  // 수신된 데이터 읽기
    SPDR = received_command;  // 수신된 데이터 그대로 송신 (Echo)

    switch (received_command) {
        case 'F':
            Go_Straight(50);
            break;
        case 'B':
            Go_Back(200);
            break;
        case 'L':
            Turn_Left(20);
            break;
        case 'R':
            Turn_Right(20);
            break;
        case 'S':
            Stop_Motor();
            break;
        default:
            Stop_Motor();
            break;
    }
   command=received_command;
}

ISR(TIMER0_COMP_vect) {
    Ts_time++;
    if (Ts_time >= 5) {
        Ts_time = 0;
        Left_count = TCNT1;
        Right_count = TCNT2;

        Left_m1 = Left_count - Left_count0;
        Left_count0 = Left_count;
        Left_Wr = (float)Left_m1 * 60.0 / 740.0 / 0.01;

        Right_m1 = Right_count - Right_count0;
        Right_count0 = Right_count;
        Right_Wr = (float)Right_m1 * 60.0 / 740.0 / 0.01;

        // Left Motor PI Control
        Left_Werr = Wref - Left_Wr;
        Left_PID = Left_PID + Kp*(Left_Werr-Left_Werr0)+Ki*Left_Werr*Ts+Kd*(Left_Werr-2*Left_Werr0+Left_Werr1)/Ts;
        Left_PWM = Left_PID;
        if (Left_PWM > 799) Left_PWM = 799;
        else if (Left_PWM < 0) Left_PWM = 0;
        //OCR3A = Left_PWM;

        // Right Motor PI Control
        Right_Werr = Wref - Right_Wr; 
        Right_PID = Right_PID + Kp*(Right_Werr-Right_Werr0)+Ki*Right_Werr*Ts+Kd*(Right_Werr-2*Right_Werr0+Right_Werr1)/Ts;
        Right_PWM = Right_PID;
        if (Right_PWM > 799) Right_PWM = 799;
        else if (Right_PWM < 0) Right_PWM = 0;
        //OCR3B = Right_PWM;

      Left_Werr0 = Left_Werr;
      Left_Werr1 = Left_Werr0;
      Right_Werr0 = Right_Werr;
      Right_Werr1 = Right_Werr0;




    }
}


int main(void) {
    DDRE |= 0x1A;
    DDRC |= 0xF0;
    DDRB |= 0x90;
    DDRD |= 0x00;

    TCCR3A = 0xA2;
    TCCR3B = 0x19;
    ICR3 = 799;
    OCR3A = 0;
    OCR3B = 0;

    TCCR0 = 0x0D;
    OCR0 = 249;
    TIMSK = 0x02;

    TCCR1A = 0x00;
    TCCR1B = 0x06;
    TCNT1 = 0x0000;

    TCCR2 = 0x06;
    TCNT2 = 0x00;

    SPI_SlaveInit();
   LCD_init();
   _delay_ms(1000);

    sei();

    while (1) {
      LCD_clear();
      LCD_goto_XY(0,0);
      LCD_write_data(command);
      _delay_ms(5);
        // Main Loop
    }
   return 0;
}
