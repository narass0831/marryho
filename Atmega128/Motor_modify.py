#define F_CPU 16000000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include "LCD.h"

#define Kp_L  10
#define Ki_L  0.1
#define Kd_L   0.05
#define Kp_R  10
#define Ki_R  0.1
#define Kd_R   0.05
#define Ts  0.01

volatile unsigned char Right_m1, Right_count, Right_count0 = 0;
volatile unsigned int Left_m1, Left_count, Left_count0 = 0;
volatile float Left_Wref, Right_Wref, Left_Wr, Right_Wr, Left_Werr, Right_Werr, Left_Werr0, Right_Werr0, Left_Werr1, Right_Werr1 = 0;
volatile float Left_Wt, Right_Wt, Left_PID, Right_PID = 0;
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
       Left_Wref = speed;
       Right_Wref = speed;
    }
    void Turn_Left(unsigned int speed) {
       Right_Wref = 0;
       PORTC = 0xE0;
       Right_Wref = speed; //오른 바퀴
    }
    
    void Turn_Right(unsigned int speed) {
       Left_Wref = 0;
       PORTC = 0xB0;
       Left_Wref = speed;   //왼 바퀴 더돌게
    }
    void Stop_Motor() {
       PORTC = 0xf0;
       Left_Wref = 0;
       Right_Wref = 0;
    }
    ISR(SPI_STC_vect) {
       char received_command = SPDR;  // 수신된 데이터 읽기
       SPDR = received_command;  // 수신된 데이터 그대로 송신 (Echo)
       switch (received_command) {
          case 'F':
          Go_Straight(150);
          break;
          case 'L':
          Turn_Left(85);
          break;
          case 'R':
          Turn_Right(85);
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
            
            // Left Motor PID Control
            Left_Werr = Left_Wref - Left_Wr;
            Left_Wt += Left_Werr*Ts;                                                                              
            Left_PID = Kp_L*Left_Werr+Ki_L*Left_Wt+Kd_L*(Left_Werr-Left_Werr0)/Ts;                                             
            Left_PWM = Left_PID;
            if (Left_PWM > 799){
               Left_PWM = 799;
               Left_Wt -= Left_Werr*Ts;
            }
            else if (Left_PWM < 0){
               Left_PWM = 0;
               Left_Wt -= Left_Werr*Ts;
            }
            
            // Right Motor PID Control
            Right_Werr = Right_Wref - Right_Wr;
            Right_Wt += Right_Werr*Ts;                                                                              
            Right_PID = Kp_R*Right_Werr+Ki_R*Right_Wt+Kd_R*(Right_Werr-Right_Werr0)/Ts;                                          
            Right_PWM = Right_PID;
            if (Right_PWM > 799){
               Right_PWM = 799;
               Right_Wt -= Right_Werr*Ts;
            }
            else if (Right_PWM < 0){
               Right_PWM = 0;
               Right_Wt -= Right_Werr*Ts;
            }
            
             OCR3A = Left_PWM;
             OCR3B = Right_PWM;

            Left_Werr1 = Left_Werr0;
            Left_Werr0 = Left_Werr;
            Right_Werr1 = Right_Werr0;
            Right_Werr0 = Right_Werr;
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
         LCD_init();
         SPI_SlaveInit();
         _delay_ms(1000);
         sei();
         while (1) {
            LCD_clear();
            LCD_goto_XY(0,0);
            LCD_write_data(command);
            //Main Loop
      }
    return 0;
    }