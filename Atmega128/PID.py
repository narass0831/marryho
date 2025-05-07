#define F_CPU 16000000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include "LCD.h"

#define Kp   0.1
#define Ki   10.0
#define Ts   0.01

volatile unsigned char  Right_m1, Right_count, Right_count0 = 0;
volatile unsigned int Left_m1, Left_count, Left_count0 = 0;   // rotary encoder pulse
volatile float Left_Wr, Right_Wr, Left_Werr, Right_Werr, Left_Werr0, Right_Werr0 = 0;            // motor speed
volatile float PIconstant, Left_PI, Right_PI=0;
volatile unsigned int Left_PWM, Right_PWM;
volatile float Wref=200;
volatile char command = 'S';

volatile unsigned char Wr_time = 0;      // Wr display time counter
volatile unsigned char Ts_time = 0;      // Ts = 0.002s time counter


ISR(TIMER0_COMP_vect)            /* OC2 interrupt function(0.01s period) */
{
   Ts_time++;
   if(Ts_time>=5){
   		Ts_time=0;
      	Left_count = TCNT1;            // read TCNT1
      	Right_count = TCNT2;            // read TCNT3
   
      	Left_m1 = Left_count - Left_count0;            // calculate m1(rotary encoder pulse)
      	Left_count0 = Left_count;
      	Left_Wr = (float)Left_m1*60./750./0.01;         // calculate Wr(motor speed)
   
      	Right_m1 = Right_count - Right_count0;            // calculate m1(rotary encoder pulse)
      	Right_count0 = Right_count;
      	Right_Wr = (float)Right_m1*60./750./0.01;         // calculate Wr(motor speed)
   
      	Left_Werr = Wref - Left_Wr;
      	Left_PI = Left_PI + PIconstant*Left_Werr - Kp*Left_Werr0;
      	Left_Werr0 = Left_Werr;
      	if(Left_PI > 799) Left_PI=799;
		else if(Left_PI < 0.) Left_PI=0.;
      
      
      	Right_Werr = Wref - Right_Wr;
      	Right_PI = Right_PI + PIconstant*Right_Werr - Kp*Right_Werr0;
      	Right_Werr0 = Right_Werr;
      	if(Right_PI > 799) Right_PI=799;
      	else if(Right_PI < 0.) Right_PI=0.;
      
      	Wr_time++;
   }
}
void Go_Straight(){
   PORTC = 0xA0;
   OCR3A=Left_PI;
   OCR3B=Right_PI;
}
void Go_Back(){
   PORTC = 0x50;
   OCR3A=Left_PI;
   OCR3B=Right_PI;
}
void Turn_Left(){
   PORTC = 0xE0;
   OCR3B=Right_PI;
}
void Turn_Right(){
   PORTC = 0xB0;
   OCR3A=Left_PI;
}

void putch(unsigned char data){
   while((UCSR0A&0x20)==0);

   UDR0=data;
   UCSR0A|=0x20;
}
void UART0_transmit(char data) {
   while (!(UCSR0A & (1 << UDRE0)));  // 버퍼가 비어있을 때까지 대기
   UDR0 = data;
}
void UART0_print(const char *str) {
   while (*str) {
      UART0_transmit(*str++);
   }
}
void UART0_print_number(int num) {
   char buffer[10];  // 충분한 크기의 버퍼
   itoa(num, buffer, 10);  // 정수를 문자열로 변환
   UART0_print(buffer);
}
int main(void)
{
   
   DDRE=0x1A;
   UCSR0A = 0x00;
   UCSR0B = 0x18;
   UCSR0C = 0x06;
   UBRR0H = 0x00;
   UBRR0L = 0x08;
   
   DDRC|=0x0f;
   DDRB|=0x90;
   DDRD=0x00;
   
   TCCR3A = 0xA2;            // Timer3 = fast PWM mode(14), output OC3A,B
   TCCR3B = 0x19;            // 16MHz/(1+799) = 20kHz
   TCCR3C = 0x00;
   ICR3 = 799;
   OCR3A = 0;
   OCR3B = 0;
   
   TCCR0=0x0D;
   OCR0=249;
   
   TIFR=0x02;
   TIMSK=0x02;
   sei();                     

   TCCR1A=0x00;
   TCCR1B=0x06;
   TCCR1C=0x00;
   TCNT1=0x0000;

   TCCR2=0x06;
   TCNT2=0x00;                           // clear Timer/Counter2
   
   PIconstant = Kp+ Ki*Ts;
	LCD_init();
	_delay_ms(1000);
   
    /* Replace with your application code */
    while (1) 
    {

      Go_Straight();
      //_delay_ms(5000);
 /*    
      Go_Back();
      _delay_ms(5000);
      
      Turn_Left();
      _delay_ms(5000);
      
      Turn_Right();
      _delay_ms(5000);
*/   

    UART0_print_number(Left_m1);
    UART0_print("  ");
    UART0_print_number(Left_Wr);
    UART0_print("  ");
    UART0_print_number(Right_m1);
    UART0_print("  ");
    UART0_print_number(Right_Wr);
    UART0_print("\r\n\v");

	LCD_clear();
	LCD_goto_XY(0,0);
	LCD_write_data(command);
	_delay_ms(5);

    }
   return 0;
}
