#define F_CPU 16000000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile unsigned int Left_m1, Right_m1, Left_count, Left_count0, Right_count, Right_count0 = 0;   // rotary encoder pulse
volatile float Left_Wr, Right_Wr;            // motor speed
volatile unsigned char Wr_time = 0;      // Wr display time counter
volatile unsigned char Ts_time = 0;      // Ts = 0.002s time counter


ISR(TIMER0_COMP_vect)            /* OC1A interrupt function(0.01s period) */
{
   Left_count = TCNT1;            // read TCNT1
   Right_count = TCNT2;            // read TCNT2
   
   Left_m1 = Left_count - Left_count0;            // calculate m1(rotary encoder pulse)
   Left_count0 = Left_count;
   Left_Wr = (float)Left_m1*60./740./0.01;         // calculate Wr(motor speed)
   
   Right_m1 = Right_count - Right_count0;            // calculate m1(rotary encoder pulse)
   Right_count0 = Right_count;
   Right_Wr = (float)Right_m1*60./740./0.01;         // calculate Wr(motor speed)
   
   Wr_time++;
}
void Go_Straight(){
   PORTC=0xA0;
   OCR0=255;
}
void Go_Back(){
   PORTC=0x50;
   OCR0=200;
}
void Turn_Left(){
   PORTC=0xE0;
   OCR0=200;
}
void Turn_Right(){
   PORTC=0xB0;
   OCR0=200;
}

int main(void)
{
   UCSR0A = 0x00;
   UCSR0B = 0x18;
   UCSR0C = 0x06;
   UBRR0H = 0x00;
   UBRR0L = 0x08;

   DDRE=0x1A;
   DDRC|=0x0f;
   DDRB|=0x90;
   DDRD=0x00;
   //7DDRE|=0x04;
   
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
   TCNT2=0x00;                           // clear Timer/Counter

   
    /* Replace with your application code */
    while (1) 
    {
      Go_Straight();
      _delay_ms(5000);

      Go_Back();
      _delay_ms(5000);
      
      Turn_Left();
      _delay_ms(5000);
      
      Turn_Right();
      _delay_ms(5000);
   
    }
   return 0;
}


