#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <math.h>
#include "LCD_16x2.h"

#define KEY_PRT 	PORTC
#define KEY_DDR		DDRC
#define KEY_PIN		PINC

volatile unsigned char colloc, rowloc;

volatile double kp = 0.093018633540374, ki = 0.004770186335404, kd = 0.477018633540379;
//volatile double kp = 1, ki = 0.1, kd = 0.01;
//volatile double kp = 5.301863354, ki = 2.0522, kd = 3.42434;
volatile double p, i, d, err = 0, PID_value = 0, err_TP = 0, err_VP = 0, pre_err = 0;
volatile double nhiet_HT, nhiet_lcd;

unsigned int keyfind()
{
	unsigned int keypad[4][4] = {{7,8,9,0},
	                             {4,5,6,0},
	                             {1,2,3,0},
	                             {0,0,0,0}};
	while(1)
	{
		KEY_DDR = 0xF0;           /* set psort direction as input-output */
		KEY_PRT = 0xFF;

		do
		{
			KEY_PRT &= 0x0F;      /* mask PORT for column read only */
			asm("NOP");
			colloc = (KEY_PIN & 0x0F); /* read status of column */
		}while(colloc != 0x0F);
		
		do
		{
			do
			{
				_delay_ms(20);             /* 20ms key debounce time */
				colloc = (KEY_PIN & 0x0F); /* read status of column */
				}while(colloc == 0x0F);        /* check for any key press */
				
				_delay_ms (40);	            /* 20 ms key debounce time */
				colloc = (KEY_PIN & 0x0F);
			}while(colloc == 0x0F);

			/* now check for rows */
			KEY_PRT = 0xEF;            /* check for pressed key in 1st row */
			asm("NOP");
			colloc = (KEY_PIN & 0x0F);
			if(colloc != 0x0F)
			{
				rowloc = 0;
				break;
			}

			KEY_PRT = 0xDF;		/* check for pressed key in 2nd row */
			asm("NOP");
			colloc = (KEY_PIN & 0x0F);
			if(colloc != 0x0F)
			{
				rowloc = 1;
				break;
			}
			
			KEY_PRT = 0xBF;		/* check for pressed key in 3rd row */
			asm("NOP");
			colloc = (KEY_PIN & 0x0F);
			if(colloc != 0x0F)
			{
				rowloc = 2;
				break;
			}

			KEY_PRT = 0x7F;		/* check for pressed key in 4th row */
			asm("NOP");
			colloc = (KEY_PIN & 0x0F);
			if(colloc != 0x0F)
			{
				rowloc = 3;
				break;
			}
		}

		if(colloc == 0x0E)
		{
			return(keypad[rowloc][0]);
		}
		else if(colloc == 0x0D)
		{
			return(keypad[rowloc][1]);
		}
		else if(colloc == 0x0B)
		{
			return(keypad[rowloc][2]);
		}
		else
		{
			return(keypad[rowloc][3]);
		}
	}

	void PID_control(unsigned int des_temp)
	{
		nhiet_HT = nhiet_lcd;
		err = des_temp - nhiet_HT;
		p = err*kp;
		err_TP = err_TP + err;
		if(err_TP > 50)
		err_TP = 50;
		else if(err_TP < -50)
		err_TP = -50;
		i = ki*err_TP;
		err_VP = err - pre_err;
		pre_err = err;
		d = kd*err_VP;
		PID_value = 255*(p+i+d);
		if(PID_value > 254)
		PID_value = 254;
		else if(PID_value < 0)
		PID_value = 0;
	}

	void ADC_init()
	{
		ADCSRA |= (1<<ADEN) | (1<<ADPS2)| (1<<ADPS1)| (1<<ADPS0); //kich hoat ADC, F = F_PCU/128
		ADMUX |= (1<<REFS0);
	}
	unsigned int ADC_read(char channel)
	{
		ADMUX = ADMUX|(channel & 0x07); //chon AVcc, kenh 2
		ADCSRA |= (1<<ADSC);
		while ( (ADCSRA&(1<<ADIF)) == 0);
		return(ADCW);
	}


	int main(void)
	{
		DDRD |= (1<<7);
		unsigned int a,b,c,d,d2,high,period;
		char frequency[14], duty_cy[7], temp[5], PID[10], x[10];
		LCD_Init();
		TCCR2 |= (1<<WGM20) | (1<<WGM21) | (1<<COM21) | (1<<CS22); //Fast PWM, non-inverted, pre = 64
		ADC_init();
		sei();
		//d = keyfind();
		while (1)
		{
			//unsigned int a,b,c,d,high,period;
			//char frequency[14], duty_cy[7], temp[5], PID[10], x[10];
			nhiet_lcd = (double)(ADC_read(2)*276.50/1023.00);
			//d = keyfind();
			//PID_control(10*d);
			OCR2 = (int)PID_value;
			//itoa(d,x,10);
			
			TCCR1A = 0;
			TCNT1 = 0;
			TIFR = (1<<ICF1);

			TCCR1B = 0x41;
			while ((TIFR&(1<<ICF1)) == 0);
			a = ICR1;
			TIFR = (1<<ICF1);
			
			TCCR1B = 0x01;
			while ((TIFR&(1<<ICF1)) == 0);
			b = ICR1;
			TIFR = (1<<ICF1);
			
			TCCR1B = 0x41;
			while ((TIFR&(1<<ICF1)) == 0);
			c = ICR1;
			TIFR = (1<<ICF1);
			
			TCCR1B = 0;
			
			if(a<b && b<c)  	// Check for valid condition, to avoid timer overflow reading
			{
				high=b-a;
				period=c-a;
				double freq = F_CPU/period;/* Calculate frequency */
				/* Calculate duty cycle */
				double duty_cycle =((double)high/(double)period)*100;
				
				
				dtostrf(PID_value, 4, 0, PID);
				dtostrf(nhiet_lcd, 4, 0, temp);
				ltoa((long)freq,frequency,10);
				itoa((int)duty_cycle,duty_cy,10);
				
				LCD_Command(0x80);
				LCD_String("Last Temp: ");
				LCD_String(temp);
				LCD_String(" *C");
				
				LCD_Command(0xC0);
				LCD_String("Set Temp : ");
				d = keyfind();
				d2= d2*10;
				if (d2<1000)
				{
					if(d2<10)
					{
						d2=d;
					}
					else
					{
						d2= d2 + d;
					}
				}
				else
				{
					d2=d;
					LCD_Clear();
				}
				
				itoa(d2,x,10);
				LCD_String(x);
			}
			else
			{
				//LCD_Clear();
				LCD_String("OUT OF RANGE!!");
			}
			_delay_ms(50);
			}
		}
