/*
 * Proyecto 2.c
 *
 * Created: 15/05/2024 15:50:09
 * Author : Paul Mateo Castañeda Paredes
 * IE2023 Programación de Microcontroladores
 * Universidad del Valle de Guatemala
 */ 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void setup(void);
void Init_T0(void);
void Init_T1(void);
void Init_T2(void);
void Init_ADC(void);
void W_Epr(void);
void R_Epr(void);



uint8_t MASK = 0x0F;
uint8_t MASK_Estado = 0b00001100;
uint8_t Comp_ADC;
uint8_t Dato_ADC;
uint8_t Estado = 1;
uint8_t Servo_Cuello;
uint8_t Servo_Cuello_1;
uint8_t Servo_Brazos;
uint8_t Servo_Brazo2;
uint8_t Servo_Brazo2_1;
uint8_t Servo_Brazos_1;
uint8_t DC;
uint8_t DC_1;
uint8_t* Dir = 0; 
uint8_t Dir_1 =0;
uint8_t Serv_C;
uint8_t Serv_B;
uint8_t Serv_B1;
uint8_t DC_V;
uint8_t Cnt_Posiciones = 1;
uint8_t Cnt_Posiciones_R = 1;

uint8_t mapeo(uint8_t x){
	return ((uint16_t)((x)*(12))/(256))+4;
}

int main(void)
{
	cli();

	setup();
	sei();
    while (1) 
    {
		ADCSRA |= (1<<ADSC);
		_delay_ms(15);
	}
}

void setup(void){
	CLKPR = (1<<CLKPCE);
	CLKPR = (1<<CLKPS0);
	DDRC = 0;
	DDRB |= (1<<DDB3) | (1<<DDB1) | (1<<DDB4) | (1<<DDB2);
	DDRD |= (1<<DDD6) | (1<<DDD3) | (1<<DDD2) | (1<<DDD5);
	PORTD = 0;
	PORTC |= (1<<PORTC4) | (1<<PORTC5);
	PORTB |= (1<<PORTB5);
	PCICR |= (1<<PCIE0) | (1<<PCIE1);
	PCMSK0 |= (1<<PCINT5);
	PCMSK1 |= (1<<PCINT12) | (1<<PCINT13);
	PORTD |= (1<<PORTD2);
	TIMSK1 |= (1<<OCIE1A);
	TIMSK0 |= (1<<OCIE0A) | (1<<OCIE0B);
	TIMSK2 |= (1<< OCIE2A);
		Init_T0();
		Init_T1();
		Init_T2();
		Init_ADC();

}

void Init_T0(void){
	TCCR0A |= (1<<COM0A1) | (1<<WGM01) | (1<<WGM00) | (1<<COM0B1);
	TCCR0B |= (1<<CS00) | (1<<CS02);
	TCNT0 = 0;
	OCR0A = 0;
	OCR0B = 0;
	
	
}
void Init_T1(void){
	TCCR1A |= (1<<COM1A1) | (1<<WGM10) | (1<<COM1B1);;
	TCCR1B |= (1<<WGM12) | (1<<CS10) | (1<<CS12);
	TCNT1 = 0;
	OCR1A = 0;	
	OCR1B = 0;
	
}

void Init_T2(void){
	TCCR2A |= (1<< COM2A1) | (1<<WGM20) | (1<<WGM21);
	TCCR2B |= (1<< CS20) | (1<<CS21) | (1<< CS22); 
	TCNT2 = 0;
	OCR2A = 0;
	OCR2B = 0;
	
}

void Init_ADC(void){
	ADMUX |= (1<<REFS0) | (1<<ADLAR);
	ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADIE);
	ADCSRB = 0x00;
	DIDR0 |= (1<<ADC5D) | (1<<ADC4D);
	
}


void W_Epr(void){
	if (Cnt_Posiciones == 1){
	eeprom_write_byte((uint8_t*) Dir, Serv_C);
	Dir++;
	eeprom_write_byte((uint8_t*) Dir, Serv_B1);
	Dir++;
	eeprom_write_byte((uint8_t*) Dir,  Serv_B);
	Dir++;
	eeprom_write_byte((uint8_t*) Dir,  DC_V);
		
	}
	else if (Cnt_Posiciones == 2){
		Dir = 0;
		for (Dir; Dir <= 4; Dir++){
			
		}
	eeprom_write_byte((uint8_t*) Dir,  Serv_C);
		Dir++;
		eeprom_write_byte((uint8_t*) Dir,  Serv_B1);
		Dir++;
		eeprom_write_byte((uint8_t*) Dir, Serv_B);
		Dir++;
		eeprom_write_byte((uint8_t*) Dir,  DC_V);
	}
	else if (Cnt_Posiciones == 3){
		Dir = 0;
		for (Dir; Dir <= 8; Dir++){
			
		}
		eeprom_write_byte((uint8_t*) Dir,  Serv_C);
		Dir++;
		eeprom_write_byte((uint8_t*) Dir,  Serv_B1);
		Dir++;
		eeprom_write_byte((uint8_t*) Dir,  Serv_B);
		Dir++;
		eeprom_write_byte((uint8_t*) Dir, DC_V);
		}
	else if (Cnt_Posiciones == 4){
		Dir = 0;
		for (Dir; Dir <= 12; Dir++){
			
		}
		eeprom_write_byte((uint8_t*) Dir,  Serv_C);
		Dir++;
		eeprom_write_byte((uint8_t*) Dir,  Serv_B1);
		Dir++;
		eeprom_write_byte((uint8_t*) Dir,  Serv_B);
		Dir++;
		eeprom_write_byte((uint8_t*) Dir,  DC_V);
	}

}

void R_Epr(void){
	if (Cnt_Posiciones_R == 1){
	Dir = 0;
	Serv_C = eeprom_read_byte((uint8_t*) Dir);
	Dir++;
	Serv_B1 = eeprom_read_byte((uint8_t*) Dir);
	Dir++;
	Serv_B = eeprom_read_byte((uint8_t*) Dir);
	Dir++;
	DC_V = eeprom_read_byte((uint8_t*) Dir);
	}
		if (Cnt_Posiciones_R == 2){
	Dir = 0;
	for (Dir; Dir <=4; Dir++){
		
	}
	Serv_C = eeprom_read_byte((uint8_t*) Dir);
	Dir++;
	Serv_B1 = eeprom_read_byte((uint8_t*) Dir);
	Dir++;
	Serv_B = eeprom_read_byte((uint8_t*) Dir);
	Dir++;
	DC_V = eeprom_read_byte((uint8_t*) Dir);
	}
		if (Cnt_Posiciones_R == 3){
	Dir = 0;
	for (Dir; Dir <=8; Dir++){
		
	}
	Serv_C = eeprom_read_byte((uint8_t*) Dir);
	Dir++;
	Serv_B1 = eeprom_read_byte((uint8_t*) Dir);
	Dir++;
	Serv_B = eeprom_read_byte((uint8_t*) Dir);
	Dir++;
	DC_V = eeprom_read_byte((uint8_t*) Dir);
	}
		if (Cnt_Posiciones_R == 4){
	Dir = 0;
	for (Dir; Dir <=12; Dir++){
		
	}
	Serv_C = eeprom_read_byte((uint8_t*) Dir);
	Dir++;
	Serv_B1 = eeprom_read_byte((uint8_t*) Dir);
	Dir++;
	Serv_B = eeprom_read_byte((uint8_t*) Dir);
	Dir++;
	DC_V = eeprom_read_byte((uint8_t*) Dir);
	}
	
}


ISR(ADC_vect){
	Comp_ADC = (ADMUX & MASK);
	Dato_ADC = ADCH;
	
	if (Comp_ADC == 0x00){
		ADMUX = 0b01100001;
		Servo_Cuello = Dato_ADC;
		Servo_Cuello_1 = mapeo(Servo_Cuello);
	}
	else if (Comp_ADC == 0b00000001){
		ADMUX = 0b01100010;
		Servo_Brazos = Dato_ADC;
		Servo_Brazos_1 = mapeo(Servo_Brazos);
	}
	else if (Comp_ADC == 0b00000010){
		ADMUX = 0b01100011;
		Servo_Brazo2 = Dato_ADC;
		Servo_Brazo2_1 = mapeo(Servo_Brazo2);
	
	}
	else if (Comp_ADC == 0b00000011){
		ADMUX = 0b01100000;
		DC = Dato_ADC;
		DC_1 = DC;
	}
	
}
	
ISR(PCINT0_vect){ 
 _delay_ms(50);
if (!(PINB & (1<<PINB5)) && Estado == 1){
	Cnt_Posiciones++;
	
	if (Cnt_Posiciones > 4)
	{
	Cnt_Posiciones = 0;
	}
	else 
	{
			Serv_C = Servo_Cuello_1;
			Serv_B = Servo_Brazos_1;
			Serv_B1 = Servo_Brazo2_1;
			DC_V = DC_1;
			W_Epr();
	}
}
else if (!(PINB & (1<<PINB5)) && Estado == 2){
	Cnt_Posiciones_R++;
	if (Cnt_Posiciones_R > 4){
		Cnt_Posiciones_R = 0;
	}
	else {
		R_Epr();
	}
}
}

ISR (PCINT1_vect){
	_delay_ms(50);
if (!(PINC &(1<<PINC4))){
	if (((PIND & (1<<PIND2))) && (!(PIND & (1<<PIND3))))
	{
		PORTD &= ~(1<<PORTD2);
		PORTD |= (1<<PORTD3);
		Estado = 2;
	}
	else if ((!(PIND & (1<<PIND2))) && ((PIND & (1<<PIND3))))
	{
		PORTD |= (1<<PORTD3);
		PORTD |= (1<<PORTD2);
		Estado = 3;
	}
	else if (((PIND & (1<<PIND2))) && ((PIND & (1<<PIND3))))
	{
		PORTD &= ~(1<<PORTD3);
		PORTD |= (1<<PORTD2);
		Estado = 1;
	}
	
}
}


ISR (TIMER0_COMPA_vect){
	 if (Estado == 1){
	 OCR0A = Servo_Brazos_1; 
	 }
	 else if (Estado == 2){
	 OCR0A = Serv_B;
	 }
}
ISR (TIMER1_COMPA_vect){
	 if (Estado == 1){
	OCR1A = Servo_Cuello_1;
	 }
	 	 else if (Estado == 2){
	 	 OCR1A = Serv_C;  
		  }
}

ISR (TIMER2_COMPA_vect){
	 if (Estado == 1){
	OCR2A = Servo_Brazo2_1;
	 }
	 else if (Estado == 2){
		 OCR2A = Serv_B1;
	 }
}

ISR (TIMER0_COMPB_vect){
	if (Estado == 1){
		OCR0B = DC_1;
	}
	else if (Estado == 2) {
	OCR0B = DC_V;
	}
}