#include <stdint.h>
#include "stm32f4xx.h"

#define COLS 3

void init(void);
void setMode(GPIO_RegDef_t *GPIO, uint8_t pin, uint8_t mode);
void setAF(GPIO_RegDef_t *GPIO, uint8_t pin, uint8_t AF_num);
void setPullRes(GPIO_RegDef_t *GPIO, uint8_t pin, uint8_t pull_type);
void servo_set_angle(uint8_t angle);
void test(void);

//Common anode
uint8_t seven_seg[] = {0B11000000, 0B11111001, 0B10100100, 0B10110000, 0B10011001, 0B10010010, 0B10000010, 0B11111000, 
	0B10000000, 0B10011000};

uint8_t volatile row = 0;
uint8_t volatile col = 0;
uint8_t volatile keypad_val;
uint8_t volatile keypad_val_units = 10;
uint8_t volatile keypad_val_tens = 10;
uint8_t volatile keypad_val_hunds = 10;
uint8_t volatile temp_unit = 0;
uint8_t volatile temp_ten = 0;
uint8_t volatile temp_hund = 0;
uint8_t volatile reset = 0;
uint8_t volatile temp1 = 0;

int main(void)
{
	init();	

	uint16_t last_angle = 0;

	for(;;) {
		GPIOD->ODR = seven_seg[keypad_val_units == 10 ? 1 : keypad_val_units];
		GPIOD->ODR |= (1 << 8);
		for(uint32_t i = 0; i < 4000; i++);

		GPIOD->ODR = seven_seg[keypad_val_tens == 10 ? 0 : keypad_val_tens];
		GPIOD->ODR |= (1 << 9);
		for(uint32_t i = 0; i < 4000; i++);

		GPIOD->ODR = seven_seg[keypad_val_hunds == 10 ? 0 : keypad_val_hunds];
		GPIOD->ODR |= (1 << 10);
		for(uint32_t i = 0; i < 4000; i++);

		// --- Servo ---
		uint16_t angle = keypad_val_units + keypad_val_tens*10 + keypad_val_hunds*100;
		if(angle != last_angle) {
			servo_set_angle(angle);   // only update when angle changes
			last_angle = angle;
		}
	}
}

void init(void) {
	// Enabling the clock for PORTA, PORTB, PORTD and PORTE 
	RCC->AHB1ENR |= (1 << 0) | (1 << 1) | (1 << 3) | (1 << 4);
	GPIOA->ODR = 0;

	// Enabling the clock for TIM2
	RCC->APB1ENR |= (1 << 0);
	
	// Enable SYSCFG clock
	RCC->APB2ENR |= (1 << 14);

	// Enabling interrupts for the pins of the keypad, and the resume button

	// Initializing
	EXTI->PR = 0xffffffff;
	SYSCFG->EXTICR[0] = 0;
	SYSCFG->EXTICR[1] = 0;
	SYSCFG->EXTICR[2] = 0;

	NVIC->ISER[0] |= (1 << 6);	// EXTI0 for the resume button, the desired pin is RA0
	SYSCFG->EXTICR[0] &= ~(0x0f);

	NVIC->ISER[0] |= (1 << 23);	// EXTI5-9 the desired pins are RE6, 7, 8, and 9
	SYSCFG->EXTICR[1] |= (0x4 << 8);	//EXTI6 is connected to RE6
	SYSCFG->EXTICR[1] |= (0x4 << 12);	//EXTI7 is connected to RE7
	SYSCFG->EXTICR[2] |= (0x4 << 0);	//EXTI8 is connected to RE8
	SYSCFG->EXTICR[2] |= (0x4 << 4);	//EXTI9 is connected to RE9

	EXTI->FTSR |= (0x0f << 6) |	(7 << 11) | (1 << 0);		//On falling edge interrupt
	EXTI->IMR |= (0x0f << 6) | (7 << 11) |(1 << 0);		//Enabling the interrupts

	// RD0-RD6 output pins to seven segments, RD8-RD10 for multiplexing
	setMode(GPIOD, 0, OUTPUT);
	setMode(GPIOD, 1, OUTPUT);
	setMode(GPIOD, 2, OUTPUT);
	setMode(GPIOD, 3, OUTPUT);
	setMode(GPIOD, 4, OUTPUT);
	setMode(GPIOD, 5, OUTPUT);
	setMode(GPIOD, 6, OUTPUT);

	setMode(GPIOD, 8, OUTPUT);
	setMode(GPIOD, 9, OUTPUT);
	setMode(GPIOD, 10, OUTPUT);

	// RA0 as input for reset/resume button, and set pull up 
	setMode(GPIOA, 0, INPUT);
	setPullRes(GPIOA, 0, PULL_UP);

	// RA11 for testing
	setMode(GPIOA, 11, OUTPUT);

	// RB10 for PWM signal for servo-motor
	setMode(GPIOB, 10, AF);
	setAF(GPIOB, 10, 1);

	// RE7-RE13 pins for keypad, RE6, RE7, RE8 and RE9 for rows, RE11, RE12, RE13 for columns and setting pull ups for all of them
	setMode(GPIOE, 6, INPUT); 
	setMode(GPIOE, 7, INPUT); 
	setMode(GPIOE, 8, INPUT); 
	setMode(GPIOE, 9, INPUT); 

	setMode(GPIOE, 11, OUTPUT); 
	setMode(GPIOE, 12, OUTPUT); 
	setMode(GPIOE, 13, OUTPUT);
	
	setPullRes(GPIOE, 6, PULL_UP);
	setPullRes(GPIOE, 7, PULL_UP);
	setPullRes(GPIOE, 8, PULL_UP);
	setPullRes(GPIOE, 9, PULL_UP);

	setPullRes(GPIOE, 11, PULL_UP);
	setPullRes(GPIOE, 12, PULL_UP);
	setPullRes(GPIOE, 13, PULL_UP);

	GPIOE->ODR = 0;

	// TIM2 configuration for PWM
	TIM2->PSC = 1599;     // Pre-scaler: 16 MHz / 1600 = 10 kHz
	TIM2->ARR = 199;      // Auto-reload: 10 kHz / (199+1) = 50 Hz
	TIM2->CCR3 = 15;      // Initial duty (1.5 ms pulse) = neutral position

	TIM2->CCMR2 &= ~(0xFF);       // Clear channel 3/4 mode bits
	TIM2->CCMR2 |= (6 << 4);      // PWM mode 1 on CH3 (OC3M = 110)
	TIM2->CCMR2 |= (1 << 3);      // Enable pre-load for CCR3 (OC3PE)

	TIM2->CCER |= (1 << 8);       // Enable output for CH3 (CC3E)
	TIM2->CR1 |= (1 << 7);        // Auto-reload pre-load enable (ARPE)
	TIM2->EGR |= (1 << 0);        // Generate update event to load registers
	TIM2->CR1 |= (1 << 0);        // Enable counter
}

void EXTI0_IRQHandler(void) {
	for (int i = 0; i < 160000; i++); //De-bouncing

	if(reset == 0) {
		temp_unit = keypad_val_units;
		temp_ten = keypad_val_tens;
		temp_hund = keypad_val_hunds;

		keypad_val_units = 0;
		keypad_val_tens = 0;
		keypad_val_hunds = 0;

		reset = 1;
	}
	else {
		keypad_val_units = temp_unit;
		keypad_val_tens = temp_ten;
		keypad_val_hunds = temp_hund;
		reset = 0;
	}

	EXTI->PR |= (1 << 0);	// Clearing the flag
}
void EXTI9_5_IRQHandler(void) {
	// De-bounce (software loop)
	for (volatile int i = 0; i < 160000; i++);

	if(reset != 0) {
		EXTI->PR = (0x0f << 6);
		return;
	}

	if(keypad_val_hunds != 10) {
		keypad_val_units = 10;
		keypad_val_tens = 10;
		keypad_val_hunds = 10;
	}

	uint8_t keypad[][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {0, 0, 0}};

	test();

	// Identify which row triggered
	if (EXTI->PR & (1 << 6)) row = 0;
	else if (EXTI->PR & (1 << 7)) row = 1;
	else if (EXTI->PR & (1 << 8)) row = 2;
	else if (EXTI->PR & (1 << 9)) row = 3;

	for(int i = 0; i < COLS; i++) {
		GPIOE->ODR |= (1 << (11 + i));
		if((GPIOE->IDR >> (6 + row)) & 1) {
			col = i;
			GPIOE->ODR &= ~(1 << (11 + i));
			break;
		}
	}
	EXTI->PR = (0x0f << 6);

	if(keypad_val_units == 10) {
		keypad_val_units = keypad[row][col];
	} else if (keypad_val_tens == 10) {
		keypad_val_tens = keypad_val_units;
		keypad_val_units = keypad[row][col];
	} else {
		keypad_val_hunds = keypad_val_tens;
		keypad_val_tens = keypad_val_units;
		keypad_val_units = keypad[row][col];
	}
}

void setMode(GPIO_RegDef_t *GPIO, uint8_t pin, uint8_t mode) {
	GPIO->MODER &= ~(3 << (pin*2));
	GPIO->MODER |= (mode << (pin*2));
}

void setAF(GPIO_RegDef_t *GPIO, uint8_t pin, uint8_t AF_num) {
	AF_num &= (0xf);

	if(pin < 8) {
		GPIO->AFRL &= ~(0x0f << (pin*4));
		GPIO->AFRL |= (AF_num << (pin*4));
	} else {
		GPIO->AFRH &= ~(0x0f << ((pin - 8)*4));
		GPIO->AFRH |= (AF_num << ((pin - 8)*4));
	}
}

void setPullRes(GPIO_RegDef_t *GPIO, uint8_t pin, uint8_t pull_type) {
	GPIO->PUPDR &= ~(3 << (pin*2));
	GPIO->PUPDR |= (pull_type << (pin*2));
}

void servo_set_angle(uint8_t angle) {
    TIM2->CCR3 = 10 + (angle * 10) / 180;
}

void test(void) {
	if(temp1 % 2 == 0) {
		GPIOA->ODR |= (1 << 11);
		for(int i = 0; i < 40000; i++);
		temp1++;
	} else {
		GPIOA->ODR &= ~(1 << 11);
		for(int i = 0; i < 40000; i++);
		temp1++;
	}
}
