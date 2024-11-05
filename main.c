#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

uint8_t message = 0x00;
uint8_t PORTF_Interrupt = 0x00;
void GPIOInterruptHandler();

int main(void)
{
    SYSCTL_RCGC2_R |= 0x00000020;       /* enable clock to GPIOF */
    //PORTF_REGISTERS
    GPIO_PORTF_LOCK_R = 0x4C4F434B;     /* unlock commit register */
    GPIO_PORTF_CR_R = 0x1F;             /* make PORTF configurable */
    GPIO_PORTF_DEN_R = 0x1F;            /* set PORTF pins 0 to pins 4 */
    GPIO_PORTF_DIR_R = 0x0E;            
    GPIO_PORTF_PUR_R = 0x11;            /* PORTF0 and PORTF4 are pulled up */
    NVIC_EN0_R = 0x40000000; // 30th bit controls PORTF
    GPIO_PORTF_IS_R = 0x00; // Set interrupt trigger type to edge-sensitive
    GPIO_PORTF_IEV_R = 0x00; // Set interrupt to trigger on falling edge
    GPIO_PORTF_IM_R = 0x10; // Enable interrupt for pin 4
    SYSCTL_RCGCUART_R |= 0x01; // enabling clock to UART module 0
    SYSCTL_RCGCGPIO_R |= 0x21; // enabling clock to PORTF, A
    GPIO_PORTA_LOCK_R = 0x4C4F434B;     /* unlock commit register */
    GPIO_PORTA_CR_R = 0x03;             /* make PORTF configurable */
    GPIO_PORTA_DEN_R = 0x03;
    GPIO_PORTA_AFSEL_R = 0x03; // UART on pins A0 and A1
    GPIO_PORTA_PCTL_R = 0x11; // Configure A0 and A1 for UART functions (Rx and Tx)
    GPIO_PORTA_DIR_R = 0x02;
    GPIO_PORTA_PUR_R = 0x02;

    UART0_IBRD_R = 104;
    UART0_FBRD_R = 11;
    UART0_LCRH_R |= 0x62;
    UART0_CC_R = 0x00;
    UART0_CTL_R |= 0x01; // enabling UART0
    uint8_t rx_reg = 0x00;
    while(1){
        NVIC_EN0_R = 0x40000000; // 30th bit controls PORTF GPIO interrupts
        GPIO_PORTF_IM_R = 0x11; // unmasking both switches
        UART0_DR_R = message; // Send the message via UART
        while (UART0_FR_R & 0x08){
            ; // wait till transmission is complete
        }
        rx_reg = UART0_DR_R & 0xFFF; // read LSB
        if ((rx_reg & 0xFF) == 0x47){
            GPIO_PORTF_DATA_R &= 0x08; //green
            GPIO_PORTF_DATA_R ^= 0x08;
            message = 0x47;
            delay(500000);
        }
        else if ((rx_reg & 0xFF) == 0x52){
            GPIO_PORTF_DATA_R &= 0x02; //red
            GPIO_PORTF_DATA_R ^= 0x02;
            message = 0x52;
            delay(500000);
        }
        else if ((rx_reg & 0xFF) == 0x42){
            GPIO_PORTF_DATA_R &= 0x04; //blue
            GPIO_PORTF_DATA_R ^= 0x04;
            message = 0x42;
            delay(500000);
        }
        else { // error bits 
            GPIO_PORTF_DATA_R &= 0x00;
            message = 0x21; // !
        }
    }
    return 0;
}
void delay(int us){
    NVIC_ST_RELOAD_R = 16*us;
    NVIC_ST_CTRL_R = 0x00000005;
    while( (NVIC_ST_CTRL_R & 0x00010000) != 0x00010000 ){;} 
    NVIC_ST_CTRL_R = 0x00000000;
}

void GPIOInterruptHandler(){
    PORTF_Interrupt = GPIO_PORTF_RIS_R & 0x11; // Check which switch caused the interrupt
    NVIC_EN0_R = 0x00000000; // 30th bit controls PORTF
    GPIO_PORTF_IM_R = 0x00; // Mask the interrupts for both switches
    if (PORTF_Interrupt == 0x01){ // If switch 0 was pressed, reduce brightness
        GPIO_PORTF_ICR_R = 0x11; // Clear interrupt status for edge-triggered interrupt
        message = 0xAA;
    }
    if (PORTF_Interrupt == 0x10){ // If switch 4 was pressed, reduce brightness
        GPIO_PORTF_ICR_R = 0x11; // Clear interrupt status
        message = 0xF0;
    }
}
