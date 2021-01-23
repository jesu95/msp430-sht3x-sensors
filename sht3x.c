/*
 * sht3.c
 *
 *  Created on: 30/9/2019
 *      Author: Jesús
 */
#include "sht3x.h"

//**********************************************************************************************************************************************************
// Global variable used for I2C communication reception
static uint8_t RXData[6] = {0};                                      // Received data from sensor

//**********************************************************************************************************************************************************
void initSht3x()
{
    // Configure USCI_B0 for I2C mode - Sending
    UCB0CTLW0 |= UCSWRST;                                            // Software reset enabled
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSYNC | UCSSEL__SMCLK;          // I2C mode, master, sync, sending, SMCLK
    UCB0BRW = 10;                                                    // Baudrate = SMCLK / 10; Bit clock prescaler. Modify only when UCSWRST = 1.
    UCB0I2CSA = SHT3X_I2C_ADDRESS;                                   // I2C slave Address.  The I2CSAx bits contain the slave address of the
                                                                     // external device to be addressed by the eUSCIx_B module. It is only used
                                                                     // in master mode. The address is right justified. In 7-bit slave addressing
                                                                     // mode, bit 6 is the MSB and bits 9-7 are ignored. In 10-bit slave addressing mode, bit 9 is the MSB
    P5SEL0 |= BIT2 | BIT3;
    UCB0CTLW0 &=~ UCSWRST;                                           // Clear reset register
}

//**********************************************************************************************************************************************************
static void initWrite(void)
{
    UCB0CTLW0 |= UCTR;                         // UCTR=1 => Transmit Mode (R/W bit = 0)
    UCB0IFG &= ~UCTXIFG0;
    UCB0IE &= ~UCRXIE0;                        // disable Receive ready interrupt
    UCB0IE |= UCTXIE0 | UCSTTIE | UCSTPIE;     // enable Transmit ready interrupt
}
//**********************************************************************************************************************************************************
static void initRead(void)
{
    UCB0CTLW0 &= ~UCTR;                         // UCTR=0 => Receive Mode (R/W bit = 1)
    UCB0IFG &= ~(UCRXIFG0 | UCSTPIFG);
    UCB0IE &= ~(UCTXIE0 | UCSTTIE | UCSTPIE);   // disable Transmit ready interrupt
    UCB0IE |= UCRXIE0;                // enable Receive ready interrupt
}

//**********************************************************************************************************************************************************
void getMeasure(void)
{
    initWrite();
	UCB0CTLW0 |= UCTXSTT;                     // Send start. Transmit START condition in master mode.
	__bis_SR_register(LPM3_bits);             // Wait for TX interrupt flag
	UCB0TXBUF = SHT3X_MEASUREMENT_MSB;        // Send temperature command
	__bis_SR_register(LPM3_bits);
	UCB0TXBUF = SHT3X_MEASUREMENT_LSB;
	__bis_SR_register(LPM3_bits);
	UCB0CTLW0 |= UCTXSTP;
    __bis_SR_register(LPM3_bits);             // Ensure stop condition got sent

	initRead();                               // Change to receive
	UCB0CTLW0 |= UCTXSTT;
    UCB0CTLW0 |= UCTXSTP;                     // I2C stop condition
}

//**********************************************************************************************************************************************************
uint16_t getTemp(void)
{
	uint32_t temp = ((((uint16_t) RXData[0]) << 8 ) | ( (uint16_t) RXData[1]));

    temp *= 175;
    temp /= 65535;
    temp -= 45;

    return (temp);
}

//**********************************************************************************************************************************************************
uint16_t getHum(void)
{
    uint32_t hum = ((((uint16_t) RXData[3]) << 8 ) | ( (uint16_t) RXData[4]));

    hum *= 100;
    hum /= 65535;

    return (hum);
}

//**********************************************************************************************************************************************************
void showTempHum(uint16_t g_temp, uint16_t g_hum)
{
    uint16_t aux;
    // Temperature
    showChar('T',pos1);
    aux=(g_temp)/10;
    showChar(aux+48,pos2);
    aux=(g_temp)%10;
    showChar(aux+48,pos3);
    // Humidity
    showChar('H',pos4);
    aux=(g_hum)/10;
    showChar(aux+48,pos5);
    aux=(g_hum)%10;
    showChar(aux+48,pos6);
}

//**********************************************************************************************************************************************************
// I2C interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B0_VECTOR
__interrupt void USCIB0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) USCIB0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  static uint8_t i = 0;
  switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
  {
    case USCI_NONE:          break;                 // Vector 0: No interrupts
    case USCI_I2C_UCALIFG:   break;                 // Vector 2: ALIFG
    case USCI_I2C_UCNACKIFG: break;                 // Vector 4: NACKIFGy
    case USCI_I2C_UCSTTIFG:                         // Vector 6: STTIFG
    case USCI_I2C_UCSTPIFG:                         // Vector 8: STPIFG
        __bic_SR_register_on_exit(LPM3_bits);
        break;
    case USCI_I2C_UCRXIFG3:  break;                 // Vector 10: RXIFG3
    case USCI_I2C_UCTXIFG3:  break;                 // Vector 14: TXIFG3
    case USCI_I2C_UCRXIFG2:  break;                 // Vector 16: RXIFG2
    case USCI_I2C_UCTXIFG2:  break;                 // Vector 18: TXIFG2
    case USCI_I2C_UCRXIFG1:  break;                 // Vector 20: RXIFG1
    case USCI_I2C_UCTXIFG1:  break;                 // Vector 22: TXIFG1
    case USCI_I2C_UCRXIFG0:                         // Vector 24: RXIFG0
        RXData[i++] = UCB0RXBUF;                    // Get RX data
        if(i == 6)
            __bic_SR_register_on_exit(LPM3_bits);   // Exit LPM0
        break;
    case USCI_I2C_UCTXIFG0:                         // Vector 26: TXIFG0
        __bic_SR_register_on_exit(LPM3_bits);
        break;
    case USCI_I2C_UCBCNTIFG: break;                 // Vector 28: BCNTIFG
    case USCI_I2C_UCCLTOIFG: break;                 // Vector 30: clock low timeout
    case USCI_I2C_UCBIT9IFG: break;                 // Vector 32: 9th bit
    default: break;
  }
}
