#include <msp430.h>
#include "board.h"

#define I2C_PINS (BIT2 | BIT3)
#define WAIT_LIMIT 30000u

void board_delay_ms(uint16_t ms)
{
    while (ms--) __delay_cycles(1016); /* REFO * 31 = 1,015,808 Hz. */
}

uint8_t board_clock_init(void)
{
    uint16_t remaining = WAIT_LIMIT;
    __bis_SR_register(SCG0);
    CSCTL3 = SELREF__REFOCLK;
    CSCTL0 = 0;
    CSCTL1 = DCORSEL_0;
    CSCTL2 = FLLD_0 | 30;
    __delay_cycles(3);
    __bic_SR_register(SCG0);
    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;
    CSCTL5 = DIVM_0 | DIVS_0;
    while (CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)) {
        if (--remaining == 0) return 0;
    }
    return 1;
}

void board_pins_init(void)
{
    /* MAXEN -> P3.0 -> MAXM86161 LDO_EN. MAXEN1/P1.7 is not used. */
    P3SEL0 &= ~BIT0;
    P3SEL1 &= ~BIT0;
    P3OUT &= ~BIT0;
    P3DIR |= BIT0;
    P3REN &= ~BIT0;
    P1SEL0 &= ~BIT7;
    P1SEL1 &= ~BIT7;
    P1DIR &= ~BIT7;
    P1REN &= ~BIT7;
    /* INT (P2.2) and sensor GPIO (P2.3) stay inputs; INTB not shown wired. */
    P2SEL0 &= ~(BIT2 | BIT3);
    P2SEL1 &= ~(BIT2 | BIT3);
    P2DIR &= ~(BIT2 | BIT3);
    P2REN &= ~(BIT2 | BIT3);
    PM5CTL0 &= ~LOCKLPM5;
}

void board_sensor_enable(uint8_t on)
{
    if (on) P3OUT |= BIT0;
    else P3OUT &= ~BIT0;
}

static uint8_t scl_released(void)
{
    uint16_t n = 1000;
    while (!(P1IN & BIT3)) {
        if (--n == 0) return 0;
    }
    return 1;
}

maxm_result_t board_i2c_init(void)
{
    uint8_t i;
    UCB0CTLW0 = UCSWRST;
    /* Release lines as inputs. Never drive an I2C line high. External pulls required. */
    P1SEL0 &= ~I2C_PINS;
    P1SEL1 &= ~I2C_PINS;
    P1REN &= ~I2C_PINS;
    P1OUT &= ~I2C_PINS;
    P1DIR &= ~I2C_PINS;
    if (!scl_released()) return MAXM_BUS_STUCK;
    if (!(P1IN & BIT2)) {
        for (i = 0; i < 9 && !(P1IN & BIT2); ++i) {
            P1DIR |= BIT3; /* SCL low. */
            __delay_cycles(6);
            P1DIR &= ~BIT3; /* SCL released. */
            if (!scl_released()) return MAXM_BUS_STUCK;
            __delay_cycles(6);
        }
        /* Generate STOP: SDA low while SCL low, release SCL, then SDA. */
        P1DIR |= I2C_PINS;
        __delay_cycles(6);
        P1DIR &= ~BIT3;
        if (!scl_released()) {
            P1DIR &= ~BIT2;
            return MAXM_BUS_STUCK;
        }
        __delay_cycles(6);
        P1DIR &= ~BIT2;
        __delay_cycles(6);
    }
    if ((P1IN & I2C_PINS) != I2C_PINS) return MAXM_BUS_STUCK;
    P1SEL0 |= I2C_PINS; /* P1.2 UCB0SDA, P1.3 UCB0SCL: primary function. */
    UCB0CTLW0 = UCSWRST | UCMST | UCMODE_3 | UCSYNC | UCSSEL__SMCLK;
    UCB0CTLW1 = UCASTP_0;
    UCB0BRW = 11; /* Approx. 92.3 kHz, below standard-mode 100 kHz. */
    UCB0I2CSA = MAXM86161_ADDRESS;
    UCB0IE = 0;
    UCB0CTLW0 &= ~UCSWRST;
    UCB0IFG = 0;
    return MAXM_OK;
}

static maxm_result_t wait_ifg(uint16_t flag)
{
    uint16_t n = WAIT_LIMIT;
    do {
        if (UCB0IFG & UCALIFG) return MAXM_BUS_STUCK;
        if (UCB0IFG & UCNACKIFG) return MAXM_NACK;
        if (UCB0IFG & flag) return MAXM_OK;
    } while (--n);
    return MAXM_TIMEOUT;
}

static maxm_result_t wait_idle(void)
{
    uint16_t n = WAIT_LIMIT;
    while ((UCB0STATW & UCBBUSY) || (UCB0CTLW0 & UCTXSTP)) {
        if (--n == 0) return MAXM_BUS_STUCK;
    }
    return MAXM_OK;
}

static maxm_result_t abort_transaction(maxm_result_t reason)
{
    uint16_t n = WAIT_LIMIT;
    if (!(UCB0IFG & UCALIFG)) {
        UCB0CTLW0 |= UCTXSTP;
        while ((UCB0CTLW0 & UCTXSTP) && --n) { }
    }
    UCB0CTLW0 |= UCSWRST;
    UCB0CTLW0 &= ~(UCTXSTT | UCTXSTP);
    UCB0IFG = 0;
    UCB0CTLW0 &= ~UCSWRST;
    return reason;
}

maxm_result_t maxm_write_reg(uint8_t reg, uint8_t value)
{
    maxm_result_t result = wait_idle();
    if (result != MAXM_OK) return abort_transaction(result);
    UCB0IFG = 0;
    UCB0CTLW1 = UCASTP_2;
    UCB0TBCNT = 2;
    UCB0CTLW0 |= UCTR | UCTXSTT;
    result = wait_ifg(UCTXIFG0);
    if (result != MAXM_OK) return abort_transaction(result);
    UCB0TXBUF = reg;
    result = wait_ifg(UCTXIFG0);
    if (result != MAXM_OK) return abort_transaction(result);
    UCB0TXBUF = value;
    result = wait_ifg(UCSTPIFG);
    if (result != MAXM_OK) return abort_transaction(result);
    result = wait_idle();
    return result == MAXM_OK ? result : abort_transaction(result);
}

maxm_result_t maxm_read_regs(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t i;
    maxm_result_t result;
    if (data == 0 || length == 0) return MAXM_ARGUMENT;
    result = wait_idle();
    if (result != MAXM_OK) return abort_transaction(result);
    UCB0IFG = 0;
    UCB0CTLW1 = UCASTP_1; /* Count pointer byte, but do not generate STOP. */
    UCB0TBCNT = 1;
    UCB0CTLW0 |= UCTR | UCTXSTT;
    result = wait_ifg(UCTXIFG0);
    if (result != MAXM_OK) return abort_transaction(result);
    UCB0TXBUF = reg;
    result = wait_ifg(UCBCNTIFG); /* Pointer byte ACK completed before repeated START. */
    if (result != MAXM_OK) return abort_transaction(result);
    UCB0IFG &= ~(UCBCNTIFG | UCTXIFG0);
    UCB0CTLW1 = UCASTP_2;
    UCB0TBCNT = length;
    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT; /* Repeated START + read address C5. */
    for (i = 0; i < length; ++i) {
        result = wait_ifg(UCRXIFG0);
        if (result != MAXM_OK) return abort_transaction(result);
        data[i] = (uint8_t)UCB0RXBUF;
    }
    /* Byte counter sends final NACK + STOP, including for one-byte reads. */
    result = wait_ifg(UCSTPIFG);
    if (result != MAXM_OK) return abort_transaction(result);
    result = wait_idle();
    return result == MAXM_OK ? result : abort_transaction(result);
}

void board_uart_init(void)
{
    /* UARTTX -> P2.6/UCA1TXD, UARTRX -> P2.5/UCA1RXD (RX unused). */
    P2SEL0 |= BIT6;
    P2SEL1 &= ~BIT6;
    UCA1CTLW0 = UCSWRST | UCSSEL__SMCLK;
    UCA1BRW = 106; /* 1,015,808 / 106 = 9583 baud, within 0.2% of 9600. */
    UCA1MCTLW = 0;
    UCA1IE = 0;
    UCA1CTLW0 &= ~UCSWRST;
}

uint8_t board_uart_char(char value)
{
    uint16_t n = WAIT_LIMIT;
    while (!(UCA1IFG & UCTXIFG)) {
        if (--n == 0) return 0;
    }
    UCA1TXBUF = (uint8_t)value;
    return 1;
}
