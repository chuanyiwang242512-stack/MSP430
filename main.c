#include <msp430.h>
#include "board.h"

/* Add these to CCS Expressions. Values are published only for a complete frame. */
volatile uint8_t g_part_id;
volatile maxm_result_t g_error;
volatile uint8_t g_ready;
volatile uint8_t g_clock_error;
volatile uint32_t g_green, g_ir, g_red;
volatile uint32_t g_frame_count, g_sample_count;
volatile uint32_t g_i2c_errors, g_fifo_overflows, g_tag_errors, g_uart_errors;
volatile uint32_t g_sensor_errors;

static maxm_sample_t samples[MAXM86161_FIFO_BATCH];
static uint32_t pending_green, pending_ir;
static uint8_t next_tag = 1;

static uint8_t uart_number(uint32_t value)
{
    char digits[10];
    uint8_t used = 0;
    do {
        digits[used++] = (char)('0' + value % 10);
        value /= 10;
    } while (value);
    while (used) {
        if (!board_uart_char(digits[--used])) return 0;
    }
    return 1;
}

static uint8_t uart_frame(void)
{
    return uart_number(g_green) && board_uart_char(',')
        && uart_number(g_ir) && board_uart_char(',')
        && uart_number(g_red) && board_uart_char('\r') && board_uart_char('\n');
}

static void accept_sample(const maxm_sample_t *sample)
{
    ++g_sample_count;
    if (sample->tag == 1) {
        if (next_tag != 1) ++g_tag_errors;
        pending_green = sample->value;
        next_tag = 2;
    } else if (sample->tag == 2 && next_tag == 2) {
        pending_ir = sample->value;
        next_tag = 3;
    } else if (sample->tag == 3 && next_tag == 3) {
        g_green = pending_green;
        g_ir = pending_ir;
        g_red = sample->value;
        ++g_frame_count;
        next_tag = 1;
    } else {
        ++g_tag_errors;
        next_tag = 1; /* Never combine readings across missing/unknown slots. */
    }
}

int main(void)
{
    uint8_t id, count, i;
    maxm_result_t result;
    uint32_t reported_frame = 0;
    WDTCTL = WDTPW | WDTHOLD;
    board_pins_init();
    if (!board_clock_init()) {
        g_clock_error = 1;
        for (;;) { } /* LDO remains off; visible in CCS. */
    }
    board_uart_init();
    for (;;) {
        g_ready = 0;
        g_part_id = 0;
        next_tag = 1;
        board_sensor_enable(0);
        board_delay_ms(10);
        board_sensor_enable(1);
        board_delay_ms(50);
        result = board_i2c_init();
        if (result == MAXM_OK) {
            result = maxm_init(&id);
            g_part_id = id;
        }
        if (result != MAXM_OK) {
            g_error = result;
            if (result == MAXM_NACK || result == MAXM_TIMEOUT || result == MAXM_BUS_STUCK)
                ++g_i2c_errors;
            else ++g_sensor_errors;
            board_sensor_enable(0);
            board_delay_ms(1000);
            continue;
        }
        g_error = MAXM_OK;
        g_ready = 1;
        for (;;) {
            result = maxm_read_fifo(samples, MAXM86161_FIFO_BATCH, &count);
            if (result == MAXM_FIFO_OVERFLOW) {
                ++g_fifo_overflows;
                next_tag = 1;
                g_error = result;
                continue;
            }
            if (result != MAXM_OK) {
                g_error = result;
                ++g_i2c_errors;
                g_ready = 0;
                board_delay_ms(1000);
                break; /* Power cycle, recover I2C bus, reconfigure sensor. */
            }
            g_error = MAXM_OK;
            for (i = 0; i < count; ++i) accept_sample(&samples[i]);
            /* About 10 complete frames/sec on UART; sensor still samples at 100 Hz. */
            if (g_frame_count - reported_frame >= 10) {
                if (!uart_frame()) ++g_uart_errors;
                reported_frame = g_frame_count;
            }
            /* Drain full batches immediately to catch up; sleep only when caught up. */
            if (count < MAXM86161_FIFO_BATCH) board_delay_ms(5);
        }
    }
}
