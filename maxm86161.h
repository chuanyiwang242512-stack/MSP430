#ifndef MAXM86161_H
#define MAXM86161_H

#include <stdint.h>

#define MAXM86161_ADDRESS 0x62u /* 7-bit; wire addresses are C4/C5. */
#define MAXM86161_FIFO_BATCH 16u

typedef enum {
    MAXM_OK = 0,
    MAXM_NACK,
    MAXM_TIMEOUT,
    MAXM_BUS_STUCK,
    MAXM_WRONG_ID,
    MAXM_RESET_TIMEOUT,
    MAXM_CAL_TIMEOUT,
    MAXM_CAL_RANGE,
    MAXM_FIFO_OVERFLOW,
    MAXM_CONFIG_MISMATCH,
    MAXM_ARGUMENT
} maxm_result_t;

typedef struct {
    uint32_t value; /* 19-bit ADC count, not heart rate or SpO2. */
    uint8_t tag;    /* Sequence slot, not a fixed LED colour. */
} maxm_sample_t;

/* Board transport; all transactions must have bounded waits. */
maxm_result_t maxm_write_reg(uint8_t reg, uint8_t value);
maxm_result_t maxm_read_regs(uint8_t reg, uint8_t *data, uint8_t length);
void board_delay_ms(uint16_t ms);
maxm_result_t board_i2c_init(void);

maxm_result_t maxm_init(uint8_t *part_id);
maxm_result_t maxm_read_fifo(maxm_sample_t *samples, uint8_t capacity,
                            uint8_t *count);

#endif
