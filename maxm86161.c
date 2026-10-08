#include "maxm86161.h"

#define REG_FIFO_OVF       0x06u
#define REG_FIFO_DATA      0x08u
#define REG_FIFO_CONFIG2   0x0Au
#define REG_SYSTEM         0x0Du
#define REG_CALIBRATION    0x50u
#define REG_PART_ID        0xFFu
#define FIFO_CONFIG        0x08u /* Status clears on read; stop on full. */
#define FIFO_FLUSH         0x10u

static maxm_result_t write_checked(uint8_t reg, uint8_t value)
{
    uint8_t actual;
    maxm_result_t result = maxm_write_reg(reg, value);
    if (result != MAXM_OK) return result;
    result = maxm_read_regs(reg, &actual, 1);
    if (result != MAXM_OK) return result;
    return actual == value ? MAXM_OK : MAXM_CONFIG_MISMATCH;
}

maxm_result_t maxm_init(uint8_t *part_id)
{
    static const uint8_t config[][2] = {
        {0x02, 0x00}, {0x03, 0x00}, /* Polling, no interrupt wiring required. */
        {0x10, 0x00},               /* No sync / timestamps / DAC-change tags. */
        {0x11, 0x0B},               /* 16 uA ADC range, 117.3 us integration. */
        {0x12, 0x18},               /* SR=3: 99.902 sps, no averaging. */
        {0x13, 0x00}, {0x15, 0x01}, {0x16, 0x00},
        {0x2A, 0x00},               /* 31 mA full scale for all LED drivers. */
        {0x23, 0x28}, {0x24, 0x28}, {0x25, 0x28}, /* Approx. 4.8 mA. */
        {0x09, 0x60}, {0x0A, FIFO_CONFIG},
        {0x21, 0x03}, {0x22, 0x00}, /* Slot 3 red; end at slot 4. */
        {0x0D, 0x08}                /* SINGLE_PPG=1, SHDN=0, LP_MODE=0. */
    };
    uint8_t i, value;
    maxm_result_t result;
    if (part_id == 0) return MAXM_ARGUMENT;
    *part_id = 0;
    result = maxm_read_regs(REG_PART_ID, part_id, 1);
    if (result != MAXM_OK) return result;
    if (*part_id != 0x36) return MAXM_WRONG_ID;
    result = maxm_write_reg(REG_SYSTEM, 0x01); /* Software reset. */
    if (result != MAXM_OK) return result;
    for (i = 0; i < 20; ++i) {
        board_delay_ms(1);
        result = maxm_read_regs(REG_SYSTEM, &value, 1);
        if (result != MAXM_OK) return result;
        if (!(value & 0x01)) break;
    }
    if (i == 20) return MAXM_RESET_TIMEOUT;

    /* Datasheet ADC/XNL calibration procedure, before starting LED sequence. */
    {
        static const uint8_t clear_regs[] = {0x02, 0x03, 0x0D, 0x10,
                                            0x12, 0x13, 0x20};
        for (i = 0; i < sizeof(clear_regs); ++i) {
            result = write_checked(clear_regs[i], 0);
            if (result != MAXM_OK) return result;
        }
    }
    result = write_checked(0x11, 0x0B);
    if (result != MAXM_OK) return result;
    result = write_checked(0x15, 0x01);
    if (result != MAXM_OK) return result;
    result = maxm_write_reg(REG_CALIBRATION, 0x04); /* START_CAL. */
    if (result != MAXM_OK) return result;
    board_delay_ms(200);
    for (i = 0; i < 100; ++i) {
        result = maxm_read_regs(REG_CALIBRATION, &value, 1);
        if (result != MAXM_OK) return result;
        if (value & 0x40) break; /* CAL_DAC_Complete. */
        board_delay_ms(1);
    }
    if (i == 100) return MAXM_CAL_TIMEOUT;
    if (value & 0x10) return MAXM_CAL_RANGE; /* CAL_DAC1_OOR. */

    for (i = 0; i < sizeof(config) / sizeof(config[0]); ++i) {
        result = write_checked(config[i][0], config[i][1]);
        if (result != MAXM_OK) return result;
    }
    result = maxm_write_reg(REG_FIFO_CONFIG2, FIFO_CONFIG | FIFO_FLUSH);
    if (result != MAXM_OK) return result;
    /* 0x21 => slot 1 green (LED1), slot 2 IR (LED2). Starts sampling last. */
    return write_checked(0x20, 0x21);
}

maxm_result_t maxm_read_fifo(maxm_sample_t *samples, uint8_t capacity,
                            uint8_t *count)
{
    uint8_t status[2], raw[3 * MAXM86161_FIFO_BATCH];
    uint8_t i, available;
    maxm_result_t result;
    if (count == 0) return MAXM_ARGUMENT;
    *count = 0;
    if (samples == 0 || capacity == 0 || capacity > MAXM86161_FIFO_BATCH)
        return MAXM_ARGUMENT;
    /* Burst read OVF_COUNTER and FIFO_DATA_COUNT (items, not bytes). */
    result = maxm_read_regs(REG_FIFO_OVF, status, 2);
    if (result != MAXM_OK) return result;
    if (status[0] & 0x7F) {
        result = maxm_write_reg(REG_FIFO_CONFIG2, FIFO_CONFIG | FIFO_FLUSH);
        return result == MAXM_OK ? MAXM_FIFO_OVERFLOW : result;
    }
    available = status[1]; /* 128 is valid, so do not mask to seven bits. */
    if (available > capacity) available = capacity;
    if (available == 0) return MAXM_OK;
    /* FIFO port stays at 0x08 throughout the burst; always read whole items. */
    result = maxm_read_regs(REG_FIFO_DATA, raw, (uint8_t)(3 * available));
    if (result != MAXM_OK) return result;
    for (i = 0; i < available; ++i) {
        uint8_t offset = (uint8_t)(3 * i);
        samples[i].tag = raw[offset] >> 3;
        samples[i].value = ((uint32_t)(raw[offset] & 0x07) << 16)
                         | ((uint32_t)raw[offset + 1] << 8)
                         | raw[offset + 2];
    }
    *count = available;
    return MAXM_OK;
}
