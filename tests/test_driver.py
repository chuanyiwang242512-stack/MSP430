"""Host regression tests for the sensor protocol and complete-frame assembly.

The fake bus exercises configuration, FIFO parsing and errors; it does not
simulate MSP430 peripherals or claim on-board I2C/optical validation.
Run: python3 tests/test_driver.py
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "maxm86161.h"

static uint8_t regs[256], raw[48], available, overflow, last_length;
static uint8_t calibration_stuck, reset_stuck, calibration_bad, mismatch;
static uint8_t injected_read_reg = 0xEE, injected_write_reg = 0xEE;
static unsigned flushes, fifo_reads, delay_total;
static char serial[100];
static unsigned serial_used;
uint16_t WDTCTL;

void board_delay_ms(uint16_t ms) { delay_total += ms; }
uint8_t board_clock_init(void) { return 1; }
void board_pins_init(void) { }
void board_sensor_enable(uint8_t on) { (void)on; }
void board_uart_init(void) { }
uint8_t board_uart_char(char c) {
    assert(serial_used < sizeof(serial) - 1);
    serial[serial_used++] = c; serial[serial_used] = 0; return 1;
}
maxm_result_t board_i2c_init(void) { return MAXM_OK; }
maxm_result_t maxm_write_reg(uint8_t reg, uint8_t value) {
    if (reg == injected_write_reg) return MAXM_NACK;
    regs[reg] = value;
    if (reg == 0x0D && value == 1 && !reset_stuck) regs[reg] = 0;
    if (reg == 0x50 && !calibration_stuck)
        regs[reg] = calibration_bad ? 0x50 : 0x40;
    if (reg == 0x0A && (value & 0x10)) {
        ++flushes; overflow = available = 0; regs[reg] &= ~0x10;
    }
    return MAXM_OK;
}
maxm_result_t maxm_read_regs(uint8_t reg, uint8_t *out, uint8_t len) {
    if (reg == injected_read_reg) return MAXM_TIMEOUT;
    if (reg == 0x06) {
        assert(len == 2); out[0] = overflow; out[1] = available;
    } else if (reg == 0x08) {
        assert(len <= sizeof(raw) && len % 3 == 0 && len / 3 <= available);
        last_length = len; ++fifo_reads; memcpy(out, raw, len);
        available -= len / 3;
    } else {
        assert(len == 1); out[0] = regs[reg];
        if (reg == 0x12 && mismatch) out[0] ^= 1;
    }
    return MAXM_OK;
}
#define main firmware_main
#include "main.c"
#undef main

static void reset_mock(void) {
    memset(regs, 0, sizeof(regs)); regs[0xFF] = 0x36;
    available = overflow = calibration_stuck = reset_stuck = calibration_bad = mismatch = 0;
    injected_read_reg = injected_write_reg = 0xEE;
    flushes = fifo_reads = delay_total = 0;
}
static void item(unsigned i, uint8_t tag, uint32_t value) {
    uint32_t word = ((uint32_t)tag << 19) | value;
    raw[3*i] = (uint8_t)(word >> 16);
    raw[3*i+1] = (uint8_t)(word >> 8); raw[3*i+2] = (uint8_t)word;
}
int main(void) {
    maxm_sample_t out[16], s;
    uint8_t id, n;
    reset_mock();
    assert(maxm_init(&id) == MAXM_OK && id == 0x36);
    assert(regs[0x0D] == 8 && regs[0x12] == 0x18);
    assert(regs[0x20] == 0x21 && regs[0x21] == 3 && regs[0x22] == 0);
    assert(regs[0x23] == 0x28 && regs[0x24] == 0x28 && regs[0x25] == 0x28);
    assert(flushes == 1 && delay_total >= 200);
    puts("PASS initialization, calibration, LED sequence and readback");

    reset_mock(); regs[0xFF] = 0x15;
    assert(maxm_init(&id) == MAXM_WRONG_ID && flushes == 0);
    reset_mock(); injected_read_reg = 0xFF;
    assert(maxm_init(&id) == MAXM_TIMEOUT);
    reset_mock(); injected_write_reg = 0x11;
    assert(maxm_init(&id) == MAXM_NACK);
    reset_mock(); reset_stuck = 1;
    assert(maxm_init(&id) == MAXM_RESET_TIMEOUT && delay_total == 20);
    reset_mock(); calibration_stuck = 1;
    assert(maxm_init(&id) == MAXM_CAL_TIMEOUT && delay_total <= 321);
    reset_mock(); calibration_bad = 1;
    assert(maxm_init(&id) == MAXM_CAL_RANGE);
    reset_mock(); mismatch = 1;
    assert(maxm_init(&id) == MAXM_CONFIG_MISMATCH);
    puts("PASS wrong chip, bus failures, bounded reset/calibration and bad readback");

    reset_mock(); available = 3;
    item(0, 1, 0x7FFFF); item(1, 2, 0x12345); item(2, 3, 0);
    assert(maxm_read_fifo(out, 16, &n) == MAXM_OK && n == 3 && last_length == 9);
    assert(out[0].tag == 1 && out[0].value == 0x7FFFF);
    assert(out[1].tag == 2 && out[1].value == 0x12345);
    assert(out[2].tag == 3 && out[2].value == 0);
    assert(maxm_read_fifo(out, 16, &n) == MAXM_OK && n == 0 && fifo_reads == 1);
    available = 128;
    assert(maxm_read_fifo(out, 16, &n) == MAXM_OK && n == 16 && last_length == 48);
    assert(available == 112);
    overflow = 7;
    assert(maxm_read_fifo(out, 16, &n) == MAXM_FIFO_OVERFLOW && n == 0);
    assert(flushes == 1 && overflow == 0 && available == 0);
    overflow = 1; injected_write_reg = 0x0A;
    assert(maxm_read_fifo(out, 16, &n) == MAXM_NACK && n == 0);
    reset_mock(); available = 1; injected_read_reg = 0x08; n = 99;
    assert(maxm_read_fifo(out, 16, &n) == MAXM_TIMEOUT && n == 0);
    assert(maxm_read_fifo(out, 17, &n) == MAXM_ARGUMENT && n == 0);
    assert(maxm_read_fifo(0, 16, &n) == MAXM_ARGUMENT && n == 0);
    assert(maxm_read_fifo(out, 0, &n) == MAXM_ARGUMENT && n == 0);
    assert(maxm_read_fifo(out, 16, 0) == MAXM_ARGUMENT);
    assert(maxm_init(0) == MAXM_ARGUMENT);
    puts("PASS FIFO byte order, 19-bit mask, capacity, 128 items, overflow and read failure");

    g_frame_count = g_sample_count = g_tag_errors = 0; next_tag = 1;
    s.tag = 1; s.value = 100; accept_sample(&s);
    s.tag = 3; s.value = 999; accept_sample(&s); /* Missing IR: reject frame. */
    assert(g_frame_count == 0 && g_tag_errors == 1);
    s.tag = 2; accept_sample(&s); assert(g_frame_count == 0);
    s.tag = 1; s.value = 101; accept_sample(&s);
    s.tag = 2; s.value = 202; accept_sample(&s);
    assert(g_frame_count == 0); /* Partial frame can cross a FIFO batch. */
    s.tag = 3; s.value = 303; accept_sample(&s);
    assert(g_frame_count == 1 && g_green == 101 && g_ir == 202 && g_red == 303);
    s.tag = 1; s.value = 404; accept_sample(&s);
    s.tag = 31; accept_sample(&s); /* Unknown tag invalidates partial frame. */
    s.tag = 2; accept_sample(&s); s.tag = 3; accept_sample(&s);
    assert(g_frame_count == 1 && g_green == 101);
    serial_used = 0; assert(uart_frame());
    assert(strcmp(serial, "101,202,303\r\n") == 0);
    serial_used = 0; assert(uart_number(0)); assert(strcmp(serial, "0") == 0);
    serial_used = 0; assert(uart_number(4294967295UL));
    assert(strcmp(serial, "4294967295") == 0);
    puts("PASS complete-frame assembly, discontinuities, unknown tags and UART CSV");
    return 0;
}
'''

with tempfile.TemporaryDirectory(prefix="maxm86161-test-") as directory:
    tmp = Path(directory)
    # Only the watchdog symbols needed by main.c; hardware driver is not mocked here.
    (tmp / "msp430.h").write_text(
        "#include <stdint.h>\nextern uint16_t WDTCTL;\n"
        "#define WDTPW 0x5A00\n#define WDTHOLD 0x0080\n"
    )
    (tmp / "test.c").write_text(HARNESS)
    binary = tmp / "test"
    subprocess.run([
        "gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-Wpedantic",
        "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-g",
        "-I", str(tmp), "-I", str(ROOT), str(tmp / "test.c"),
        str(ROOT / "maxm86161.c"), "-o", str(binary)
    ], check=True)
    subprocess.run([str(binary)], check=True)
