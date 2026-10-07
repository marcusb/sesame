#include "pic_updater.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "pic_uart.h"

LOG_MODULE_REGISTER(pic_updater, LOG_LEVEL_DBG);

#define HEX_LINE_BUF_SIZE 256

static char line_buf[HEX_LINE_BUF_SIZE];
static size_t line_len;
static uint32_t ext_addr;
static bool sbl_started;
static bool eof_received;
static size_t data_records_written;

static int hex_char_to_val(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

static int hex_byte(const char* s) {
    int hi = hex_char_to_val(s[0]);
    int lo = hex_char_to_val(s[1]);
    if (hi < 0 || lo < 0) {
        return -1;
    }
    return (hi << 4) | lo;
}

static int parse_and_process_hex_line(const char* line) {
    while (*line == ' ' || *line == '\t') {
        line++;
    }
    if (*line != ':') {
        LOG_DBG("Ignoring non-record line: %s", line);
        return 0;
    }

    size_t str_len = strlen(line);
    if (str_len < 11) {
        LOG_ERR("Line too short: %s", line);
        return -1;
    }

    int byte_count = hex_byte(line + 1);
    if (byte_count < 0) {
        LOG_ERR("Invalid byte count in line: %s", line);
        return -1;
    }

    size_t expected_chars = 1 + 2 + 4 + 2 + (size_t)byte_count * 2 + 2;
    if (str_len < expected_chars) {
        LOG_ERR("Line truncated: expected %zu chars, got %zu", expected_chars,
                str_len);
        return -1;
    }

    // Verify Intel HEX checksum
    uint8_t sum = 0;
    for (size_t i = 1; i < expected_chars; i += 2) {
        int b = hex_byte(line + i);
        if (b < 0) {
            LOG_ERR("Invalid hex char in line: %s", line);
            return -1;
        }
        sum += (uint8_t)b;
    }
    if (sum != 0) {
        LOG_ERR("Checksum mismatch in hex line: %s", line);
        return -1;
    }

    int addr_hi = hex_byte(line + 3);
    int addr_lo = hex_byte(line + 5);
    uint16_t addr = (uint16_t)((addr_hi << 8) | addr_lo);
    int rectype = hex_byte(line + 7);

    if (rectype == 0x00) {  // Data Record
        uint32_t full_byte_addr = (ext_addr << 16) | addr;
        uint32_t word_addr = full_byte_addr >> 1;

        // Protect bootloader memory area (0x0000 - 0x03FF)
        if (word_addr < 0x0400) {
            LOG_WRN(
                "Ignoring write to protected bootloader area at word 0x%04x",
                word_addr);
            return 0;
        }
        // Configuration words / ID locations are at 0x8000+
        if (word_addr >= 0x8000) {
            LOG_DBG("Skipping config words at word 0x%04x", word_addr);
            return 0;
        }

        uint8_t payload[64];
        if (byte_count > sizeof(payload)) {
            LOG_ERR("Byte count %d exceeds payload buffer", byte_count);
            return -1;
        }

        for (int i = 0; i < byte_count; i++) {
            int b = hex_byte(line + 9 + i * 2);
            payload[i] = (uint8_t)b;
        }

        int ret = pic_sbl_write_chunk(word_addr, payload, byte_count);
        if (ret != 0) {
            LOG_ERR("Failed to write SBL block at word addr 0x%04x: %d",
                    word_addr, ret);
            return ret;
        }
        data_records_written++;
    } else if (rectype == 0x04) {  // Extended Linear Address Record
        int ext_hi = hex_byte(line + 9);
        int ext_lo = hex_byte(line + 11);
        ext_addr = (uint32_t)((ext_hi << 8) | ext_lo);
        LOG_DBG("SBL Extended Linear Address: 0x%04x", ext_addr);
    } else if (rectype == 0x01) {  // End Of File Record
        if (data_records_written == 0) {
            LOG_ERR("Intel HEX EOF record received but 0 data records written");
            pic_exit_sbl_mode();
            return -1;
        }
        LOG_INF("Intel HEX EOF record received (%zu data records written)",
                data_records_written);
        int ret = pic_sbl_finish();
        if (ret != 0) {
            LOG_ERR("SBL finish/EOF failed: %d", ret);
            return ret;
        }
        eof_received = true;
        sbl_started = false;
        return 0;
    }

    return 0;
}

void pic_updater_reset(void) {
    line_len = 0;
    ext_addr = 0;
    sbl_started = false;
    eof_received = false;
    data_records_written = 0;
}

int pic_updater_process_chunk(const uint8_t* data, size_t len) {
    if (eof_received) {
        return 0;
    }

    if (!sbl_started) {
        LOG_INF("First chunk received, entering SBL mode");
        int ret = pic_enter_sbl_mode();
        if (ret != 0) {
            LOG_ERR("Failed to enter SBL mode: %d", ret);
            return ret;
        }
        sbl_started = true;
        ext_addr = 0;
        eof_received = false;
        line_len = 0;
    }

    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\r' || c == '\n') {
            if (line_len > 0) {
                line_buf[line_len] = '\0';
                int ret = parse_and_process_hex_line(line_buf);
                line_len = 0;
                if (ret != 0) {
                    LOG_ERR("Error processing hex line, aborting");
                    pic_exit_sbl_mode();
                    sbl_started = false;
                    return ret;
                }
            }
        } else if (c == ':' && line_len > 0) {
            line_buf[line_len] = '\0';
            int ret = parse_and_process_hex_line(line_buf);
            line_len = 0;
            if (ret != 0) {
                LOG_ERR("Error processing hex line, aborting");
                pic_exit_sbl_mode();
                sbl_started = false;
                return ret;
            }
            line_buf[line_len++] = c;
        } else {
            if (line_len < HEX_LINE_BUF_SIZE - 1) {
                line_buf[line_len++] = c;
            } else {
                LOG_ERR("Hex line buffer overflow");
                pic_exit_sbl_mode();
                sbl_started = false;
                return -1;
            }
        }
    }
    return 0;
}

int pic_updater_complete(void) {
    if (!sbl_started && !eof_received) {
        LOG_ERR("Updater completed without starting");
        return -1;
    }

    if (!eof_received && line_len > 0) {
        line_buf[line_len] = '\0';
        int ret = parse_and_process_hex_line(line_buf);
        line_len = 0;
        if (ret != 0) {
            pic_exit_sbl_mode();
            sbl_started = false;
            return ret;
        }
    }

    sbl_started = false;
    if (!eof_received || data_records_written == 0) {
        LOG_ERR(
            "PIC update failed: EOF record not received/acknowledged or no "
            "data "
            "written (eof=%d, records=%zu)",
            eof_received, data_records_written);
        pic_exit_sbl_mode();
        return -1;
    }

    LOG_INF("PIC firmware update completed successfully (%zu data blocks)",
            data_records_written);
    return 0;
}

void pic_updater_abort(void) {
    if (sbl_started) {
        LOG_WRN("Aborting PIC update");
        pic_exit_sbl_mode();
        sbl_started = false;
    }
    line_len = 0;
}
