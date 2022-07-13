/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "as3935.h"


#define ALERT_SENSE_PIN 15

uint32_t irq_fired = 0;

void print_registers();

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
}

static void write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = 0x00 | (reg & 0x3F);
    buf[1] = data;
    cs_select();
    spi_write_blocking(spi_default, buf, 2);
    cs_deselect();
    sleep_ms(10);
}

static void read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    reg |= 0x40; // bit 6 needs to be one.
    reg &= 0x7F; // make sure topmost bit is clear.
    cs_select();
    spi_write_blocking(spi_default, &reg, 1);
    sleep_us(10);
    spi_read_blocking(spi_default, 0, buf, len);
    cs_deselect();
    sleep_us(10);
}

void gpio_isr( uint gpio, uint32_t events )
{
    printf("IRQ Detected!\n");
    // uint8_t b;
    // busy_wait_us(2000);
    // read_registers(3, &b, 1);
    // printf("REG 03 : 0x%02X\n", b);
    // //enable_irq( false );
    // print_registers();
    irq_fired = 1;
}

void enable_irq(bool state) {
    gpio_set_irq_enabled_with_callback(ALERT_SENSE_PIN,
                                       GPIO_IRQ_EDGE_RISE,
                                       state,
                                       &gpio_isr);
}


void print_registers()
{
    uint8_t register_values[11];
    int count = 0;

    for(int i = 0; i < 0x09; i++)
    {
        read_registers(i, &register_values[count], 1);
        count++;
    }
    for(int i = 0x3A; i <= 0x3B; i++)
    {
        read_registers(i, &register_values[count], 1);
        count++;
    }
    for(int j = 0; j < 11; j++)
    {
        switch( j ){
            case(0):
                printf("REGISTER 0:\n");
                printf("      RESV[7:6]            : 0x%02X\n", (register_values[j] & 0xC0) >> 6);
                printf("      AFE_CB[5:1]          : 0x%02X\n", (register_values[j] & 0x3E) >> 1);
                printf("      PWD[0]               : 0x%02X\n", (register_values[j] & 0x01) >> 0);
                break;
            case(1):
                printf("REGISTER 1:\n");
                printf("      RESV[7]              : 0x%02X\n", (register_values[j] & 0x80) >> 7);
                printf("      NF_LEV[6:4]          : 0x%02X\n", (register_values[j] & 0x70) >> 4);
                printf("      WDTH[3:0]            : 0x%02X\n", (register_values[j] & 0x0F) >> 0);
                break;
            case(2):
                printf("REGISTER 2:\n");
                printf("      RESV[7]              : 0x%02X\n", (register_values[j] & 0x80) >> 7);
                printf("      CL_STAT[6]           : 0x%02X\n", (register_values[j] & 0x40) >> 6);
                printf("      MIN_NUM_LIGH[5:4]    : 0x%02X\n", (register_values[j] & 0x30) >> 4);
                printf("      SREJ[3:0]            : 0x%02X\n", (register_values[j] & 0x0F) >> 0);
                break;
            case(3):
                printf("REGISTER 3:\n");
                printf("      LCO_FDIV[7:6]        : 0x%02X\n", (register_values[j] & 0xC0) >> 6);
                printf("      MASK_DST[5]          : 0x%02X\n", (register_values[j] & 0x20) >> 5);
                printf("      RESV[4]              : 0x%02X\n", (register_values[j] & 0x10) >> 4);
                printf("      INT[3:0]             : 0x%02X\n", (register_values[j] & 0x0F) >> 0);
                break;
            case(4):
                printf("REGISTER 4:\n");
                printf("      S_LIG_L[7:0]         : 0x%02X\n", (register_values[j] & 0xFF));
                break;
            case(5):
                printf("REGISTER 5:\n");
                printf("      S_LIG_M[7:0]         : 0x%02X\n", (register_values[j] & 0xFF));
                break;
            case(6):
                printf("REGISTER 6:\n");
                printf("      RESV[7:5]            : 0x%02X\n", (register_values[j] & 0xE0) >> 5);
                printf("      S_LIG_MM[4:0]        : 0x%02X\n", (register_values[j] & 0x1F) >> 0);
                break;
            case(7):
                printf("REGISTER 7:\n");
                printf("      RESV[7:6]            : 0x%02X\n", (register_values[j] & 0xC0) >> 6);
                printf("      distance[5:0]        : 0x%02X\n", (register_values[j] & 0x3F) >> 0);
                break;
            case(8):
                printf("REGISTER 8:\n");
                printf("      DISP_LCO[7]          : 0x%02X\n", (register_values[j] & 0x80) >> 7);
                printf("      DISP_SRCO[6]         : 0x%02X\n", (register_values[j] & 0x40) >> 6);
                printf("      DISP_TRCO[5]         : 0x%02X\n", (register_values[j] & 0x20) >> 5);
                printf("      RESV[4]              : 0x%02X\n", (register_values[j] & 0x10) >> 4);
                printf("      TUN_CAP[3:0]         : 0x%02X\n", (register_values[j] & 0x0F) >> 0);
                break;
            case(9):
                printf("REGISTER 8:\n");
                printf("      TRCO_CALIB_DONE[7]   : 0x%02X\n", (register_values[j] & 0x80) >> 7);
                printf("      TRCO_CALIB_NOK[6]    : 0x%02X\n", (register_values[j] & 0x40) >> 6);
                break;
            case(10):
                printf("REGISTER 8:\n");
                printf("      SRCO_CALIB_DONE[7]   : 0x%02X\n", (register_values[j] & 0x80) >> 7);
                printf("      SRCO_CALIB_NOK[6]    : 0x%02X\n", (register_values[j] & 0x40) >> 6);
                break;
        }
    }
}


int main() {
    stdio_init_all();
    if (cyw43_arch_init()) {
        printf("WiFi init failed");
        return -1;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(250);
#if !defined(spi_default) || !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || !defined(PICO_DEFAULT_SPI_RX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)
        #warning spi/bme280_spi example requires a board with SPI pins
    puts("Default SPI pins were not defined");
#endif

    printf("Hello, Reading raw data from registers via SPI...\n");

    // This example will use SPI0 at 0.5MHz.
    spi_init(spi_default, 1000000);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI));


    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "SPI CS"));

    gpio_init( ALERT_SENSE_PIN );
    gpio_set_dir( ALERT_SENSE_PIN, GPIO_IN );
    enable_irq(true);

    write_register( 0x0 , (18 << 1));

    int i = 0;
    for( i = 0; i < 0x09; i++)
    {
        uint8_t buffer;
        read_registers(i, &buffer, 1);
        printf("REGISTER [0x%02X] = 0x%02X\n", i, buffer);
    }
    for( i = 0x3A; i <= 0x3B; i++)
    {
        uint8_t buffer;
        read_registers(i, &buffer, 1);
        printf("REGISTER [0x%02X] = 0x%02X\n", i, buffer);
    }
    print_registers();
    while(1)
    {
        sleep_ms(1000);
        if( irq_fired )
        {
            print_registers();
            irq_fired = 0;
        }
        //print_registers();
    }
}

