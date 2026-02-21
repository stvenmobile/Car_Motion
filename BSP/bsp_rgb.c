#include "bsp_rgb.h"

#define DWT_CONTROL *(volatile uint32_t *)0xE0001000
#define DWT_CYCCNT  *(volatile uint32_t *)0xE0001004
#define DEM_CR      *(volatile uint32_t *)0xE000EDFC

#define LED_COUNT 7

// Helper for ultra-fast pin toggling using the BSRR register
#define LED_HIGH() GPIOB->BSRR = GPIO_PIN_5
#define LED_LOW()  GPIOB->BSRR = (uint32_t)GPIO_PIN_5 << 16

void RGB_Init(void) {
    // Enable the Cycle Counter for precise timing
    DEM_CR |= (1 << 24);
    DWT_CYCCNT = 0;
    DWT_CONTROL |= 1;
}

static inline void delay_cycles(uint32_t cycles) {
    uint32_t start = DWT_CYCCNT;
    while ((DWT_CYCCNT - start) < cycles);
}

static void send_bit(uint8_t bit) {
    if (bit) {
        // T1H: 0.7us (50 cycles @ 72MHz), T1L: 0.6us (43 cycles)
        GPIOB->BSRR = GPIO_PIN_5;
        delay_cycles(50);
        GPIOB->BSRR = (uint32_t)GPIO_PIN_5 << 16;
        delay_cycles(43);
    } else {
        // T0H: 0.35us (25 cycles @ 72MHz), T0L: 0.8us (57 cycles)
        GPIOB->BSRR = GPIO_PIN_5;
        delay_cycles(25);
        GPIOB->BSRR = (uint32_t)GPIO_PIN_5 << 16;
        delay_cycles(57);
    }
}
static void RGB_Send_Color(uint8_t r, uint8_t g, uint8_t b) {
    // WS2812 is GRB order
    uint32_t color = (g << 16) | (r << 8) | b;
    for (int i = 23; i >= 0; i--) {
        send_bit((color >> i) & 0x01);
    }
}

void RGB_Set_All(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < LED_COUNT; i++) {
        RGB_Send_Color(r, g, b);
    }
    // WS2812 Reset pulse: holding low for >50us
    HAL_Delay(1);
}

void RGB_Clear(void) {
    RGB_Set_All(0, 0, 0);
}
