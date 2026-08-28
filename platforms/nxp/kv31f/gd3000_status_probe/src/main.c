#include "asr_fc/platform/gd3000_protocol.h"

#include "board.h"
#include "clock_config.h"
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_uart.h"
#include "pin_mux.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GD3000_RESET_GPIO GPIOA
#define GD3000_RESET_PORT PORTA
#define GD3000_RESET_PIN 1u

#define GD3000_ENABLE_GPIO GPIOC
#define GD3000_ENABLE_PORT PORTC
#define GD3000_ENABLE_PIN 6u

#define GD3000_CS_GPIO GPIOC
#define GD3000_CS_PORT PORTC
#define GD3000_CS_PIN 19u
#define GD3000_MOSI_GPIO GPIOC
#define GD3000_MOSI_PORT PORTC
#define GD3000_MOSI_PIN 18u
#define GD3000_MISO_GPIO GPIOC
#define GD3000_MISO_PORT PORTC
#define GD3000_MISO_PIN 17u
#define GD3000_CLOCK_GPIO GPIOC
#define GD3000_CLOCK_PORT PORTC
#define GD3000_CLOCK_PIN 16u

#define GD3000_INTERRUPT_GPIO GPIOE
#define GD3000_INTERRUPT_PORT PORTE
#define GD3000_INTERRUPT_PIN 1u

typedef struct {
    GPIO_Type *gpio;
    PORT_Type *port;
    uint32_t pin;
} output_pin_t;

static const output_pin_t kPwmPins[] = {
    {GPIOC, PORTC, 1u},
    {GPIOC, PORTC, 2u},
    {GPIOC, PORTC, 5u},
    {GPIOC, PORTC, 4u},
    {GPIOD, PORTD, 4u},
    {GPIOD, PORTD, 5u},
};

static void delay_us(uint32_t microseconds) {
    SDK_DelayAtLeastUs(microseconds, SystemCoreClock);
}

static void output_low_before_mux(const output_pin_t *pin) {
    const gpio_pin_config_t output = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0u,
    };
    GPIO_PinWrite(pin->gpio, pin->pin, 0u);
    PORT_SetPinMux(pin->port, pin->pin, kPORT_MuxAsGpio);
    GPIO_PinInit(pin->gpio, pin->pin, &output);
}

static void initialize_safe_outputs(void) {
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortD);
    CLOCK_EnableClock(kCLOCK_PortE);

    for (size_t index = 0u; index < sizeof(kPwmPins) / sizeof(kPwmPins[0]);
         ++index) {
        output_low_before_mux(&kPwmPins[index]);
    }

    const output_pin_t enable = {
        GD3000_ENABLE_GPIO,
        GD3000_ENABLE_PORT,
        GD3000_ENABLE_PIN,
    };
    const output_pin_t reset = {
        GD3000_RESET_GPIO,
        GD3000_RESET_PORT,
        GD3000_RESET_PIN,
    };
    const output_pin_t chip_select = {
        GD3000_CS_GPIO,
        GD3000_CS_PORT,
        GD3000_CS_PIN,
    };
    const output_pin_t mosi = {
        GD3000_MOSI_GPIO,
        GD3000_MOSI_PORT,
        GD3000_MOSI_PIN,
    };
    const output_pin_t clock = {
        GD3000_CLOCK_GPIO,
        GD3000_CLOCK_PORT,
        GD3000_CLOCK_PIN,
    };

    output_low_before_mux(&enable);
    output_low_before_mux(&reset);
    output_low_before_mux(&mosi);
    output_low_before_mux(&clock);
    output_low_before_mux(&chip_select);
    GPIO_PinWrite(GD3000_CS_GPIO, GD3000_CS_PIN, 1u);

    const gpio_pin_config_t input = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0u,
    };
    PORT_SetPinMux(GD3000_MISO_PORT, GD3000_MISO_PIN, kPORT_MuxAsGpio);
    GD3000_MISO_PORT->PCR[GD3000_MISO_PIN] |= PORT_PCR_PE_MASK;
    GD3000_MISO_PORT->PCR[GD3000_MISO_PIN] &= ~PORT_PCR_PS_MASK;
    GPIO_PinInit(GD3000_MISO_GPIO, GD3000_MISO_PIN, &input);
    PORT_SetPinMux(
        GD3000_INTERRUPT_PORT, GD3000_INTERRUPT_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(
        GD3000_INTERRUPT_GPIO, GD3000_INTERRUPT_PIN, &input);
}

static uint8_t gd3000_transfer(uint8_t command) {
    uint8_t response = 0u;

    GPIO_PinWrite(GD3000_CS_GPIO, GD3000_CS_PIN, 0u);
    delay_us(2u);
    for (uint8_t bit = 0u; bit < 8u; ++bit) {
        const uint8_t mask = (uint8_t)(0x80u >> bit);
        GPIO_PinWrite(
            GD3000_MOSI_GPIO, GD3000_MOSI_PIN,
            (command & mask) != 0u ? 1u : 0u);
        delay_us(2u);
        GPIO_PinWrite(GD3000_CLOCK_GPIO, GD3000_CLOCK_PIN, 1u);
        delay_us(2u);
        GPIO_PinWrite(GD3000_CLOCK_GPIO, GD3000_CLOCK_PIN, 0u);
        delay_us(2u);
        response = (uint8_t)(response << 1u);
        response |= (uint8_t)GPIO_PinRead(
            GD3000_MISO_GPIO, GD3000_MISO_PIN);
    }
    GPIO_PinWrite(GD3000_MOSI_GPIO, GD3000_MOSI_PIN, 0u);
    GPIO_PinWrite(GD3000_CS_GPIO, GD3000_CS_PIN, 1u);
    delay_us(2u);
    return response;
}

static bool gd3000_read_status(uint8_t status_register, uint8_t *value) {
    asr_fc_gd3000_status_read_t sequence;
    if (value == NULL ||
        asr_fc_gd3000_status_read_sequence(status_register, &sequence) !=
            ASR_FC_GD3000_OK) {
        return false;
    }

    (void)gd3000_transfer(sequence.request);
    *value = gd3000_transfer(sequence.flush);
    return true;
}

static void uart_write(const char *text) {
    size_t size = 0u;
    while (text[size] != '\0') {
        ++size;
    }
    (void)UART_WriteBlocking(UART0, (const uint8_t *)text, size);
}

static void uart_write_hex(uint8_t value) {
    static const char digits[] = "0123456789ABCDEF";
    const uint8_t output[] = {
        (uint8_t)digits[(value >> 4u) & 0x0fu],
        (uint8_t)digits[value & 0x0fu],
    };
    (void)UART_WriteBlocking(UART0, output, sizeof(output));
}

static void uart_write_level(GPIO_Type *gpio, uint32_t pin) {
    uart_write(GPIO_PinRead(gpio, pin) != 0u ? "1" : "0");
}

static void print_pad_levels(const char *label) {
    uart_write(label);
    uart_write(" rst=");
    uart_write_level(GD3000_RESET_GPIO, GD3000_RESET_PIN);
    uart_write(" en=");
    uart_write_level(GD3000_ENABLE_GPIO, GD3000_ENABLE_PIN);
    uart_write(" cs=");
    uart_write_level(GD3000_CS_GPIO, GD3000_CS_PIN);
    uart_write(" clk=");
    uart_write_level(GD3000_CLOCK_GPIO, GD3000_CLOCK_PIN);
    uart_write(" mosi=");
    uart_write_level(GD3000_MOSI_GPIO, GD3000_MOSI_PIN);
    uart_write(" miso=");
    uart_write_level(GD3000_MISO_GPIO, GD3000_MISO_PIN);
    uart_write(" int=");
    uart_write_level(GD3000_INTERRUPT_GPIO, GD3000_INTERRUPT_PIN);
    uart_write("\r\n");
}

static void print_status0(uint8_t raw) {
    asr_fc_gd3000_status0_t status;
    if (asr_fc_gd3000_decode_status0(raw, &status) != ASR_FC_GD3000_OK) {
        uart_write("decode-error\r\n");
        return;
    }

    uart_write(" status0=0x");
    uart_write_hex(raw);
    uart_write(" int=");
    uart_write(GPIO_PinRead(
                   GD3000_INTERRUPT_GPIO, GD3000_INTERRUPT_PIN) != 0u
                   ? "1"
                   : "0");
    uart_write(" reset=");
    uart_write(status.reset_event ? "1" : "0");
    uart_write(" fault=");
    uart_write(asr_fc_gd3000_status0_has_fault(&status) ? "1" : "0");
}

int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();

    initialize_safe_outputs();

    uart_config_t uart_config;
    UART_GetDefaultConfig(&uart_config);
    uart_config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    uart_config.enableTx = true;
    uart_config.enableRx = true;
    if (UART_Init(UART0, &uart_config, BOARD_DEBUG_UART_CLK_FREQ) !=
        kStatus_Success) {
        return 1;
    }

    uart_write("ASR GD3000 read-only probe\r\n");
    uart_write(
        "FRDM-GD3000 J2A ROUTING / MODE-1 GPIO SPI / PIPELINED STATUS READ / "
        "MISO=PULLDOWN / "
        "EN=LOW / "
        "MOTOR=INHIBITED\r\n");
    print_pad_levels("RESET-LOW");

    delay_us(1000u);
    GPIO_PinWrite(GD3000_RESET_GPIO, GD3000_RESET_PIN, 1u);
    delay_us(5000u);
    print_pad_levels("RESET-HIGH");

    for (;;) {
        uint8_t raw[4] = {0u};
        for (uint8_t index = 0u; index < 4u; ++index) {
            if (!gd3000_read_status(index, &raw[index])) {
                uart_write("status-read-contract-error\r\n");
            }
        }

        uart_write("NULL0..3:");
        for (size_t index = 0u; index < sizeof(raw); ++index) {
            uart_write(" 0x");
            uart_write_hex(raw[index]);
        }
        print_status0(raw[0]);
        uart_write("\r\n");
        delay_us(1000000u);
    }
}
