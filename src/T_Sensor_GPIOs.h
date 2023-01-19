#ifndef T_Sensor_GPIOs_H
#define T_Sensor_GPIOs_H
#include <stdint.h>

// Platform specific I/O definitions

#if defined(__AVR__)
#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint8_t
#define IO_REG_BASE_ATTR asm("r30")
#define IO_REG_MASK_ATTR
#if defined(__AVR_ATmega4809__)
#define DIRECT_READ(base, mask)         (((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   ((*((base)-8)) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*((base)-8)) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)    ((*((base)-4)) &= ~(mask))
#define DIRECT_WRITE_HIGH(base, mask)   ((*((base)-4)) |= (mask))
#else
#define DIRECT_READ(base, mask)         (((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   ((*((base)+1)) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*((base)+1)) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)    ((*((base)+2)) &= ~(mask))
#define DIRECT_WRITE_HIGH(base, mask)   ((*((base)+2)) |= (mask))
#endif
#elif defined(ARDUINO_ARCH_ESP8266)

#define PIN_TO_BASEREG(pin)             ((volatile uint32_t*) GPO)
#define PIN_TO_BITMASK(pin)             (1 << pin)
#define IO_REG_TYPE uint32_t
#define IO_REG_BASE_ATTR
#define IO_REG_MASK_ATTR
#define DIRECT_READ(base, mask)         ((GPI & (mask)) ? 1 : 0)    //GPIO_IN_ADDRESS
#define DIRECT_MODE_INPUT(base, mask)   (GPE &= ~(mask))            //GPIO_ENABLE_W1TC_ADDRESS
#define DIRECT_MODE_OUTPUT(base, mask)  (GPE |= (mask))             //GPIO_ENABLE_W1TS_ADDRESS
#define DIRECT_WRITE_LOW(base, mask)    (GPOC = (mask))             //GPIO_OUT_W1TC_ADDRESS
#define DIRECT_WRITE_HIGH(base, mask)   (GPOS = (mask))             //GPIO_OUT_W1TS_ADDRESS

#elif defined(ARDUINO_ARCH_ESP32)
#include <driver/rtc_io.h>
#define PIN_TO_BASEREG(pin)             (0)
#define PIN_TO_BITMASK(pin)             (pin)
#define IO_REG_TYPE uint32_t
#define IO_REG_BASE_ATTR
#define IO_REG_MASK_ATTR

static inline __attribute__((always_inline))
IO_REG_TYPE directRead(IO_REG_TYPE pin)
{
#if CONFIG_IDF_TARGET_ESP32C3
    return (GPIO.in.val >> pin) & 0x1;
#else // plain ESP32
    if ( pin < 32 )
        return (GPIO.in >> pin) & 0x1;
    else if ( pin < 46 )
        return (GPIO.in1.val >> (pin - 32)) & 0x1;
#endif

    return 0;
}

static inline __attribute__((always_inline))
void directWriteLow(IO_REG_TYPE pin)
{
#if CONFIG_IDF_TARGET_ESP32C3
    GPIO.out_w1tc.val = ((uint32_t)1 << pin);
#else // plain ESP32
    if ( pin < 32 )
        GPIO.out_w1tc = ((uint32_t)1 << pin);
    else if ( pin < 46 )
        GPIO.out1_w1tc.val = ((uint32_t)1 << (pin - 32));
#endif
}

static inline __attribute__((always_inline))
void directWriteHigh(IO_REG_TYPE pin)
{
#if CONFIG_IDF_TARGET_ESP32C3
    GPIO.out_w1ts.val = ((uint32_t)1 << pin);
#else // plain ESP32
    if ( pin < 32 )
        GPIO.out_w1ts = ((uint32_t)1 << pin);
    else if ( pin < 46 )
        GPIO.out1_w1ts.val = ((uint32_t)1 << (pin - 32));
#endif
}

static inline __attribute__((always_inline))
void directModeInput(IO_REG_TYPE pin)
{
#if CONFIG_IDF_TARGET_ESP32C3
    GPIO.enable_w1tc.val = ((uint32_t)1 << (pin));
#else
    if ( digitalPinIsValid(pin) )
    {
#if ESP_IDF_VERSION_MAJOR < 4      // IDF 3.x ESP32/PICO-D4
        uint32_t rtc_reg(rtc_gpio_desc[pin].reg);

        if ( rtc_reg ) // RTC pins PULL settings
        {
            ESP_REG(rtc_reg) = ESP_REG(rtc_reg) & ~(rtc_gpio_desc[pin].mux);
            ESP_REG(rtc_reg) = ESP_REG(rtc_reg) & ~(rtc_gpio_desc[pin].pullup | rtc_gpio_desc[pin].pulldown);
        }
#endif
	// Input
        if ( pin < 32 )
            GPIO.enable_w1tc = ((uint32_t)1 << pin);
        else
            GPIO.enable1_w1tc.val = ((uint32_t)1 << (pin - 32));
    }
#endif
}

static inline __attribute__((always_inline))
void directModeOutput(IO_REG_TYPE pin)
{
#if CONFIG_IDF_TARGET_ESP32C3
    GPIO.enable_w1ts.val = ((uint32_t)1 << (pin));
#else
    if ( digitalPinIsValid(pin) && pin <= 33 ) // pins above 33 can be only inputs
    {
#if ESP_IDF_VERSION_MAJOR < 4      // IDF 3.x ESP32/PICO-D4
        uint32_t rtc_reg(rtc_gpio_desc[pin].reg);

        if ( rtc_reg ) // RTC pins PULL settings
        {
            ESP_REG(rtc_reg) = ESP_REG(rtc_reg) & ~(rtc_gpio_desc[pin].mux);
            ESP_REG(rtc_reg) = ESP_REG(rtc_reg) & ~(rtc_gpio_desc[pin].pullup | rtc_gpio_desc[pin].pulldown);
        }
#endif
        // Output
        if ( pin < 32 )
            GPIO.enable_w1ts = ((uint32_t)1 << pin);
        else // already validated to pins <= 33
            GPIO.enable1_w1ts.val = ((uint32_t)1 << (pin - 32));
    }
#endif
}

#define DIRECT_READ(base, pin)          directRead(pin)
#define DIRECT_WRITE_LOW(base, pin)     directWriteLow(pin)
#define DIRECT_WRITE_HIGH(base, pin)    directWriteHigh(pin)
#define DIRECT_MODE_INPUT(base, pin)    directModeInput(pin)
#define DIRECT_MODE_OUTPUT(base, pin)   directModeOutput(pin)

#ifdef  interrupts
#undef  interrupts
#endif
#ifdef  noInterrupts
#undef  noInterrupts
#endif
#define noInterrupts() {portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;portENTER_CRITICAL(&mux)
#define interrupts() portEXIT_CRITICAL(&mux);}
//#warning "ESP32 OneWire testing"

#elif defined(ARDUINO_ARCH_STM32)
#define PIN_TO_BASEREG(pin)             (0)
#define PIN_TO_BITMASK(pin)             ((uint32_t)digitalPinToPinName(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_BASE_ATTR
#define IO_REG_MASK_ATTR
#define DIRECT_READ(base, pin)          digitalReadFast((PinName)pin)
#define DIRECT_WRITE_LOW(base, pin)     digitalWriteFast((PinName)pin, LOW)
#define DIRECT_WRITE_HIGH(base, pin)    digitalWriteFast((PinName)pin, HIGH)
#define DIRECT_MODE_INPUT(base, pin)    pin_function((PinName)pin, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0))
#define DIRECT_MODE_OUTPUT(base, pin)   pin_function((PinName)pin, STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0))

#endif

#endif