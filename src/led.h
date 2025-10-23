// led.h
#ifndef LED_H
#define LED_H

#include <stdint.h>

/* MIO pin numbers */
#define MIO_PIN_10  10
#define MIO_PIN_12  12

/* SLCR (pinmux) */
#define SLCR_BASE       0xF8000000UL
#define SLCR_MAP_SIZE   0x1000
#define SLCR_UNLOCK_OFF 0x08
#define SLCR_LOCK_OFF   0x04
#define SLCR_UNLOCK_KEY 0x0000DF0D
#define SLCR_LOCK_KEY   0x0000767B
#define MIO_PIN_BASE    0x700     // SLCR MIO_PIN_0 base
#define MIO_PIN(n)      (MIO_PIN_BASE + ((n) * 4))

/* Configuration value to set MIO pin as GPIO (FUNCSEL=0 with reasonable IO settings).
 * You can change to 0x0 if you want the simplest value.
 */
#define MIO_GPIO_CFG  0x00001600U 

/* PS GPIO */
#define GPIO_BASE       0xE000A000UL
#define GPIO_MAP_SIZE   0x1000
#define BANK0           0
#define DIRM_OFFSET     0x204
#define OUTEN_OFFSET    0x208
#define DATA_OFFSET     0x40
#define BANK_STRIDE     0x40
#define DATA_BANK_STRIDE 0x4

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize SLCR and PS GPIO mapping and configure pins as outputs.
 * Returns 0 on success, negative on error.
 */
int leds_init(void);

/* Set a MIO pin (use MIO_PIN_10 or MIO_PIN_12) to value 0 or 1.
 * Returns 0 on success, negative on error.
 */
int leds_set(uint32_t pin, int value);

/* Cleanup resources (unmap / close) */
void leds_cleanup(void);

/* Use externally provided mappings (no open, no mmap, no close, no munmap) */
int leds_init_from_maps(void *slcr_map, void *gpio_map);


#ifdef __cplusplus
}
#endif

#endif // LED_H
