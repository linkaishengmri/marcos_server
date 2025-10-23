// led.c
#define _GNU_SOURCE
#include "led.h"
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdlib.h>



/* Internal state */
static int g_memfd = -1;
static void *g_slcr_map = MAP_FAILED;
static void *g_gpio_map = MAP_FAILED;
static int g_owns_mappings = 0;

/* helper to map physical region (page aligned mapping) */
static void *map_phys_region(int fd, off_t phys_base, size_t map_size) {
    /* phys_base is page aligned in our uses (SLCR and GPIO are page aligned),
       so a direct mmap is OK. */
    void *m = mmap(NULL, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, phys_base);
    if (m == MAP_FAILED) {
        return MAP_FAILED;
    }
    return m;
}


static int leds_configure_gpio_after_maps(void) {
    if (g_slcr_map == MAP_FAILED || g_gpio_map == MAP_FAILED) return -1;

    /* Unlock SLCR */
    volatile uint32_t *slcr_unlock = (volatile uint32_t *)((uint8_t*)g_slcr_map + SLCR_UNLOCK_OFF);
    volatile uint32_t *slcr_lock   = (volatile uint32_t *)((uint8_t*)g_slcr_map + SLCR_LOCK_OFF);

    *slcr_unlock = SLCR_UNLOCK_KEY;
    usleep(1000);

    /* Set MIO10 and MIO12 to GPIO by writing MIO_PIN_x registers */
    volatile uint32_t *mio10_reg = (volatile uint32_t *)((uint8_t*)g_slcr_map + MIO_PIN(MIO_PIN_10));
    volatile uint32_t *mio12_reg = (volatile uint32_t *)((uint8_t*)g_slcr_map + MIO_PIN(MIO_PIN_12));

    *mio10_reg = MIO_GPIO_CFG;
    *mio12_reg = MIO_GPIO_CFG;
    usleep(1000);

    /* Lock SLCR */
    *slcr_lock = SLCR_LOCK_KEY;
    usleep(1000);

    /* Configure PS GPIO: set DIRM and OUTEN bits for MIO10 & MIO12 in bank0 */
    volatile uint32_t *dirm  = (volatile uint32_t *)((uint8_t*)g_gpio_map + DIRM_OFFSET + BANK0 * BANK_STRIDE);
    volatile uint32_t *outen = (volatile uint32_t *)((uint8_t*)g_gpio_map + OUTEN_OFFSET + BANK0 * BANK_STRIDE);

    uint32_t mask = (1u << MIO_PIN_10) | (1u << MIO_PIN_12);

    /* set direction bits (output) */
    uint32_t tmp = *dirm;
    tmp |= mask;
    *dirm = tmp;

    /* enable output */
    tmp = *outen;
    tmp |= mask;
    *outen = tmp;

    return 0;
}


int leds_init(void) {
    if (g_memfd >= 0) return 0; 

    g_memfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (g_memfd < 0) {
        perror("open(/dev/mem)");
        return -1;
    }

    g_slcr_map = map_phys_region(g_memfd, SLCR_BASE, SLCR_MAP_SIZE);
    if (g_slcr_map == MAP_FAILED) {
        perror("mmap SLCR");
        close(g_memfd);
        g_memfd = -1;
        return -2;
    }

    g_gpio_map = map_phys_region(g_memfd, GPIO_BASE, GPIO_MAP_SIZE);
    if (g_gpio_map == MAP_FAILED) {
        perror("mmap GPIO");
        munmap(g_slcr_map, SLCR_MAP_SIZE);
        g_slcr_map = MAP_FAILED;
        close(g_memfd);
        g_memfd = -1;
        return -3;
    }

    g_owns_mappings = 1; 
    return leds_configure_gpio_after_maps();
}


int leds_init_from_maps(void *slcr_map, void *gpio_map) {
    if (g_memfd >= 0 || g_owns_mappings) {
        return -1;
    }
    if (slcr_map == NULL || gpio_map == NULL) return -2;

    g_slcr_map = slcr_map;
    g_gpio_map = gpio_map;
    g_owns_mappings = 0; 

    return leds_configure_gpio_after_maps();
}

/* Set pin value */
int leds_set(uint32_t pin, int value) {
    if (g_gpio_map == MAP_FAILED) return -1;
    if (!(pin == MIO_PIN_10 || pin == MIO_PIN_12)) return -2;

    volatile uint32_t *data_reg = (volatile uint32_t *)((uint8_t*)g_gpio_map + DATA_OFFSET + BANK0 * DATA_BANK_STRIDE);

    uint32_t cur = *data_reg;
    if (value)
        cur |= (1u << pin);
    else
        cur &= ~(1u << pin);
    *data_reg = cur;

    return 0;
}


void leds_cleanup(void) {
    if (g_owns_mappings) {
        if (g_gpio_map != MAP_FAILED) {
            munmap(g_gpio_map, GPIO_MAP_SIZE);
            g_gpio_map = MAP_FAILED;
        }
        if (g_slcr_map != MAP_FAILED) {
            munmap(g_slcr_map, SLCR_MAP_SIZE);
            g_slcr_map = MAP_FAILED;
        }
        if (g_memfd >= 0) {
            close(g_memfd);
            g_memfd = -1;
        }
        g_owns_mappings = 0;
    } else {
        g_slcr_map = MAP_FAILED;
        g_gpio_map = MAP_FAILED;
        g_memfd = -1;
    }
}
