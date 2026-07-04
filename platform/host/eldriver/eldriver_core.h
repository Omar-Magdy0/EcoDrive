#pragma once

#include <stdint.h>
#include <pthread.h>



#ifdef __cplusplus
extern "C"{
#endif

extern pthread_mutex_t eldriver_atomic_mutex;

typedef struct{



}eldriver_core_t;

void eldriver_core_init(eldriver_core_t *h);


static inline uint32_t eldriver_prof_tick()
{
    return 0;
};

static inline uint32_t eldriver_prof_tock(uint32_t start)
{
    (void)start;
    return 0;
};

/* Types */
/** @brief Type definition for global pin indexing. */
typedef uint16_t eldriver_pin_t;

/** @brief GPIO Mode enumeration mapped to LL Driver. */
typedef enum {
    ELDRIVER_GPIO_MODE_INPUT,
    ELDRIVER_GPIO_MODE_OUTPUT,
    ELDRIVER_GPIO_MODE_ALT,
    ELDRIVER_GPIO_MODE_ANALOG
} eldriver_gpio_mode_t;

/** @brief Logic state enumeration for GPIO pins. */
typedef enum {
    ELDRIVER_GPIO_LOW  = 0,
    ELDRIVER_GPIO_HIGH = 1
} eldriver_gpio_state_t;

/* Static Inline Implementation */

/**
 * @brief Sets the pin mode for a specific global pin number.
 */
static inline void eldriver_gpio_mode(eldriver_pin_t pin_num, eldriver_gpio_mode_t mode) {

}

/**
 * @brief Writes to a pin using the BSRR register (Atomic).
 * If state is HIGH, it writes to the lower 16 bits; if LOW, the upper 16 bits.
 */
static inline void eldriver_gpio_write(eldriver_pin_t pin_num, eldriver_gpio_state_t state) {

}

/**
 * @brief Toggles the pin state.
 */
static inline void eldriver_gpio_toggle(eldriver_pin_t pin_num) {

}

/**
 * @brief Reads the current input state of the pin.
 */
static inline eldriver_gpio_state_t eldriver_gpio_read(eldriver_pin_t pin_num) {
    return ELDRIVER_GPIO_LOW;
}


/**
 * @brief  Locks mutex to start an atomic section (pThread-based interrupt emulation).
 * @retval uint32_t: Lock indicator (non-zero = locked).
 * @note   Host implementation: Uses pthread_mutex_t to emulate interrupt disabling.
 */
static inline uint32_t eldriver_atomic_start(void) {
    pthread_mutex_lock(&eldriver_atomic_mutex);
    return 1;
}

/**
 * @brief  Unlocks mutex to end an atomic section.
 * @param  primask: Lock state indicator (unused, kept for API compatibility).
 * @note   Host implementation: Uses pthread_mutex_t to emulate interrupt enabling.
 */
static inline void eldriver_atomic_end(uint32_t primask) {
    (void)primask;
    pthread_mutex_unlock(&eldriver_atomic_mutex);
}

#ifdef _WIN32
    #include <windows.h>
#else
    #include <unistd.h>
#endif
static inline void eldriver_delay(uint32_t delay_ms)
{
#ifdef _WIN32
    Sleep(delay_ms);
#else
    usleep(delay_ms * 1000);
#endif
}

#ifdef __cplusplus
}
#endif
