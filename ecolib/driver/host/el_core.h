#pragma once

#include <stdint.h>
#include <pthread.h>



#ifdef __cplusplus
extern "C"{
#endif

extern pthread_mutex_t el_atomic_mutex;

typedef struct{



}el_core_t;

void el_core_init(el_core_t *h);


static inline uint32_t el_prof_tick()
{
    return 0;
};

static inline uint32_t el_prof_tock(uint32_t start)
{
    (void)start;
    return 0;
};

/* Types */
/** @brief Type definition for global pin indexing. */
typedef uint16_t el_pin_t;

/** @brief GPIO Mode enumeration mapped to LL Driver. */
typedef enum {
    EL_GPIO_MODE_INPUT,
    EL_GPIO_MODE_OUTPUT,
    EL_GPIO_MODE_ALT,
    EL_GPIO_MODE_ANALOG
} el_gpio_mode_t;

/** @brief Logic state enumeration for GPIO pins. */
typedef enum {
    EL_GPIO_LOW  = 0,
    EL_GPIO_HIGH = 1
} el_gpio_state_t;

/* Static Inline Implementation */

/**
 * @brief Sets the pin mode for a specific global pin number.
 */
static inline void el_gpio_mode(el_pin_t pin_num, el_gpio_mode_t mode) {

}

/**
 * @brief Writes to a pin using the BSRR register (Atomic).
 * If state is HIGH, it writes to the lower 16 bits; if LOW, the upper 16 bits.
 */
static inline void el_gpio_write(el_pin_t pin_num, el_gpio_state_t state) {

}

/**
 * @brief Toggles the pin state.
 */
static inline void el_gpio_toggle(el_pin_t pin_num) {

}

/**
 * @brief Reads the current input state of the pin.
 */
static inline el_gpio_state_t el_gpio_read(el_pin_t pin_num) {
    return EL_GPIO_LOW;
}


/**
 * @brief  Locks mutex to start an atomic section (pThread-based interrupt emulation).
 * @retval uint32_t: Lock indicator (non-zero = locked).
 * @note   Host implementation: Uses pthread_mutex_t to emulate interrupt disabling.
 */
static inline uint32_t el_atomic_start(void) {
    pthread_mutex_lock(&el_atomic_mutex);
    return 1;
}

/**
 * @brief  Unlocks mutex to end an atomic section.
 * @param  primask: Lock state indicator (unused, kept for API compatibility).
 * @note   Host implementation: Uses pthread_mutex_t to emulate interrupt enabling.
 */
static inline void el_atomic_end(uint32_t primask) {
    (void)primask;
    pthread_mutex_unlock(&el_atomic_mutex);
}

#ifdef _WIN32
    #include <windows.h>
#else
    #include <unistd.h>
#endif
static inline void el_delay(uint32_t delay_ms)
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
