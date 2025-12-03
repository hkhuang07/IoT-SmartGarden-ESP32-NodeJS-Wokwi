#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef uint32_t pin_t;

#define ANALOG_OUTPUT 2
  
extern pin_t pin_init(const char *name, uint32_t mode);
extern void pin_write(pin_t pin, uint32_t value);
extern void chip_state_set(void *state);
extern void *chip_state_get();
extern float get_param_float(const char *name);

typedef struct {
  pin_t ao_pin;
} chip_state_t;

void chip_init() {
  chip_state_t *chip = (chip_state_t*)malloc(sizeof(chip_state_t));
  if (chip == NULL) {
    printf("ERROR: Failed to allocate memory for soil moisture chip!\n");
    return;
  }

  chip->ao_pin = pin_init("AO", ANALOG_OUTPUT);
  chip_state_set(chip);

  printf("Soil Moisture Sensor Chip initialized.\n");
}

void chip_loop() {
  chip_state_t *chip = (chip_state_t*)chip_state_get();
  if (chip == NULL) {
    printf("ERROR: Soil moisture chip state is NULL!\n");
    return;
  }

  uint32_t moisture_value = (uint32_t)get_param_float("moisture_value");
  pin_write(chip->ao_pin, moisture_value);
}