#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef uint32_t pin_t;

#define ANALOG_OUTPUT   (2) 
  
extern pin_t pin_init(const char *name, uint32_t mode);
extern void pin_write(pin_t pin, uint32_t value);
extern void chip_state_set(void *state);
extern void *chip_state_get();
extern float get_param_float(const char *name);


typedef struct {
  pin_t ao_n_pin;           
  pin_t bo_p_pin;
  pin_t co_k_pin;
} chip_state_t;

void chip_init() {
  chip_state_t *chip = (chip_state_t*)malloc(sizeof(chip_state_t));
  if (chip == NULL) {
    printf("ERROR: Failed to allocate memory for chip data!\n");
    return;
  }

  chip->ao_n_pin = pin_init("AO_N", ANALOG_OUTPUT); 
  chip->bo_p_pin = pin_init("BO_P", ANALOG_OUTPUT); 
  chip->co_k_pin = pin_init("CO_K", ANALOG_OUTPUT); 
  
  chip_state_set(chip);

  printf("NPK Sensor Chip initialized.\n");
}

void chip_loop() {
  chip_state_t *chip = (chip_state_t*)chip_state_get(); 
  if (chip == NULL) return;

  uint32_t n_value = (uint32_t)get_param_float("nitrogen_value"); 
  uint32_t p_value = (uint32_t)get_param_float("phosphorus_value");
  uint32_t k_value = (uint32_t)get_param_float("potassium_value");

  pin_write(chip->ao_n_pin, n_value);
  pin_write(chip->bo_p_pin, p_value);
  pin_write(chip->co_k_pin, k_value);
}