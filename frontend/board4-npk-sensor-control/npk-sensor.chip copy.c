#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

typedef uint32_t pin_t;
#define ANALOG_OUTPUT 2
  
extern pin_t pin_init(const char *name, uint32_t mode);
extern void pin_write(pin_t pin, uint32_t value);
extern void chip_state_set(void *state);
extern void *chip_state_get();
extern float get_param_float(const char *name);

typedef struct {
  pin_t ao_n_pin;
  pin_t bo_p_pin;
  pin_t co_k_pin;
  
  // Separate tracking for each independent parameter
  float nitrogen_current;
  float phosphorus_current;
  float potassium_current;
  
  // Pin output values (what's actually written to pins)
  uint32_t nitrogen_output;
  uint32_t phosphorus_output;
  uint32_t potassium_output;
  
  // Update flags to track changes (int instead of bool for Wokwi compatibility)
  int nitrogen_updated;
  int phosphorus_updated;
  int potassium_updated;
  
  // Loop counter for debugging
  unsigned long loop_count;
} chip_state_t;

void chip_init() {
  chip_state_t *chip = (chip_state_t*)malloc(sizeof(chip_state_t));
  if (chip == NULL) {
    printf("ERROR: Failed to allocate memory for NPK chip!\n");
    return;
  }

  // Initialize all values to zero
  memset(chip, 0, sizeof(chip_state_t));
  
  // Set default independent values
  chip->nitrogen_current = 2000.0f;
  chip->phosphorus_current = 1500.0f;
  chip->potassium_current = 1000.0f;
  
  // Initialize pins
  chip->ao_n_pin = pin_init("AO_N", ANALOG_OUTPUT);
  chip->bo_p_pin = pin_init("BO_P", ANALOG_OUTPUT);
  chip->co_k_pin = pin_init("CO_K", ANALOG_OUTPUT);
  
  // Write initial values to pins
  chip->nitrogen_output = (uint32_t)chip->nitrogen_current;
  chip->phosphorus_output = (uint32_t)chip->phosphorus_current;
  chip->potassium_output = (uint32_t)chip->potassium_current;
  
  pin_write(chip->ao_n_pin, chip->nitrogen_output);
  pin_write(chip->bo_p_pin, chip->phosphorus_output);
  pin_write(chip->co_k_pin, chip->potassium_output);
  
  chip_state_set(chip);
  
  printf("NPK Sensor Chip v2.4-STRING-CONCATENATION-FIXED initialized!\n");
  printf("Pins: AO_N=%u, BO_P=%u, CO_K=%u\n", 
         (unsigned int)chip->ao_n_pin,
         (unsigned int)chip->bo_p_pin,
         (unsigned int)chip->co_k_pin);
  printf("Independent Control: N=%.0f, P=%.0f, K=%.0f\n", 
         chip->nitrogen_current, chip->phosphorus_current, chip->potassium_current);
  printf("Each slider now controls exactly ONE parameter!\n");
}

void chip_loop() {
  chip_state_t *chip = (chip_state_t*)chip_state_get();
  if (chip == NULL) {
    printf("ERROR: NPK chip state is NULL!\n");
    return;
  }

  chip->loop_count++;
  
  // Reset update flags
  chip->nitrogen_updated = 0;
  chip->phosphorus_updated = 0;
  chip->potassium_updated = 0;
  
  // READ EACH PARAMETER INDEPENDENTLY
  // Nitrogen slider controls ONLY nitrogen
  float new_nitrogen = get_param_float("nitrogen_slider");
  if (isnan(new_nitrogen)) new_nitrogen = chip->nitrogen_current;
  
  // Phosphorus slider controls ONLY phosphorus  
  float new_phosphorus = get_param_float("phosphorus_slider");
  if (isnan(new_phosphorus)) new_phosphorus = chip->phosphorus_current;
  
  // Potassium slider controls ONLY potassium
  float new_potassium = get_param_float("potassium_slider");
  if (isnan(new_potassium)) new_potassium = chip->potassium_current;
  
  // Validate ranges (500-3000 ADC)
  if (new_nitrogen < 500.0f) new_nitrogen = 500.0f;
  if (new_nitrogen > 3000.0f) new_nitrogen = 3000.0f;
  
  if (new_phosphorus < 500.0f) new_phosphorus = 500.0f;
  if (new_phosphorus > 3000.0f) new_phosphorus = 3000.0f;
  
  if (new_potassium < 500.0f) new_potassium = 500.0f;
  if (new_potassium > 3000.0f) new_potassium = 3000.0f;
  
  // UPDATE NITROGEN ONLY if changed
  if (fabsf(new_nitrogen - chip->nitrogen_current) > 0.5f) {
    chip->nitrogen_current = new_nitrogen;
    chip->nitrogen_output = (uint32_t)roundf(new_nitrogen);
    pin_write(chip->ao_n_pin, chip->nitrogen_output);
    chip->nitrogen_updated = 1;
    printf("NITROGEN updated: %.0f -> %.0f (Pin AO_N: %u)\n", 
           chip->nitrogen_current, new_nitrogen, chip->nitrogen_output);
  }
  
  // UPDATE PHOSPHORUS ONLY if changed
  if (fabsf(new_phosphorus - chip->phosphorus_current) > 0.5f) {
    chip->phosphorus_current = new_phosphorus;
    chip->phosphorus_output = (uint32_t)roundf(new_phosphorus);
    pin_write(chip->bo_p_pin, chip->phosphorus_output);
    chip->phosphorus_updated = 1;
    printf("PHOSPHORUS updated: %.0f -> %.0f (Pin BO_P: %u)\n", 
           chip->phosphorus_current, new_phosphorus, chip->phosphorus_output);
  }
  
  // UPDATE POTASSIUM ONLY if changed  
  if (fabsf(new_potassium - chip->potassium_current) > 0.5f) {
    chip->potassium_current = new_potassium;
    chip->potassium_output = (uint32_t)roundf(new_potassium);
    pin_write(chip->co_k_pin, chip->potassium_output);
    chip->potassium_updated = 1;
    printf("POTASSIUM updated: %.0f -> %.0f (Pin CO_K: %u)\n", 
           chip->potassium_current, new_potassium, chip->potassium_output);
  }
  
  // Print status every 500 loops
  if (chip->loop_count % 500 == 0) {
    printf("\n=== NPK STATUS (Loop %lu) ===\n", chip->loop_count);
    printf("Current Independent Values:\n");
    printf("  Nitrogen:   %.0f (Pin AO_N: %u) %s\n", 
           chip->nitrogen_current, chip->nitrogen_output, 
           chip->nitrogen_updated ? "[UPDATED]" : "");
    printf("  Phosphorus: %.0f (Pin BO_P: %u) %s\n", 
           chip->phosphorus_current, chip->phosphorus_output,
           chip->phosphorus_updated ? "[UPDATED]" : "");
    printf("  Potassium:  %.0f (Pin CO_K: %u) %s\n", 
           chip->potassium_current, chip->potassium_output,
           chip->potassium_updated ? "[UPDATED]" : "");
    printf("Each slider controls exactly ONE output pin!\n");
    printf("=============================\n\n");
  }
}