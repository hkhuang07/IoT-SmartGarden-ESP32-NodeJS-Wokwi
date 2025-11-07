#include <stdio.h>
#include <stdlib.h>
#include <stdint.h> // Đảm bảo uint32_t được định nghĩa

typedef uint32_t pin_t;

// Định nghĩa các loại pin (Các định nghĩa này vẫn cần thiết cho chip.c)
#define INPUT           (0)
#define OUTPUT          (1)
#define ANALOG_OUTPUT   (2) 
  
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
    printf("ERROR: Failed to allocate memory for chip data!\n");
    return;
  }

  // Khởi tạo chân AO là ANALOG_OUTPUT
  chip->ao_pin = pin_init("AO", ANALOG_OUTPUT); 
  
  chip_state_set(chip);

  printf("Soil Moisture Sensor Chip initialized.\n");
}

void chip_loop() {
  chip_state_t *chip = (chip_state_t*)chip_state_get(); 
  if (chip == NULL) return;

  // Lấy giá trị từ slider (Float)
  float adc_value_float = get_param_float("adc_value"); 
  
  // Ép kiểu sang uint32_t để ghi vào chân pin
  uint32_t adc_value = (uint32_t)adc_value_float;

  // Ghi giá trị ADC giả lập vào chân AO
  pin_write(chip->ao_pin, adc_value);
}