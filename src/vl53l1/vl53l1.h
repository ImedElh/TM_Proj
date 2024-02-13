#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/logging/log.h>

// this near top of sketch
typedef union tempFloat
{
  uint8_t byte[4];
  float f;
} temperatureFloat_t;

typedef union tempDoub
{
  uint8_t byte[8];
  double d;
} temperatureDoub_t;