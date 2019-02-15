#ifndef LEDSTRIP_H
#define LEDSTRIP_H

#include "libs/Kernel.h"
#include "libs/Pwm.h"

class LedStrip : public Module
{
public:
  LedStrip();
  void on_module_loaded();
  void on_config_reload(void *argument);
  void on_gcode_received(void *argument);

private:
  Pwm *leds[4];
  int prev_values[4];
  int values[4];
  struct
  {
    int duration;
    bool duration_set;
  };

  void set_color(float h, float s, float i);
  void set_pins(int r, int g, int b, int w);

  uint32_t led_tick(uint32_t);
};

#endif //LEDSTRIP_H