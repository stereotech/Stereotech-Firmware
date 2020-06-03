#ifndef FIBER_CUTTER_H
#define FIBER_CUTTER_H

#include "libs/Kernel.h"
#include "libs/Pwm.h"

class FiberCutter : public Module
{
public:
    FiberCutter();
    void on_module_loaded();
    void on_config_reload(void *argument);
    void on_gcode_received(void *argument);

private:
    Pwm *futter_pin;
    struct
    {
        int pause_duration;
        int on_value;
        int off_value;
    };

    uint32_t cutter_tick(uint32_t);

    std::string on_command;
};

#endif //FIBER_CUTTER_H