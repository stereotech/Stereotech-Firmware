
#include "libs/Module.h"
#include "libs/Kernel.h"
#include <math.h>
#include "libs/Pin.h"
#include "modules/robot/Conveyor.h"
#include "LedStrip.h"
#include "Config.h"
#include "Gcode.h"
#include "checksumm.h"
#include "SlowTicker.h"
#include "ConfigValue.h"
#include "PwmOut.h"
#include "PublicDataRequest.h"

#define DEG_TO_RAD 0.0174532925F
#define BYTE_TO_360 1.41176471F
#define RED 0
#define GREEN 1
#define BLUE 2
#define WHITE 3

#define led_strip_enable_checksum CHECKSUM("led_strip_enable")
#define led_strip_red_pin_checksum CHECKSUM("led_strip_red_pin")
#define led_strip_green_pin_checksum CHECKSUM("led_strip_green_pin")
#define led_strip_blue_pin_checksum CHECKSUM("led_strip_blue_pin")
#define led_strip_white_pin_checksum CHECKSUM("led_strip_white_pin")

LedStrip::LedStrip() {}

void LedStrip::on_module_loaded()
{
    if (!THEKERNEL->config->value(led_strip_enable_checksum)->by_default(false)->as_bool())
    {
        delete this;
        return;
    }

    this->duration = 0;
    this->duration_set = false;

    this->register_for_event(ON_GCODE_RECEIVED);

    //settings
    this->on_config_reload(this);
}

void LedStrip::on_config_reload(void *argument)
{

    this->leds[RED] = new Pwm();
    this->leds[RED]->from_string(THEKERNEL->config->value(led_strip_red_pin_checksum)->by_default("nc")->as_string())->as_output();
    this->leds[GREEN] = new Pwm();
    this->leds[GREEN]->from_string(THEKERNEL->config->value(led_strip_green_pin_checksum)->by_default("nc")->as_string())->as_output();
    this->leds[BLUE] = new Pwm();
    this->leds[BLUE]->from_string(THEKERNEL->config->value(led_strip_blue_pin_checksum)->by_default("nc")->as_string())->as_output();
    this->leds[WHITE] = new Pwm();
    this->leds[WHITE]->from_string(THEKERNEL->config->value(led_strip_white_pin_checksum)->by_default("nc")->as_string())->as_output();

    for (uint8_t i = 0; i < 4; i++)
    {
        this->leds[i]->max_pwm(255);
    }

    this->set_color(0.0F, 0.0F, 255.0F);

    THEKERNEL->slow_ticker->attach(1000, this->leds[RED], &Pwm::on_tick);
    THEKERNEL->slow_ticker->attach(1000, this->leds[GREEN], &Pwm::on_tick);
    THEKERNEL->slow_ticker->attach(1000, this->leds[BLUE], &Pwm::on_tick);
    THEKERNEL->slow_ticker->attach(1000, this->leds[WHITE], &Pwm::on_tick);

    THEKERNEL->slow_ticker->attach(1, this, &LedStrip::led_tick);
}

void LedStrip::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if (gcode->has_m)
    {
        if (gcode->m == 150)
        {
            if (gcode->has_letter('H') && gcode->has_letter('S') && gcode->has_letter('I'))
            {
                //set color
                float h = gcode->get_value('H') * this->leds[0]->max_pwm() / 255.0F;
                float s = gcode->get_value('S') * this->leds[0]->max_pwm() / 255.0F;
                float i = gcode->get_value('I') * this->leds[0]->max_pwm() / 255.0F;

                this->set_color(h, s, i);

                if (gcode->has_letter('D'))
                {
                    //set duration
                    this->duration = roundf(gcode->get_value('D'));
                    this->duration_set = this->duration > 0;
                }
                else
                {
                    this->duration = 0;
                    this->duration_set = false;
                }
            }
            else
            {
                set_pins(this->values[RED], values[GREEN], values[BLUE], values[WHITE]);
            }
        }
    }
}

void LedStrip::set_color(float h, float s, float i)
{
    int r, g, b, w;
    float cos_h, cos_1047_h;
    h = DEG_TO_RAD * fmod(h * BYTE_TO_360, 360.0F);  // Convert to radians.
    s = s > 0.0F ? (s < 255.0F ? s : 255.0F) : 0.0F; // clamp S and I to interval [0,1]
    i = i > 0.0F ? (i < 255.0F ? i : 255.0F) : 0.0F;

    if (h < 2.09439F)
    {
        cos_h = cos(h);
        cos_1047_h = cos(1.047196667F - h);
        r = roundf(s * i / 3 * (1 + cos_h / cos_1047_h));
        g = roundf(s * i / 3 * (1 + (1 - cos_h / cos_1047_h)));
        b = 0;
    }
    else if (h < 4.188787F)
    {
        h = h - 2.09439;
        cos_h = cos(h);
        cos_1047_h = cos(1.047196667 - h);
        g = roundf(s * i / 3 * (1 + cos_h / cos_1047_h));
        b = roundf(s * i / 3 * (1 + (1 - cos_h / cos_1047_h)));
        r = 0;
    }
    else
    {
        h = h - 4.188787F;
        cos_h = cos(h);
        cos_1047_h = cos(1.047196667F - h);
        b = roundf(s * i / 3 * (1 + cos_h / cos_1047_h));
        r = roundf(s * i / 3 * (1 + (1 - cos_h / cos_1047_h)));
        g = 0;
    }
    w = roundf((255.0F - s) * i);

    this->set_pins(r, g, b, w);
}

void LedStrip::set_pins(int r, int g, int b, int w)
{
    if (r != this->leds[RED]->get_pwm() ||
        g != this->leds[GREEN]->get_pwm() ||
        b != this->leds[BLUE]->get_pwm() ||
        w != this->leds[WHITE]->get_pwm())
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            this->prev_values[i] = this->leds[i]->get_pwm();
        }
        //drain queue
        THEKERNEL->conveyor->wait_for_idle();
        this->leds[RED]->pwm(r);
        this->leds[GREEN]->pwm(g);
        this->leds[BLUE]->pwm(b);
        this->leds[WHITE]->pwm(w);
        this->values[RED] = r;
        this->values[GREEN] = g;
        this->values[BLUE] = b;
        this->values[WHITE] = w;
    }
}

uint32_t LedStrip::led_tick(uint32_t)
{
    if (duration_set)
    {
        if (--duration <= 0)
        {
            duration = 0;
            duration_set = false;
            set_pins(prev_values[0], prev_values[1], prev_values[2], prev_values[3]);
        }
    }
    return 0;
}
