#include "libs/Module.h"
#include "libs/Kernel.h"
#include <math.h>
#include "libs/Pin.h"
#include "modules/robot/Conveyor.h"
#include "FiberCutter.h"
#include "Config.h"
#include "Gcode.h"
#include "checksumm.h"
#include "SlowTicker.h"
#include "ConfigValue.h"
#include "PwmOut.h"
#include "PublicDataRequest.h"

#define fiber_cutter_enable_checksum CHECKSUM("fiber_cutter_enable")
#define fiber_cutter_pin_checksum CHECKSUM("fiber_cutter_pin")
#define fiber_cutter_on_value_checksum CHECKSUM("fiber_cutter_on_value")
#define fiber_cutter_off_value_checksum CHECKSUM("fiber_cutter_off_value")