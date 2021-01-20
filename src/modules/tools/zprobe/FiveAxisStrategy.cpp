#include "FiveAxisStrategy.h"

#include "Kernel.h"
#include "Config.h"
#include "Robot.h"
#include "StreamOutputPool.h"
#include "Gcode.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "Conveyor.h"
#include "ZProbe.h"
#include "nuts_bolts.h"
#include "utils.h"
#include "platform_memory.h"

#include <string>
#include <memory>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <fastmath.h>

#define probe_point_1_checksum CHECKSUM("point1")
#define probe_point_2_checksum CHECKSUM("point2")
#define probe_point_3_checksum CHECKSUM("point3")
#define probe_point_4_checksum CHECKSUM("point4")
#define probe_point_5_checksum CHECKSUM("point5")
#define probe_point_6_checksum CHECKSUM("point6")
#define probe_point_7_checksum CHECKSUM("point7")
#define probe_point_8_checksum CHECKSUM("point8")
#define probe_point_9_checksum CHECKSUM("point9")

#define probe_device_big_part_length_checksum CHECKSUM("big_part_length")
#define probe_device_small_part_length_checksum CHECKSUM("small_part_length")
#define five_axis_home_checksum CHECKSUM("do_home_first")

#define CALIBRFILE "/sd/fiveaxis.calibr"

#define B 0
#define X0 1
#define Z0 2
#define Y0 3
#define EXA 4
#define EYA 5
#define EZA 6
#define XOC 7
#define YOC 8
#define ZOC 9

template <typename... Args>
std::string string_format(const std::string &format, Args... args)
{
    size_t size = snprintf(nullptr, 0, format.c_str(), args...) + 1; // Extra space for '\0'
    if (size <= 0)
    {
        return "";
    }
    std::unique_ptr<char[]> buf(new char[size]);
    snprintf(buf.get(), size, format.c_str(), args...);
    return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
}

FiveAxisStrategy::FiveAxisStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe)
{
    for (size_t i = 0; i < 10; i++)
    {
        this->probe_points[i] = std::make_tuple(NAN, NAN, NAN);
    }
}

FiveAxisStrategy::~FiveAxisStrategy() {}

bool FiveAxisStrategy::handleConfig()
{
    big_part_length = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_device_big_part_length_checksum)->by_default(10.0F)->as_number();
    small_part_length = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_device_small_part_length_checksum)->by_default(10.0F)->as_number();
    home = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, five_axis_home_checksum)->by_default(true)->as_bool();

    std::string p1 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_1_checksum)->by_default("")->as_string();
    std::string p2 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_2_checksum)->by_default("")->as_string();
    std::string p3 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_3_checksum)->by_default("")->as_string();
    std::string p4 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_4_checksum)->by_default("")->as_string();
    std::string p5 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_5_checksum)->by_default("")->as_string();
    std::string p6 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_6_checksum)->by_default("")->as_string();
    std::string p7 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_7_checksum)->by_default("")->as_string();
    std::string p8 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_8_checksum)->by_default("")->as_string();
    std::string p9 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_9_checksum)->by_default("")->as_string();
    //std::string p10 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_10_checksum)->by_default("")->as_string();

    if (!p1.empty())
        probe_points[0] = parseXYZ(p1.c_str());
    if (!p2.empty())
        probe_points[1] = parseXYZ(p2.c_str());
    if (!p3.empty())
        probe_points[2] = parseXYZ(p3.c_str());
    if (!p4.empty())
        probe_points[3] = parseXYZ(p4.c_str());
    if (!p5.empty())
        probe_points[4] = parseXYZ(p5.c_str());
    if (!p6.empty())
        probe_points[5] = parseXYZ(p6.c_str());
    if (!p7.empty())
        probe_points[6] = parseXYZ(p7.c_str());
    if (!p8.empty())
        probe_points[7] = parseXYZ(p8.c_str());

    for (int i = 0; i < 9; i++)
    {
        actual_probe_points[i] = std::make_tuple(0, 0, 0);
    }

    reset_calibr();

    return true;
}

bool FiveAxisStrategy::handleGcode(Gcode *gcode)
{
    if (gcode->has_m)
    {
        if (gcode->m == 557)
        { // M557 - set probe points eg M557 P0 X30 Y40.5  where P is 0,1,2
            int idx = 0;
            float x = NAN, y = NAN, z = NAN;
            if (gcode->has_letter('P'))
                idx = gcode->get_value('P');
            if (gcode->has_letter('X'))
                x = gcode->get_value('X');
            if (gcode->has_letter('Y'))
                y = gcode->get_value('Y');
            if (gcode->has_letter('Z'))
                z = gcode->get_value('Z');
            if (idx >= 0 && idx <= 8)
            {
                probe_points[idx] = std::make_tuple(x, y, z);
            }
            else
            {
                gcode->stream->printf("only 8 probe points allowed P0-P8\n");
            }
            return true;
        }
        else if (gcode->m == 565)
        {
            int idx = 0;
            float value = 0;
            if (gcode->has_letter('S'))
                idx = gcode->get_value('S');
            if (gcode->has_letter('P'))
                value = gcode->get_value('P');
            if (idx >= 0 && idx <= 9)
            {
                calibration[idx] = value;
            }
            else
            {
                gcode->stream->printf("only 10 calibration values allowed S0-S9\n");
            }
            return true;
        }
        else if (gcode->m == 370 || gcode->m == 561)
        {
            setFirstAdjustFunction(false);
            reset_calibr();
            gcode->stream->printf("calibration cleared and disabled\n");
            return true;
        }
        else if (gcode->m == 374)
        {
            if (gcode->subcode == 1)
            {
                remove(CALIBRFILE);
                gcode->stream->printf("%s deleted\n", CALIBRFILE);
            }
            else
            {
                save_calibr(gcode->stream);
            }
            return true;
        }
        else if (gcode->m == 375)
        {
            if (gcode->subcode == 1)
            {
                print_calibr(gcode->stream);
            }
            else
            {
                if (load_calibr(gcode->stream))
                    setFinalAdjustFunction(true);
            }
            return true;
        }
        else if (gcode->m == 500 || gcode->m == 503)
        {
            gcode->stream->printf(";Home: %s, big: %1.3f, small: %1.3f\n", home ? "true" : "false", big_part_length, small_part_length);
            for (size_t i = 0; i < 10; i++)
            {
                float x, y, z;
                std::tie(x, y, z) = probe_points[i];
                gcode->stream->printf("Probe point %d: %1.3f,%1.3f,%1.3f", i, x, y, z);
            }
            gcode->stream->printf(";Load saved calibration\nM375\n");
            return true;
        }
        else if (gcode->m == 1005)
        {
            uint8_t stepNum = 0;
            if (gcode->has_letter('S'))
                stepNum = gcode->get_uint('S');
            gotoStep(stepNum, gcode->stream);
            return true;
        }
        else if (gcode->m == 1006)
        {
            uint8_t functionNum = 0;
            uint8_t enable = 0;
            if (gcode->has_letter('S'))
                functionNum = gcode->get_uint('S');
            if (gcode->has_letter('P'))
                enable = gcode->get_uint('P');
            if (functionNum == 0)
            {
                setFirstAdjustFunction(enable > 0);
            }
            else if (functionNum == 1)
            {
                setSecondAdjustFunction(enable > 0);
            }
            else if (functionNum == 2)
            {
                setFinalAdjustFunction(enable > 0);
            }
            else
            {
                gcode->stream->printf(";Wrong function number\n");
            }
        }
    }
    return false;
}

void FiveAxisStrategy::save_calibr(StreamOutput *stream)
{
    FILE *fp = fopen(CALIBRFILE, "w");
    if (fp == NULL)
    {
        stream->printf("error:Failed to open calibration file %s\n", CALIBRFILE);
        return;
    }
    for (size_t i = 0; i < 10; i++)
    {
        if (fwrite(&calibration[i], sizeof(float), 1, fp) != 1)
        {
            stream->printf("error:Failed to write calibration\n");
            fclose(fp);
            return;
        }
        else
        {
            stream->printf("Write %1.3f\n", calibration[i]);
        }
    }
    stream->printf("calibration saved to %s\n", CALIBRFILE);
    fclose(fp);
}

bool FiveAxisStrategy::load_calibr(StreamOutput *stream)
{
    FILE *fp = fopen(CALIBRFILE, "r");
    if (fp == NULL)
    {
        stream->printf("error:Failed to open calibration file %s\n", CALIBRFILE);
        return false;
    }
    for (size_t i = 0; i < 10; i++)
    {
        if (fread(&calibration[i], sizeof(float), 1, fp) != 1)
        {
            stream->printf("error:Failed to read calibration\n");
            fclose(fp);
            return false;
        }
        else
        {
            stream->printf("Read %1.3f\n", calibration[i]);
        }
    }
    stream->printf("calibration loaded");
    fclose(fp);
    return true;
}

void FiveAxisStrategy::reset_calibr()
{
    for (size_t i = 0; i < 10; i++)
    {
        calibration[i] = 0;
    }
}

void FiveAxisStrategy::print_calibr(StreamOutput *stream)
{
    stream->printf("Beta correction: %1.4f\n", calibration[B]);
    stream->printf("Real A axis rotation point: X - %1.4f, Y - %1.4f, Z - %1.4f\n", calibration[X0], calibration[Y0], calibration[Z0]);
    stream->printf("A axis correction: %1.4f %1.4f %1.4f\n", calibration[EXA], calibration[EYA], calibration[EZA]);
    stream->printf("C axis correction: %1.4f %1.4f %1.4f\n", calibration[XOC], calibration[YOC], calibration[ZOC]);
}

void FiveAxisStrategy::gotoStep(uint8_t step, StreamOutput *stream)
{
    if (step == 0)
    {
        THEKERNEL->conveyor->wait_for_idle();
        setFirstAdjustFunction(false);

        if (home)
        {
            Gcode gc(THEKERNEL->is_grbl_mode() ? "G28.2" : "G28", &(StreamOutput::NullStream));
            THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
        }

        Gcode beforeHome("M206 A0", &(StreamOutput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &beforeHome);
        Gcode homeA("G28 A0", &(StreamOutput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &homeA);
        Gcode afterHome("G0 Z100", &(StreamOutput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &afterHome);
    }
    else if (step == 1)
    {
        Gcode rotateA("G0 A0", &(StreamOutput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &rotateA);

        //Move to the first point
        float x, y, z;
        std::tie(x, y, z) = probe_points[0];
        zprobe->coordinated_move(x, y, z, zprobe->getFastFeedrate());
        stream->printf("Moving to probe point 1: x%1.3f y%1.3f z%1.3f\n", x, y, z);
    }
    else if (step == 2)
    {
        //Save first point
        float position[3];
        THEROBOT->get_axis_position(position);
        actual_probe_points[0] = std::make_tuple(position[0], position[1], position[2]);
        stream->printf("Probe point 1 at: x%1.3f y%1.3f z%1.3f\n", position[0], position[1], position[2]);
        //Move to the second point
        float x, y, z;
        std::tie(x, y, z) = probe_points[1];
        zprobe->coordinated_move(x, y, z, zprobe->getFastFeedrate());
        stream->printf("Moving to probe point 2: x%1.3f y%1.3f z%1.3f\n", x, y, z);
    }
    else if (step == 3)
    {
        setAAxisZero(stream);
    }
    else if (step == 4)
    {
        preLinearCorrection(stream);
    }
    else if (step == 5)
    {
        setBAxisCorrection(stream);
    }
    else if (step == 6)
    {
        linearCorrection(stream);
    }
    else if (step == 7)
    {
        preCAxisBeatingCorrection(stream);
    }
    else if (step == 8)
    {
        cAxisBeatingCorrection(stream);
    }
    else if (step == 9)
    {
        preAAxisBeatingCorrection(stream);
    }
    else if (step == 10)
    {
        aAxisBeatingCorrection(stream);
    }
}

void FiveAxisStrategy::setAAxisZero(StreamOutput *stream)
{
    //Save second point
    float position[3];
    THEROBOT->get_axis_position(position);
    actual_probe_points[1] = std::make_tuple(position[0], position[1], position[2]);
    stream->printf("Probe point 2 at: x%1.3f y%1.3f z%1.3f\n", position[0], position[1], position[2]);

    float c_offset = 0;
    float x1, x2;
    x1 = std::get<0>(actual_probe_points[0]);
    x2 = std::get<0>(actual_probe_points[1]);
    //std::tie(x1, y1, z1) = actual_probe_points[0];
    //std::tie(x2, y2, z2) = actual_probe_points[1];
    c_offset = 57.2958 * asinf((x2 - x1) / (big_part_length + small_part_length));
    c_offset /= 3.0;

    Gcode homeC("G92 C0\n", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &homeC);

    if (!isnan(c_offset))
    {
        string cmdc = string_format("G0 C%1.3f\n", c_offset);
        Gcode cOffsetGcode(cmdc, &(StreamOutput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &cOffsetGcode);
        stream->printf("C axis offset is:%1.3f\n", c_offset);
        stream->printf("Big part:%1.3f\n", big_part_length);
        stream->printf("Small part:%1.3f\n", small_part_length);
        Gcode homeC("G92 C0\n", &(StreamOutput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &homeC);
    }

    float a_offset = 0;
    float z1, z2;
    z1 = std::get<2>(actual_probe_points[0]);
    z2 = std::get<2>(actual_probe_points[1]);
    a_offset = -57.2958 * asinf((z2 - z1) / ((big_part_length + small_part_length) * cosf(c_offset / 57.2958)));
    a_offset /= 1.5;

    zprobe->coordinated_move(NAN, NAN, position[2] + 20, zprobe->getFastFeedrate());
    stream->printf("A axis offset is:%1.3f\n", a_offset);
    if (!isnan(a_offset))
    {
        string cmd = string_format("M206 A%1.3f\n", a_offset);
        Gcode aOffsetGcode(cmd, &(StreamOutput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &aOffsetGcode);
    }

    Gcode homeA("G28 A0\n", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &homeA);
    Gcode zeroA("G0 A0\n", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &zeroA);

    //Move to the third point
    float x, y, z;
    std::tie(x, y, z) = probe_points[2];
    zprobe->coordinated_move(x, y, z, zprobe->getFastFeedrate());
    stream->printf("Moving to probe point 3: x%1.3f y%1.3f z%1.3f\n", x, y, z);
}

void FiveAxisStrategy::preLinearCorrection(StreamOutput *stream)
{
    //Save third point
    float position[3];
    THEROBOT->get_axis_position(position);
    actual_probe_points[2] = std::make_tuple(position[0], position[1], position[2]);
    stream->printf("Probe point 3 at: x%1.3f y%1.3f z%1.3f\n", position[0], position[1], position[2]);

    //Move to the fourth point
    float x, y, z;
    std::tie(x, y, z) = probe_points[3];
    zprobe->coordinated_move(x, y, z, zprobe->getFastFeedrate());
    stream->printf("Moving to probe point 4: x%1.3f y%1.3f z%1.3f\n", x, y, z);
}

void FiveAxisStrategy::setBAxisCorrection(StreamOutput *stream)
{
    //Save fourth point
    float position[3];
    THEROBOT->get_axis_position(position);
    actual_probe_points[3] = std::make_tuple(position[0], position[1], position[2]);
    stream->printf("Probe point 4 at: x%1.3f y%1.3f z%1.3f\n", position[0], position[1], position[2]);

    calibration[B] = 0;
    float z3, z4;
    z3 = std::get<2>(actual_probe_points[2]);
    z4 = std::get<2>(actual_probe_points[3]);
    calibration[B] = asinf((z4 - z3) / ((this->big_part_length + this->small_part_length)));
    stream->printf("B axis angle: b%1.3f\n", calibration[B] * 57.2958);
    float x3;
    x3 = std::get<0>(actual_probe_points[2]);
    float cosb_2 = cosf(calibration[B] / 2);
    float sinb_2 = sinf(calibration[B] / 2);
    float cosb_m1 = 1 - cosf(calibration[B]);
    float l = 2 * (this->big_part_length + this->small_part_length) * (this->big_part_length + this->small_part_length);
    calibration[X0] = x3 - (calibration[B] / abs(calibration[B])) * cosb_2 * l * cosb_m1;
    calibration[Y0] = position[1];
    calibration[Z0] = z3 - abs(sinb_2) * l * cosb_m1 + (this->big_part_length + this->small_part_length);
    stream->printf("Real A axis rotation point: x%1.3f y%1.3f z%1.3f\n", calibration[X0], position[Y0], calibration[Z0]);
    setFirstAdjustFunction(true);

    //Move to the fifth point
    float x, y, z;
    std::tie(x, y, z) = probe_points[4];
    zprobe->coordinated_move(x, y, z, zprobe->getFastFeedrate());
    stream->printf("Moving to probe point 5: x%1.3f y%1.3f z%1.3f\n", x, y, z);
}

void FiveAxisStrategy::linearCorrection(StreamOutput *stream)
{
    //Save fifth point
    float position[3];
    THEROBOT->get_axis_position(position);
    actual_probe_points[4] = std::make_tuple(position[0], position[1], position[2]);
    stream->printf("Probe point 5 at: x%1.3f y%1.3f z%1.3f\n", position[0], position[1], position[2]);

    //Move to the sixth point
    float x, y, z;
    std::tie(x, y, z) = probe_points[5];
    zprobe->coordinated_move(x, y, z, zprobe->getFastFeedrate());
    stream->printf("Moving to probe point 6: x%1.3f y%1.3f z%1.3f\n", x, y, z);
}

void FiveAxisStrategy::preCAxisBeatingCorrection(StreamOutput *stream)
{
    //Save six point
    float position[3];
    THEROBOT->get_axis_position(position);
    actual_probe_points[5] = std::make_tuple(position[0], position[1], position[2]);
    stream->printf("Probe point 6 at: x%1.3f y%1.3f z%1.3f\n", position[0], position[1], position[2]);

    Gcode offset("G0 C60 F1200", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &offset);

    //Move to the seven point
    float x, y, z;
    std::tie(x, y, z) = probe_points[6];
    zprobe->coordinated_move(x, y, z, zprobe->getFastFeedrate());
    stream->printf("Moving to probe point 7: x%1.3f y%1.3f z%1.3f\n", x, y, z);
}

void FiveAxisStrategy::cAxisBeatingCorrection(StreamOutput *stream)
{
    //Save seven point
    float position[3];
    THEROBOT->get_axis_position(position);
    actual_probe_points[6] = std::make_tuple(position[0], position[1], position[2]);
    stream->printf("Probe point 7 at: x%1.3f y%1.3f z%1.3f\n", position[0], position[1], position[2]);

    float x6, y6, z6;
    float x7, y7, z7;
    std::tie(x6, y6, z6) = actual_probe_points[5];
    std::tie(x7, y7, z7) = actual_probe_points[6];

    calibration[XOC] = (abs(x7 + x6) / 2) - x7;
    calibration[YOC] = (abs(y7 + y6) / 2) - y7;
    calibration[ZOC] = (abs(z7 + z6) / 2) - z7;

    setSecondAdjustFunction(true);

    //Move to the eight point
    float x, y, z;
    std::tie(x, y, z) = probe_points[7];
    zprobe->coordinated_move(x, y, z, zprobe->getFastFeedrate());
    stream->printf("Moving to probe point 8: x%1.3f y%1.3f z%1.3f\n", x, y, z);

    Gcode offset("G0 C90 F1200", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &offset);
}

void FiveAxisStrategy::preAAxisBeatingCorrection(StreamOutput *stream)
{
    //Save eight point
    float position[3];
    THEROBOT->get_axis_position(position);
    actual_probe_points[7] = std::make_tuple(position[0], position[1], position[2]);
    stream->printf("Probe point 8 at: x%1.3f y%1.3f z%1.3f\n", position[0], position[1], position[2]);

    zprobe->coordinated_move(position[0], position[1], position[2] + 40, zprobe->getFastFeedrate());

    Gcode offset("G0 A90 F1200", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &offset);

    //Move to the nine point
    float x, y, z;
    std::tie(x, y, z) = probe_points[8];
    zprobe->coordinated_move(x, y, z, zprobe->getFastFeedrate());
    stream->printf("Moving to probe point 9: x%1.3f y%1.3f z%1.3f\n", x, y, z);
}

void FiveAxisStrategy::aAxisBeatingCorrection(StreamOutput *stream)
{
    //Save nine point
    float position[3];
    THEROBOT->get_axis_position(position);
    actual_probe_points[8] = std::make_tuple(position[0], position[1], position[2]);
    stream->printf("Probe point 9 at: x%1.3f y%1.3f z%1.3f\n", position[0], position[1], position[2]);

    float x8, y8, z8;
    float x9, y9, z9;
    std::tie(x8, y8, z8) = actual_probe_points[7];
    std::tie(x9, y9, z9) = actual_probe_points[8];
    calibration[EXA] = (x9 - x8) / 90;
    calibration[EYA] = (y9 - y8) / 90;
    calibration[EZA] = (z9 - z8) / 90;

    setFinalAdjustFunction(true);

    print_calibr(stream);

    Gcode afterHome("G0 Z100", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &afterHome);
}

float FiveAxisStrategy::matrixDeterminant(float a, float b, float c, float d, float e, float f, float g, float h, float i)
{
    float det = a * e * i + b * f * g + d * h * c - c * e * g - b * d * i - a * h * f;
    //THEKERNEL->streams("Det : %1.3f\n", det);
    return det;
}

void FiveAxisStrategy::makeHome()
{
    Gcode gc("G28", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
}

void FiveAxisStrategy::setFirstAdjustFunction(bool on)
{
    if (on)
    {
        // set compensation function
        using std::placeholders::_1;
        using std::placeholders::_2;
        THEROBOT->compensationTransform = std::bind(&FiveAxisStrategy::firstCompensationFunction, this, _1, _2);
    }
    else
    {
        THEROBOT->compensationTransform = nullptr;
    }
}

void FiveAxisStrategy::setSecondAdjustFunction(bool on)
{
    if (on)
    {
        // set compensation function
        using std::placeholders::_1;
        using std::placeholders::_2;
        THEROBOT->compensationTransform = std::bind(&FiveAxisStrategy::secondCompensationFunction, this, _1, _2);
    }
    else
    {
        THEROBOT->compensationTransform = nullptr;
    }
}

void FiveAxisStrategy::setFinalAdjustFunction(bool on)
{
    if (on)
    {
        // set compensation function
        using std::placeholders::_1;
        using std::placeholders::_2;
        THEROBOT->compensationTransform = std::bind(&FiveAxisStrategy::finalCompensationFunction, this, _1, _2);
    }
    else
    {
        THEROBOT->compensationTransform = nullptr;
    }
}

void FiveAxisStrategy::firstCompensationFunction(float *target, bool inverse)
{
    float sin_a = sinf((target[A_AXIS] * 1.5F) / 57.2958);
    float cos_a = cosf((target[A_AXIS] * 1.5F) / 57.2958);
    float sin_b = sinf(calibration[B]);
    float cos_b = 1 - cosf(calibration[B]);
    float cos_b2 = cosf(calibration[B] / 2);
    float sin_b2 = sinf(calibration[B] / 2);
    float x = target[X_AXIS] - calibration[X0];
    float y = target[Y_AXIS] - calibration[Y0];
    float z = target[Z_AXIS] - calibration[Z0];
    float r = hypotf(hypotf(x, y), z);
    if (inverse)
    {
        target[X_AXIS] -= r * sin_b * sin_a + calibration[B] / abs(calibration[B]) * 2 * r * r * cos_b * cos_b2 * cos_a;
        target[Y_AXIS] -= r * cos_b * sin_a;
        target[Z_AXIS] += 2 * r * r * cos_b * abs(sin_b2) * cos_a;
    }
    else
    {
        target[X_AXIS] += r * sin_b * sin_a + calibration[B] / abs(calibration[B]) * 2 * r * r * cos_b * cos_b2 * cos_a;
        target[Y_AXIS] += r * cos_b * sin_a;
        target[Z_AXIS] -= 2 * r * r * cos_b * abs(sin_b2) * cos_a;
    }
}

void FiveAxisStrategy::secondCompensationFunction(float *target, bool inverse)
{
    firstCompensationFunction(target, inverse);
    if (inverse)
    {
        //THEKERNEL->streams("SecondFunction input: x%1.3f, y%1.3f, z%1.3f\n", target[X_AXIS], target[Y_AXIS], target[Z_AXIS]);
        target[X_AXIS] -= calibration[XOC];
        target[Y_AXIS] -= calibration[YOC];
        target[Z_AXIS] -= calibration[ZOC];
        //THEKERNEL->streams("SecondFunction output: x%1.3f, y%1.3f, z%1.3f\n", target[X_AXIS], target[Y_AXIS], target[Z_AXIS]);
    }
    else
    {
        //THEKERNEL->streams("SecondFunction input: x%1.3f, y%1.3f, z%1.3f\n", target[X_AXIS], target[Y_AXIS], target[Z_AXIS]);
        target[X_AXIS] += calibration[XOC];
        target[Y_AXIS] += calibration[YOC];
        target[Z_AXIS] += calibration[ZOC];
        //THEKERNEL->streams("SecondFunction output: x%1.3f, y%1.3f, z%1.3f\n", target[X_AXIS], target[Y_AXIS], target[Z_AXIS]);
    }
}

void FiveAxisStrategy::finalCompensationFunction(float *target, bool inverse)
{
    secondCompensationFunction(target, inverse);
    if (inverse)
    {
        //THEKERNEL->streams("FinalFunction input: x%1.3f, y%1.3f, z%1.3f\n", target[X_AXIS], target[Y_AXIS], target[Z_AXIS]);
        target[X_AXIS] -= calibration[EXA] * target[A_AXIS];
        target[Y_AXIS] -= calibration[EYA] * target[A_AXIS];
        target[Z_AXIS] -= calibration[EZA] * target[A_AXIS];
        //THEKERNEL->streams("FinalFunction output: x%1.3f, y%1.3f, z%1.3f\n", target[X_AXIS], target[Y_AXIS], target[Z_AXIS]);
    }
    else
    {
        //THEKERNEL->streams("FinalFunction input: x%1.3f, y%1.3f, z%1.3f\n", target[X_AXIS], target[Y_AXIS], target[Z_AXIS]);
        target[X_AXIS] += calibration[EXA] * target[A_AXIS];
        target[Y_AXIS] += calibration[EYA] * target[A_AXIS];
        target[Z_AXIS] += calibration[EZA] * target[A_AXIS];
        //THEKERNEL->streams("FinalFunction output: x%1.3f, y%1.3f, z%1.3f\n", target[X_AXIS], target[Y_AXIS], target[Z_AXIS]);
    }
}

std::tuple<float, float, float>
FiveAxisStrategy::parseXYZ(const char *str)
{
    float x = 0, y = 0, z = 0;
    char *p;
    x = strtof(str, &p);
    if (p + 1 < str + strlen(str))
    {
        y = strtof(p + 1, &p);
        if (p + 1 < str + strlen(str))
        {
            z = strtof(p + 1, nullptr);
        }
    }
    return std::make_tuple(x, y, z);
}
