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
#define probe_point_10_checksum CHECKSUM("point10")
#define probe_device_big_part_length CHECKSUM("big_part_length")
#define probe_device_small_part_length CHECKSUM("small_part_length")
#define home_checksum CHECKSUM("home_first")

#define CALIBRFILE "/sd/fiveaxis.calibr"

#define B 0
#define X0 1
#define Z0 2
#define T 3
#define EXX 4
#define EXY 5
#define EXZ 6
#define EYX 7
#define EYY 8
#define EYZ 9
#define EZX 10
#define EZY 11
#define EZZ 12
#define EXA 13
#define EYA 14
#define EZA 15
#define XOC 16
#define YOC 17
#define ZOC 18

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
    std::string p1 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_1_checksum)->by_default("")->as_string();
    std::string p2 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_2_checksum)->by_default("")->as_string();
    std::string p3 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_3_checksum)->by_default("")->as_string();
    std::string p4 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_4_checksum)->by_default("")->as_string();
    std::string p5 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_5_checksum)->by_default("")->as_string();
    std::string p6 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_6_checksum)->by_default("")->as_string();
    std::string p7 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_7_checksum)->by_default("")->as_string();
    std::string p8 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_8_checksum)->by_default("")->as_string();
    std::string p9 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_9_checksum)->by_default("")->as_string();
    std::string p10 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_10_checksum)->by_default("")->as_string();

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
    if (!p9.empty())
        probe_points[8] = parseXYZ(p9.c_str());
    if (!p10.empty())
        probe_points[9] = parseXYZ(p10.c_str());

    this->big_part_length = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_device_big_part_length)->by_default(10.0F)->as_number();
    this->small_part_length = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_device_small_part_length)->by_default(10.0F)->as_number();
    this->home = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, home_checksum)->by_default(true)->as_bool();

    for (int i = 0; i < 10; i++)
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
            if (idx >= 0 && idx <= 9)
            {
                probe_points[idx] = std::make_tuple(x, y, z);
            }
            else
            {
                gcode->stream->printf("only 9 probe points allowed P0-P9\n");
            }
            return true;
        }
        if (gcode->m == 370 || gcode->m == 561)
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
    for (size_t i = 0; i < 19; i++)
    {
        if (fwrite(&calibration[i], sizeof(float), 1, fp) != 1)
        {
            stream->printf("error:Failed to write calibration\n");
            fclose(fp);
            return;
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
    for (size_t i = 0; i < 19; i++)
    {
        if (fread(&calibration[i], sizeof(float), 1, fp) != 1)
        {
            stream->printf("error:Failed to read calibration\n");
            fclose(fp);
            return false;
        }
    }
    stream->printf("calibration loaded");
    fclose(fp);
    return true;
}

void FiveAxisStrategy::reset_calibr()
{
    for (size_t i = 0; i < 19; i++)
    {
        calibration[i] = 0;
    }
}

void FiveAxisStrategy::print_calibr(StreamOutput *stream)
{
    stream->printf("Betta correction: %1.4f\n", calibration[B]);
    stream->printf("Real A axis rotation point: X - %1.4f, Z - %1.4f\n", calibration[X0], calibration[Z0]);
    stream->printf("T correction: %1.4f\n", calibration[T]);
    stream->printf("X axis correction: %1.4f %1.4f %1.4f\n", calibration[EXX], calibration[EXY], calibration[EXZ]);
    stream->printf("Y axis correction: %1.4f %1.4f %1.4f\n", calibration[EYX], calibration[EYY], calibration[EYZ]);
    stream->printf("Z axis correction: %1.4f %1.4f %1.4f\n", calibration[EZX], calibration[EZY], calibration[EZZ]);
    stream->printf("A axis correction: %1.4f %1.4f %1.4f\n", calibration[EXA], calibration[EYA], calibration[EZA]);
    stream->printf("B axis correction: %1.4f %1.4f %1.4f\n", calibration[XOC], calibration[YOC], calibration[ZOC]);
}

void FiveAxisStrategy::gotoStep(uint8_t step, StreamOutput *stream)
{
    if (step == 0)
    {
        THEKERNEL->conveyor->wait_for_idle();
        setFirstAdjustFunction(false);

        if (this->home)
        {
            zprobe->home();
        }

        Gcode rotateA("G0 A0", &(StreamOutput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &rotateA);

        //Move to the first point
        float x, y, z;
        std::tie(x, y, z) = probe_points[0];
        zprobe->coordinated_move(x, y, z, zprobe->getFastFeedrate());
        stream->printf("Moving to probe point 1: x%1.3f y%1.3f z%1.3f\n", x, y, z);
    }
    else if (step == 1)
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
    else if (step == 2)
    {
        setAAxisZero(stream);
    }
    else if (step == 3)
    {
        setCAxisRegardingXY(stream);
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
    else if (step == 11)
    {
        preZAxisCorrectionResetting(stream);
    }
    else if (step == 12)
    {
        zAxisCorrectionResetting(stream);
    }
}

void FiveAxisStrategy::setAAxisZero(StreamOutput *stream)
{
    float a_offset = 0;
    //Save second point
    float position[3];
    THEROBOT->get_axis_position(position);
    actual_probe_points[1] = std::make_tuple(position[0], position[1], position[2]);
    stream->printf("Probe point 2 at: x%1.3f y%1.3f z%1.3f\n", position[0], position[1], position[2]);

    float z1, z2;
    z1 = std::get<2>(actual_probe_points[0]);
    z2 = std::get<2>(actual_probe_points[1]);
    a_offset = -57.2958 * asinf((z2 - z1) / (2 * (this->big_part_length + this->small_part_length)));

    char *cmd = new char[128];
    if (!isnan(a_offset))
    {
        size_t n = strlen(cmd);
        snprintf(&cmd[n], 128 - n, "M206 A%1.3f", a_offset);
        stream->printf("A axis offset is:%1.3f", a_offset);
        Gcode offset(cmd, &(StreamOutput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &offset);
    }
    delete[] cmd;

    zprobe->coordinated_move(NAN, NAN, position[2] + 20, zprobe->getFastFeedrate(), true);
    Gcode homeA("G28 A0", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &homeA);
    Gcode rotateA("G0 A0", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &rotateA);
}

void FiveAxisStrategy::setCAxisRegardingXY(StreamOutput *stream)
{
    float c_offset = 0;
    float y1, y2, x1, x2, z1, z2;
    std::tie(x1, y1, z1) = actual_probe_points[0];
    std::tie(x2, y2, z2) = actual_probe_points[1];
    c_offset = atan2f((y2 - y1), (x2 - x1));

    char *cmd = new char[128];
    if (!isnan(c_offset))
    {
        size_t n = strlen(cmd);
        snprintf(&cmd[n], 128 - n, "G0 C%1.3f", c_offset);
        stream->printf("C axis offset is:%1.3f", c_offset);
        Gcode offset(cmd, &(StreamOutput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &offset);
        Gcode homeB("G28 C0", &(StreamOutput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &homeB);
    }
    delete[] cmd;

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
    calibration[T] = (sinf(calibration[B]) * sinf(calibration[B])) / (cosf(calibration[B] / 2) * cosf(calibration[B] / 2));
    float x3;
    x3 = std::get<0>(actual_probe_points[2]);
    calibration[X0] = x3 + (this->big_part_length + this->small_part_length) * sinf(calibration[B]);
    calibration[Z0] = z3 - (this->big_part_length + this->small_part_length) * sinf(calibration[B]);
    stream->printf("Real A axis rotation point: x%1.3f y%1.3f z%1.3f\n", calibration[X0], position[1], calibration[Z0]);
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

    float x2, y2, z2;
    float x3, y3, z3;
    float x4, y4, z4;
    float x5, y5, z5;
    std::tie(x2, y2, z2) = actual_probe_points[1];
    std::tie(x3, y3, z3) = actual_probe_points[2];
    std::tie(x4, y4, z4) = actual_probe_points[3];
    std::tie(x5, y5, z5) = actual_probe_points[4];

    float matrix[3][3] = {
        {x3 - x2, y3 - y2, z3 - z2},
        {x4 - x3, y4 - y3, z4 - z3},
        {x5 - x4, y5 - y4, z5 - z4}};

    float commonDeterminant = matrixDeterminant(matrix[0][0], matrix[0][1], matrix[0][2],
                                                matrix[1][0], matrix[1][1], matrix[1][2],
                                                matrix[2][0], matrix[2][1], matrix[2][2]);
    if (commonDeterminant == 0)
    {
        stream->printf("Error getting corrections, common determinant equals zero");
        return;
    }

    float xxmatrix[3] = {x3 - x2, x4 - x3 - (probe_device_small_part_length + probe_device_big_part_length) * cosf(calibration[B]), x5 - x4 + 2 * (probe_device_small_part_length + probe_device_big_part_length)};
    float detXX = matrixDeterminant(xxmatrix[0], matrix[0][1], matrix[0][2],
                                    xxmatrix[1], matrix[1][1], matrix[1][2],
                                    xxmatrix[2], matrix[2][1], matrix[2][2]);
    float detXY = matrixDeterminant(matrix[0][0], xxmatrix[0], matrix[0][2],
                                    matrix[1][0], xxmatrix[1], matrix[1][2],
                                    matrix[2][0], xxmatrix[2], matrix[2][2]);
    float detXZ = matrixDeterminant(matrix[0][0], matrix[0][1], xxmatrix[0],
                                    matrix[1][0], matrix[1][1], xxmatrix[1],
                                    matrix[2][0], matrix[2][1], xxmatrix[2]);
    calibration[EXX] = detXX / commonDeterminant;
    calibration[EXY] = detXY / commonDeterminant;
    calibration[EXZ] = detXZ / commonDeterminant;

    float yymatrix[3] = {y3 - y2 + (probe_device_small_part_length + probe_device_big_part_length), y4 - y3, y5 - y4};
    float detYX = matrixDeterminant(yymatrix[0], matrix[0][1], matrix[0][2],
                                    yymatrix[1], matrix[1][1], matrix[1][2],
                                    yymatrix[2], matrix[2][1], matrix[2][2]);
    float detYY = matrixDeterminant(matrix[0][0], yymatrix[0], matrix[0][2],
                                    matrix[1][0], yymatrix[1], matrix[1][2],
                                    matrix[2][0], yymatrix[2], matrix[2][2]);
    float detYZ = matrixDeterminant(matrix[0][0], matrix[0][1], yymatrix[0],
                                    matrix[1][0], matrix[1][1], yymatrix[1],
                                    matrix[2][0], matrix[2][1], yymatrix[2]);
    calibration[EYX] = detYX / commonDeterminant;
    calibration[EYY] = detYY / commonDeterminant;
    calibration[EYZ] = detYZ / commonDeterminant;

    float zzmatrix[3] = {z3 - z2, z4 - z3 - (probe_device_big_part_length + probe_device_small_part_length) * sinf(calibration[B]), z5 - z4};
    float detZX = matrixDeterminant(zzmatrix[0], matrix[0][1], matrix[0][2],
                                    zzmatrix[1], matrix[1][1], matrix[1][2],
                                    zzmatrix[2], matrix[2][1], matrix[2][2]);
    float detZY = matrixDeterminant(matrix[0][0], zzmatrix[0], matrix[0][2],
                                    matrix[1][0], zzmatrix[1], matrix[1][2],
                                    matrix[2][0], zzmatrix[2], matrix[2][2]);
    float detZZ = matrixDeterminant(matrix[0][0], matrix[0][1], zzmatrix[0],
                                    matrix[1][0], matrix[1][1], zzmatrix[1],
                                    matrix[2][0], matrix[2][1], zzmatrix[2]);
    calibration[EZX] = detZX / commonDeterminant;
    calibration[EZY] = detZY / commonDeterminant;
    calibration[EZZ] = detZZ / commonDeterminant;

    setSecondAdjustFunction(true);

    //Move to the sixth point
    float x, y, z;
    std::tie(x, y, z) = probe_points[5];
    zprobe->coordinated_move(x, y, z, zprobe->getFastFeedrate());
    stream->printf("Moving to probe point 6: x%1.3f y%1.3f z%1.3f\n", x, y, z);
}

void FiveAxisStrategy::preCAxisBeatingCorrection(StreamOutput *stream)
{
    //Save sixth point
    float position[3];
    THEROBOT->get_axis_position(position);
    actual_probe_points[5] = std::make_tuple(position[0], position[1], position[2]);
    stream->printf("Probe point 6 at: x%1.3f y%1.3f z%1.3f\n", position[0], position[1], position[2]);

    //Move to the seventh point
    float x, y, z;
    std::tie(x, y, z) = probe_points[6];
    zprobe->coordinated_move(x, y, z, zprobe->getFastFeedrate());
    stream->printf("Moving to probe point 7: x%1.3f y%1.3f z%1.3f\n", x, y, z);

    Gcode offset("G0 C180", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &offset);
}

void FiveAxisStrategy::cAxisBeatingCorrection(StreamOutput *stream)
{
    //Save seventh point
    float position[3];
    THEROBOT->get_axis_position(position);
    actual_probe_points[6] = std::make_tuple(position[0], position[1], position[2]);
    stream->printf("Probe point 7 at: x%1.3f y%1.3f z%1.3f\n", position[0], position[1], position[2]);

    float x6, y6, z6;
    float x7, y7, z7;
    std::tie(x6, y6, z6) = actual_probe_points[5];
    std::tie(x7, y7, z7) = actual_probe_points[6];

    calibration[XOC] = (abs(x7 - x6) / 2) - x7;
    calibration[YOC] = (abs(y7 - y6) / 2) - y7;
    calibration[ZOC] = (abs(z7 - z6) / 2) - z7;

    setThirdAdjustFunction(true);

    Gcode offset("G0 C270", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &offset);

    //Move to the eight point
    float x, y, z;
    std::tie(x, y, z) = probe_points[7];
    zprobe->coordinated_move(x, y, z, zprobe->getFastFeedrate());
    stream->printf("Moving to probe point 8: x%1.3f y%1.3f z%1.3f\n", x, y, z);
}

void FiveAxisStrategy::preAAxisBeatingCorrection(StreamOutput *stream)
{
    //Save eight point
    float position[3];
    THEROBOT->get_axis_position(position);
    actual_probe_points[7] = std::make_tuple(position[0], position[1], position[2]);
    stream->printf("Probe point 8 at: x%1.3f y%1.3f z%1.3f\n", position[0], position[1], position[2]);

    zprobe->coordinated_move(position[0], position[1], position[2] + 40, zprobe->getFastFeedrate());

    Gcode offset("G0 A90", &(StreamOutput::NullStream));
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
}

void FiveAxisStrategy::preZAxisCorrectionResetting(StreamOutput *stream)
{
    zprobe->coordinated_move(NAN, NAN, probe_device_small_part_length + probe_device_big_part_length, zprobe->getFastFeedrate());

    Gcode offset("G0 C360", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &offset);

    //Move to the ten point
    float x, y, z;
    std::tie(x, y, z) = probe_points[9];
    zprobe->coordinated_move(x, y, z, zprobe->getFastFeedrate());
    stream->printf("Moving to probe point 10: x%1.3f y%1.3f z%1.3f\n", x, y, z);
}

void FiveAxisStrategy::zAxisCorrectionResetting(StreamOutput *stream)
{
    //Save ten point
    float position[3];
    THEROBOT->get_axis_position(position);
    actual_probe_points[9] = std::make_tuple(position[0], position[1], position[2]);
    stream->printf("Probe point 10 at: x%1.3f y%1.3f z%1.3f\n", position[0], position[1], position[2]);

    float x9, y9, z9;
    float x10, y10, z10;
    std::tie(x9, y9, z9) = actual_probe_points[8];
    std::tie(x10, y10, z10) = actual_probe_points[9];
    calibration[EXZ] = (x10 - x9) / x10;
    calibration[EYZ] = (y10 - y9) / y10;
    calibration[EZZ] = (z10 - z9) / z10;

    print_calibr(stream);
}

float FiveAxisStrategy::matrixDeterminant(float a, float b, float c, float d, float e, float f, float g, float h, float i, StreamOutput *stream)
{
    float det = a * e * i + b * f * g + d * h * c - c * e * g - b * d * i - a * h * f;
    stream->printf("Det : %1.3f\n", det);
    return det;
}

float FiveAxisStrategy::helperL1(float x, float z, StreamOutput *stream)
{
    float l1 = sqrtf((x - calibration[X0]) * (x - calibration[X0]) + (z - calibration[Z0]) * (z - calibration[Z0]));
    stream->printf("L1 : %1.3f\n", l1);
    return l1;
}

float FiveAxisStrategy::helperP(float x, float z, StreamOutput *stream)
{
    float p = 4 * ((x + calibration[X0]) * (x + calibration[X0]) + (z + calibration[Z0]) * (z + calibration[Z0]));
    stream->printf("P : %1.3f\n", p);
    return p
}

float FiveAxisStrategy::helperQ(float x, float z, StreamOutput *stream)
{
    float q = -4 * (x + calibration[X0]) * ((x + calibration[X0]) * (x + calibration[X0]) + (z + calibration[Z0]) * (z + calibration[Z0]) + helperL1(x, z) * helperL1(x, z) * (1 - calibration[T]));
    stream->printf("Q : %1.3f\n", q);
    return q;
}

float FiveAxisStrategy::helperR(float x, float z, StreamOutput *stream)
{
    float a = (x + calibration[X0]) * (x + calibration[X0]) + (z + calibration[Z0]) * (z + calibration[Z0]);
    float b = helperL1(x, z) * helperL1(x, z) * helperL1(x, z) * helperL1(x, z) * (1 - calibration[T]) * (1 - calibration[T]);
    float c = 2 * helperL1(x, z) * helperL1(x, z) * (1 - calibration[T]) * (a - 4 * helperL1(x, z) * helperL1(x, z) * (z + calibration[Z0]) * (z + calibration[Z0]));
    float r = a * a + b + c;
    stream->printf("R : %1.3f (a-%1.3f, b-%1.3f, c-%1.3f)\n", r, a, b, c);
    return r;
}

float FiveAxisStrategy::helperXi(float x, float z, StreamOutput *stream)
{
    float variantA = (-helperQ(x, z) + sqrtf(helperQ(x, z) * helperQ(x, z) - 4 * helperP(x, z) * helperR(x, z))) / (2 * helperP(x, z));
    float variantB = (-helperQ(x, z) - sqrtf(helperQ(x, z) * helperQ(x, z) - 4 * helperP(x, z) * helperR(x, z))) / (2 * helperP(x, z));
    stream->printf("Xi : varA-%1.3f, varb-%1.3f\n", variantA, variantB);
    if (variantA > helperL1(x, z))
    {
        return variantB;
    }
    else
    {
        return variantA;
    }
}

float FiveAxisStrategy::helperDzeta(float x, float z, StreamOutput *stream)
{
    float dzeta = sqrtf(helperL1(x, z) * helperL1(x, z) - helperXi(x, z) * helperXi(x, z));
    stream->printf("Dzeta : %1.3f\n", dzeta);
    return dzeta;
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

void FiveAxisStrategy::setThirdAdjustFunction(bool on)
{
    if (on)
    {
        // set compensation function
        using std::placeholders::_1;
        using std::placeholders::_2;
        THEROBOT->compensationTransform = std::bind(&FiveAxisStrategy::thirdCompensationFunction, this, _1, _2);
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
    if (inverse)
    {
        target[X_AXIS] = -helperXi(target[X_AXIS], target[Z_AXIS]) + calibration[X0];
        target[Z_AXIS] = -helperDzeta(target[X_AXIS], target[Z_AXIS]) + calibration[Z0];
    }
    else
    {
        target[X_AXIS] = helperXi(target[X_AXIS], target[Z_AXIS]) + calibration[X0];
        target[Z_AXIS] = helperDzeta(target[X_AXIS], target[Z_AXIS]) + calibration[Z0];
    }
}

void FiveAxisStrategy::secondCompensationFunction(float *target, bool inverse)
{
    firstCompensationFunction(target, inverse);
    if (inverse)
    {
        target[X_AXIS] += calibration[EXX] * target[X_AXIS] + calibration[EXY] * target[Y_AXIS] + calibration[EXZ] * target[Z_AXIS];
        target[Y_AXIS] += calibration[EYX] * target[X_AXIS] + calibration[EYY] * target[Y_AXIS] + calibration[EYZ] * target[Z_AXIS];
        target[Z_AXIS] += calibration[EZX] * target[X_AXIS] + calibration[EZY] * target[Y_AXIS] + calibration[EZZ] * target[Z_AXIS];
    }
    else
    {
        target[X_AXIS] -= calibration[EXX] * target[X_AXIS] + calibration[EXY] * target[Y_AXIS] + calibration[EXZ] * target[Z_AXIS];
        target[Y_AXIS] -= calibration[EYX] * target[X_AXIS] + calibration[EYY] * target[Y_AXIS] + calibration[EYZ] * target[Z_AXIS];
        target[Z_AXIS] -= calibration[EZX] * target[X_AXIS] + calibration[EZY] * target[Y_AXIS] + calibration[EZZ] * target[Z_AXIS];
    }
}

void FiveAxisStrategy::thirdCompensationFunction(float *target, bool inverse)
{
    secondCompensationFunction(target, inverse);
    if (inverse)
    {
        target[X_AXIS] -= calibration[XOC];
        target[Y_AXIS] -= calibration[YOC];
        target[Z_AXIS] -= calibration[ZOC];
    }
    else
    {
        target[X_AXIS] += calibration[XOC];
        target[Y_AXIS] += calibration[YOC];
        target[Z_AXIS] += calibration[ZOC];
    }
}

void FiveAxisStrategy::finalCompensationFunction(float *target, bool inverse)
{
    thirdCompensationFunction(target, inverse);
    if (inverse)
    {
        target[X_AXIS] += calibration[EXA] * target[A_AXIS];
        target[Y_AXIS] += calibration[EYA] * target[A_AXIS];
        target[Z_AXIS] += calibration[EZA] * target[A_AXIS];
    }
    else
    {
        target[X_AXIS] -= calibration[EXA] * target[A_AXIS];
        target[Y_AXIS] -= calibration[EYA] * target[A_AXIS];
        target[Z_AXIS] -= calibration[EZA] * target[A_AXIS];
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