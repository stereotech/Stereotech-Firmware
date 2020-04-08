#ifndef _5AXISSTRATEGY
#define _5AXISSTRATEGY

#include "LevelingStrategy.h"

#include <tuple>

#define five_axis_strategy_checksum CHECKSUM("five-axis")

class StreamOutput;
class Gcode;

class FiveAxisStrategy : public LevelingStrategy
{
private:
  std::tuple<float, float, float> parseXYZ(const char *str);
  void makeHome();
  void gotoStep(uint8_t step, StreamOutput *stream);
  void setAAxisZero(StreamOutput *stream);
  void preLinearCorrection(StreamOutput *stream);
  void setBAxisCorrection(StreamOutput *stream);
  void linearCorrection(StreamOutput *stream);
  void preCAxisBeatingCorrection(StreamOutput *stream);
  void cAxisBeatingCorrection(StreamOutput *stream);
  void preAAxisBeatingCorrection(StreamOutput *stream);
  void aAxisBeatingCorrection(StreamOutput *stream);
  //void preZAxisCorrectionResetting(StreamOutput *stream);
  //void zAxisCorrectionResetting(StreamOutput *stream);

  void save_calibr(StreamOutput *stream);
  bool load_calibr(StreamOutput *stream);
  void reset_calibr();
  void print_calibr(StreamOutput *stream);

  float helperL1(float x, float z);
  float helperP(float x, float z);
  float helperQ(float x, float z);
  float helperR(float x, float z);
  float helperXi(float x, float z);
  float helperDzeta(float x, float z);

  float matrixDeterminant(float a, float b, float c, float d, float e, float f, float g, float h, float i);

  std::tuple<float, float, float> probe_points[8];
  std::tuple<float, float, float> actual_probe_points[8];
  float big_part_length;
  float small_part_length;
  bool home;
  float calibration[10];
  StreamOutput *stream;

public:
  FiveAxisStrategy(ZProbe *zprobe);
  ~FiveAxisStrategy();
  bool handleGcode(Gcode *gcode);
  bool handleConfig();
  void doCompensation(float *target, bool inverse);
  void setFirstAdjustFunction(bool);
  void firstCompensationFunction(float *target, bool inverse);
  void setSecondAdjustFunction(bool);
  void secondCompensationFunction(float *target, bool inverse);
  //void setThirdAdjustFunction(bool);
  //void thirdCompensationFunction(float *target, bool inverse);
  void setFinalAdjustFunction(bool);
  void finalCompensationFunction(float *target, bool inverse);
};

#endif
