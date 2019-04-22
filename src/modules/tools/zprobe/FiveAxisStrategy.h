#ifndef _5AXISSTRATEGY
#define _5AXISSTRATEGY

#include "LevelingStrategy.h"

#define five_axis_strategy_checksum CHECKSUM("five-axis")

class FiveAxisStrategy : public LevelingStrategy
{
private:
  std::tuple<float, float, float> parseXYZ(const char *str);
  void home();
  void gotoStep(uint8_t step, StreamOutput *stream);
  void setAAxisZero(StreamOutput *stream);
  void setCAxisRegardingXY(StreamOutput *stream);
  void preLinearCorrection(StreamOutput *stream);
  void setBAxisCorrection(StreamOutput *stream);
  void linearCorrection(StreamOutput *stream);
  void preCAxisBeatingCorrection(StreamOutput *stream);
  void cAxisBeatingCorrection(StreamOutput *stream);
  void preAAxisBeatingCorrection(StreamOutput *stream);
  void aAxisBeatingCorrection(StreamOutput *stream);
  void preZAxisCorrectionResetting(StreamOutput *stream);
  void zAxisCorrectionResetting(StreamOutput *stream);
  std::tuple<float, float, float> probe_points[10];
  std::tuple<float, float, float> actual_probe_points[10];
  float big_part_length;
  float small_part_length;
  bool home;

public:
  FiveAxisStrategy(ZProbe *zprobe);
  ~FiveAxisStrategy();
  bool handleGcode(Gcode *gcode);
  bool handleConfig();
  void setAdjustFunction(bool);
  void doCompensation(float *target, bool inverse);
};

#endif
