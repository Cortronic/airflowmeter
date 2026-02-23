
typedef enum {
  MT_SELECT = 0,
  MT_SELECT_HOOD = 1,
  MT_MEASURE = 2,
  MT_CALIBRATE_ZERO_COMPENSATION = 3,
  MT_CALIBRATE_FLOW = 4,
  MT_PID_TUNE_P = 5,
  MT_PID_TUNE_I = 6,
  MT_PID_TUNE_D = 7,
} ModeType;
extern ModeType modeType;

typedef enum {
  HOOD_A_RETURN_VALVE = 0,
  HOOD_A_SUPPLY_VALVE = 1,
  HOOD_B_RETURN_VALVE = 2,
  HOOD_B_SUPPLY_VALVE = 3,
} HoodValveType;
extern HoodValveType hoodValveType;+