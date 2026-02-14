
typedef enum {
  SELECT_MODE = 0,
  SELECT_HOOD = 1,
  MEASURE_MODE = 2,
  CALIBRATE_ZERO_COMPENSATION_MODE = 3,
  CALIBRATE_FLOW_MODE = 4,
} ModeType;
extern ModeType modeType;

typedef enum {
  HOOD_A_RETURN_VALVE = 0,
  HOOD_A_SUPPLY_VALVE = 1,
  HOOD_B_RETURN_VALVE = 2,
  HOOD_B_SUPPLY_VALVE = 3,
} HoodValveType;
extern HoodValveType hoodValveType;