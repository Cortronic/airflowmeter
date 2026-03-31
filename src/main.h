
typedef enum {
  MT_SELECT = 0,
  MT_SELECT_VALVE = 1,
  MT_MEASURE = 2,
  MT_TUNE_ZERO_COMPENSATION = 3,
  MT_CALIBRATE_FLOW = 4,
  MT_TUNE_PID = 5,
  MT_ADJUST_OFFSET = 6,
} ModeType;
extern ModeType modeType;

typedef enum {
  VT_EXTRACT_AXIAL = 0,
  VT_EXTRACT_RADIAL = 1,
  VT_SUPPLY_AXIAL = 2,
  VT_SUPPLY_RADIAL = 3,
} ValveType;
extern ValveType valveType;

typedef enum {
  PID_TUNE_NONE = 0,
  PID_TUNE_P = 1,
  PID_TUNE_I = 2,
  PID_TUNE_D = 3,
} PidTuneType;
extern PidTuneType pidTuneType;

#define ZERO_COMP_MIN -0.5
#define ZERO_COMP_MAX 0.5
#define ZERO_COMP_STEP 0.001

// preferences keys
#define OFFSET_ZERO_PRESSURE "offsetZero"
#define OFFSET_FLOW_PRESSURE "offsetFlow"
#define COEF_EXTRACT_AXIAL "coefExtractAx"
#define COEF_EXTRACT_RADIAL "coefExtractRd"
#define COEF_SUPPLY_AXIAL "coefSupplyAx"
#define COEF_SUPPLY_RADIAL "coefSupplyRd"
#define ZERO_COMPENSATION_FACTOR_EA "compFactEA"
#define ZERO_COMPENSATION_FACTOR_ER "compFactER"
#define ZERO_COMPENSATION_FACTOR_SA "compFactSA"
#define ZERO_COMPENSATION_FACTOR_SR "compFactSR"
#define Kp_KEY "Kp"
#define Ki_KEY "Ki"
#define Kd_KEY "Kd"