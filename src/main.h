
typedef struct {
  float inletDiameter; // Diameter of the inlet of the venturi in meters
  float throatDiameter; // Diameter of the throat in meters
  float areaInlet; // Cross-sectional area of the inlet in square meters
  float areaThroat; // Cross-sectional area of the throat in square meters
  float betaRatio; // Ratio of throat diameter to inlet diameter (dimensionless)
  float betaCoefficient; // Coefficient for calculating beta ratio effects (1 - pow(betaRatio, 4))
  float dischargeCoefficient; // Discharge coefficient of the venturi (dimensionless)
} VenturiConstants;
extern VenturiConstants venturi;

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
#define ZERO_COMP_DEFAULT 0.0

#define FLOW_COEF_MIN 0.7
#define FLOW_COEF_MAX 1.0
#define FLOW_COEF_STEP 0.001
#define FLOW_COEF_DEFAULT 0.965


// preferences keys
#define KEY_VENTURI_INLET_DIAMETER "inletDia"
#define KEY_VENTURI_THROAT_DIAMETER "throadDia"
#define KEY_VENTURI_CD "Cd"
#define OFFSET_ZERO_PRESSURE "offsetZero"
#define OFFSET_FLOW_PRESSURE "offsetFlow"
#define FLOW_COEF_EXTRACT_AXIAL "coefExtractAx"
#define FLOW_COEF_EXTRACT_RADIAL "coefExtractRd"
#define FLOW_COEF_SUPPLY_AXIAL "coefSupplyAx"
#define FLOW_COEF_SUPPLY_RADIAL "coefSupplyRd"
#define ZERO_COMPENSATION_FACTOR_EA "compFactEA"
#define ZERO_COMPENSATION_FACTOR_ER "compFactER"
#define ZERO_COMPENSATION_FACTOR_SA "compFactSA"
#define ZERO_COMPENSATION_FACTOR_SR "compFactSR"
#define Kp_KEY "Kp"
#define Ki_KEY "Ki"
#define Kd_KEY "Kd"