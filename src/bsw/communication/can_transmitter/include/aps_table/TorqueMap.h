/*
 * TorqueMap.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "TorqueMap".
 *
 * Model version              : 1.6
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Thu Oct  5 15:40:01 2023
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_TorqueMap_h_
#define RTW_HEADER_TorqueMap_h_
#include "aps_table/rtwtypes.h"
#include "aps_table/rtw_continuous.h"
#include "aps_table/rtw_solver.h"
#include "aps_table/TorqueMap_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Constant parameters (default storage) */
struct ConstP_TorqueMap_T {
  /* Expression: gas_grid
   * Referenced by: '<Root>/2-D Lookup Table'
   */
  real_T uDLookupTable_tableData[162541];
};

/* External inputs (root inport signals with default storage) */
struct ExtU_TorqueMap_T {
  real_T In1;                          /* '<Root>/In1' */
  real_T In2;                          /* '<Root>/In2' */
};

/* External outputs (root outports fed by signals with default storage) */
struct ExtY_TorqueMap_T {
  real_T Out1;                         /* '<Root>/Out1' */
};

/* Real-time Model Data Structure */
struct tag_RTM_TorqueMap_T {
  const char_T *errorStatus;
};

/* Constant parameters (default storage) */
extern const ConstP_TorqueMap_T TorqueMap_ConstP;

/* Class declaration for model TorqueMap */
class TorqueMap final
{
  /* public data and function members */
 public:
  /* Copy Constructor */
  TorqueMap(TorqueMap const&) = delete;

  /* Assignment Operator */
  TorqueMap& operator= (TorqueMap const&) & = delete;

  /* Move Constructor */
  TorqueMap(TorqueMap &&) = delete;

  /* Move Assignment Operator */
  TorqueMap& operator= (TorqueMap &&) = delete;

  /* Real-Time Model get method */
  RT_MODEL_TorqueMap_T * getRTM();

  /* External inputs */
  ExtU_TorqueMap_T TorqueMap_U;

  /* External outputs */
  ExtY_TorqueMap_T TorqueMap_Y;

  /* Initial conditions function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  static void terminate();

  /* Constructor */
  TorqueMap();

  /* Destructor */
  ~TorqueMap();

  /* private data and function members */
 private:
  /* Real-Time Model */
  RT_MODEL_TorqueMap_T TorqueMap_M;
};

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Scope' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'TorqueMap'
 */
#endif                                 /* RTW_HEADER_TorqueMap_h_ */
