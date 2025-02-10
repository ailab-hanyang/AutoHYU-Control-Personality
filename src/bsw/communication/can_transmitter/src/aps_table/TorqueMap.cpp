/*
 * TorqueMap.cpp
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

#include "aps_table/TorqueMap.h"
#include "aps_table/TorqueMap_private.h"
#include "aps_table/rtwtypes.h"

uint32_T plook_u32d_evenckan(real_T u, real_T bp0, real_T bpSpace, uint32_T
  maxIndex)
{
  uint32_T bpIndex;

  /* Prelookup - Index only
     Index Search method: 'even'
     Interpolation method: 'Use nearest'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp0) {
    bpIndex = 0U;
  } else {
    real_T fbpIndex;
    fbpIndex = (u - bp0) * (1.0 / bpSpace);
    if (fbpIndex < maxIndex) {
      bpIndex = static_cast<uint32_T>(fbpIndex);
      if ((static_cast<real_T>(static_cast<uint32_T>(fbpIndex) + 1U) * bpSpace +
           bp0) - u <= u - (static_cast<real_T>(static_cast<uint32_T>(fbpIndex))
                            * bpSpace + bp0)) {
        bpIndex = static_cast<uint32_T>(fbpIndex) + 1U;
      }
    } else {
      bpIndex = maxIndex;
    }
  }

  return bpIndex;
}

/* Model step function */
void TorqueMap::step()
{
  /* Outport: '<Root>/Out1' incorporates:
   *  Inport: '<Root>/In1'
   *  Inport: '<Root>/In2'
   *  Lookup_n-D: '<Root>/2-D Lookup Table'
   */
  TorqueMap_Y.Out1 =
    TorqueMap_ConstP.uDLookupTable_tableData[plook_u32d_evenckan(TorqueMap_U.In2,
    10.0, 1.0, 190U) * 851U + plook_u32d_evenckan(TorqueMap_U.In1, -2000.0, 10.0,
    850U)];
}

/* Model initialize function */
void TorqueMap::initialize()
{
  /* (no initialization code required) */
}

/* Model terminate function */
void TorqueMap::terminate()
{
  /* (no terminate code required) */
}

/* Constructor */
TorqueMap::TorqueMap() :
  TorqueMap_U(),
  TorqueMap_Y(),
  TorqueMap_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
TorqueMap::~TorqueMap()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_TorqueMap_T * TorqueMap::getRTM()
{
  return (&TorqueMap_M);
}
