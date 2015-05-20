/* Include files */

#include <stddef.h>
#include "blas.h"
#include "wheel2_sfun.h"
#include "c2_wheel2.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "wheel2_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c2_debug_family_names[13] = { "n", "m", "a2", "ad", "ad2",
  "nargin", "nargout", "array", "times", "t", "td", "dtd", "d2td" };

/* Function Declarations */
static void initialize_c2_wheel2(SFc2_wheel2InstanceStruct *chartInstance);
static void initialize_params_c2_wheel2(SFc2_wheel2InstanceStruct *chartInstance);
static void enable_c2_wheel2(SFc2_wheel2InstanceStruct *chartInstance);
static void disable_c2_wheel2(SFc2_wheel2InstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_wheel2(SFc2_wheel2InstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c2_wheel2(SFc2_wheel2InstanceStruct
  *chartInstance);
static void set_sim_state_c2_wheel2(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_st);
static void finalize_c2_wheel2(SFc2_wheel2InstanceStruct *chartInstance);
static void sf_gateway_c2_wheel2(SFc2_wheel2InstanceStruct *chartInstance);
static void c2_chartstep_c2_wheel2(SFc2_wheel2InstanceStruct *chartInstance);
static void initSimStructsc2_wheel2(SFc2_wheel2InstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static real_T c2_emlrt_marshallIn(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_d2td, const char_T *c2_identifier);
static real_T c2_b_emlrt_marshallIn(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, real_T
  c2_inData_data[], int32_T c2_inData_sizes[2]);
static void c2_c_emlrt_marshallIn(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y_data[],
  int32_T c2_y_sizes[2]);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real_T c2_outData_data[], int32_T
  c2_outData_sizes[2]);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, real_T
  c2_inData_data[], int32_T c2_inData_sizes[2]);
static void c2_d_emlrt_marshallIn(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y_data[],
  int32_T c2_y_sizes[2]);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real_T c2_outData_data[], int32_T
  c2_outData_sizes[2]);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_e_emlrt_marshallIn(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[4]);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, real_T
  c2_inData_data[], int32_T *c2_inData_sizes);
static void c2_f_emlrt_marshallIn(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y_data[],
  int32_T *c2_y_sizes);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real_T c2_outData_data[], int32_T
  *c2_outData_sizes);
static void c2_info_helper(const mxArray **c2_info);
static const mxArray *c2_emlrt_marshallOut(const char * c2_u);
static const mxArray *c2_b_emlrt_marshallOut(const uint32_T c2_u);
static void c2_polyder(SFc2_wheel2InstanceStruct *chartInstance, real_T c2_u[4],
  real_T c2_a_data[], int32_T c2_a_sizes[2]);
static boolean_T c2_isfinite(SFc2_wheel2InstanceStruct *chartInstance, real_T
  c2_x);
static real_T c2_polyval(SFc2_wheel2InstanceStruct *chartInstance, real_T
  c2_p_data[], int32_T c2_p_sizes[2], real_T c2_x);
static void c2_trim_leading_zeros(SFc2_wheel2InstanceStruct *chartInstance,
  real_T c2_x_data[], int32_T c2_x_sizes[2], real_T c2_y_data[], int32_T
  c2_y_sizes[2]);
static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_g_emlrt_marshallIn(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_h_emlrt_marshallIn(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_b_is_active_c2_wheel2, const char_T *c2_identifier);
static uint8_T c2_i_emlrt_marshallIn(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void init_dsm_address_info(SFc2_wheel2InstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c2_wheel2(SFc2_wheel2InstanceStruct *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_is_active_c2_wheel2 = 0U;
}

static void initialize_params_c2_wheel2(SFc2_wheel2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c2_wheel2(SFc2_wheel2InstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c2_wheel2(SFc2_wheel2InstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c2_update_debugger_state_c2_wheel2(SFc2_wheel2InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c2_wheel2(SFc2_wheel2InstanceStruct
  *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_u;
  const mxArray *c2_b_y = NULL;
  real_T c2_b_hoistedGlobal;
  real_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  real_T c2_c_hoistedGlobal;
  real_T c2_c_u;
  const mxArray *c2_d_y = NULL;
  uint8_T c2_d_hoistedGlobal;
  uint8_T c2_d_u;
  const mxArray *c2_e_y = NULL;
  real_T *c2_d2td;
  real_T *c2_dtd;
  real_T *c2_td;
  c2_d2td = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_dtd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_td = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellmatrix(4, 1), false);
  c2_hoistedGlobal = *c2_d2td;
  c2_u = c2_hoistedGlobal;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_b_hoistedGlobal = *c2_dtd;
  c2_b_u = c2_b_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_c_hoistedGlobal = *c2_td;
  c2_c_u = c2_c_hoistedGlobal;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  c2_d_hoistedGlobal = chartInstance->c2_is_active_c2_wheel2;
  c2_d_u = c2_d_hoistedGlobal;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_d_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 3, c2_e_y);
  sf_mex_assign(&c2_st, c2_y, false);
  return c2_st;
}

static void set_sim_state_c2_wheel2(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T *c2_d2td;
  real_T *c2_dtd;
  real_T *c2_td;
  c2_d2td = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_dtd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_td = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = true;
  c2_u = sf_mex_dup(c2_st);
  *c2_d2td = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u,
    0)), "d2td");
  *c2_dtd = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 1)),
    "dtd");
  *c2_td = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 2)),
    "td");
  chartInstance->c2_is_active_c2_wheel2 = c2_h_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 3)), "is_active_c2_wheel2");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_wheel2(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_wheel2(SFc2_wheel2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c2_wheel2(SFc2_wheel2InstanceStruct *chartInstance)
{
  int32_T c2_i0;
  int32_T c2_i1;
  real_T *c2_td;
  real_T *c2_t;
  real_T *c2_dtd;
  real_T *c2_d2td;
  real_T (*c2_times)[8];
  real_T (*c2_array)[32];
  c2_d2td = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_dtd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_td = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_times = (real_T (*)[8])ssGetInputPortSignal(chartInstance->S, 1);
  c2_array = (real_T (*)[32])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  for (c2_i0 = 0; c2_i0 < 32; c2_i0++) {
    _SFD_DATA_RANGE_CHECK((*c2_array)[c2_i0], 0U);
  }

  for (c2_i1 = 0; c2_i1 < 8; c2_i1++) {
    _SFD_DATA_RANGE_CHECK((*c2_times)[c2_i1], 1U);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_wheel2(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_wheel2MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*c2_td, 2U);
  _SFD_DATA_RANGE_CHECK(*c2_t, 3U);
  _SFD_DATA_RANGE_CHECK(*c2_dtd, 4U);
  _SFD_DATA_RANGE_CHECK(*c2_d2td, 5U);
}

static void c2_chartstep_c2_wheel2(SFc2_wheel2InstanceStruct *chartInstance)
{
  real_T c2_hoistedGlobal;
  int32_T c2_i2;
  real_T c2_array[32];
  int32_T c2_i3;
  real_T c2_times[8];
  real_T c2_t;
  uint32_T c2_debug_family_var_map[13];
  int32_T c2_n_sizes;
  real_T c2_n_data[8];
  real_T c2_m[2];
  real_T c2_a2[4];
  int32_T c2_ad_sizes[2];
  real_T c2_ad_data[3];
  int32_T c2_ad2_sizes[2];
  real_T c2_ad2_data[2];
  real_T c2_n;
  real_T c2_nargin = 3.0;
  real_T c2_nargout = 3.0;
  real_T c2_td;
  real_T c2_dtd;
  real_T c2_d2td;
  int32_T c2_i4;
  boolean_T c2_x[8];
  int32_T c2_idx;
  static int32_T c2_iv0[1] = { 8 };

  int32_T c2_ii_sizes;
  int32_T c2_ii;
  int32_T c2_b_ii;
  int32_T c2_a;
  int32_T c2_b_a;
  int32_T c2_ii_data[8];
  boolean_T c2_b0;
  boolean_T c2_b1;
  boolean_T c2_b2;
  int32_T c2_i5;
  int32_T c2_tmp_sizes;
  int32_T c2_loop_ub;
  int32_T c2_i6;
  int32_T c2_tmp_data[8];
  int32_T c2_b_ii_sizes;
  int32_T c2_b_loop_ub;
  int32_T c2_i7;
  int32_T c2_b_ii_data[8];
  int32_T c2_c_loop_ub;
  int32_T c2_i8;
  int32_T c2_d_loop_ub;
  int32_T c2_i9;
  int32_T c2_i10;
  int32_T c2_b_n;
  int32_T c2_i11;
  int32_T c2_i12;
  real_T c2_p[4];
  real_T c2_b_x;
  int32_T c2_k;
  real_T c2_b_k;
  int32_T c2_i13;
  real_T c2_b_a2[4];
  int32_T c2_b_tmp_sizes[2];
  real_T c2_b_tmp_data[3];
  int32_T c2_ad;
  int32_T c2_b_ad;
  int32_T c2_e_loop_ub;
  int32_T c2_i14;
  int32_T c2_b_ad_sizes[2];
  int32_T c2_c_ad;
  int32_T c2_d_ad;
  int32_T c2_f_loop_ub;
  int32_T c2_i15;
  real_T c2_b_ad_data[3];
  int32_T c2_u_sizes[2];
  int32_T c2_u;
  int32_T c2_b_u;
  int32_T c2_g_loop_ub;
  int32_T c2_i16;
  real_T c2_u_data[3];
  int32_T c2_b_u_sizes[2];
  int32_T c2_c_u;
  int32_T c2_d_u;
  int32_T c2_h_loop_ub;
  int32_T c2_i17;
  real_T c2_b_u_data[3];
  int32_T c2_c_tmp_sizes[2];
  real_T c2_c_tmp_data[2];
  int32_T c2_ad2;
  int32_T c2_b_ad2;
  int32_T c2_i_loop_ub;
  int32_T c2_i18;
  int32_T c2_c_n;
  int32_T c2_i19;
  int32_T c2_c_k;
  boolean_T c2_b3;
  int32_T c2_b_ad2_sizes[2];
  int32_T c2_c_ad2;
  int32_T c2_d_ad2;
  int32_T c2_j_loop_ub;
  int32_T c2_i20;
  real_T c2_b_ad2_data[2];
  real_T *c2_b_t;
  real_T *c2_b_td;
  real_T *c2_b_dtd;
  real_T *c2_b_d2td;
  real_T (*c2_b_times)[8];
  real_T (*c2_b_array)[32];
  boolean_T exitg1;
  boolean_T guard1 = false;
  c2_b_d2td = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_b_dtd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_b_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_b_td = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_times = (real_T (*)[8])ssGetInputPortSignal(chartInstance->S, 1);
  c2_b_array = (real_T (*)[32])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *c2_b_t;
  for (c2_i2 = 0; c2_i2 < 32; c2_i2++) {
    c2_array[c2_i2] = (*c2_b_array)[c2_i2];
  }

  for (c2_i3 = 0; c2_i3 < 8; c2_i3++) {
    c2_times[c2_i3] = (*c2_b_times)[c2_i3];
  }

  c2_t = c2_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 13U, 14U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c2_n_data, (const int32_T *)
    &c2_n_sizes, NULL, 0, -1, (void *)c2_h_sf_marshallOut, (void *)
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_m, 1U, c2_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_a2, 2U, c2_f_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c2_ad_data, (const int32_T *)
    &c2_ad_sizes, NULL, 0, 3, (void *)c2_e_sf_marshallOut, (void *)
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c2_ad2_data, (const int32_T *)
    &c2_ad2_sizes, NULL, 0, 4, (void *)c2_d_sf_marshallOut, (void *)
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_n, MAX_uint32_T, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 5U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 6U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_array, 7U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_times, 8U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_t, 9U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_td, 10U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_dtd, 11U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_d2td, 12U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 2);
  for (c2_i4 = 0; c2_i4 < 8; c2_i4++) {
    c2_x[c2_i4] = (c2_t <= c2_times[c2_i4]);
  }

  c2_idx = 0;
  c2_ii_sizes = c2_iv0[0];
  c2_ii = 1;
  exitg1 = false;
  while ((exitg1 == false) && (c2_ii < 9)) {
    c2_b_ii = c2_ii;
    guard1 = false;
    if (c2_x[c2_b_ii - 1]) {
      c2_a = c2_idx;
      c2_b_a = c2_a + 1;
      c2_idx = c2_b_a;
      c2_ii_data[c2_idx - 1] = c2_b_ii;
      if (c2_idx >= 8) {
        exitg1 = true;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1 == true) {
      c2_ii++;
    }
  }

  c2_b0 = (1 > c2_idx);
  c2_b1 = c2_b0;
  c2_b2 = c2_b1;
  if (c2_b2) {
    c2_i5 = 0;
  } else {
    c2_i5 = _SFD_EML_ARRAY_BOUNDS_CHECK("", c2_idx, 1, 8, 0, 0);
  }

  c2_tmp_sizes = c2_i5;
  c2_loop_ub = c2_i5 - 1;
  for (c2_i6 = 0; c2_i6 <= c2_loop_ub; c2_i6++) {
    c2_tmp_data[c2_i6] = 1 + c2_i6;
  }

  _SFD_VECTOR_VECTOR_INDEX_CHECK(8, 1, 1, c2_tmp_sizes);
  c2_b_ii_sizes = c2_tmp_sizes;
  c2_b_loop_ub = c2_tmp_sizes - 1;
  for (c2_i7 = 0; c2_i7 <= c2_b_loop_ub; c2_i7++) {
    c2_b_ii_data[c2_i7] = c2_ii_data[c2_tmp_data[c2_i7] - 1];
  }

  c2_ii_sizes = c2_b_ii_sizes;
  c2_c_loop_ub = c2_b_ii_sizes - 1;
  for (c2_i8 = 0; c2_i8 <= c2_c_loop_ub; c2_i8++) {
    c2_ii_data[c2_i8] = c2_b_ii_data[c2_i8];
  }

  c2_n_sizes = c2_ii_sizes;
  c2_d_loop_ub = c2_ii_sizes - 1;
  for (c2_i9 = 0; c2_i9 <= c2_d_loop_ub; c2_i9++) {
    c2_n_data[c2_i9] = (real_T)c2_ii_data[c2_i9];
  }

  _SFD_SYMBOL_SWITCH(0U, 0U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 3);
  if (CV_EML_IF(0, 1, 0, c2_n_sizes == 0)) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
    c2_n = 8.0;
    _SFD_SYMBOL_SWITCH(0U, 5U);
  } else {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 6);
    c2_n = c2_n_data[0];
    _SFD_SYMBOL_SWITCH(0U, 5U);
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 8);
  for (c2_i10 = 0; c2_i10 < 2; c2_i10++) {
    c2_m[c2_i10] = 8.0 + -4.0 * (real_T)c2_i10;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 9);
  CV_EML_IF(0, 1, 1, true);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 10);
  c2_b_n = (int32_T)c2_n - 1;
  for (c2_i11 = 0; c2_i11 < 4; c2_i11++) {
    c2_a2[c2_i11] = c2_array[c2_b_n + (c2_i11 << 3)];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 14);
  for (c2_i12 = 0; c2_i12 < 4; c2_i12++) {
    c2_p[c2_i12] = c2_a2[c2_i12];
  }

  c2_b_x = c2_t;
  c2_td = c2_p[0];
  for (c2_k = 0; c2_k < 3; c2_k++) {
    c2_b_k = 2.0 + (real_T)c2_k;
    c2_td = c2_b_x * c2_td + c2_p[(int32_T)c2_b_k - 1];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 16);
  for (c2_i13 = 0; c2_i13 < 4; c2_i13++) {
    c2_b_a2[c2_i13] = c2_a2[c2_i13];
  }

  c2_polyder(chartInstance, c2_b_a2, c2_b_tmp_data, c2_b_tmp_sizes);
  c2_ad_sizes[0] = 1;
  c2_ad_sizes[1] = c2_b_tmp_sizes[1];
  c2_ad = c2_ad_sizes[0];
  c2_b_ad = c2_ad_sizes[1];
  c2_e_loop_ub = c2_b_tmp_sizes[0] * c2_b_tmp_sizes[1] - 1;
  for (c2_i14 = 0; c2_i14 <= c2_e_loop_ub; c2_i14++) {
    c2_ad_data[c2_i14] = c2_b_tmp_data[c2_i14];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 17);
  c2_b_ad_sizes[0] = 1;
  c2_b_ad_sizes[1] = c2_ad_sizes[1];
  c2_c_ad = c2_b_ad_sizes[0];
  c2_d_ad = c2_b_ad_sizes[1];
  c2_f_loop_ub = c2_ad_sizes[0] * c2_ad_sizes[1] - 1;
  for (c2_i15 = 0; c2_i15 <= c2_f_loop_ub; c2_i15++) {
    c2_b_ad_data[c2_i15] = c2_ad_data[c2_i15];
  }

  c2_dtd = c2_polyval(chartInstance, c2_b_ad_data, c2_b_ad_sizes, c2_t);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 19);
  c2_u_sizes[0] = 1;
  c2_u_sizes[1] = c2_ad_sizes[1];
  c2_u = c2_u_sizes[0];
  c2_b_u = c2_u_sizes[1];
  c2_g_loop_ub = c2_ad_sizes[0] * c2_ad_sizes[1] - 1;
  for (c2_i16 = 0; c2_i16 <= c2_g_loop_ub; c2_i16++) {
    c2_u_data[c2_i16] = c2_ad_data[c2_i16];
  }

  c2_b_u_sizes[0] = 1;
  c2_b_u_sizes[1] = c2_u_sizes[1];
  c2_c_u = c2_b_u_sizes[0];
  c2_d_u = c2_b_u_sizes[1];
  c2_h_loop_ub = c2_u_sizes[0] * c2_u_sizes[1] - 1;
  for (c2_i17 = 0; c2_i17 <= c2_h_loop_ub; c2_i17++) {
    c2_b_u_data[c2_i17] = c2_u_data[c2_i17];
  }

  c2_trim_leading_zeros(chartInstance, c2_b_u_data, c2_b_u_sizes, c2_c_tmp_data,
                        c2_c_tmp_sizes);
  c2_ad2_sizes[0] = 1;
  c2_ad2_sizes[1] = c2_c_tmp_sizes[1];
  c2_ad2 = c2_ad2_sizes[0];
  c2_b_ad2 = c2_ad2_sizes[1];
  c2_i_loop_ub = c2_c_tmp_sizes[0] * c2_c_tmp_sizes[1] - 1;
  for (c2_i18 = 0; c2_i18 <= c2_i_loop_ub; c2_i18++) {
    c2_ad2_data[c2_i18] = c2_c_tmp_data[c2_i18];
  }

  c2_c_n = c2_ad2_sizes[1];
  c2_i19 = c2_c_n;
  c2_c_k = 1;
  while (c2_c_k <= c2_i19 - 1) {
    _SFD_EML_ARRAY_BOUNDS_CHECK("", 1, 1, c2_ad2_sizes[1], 1, 0);
    _SFD_EML_ARRAY_BOUNDS_CHECK("", 1, 1, c2_ad2_sizes[1], 1, 0);
    c2_ad2_data[0] *= (real_T)c2_c_n;
    c2_c_k = 2;
  }

  c2_b3 = (c2_u_sizes[1] == 0);
  if (!c2_b3) {
    if (!c2_isfinite(chartInstance, c2_u_data[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          c2_u_sizes[1], 1, c2_u_sizes[1], 1, 0) - 1])) {
      c2_ad2_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_ad2_sizes[1], 1,
        c2_ad2_sizes[1], 1, 0) - 1] = rtNaN;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 20);
  c2_b_ad2_sizes[0] = 1;
  c2_b_ad2_sizes[1] = c2_ad2_sizes[1];
  c2_c_ad2 = c2_b_ad2_sizes[0];
  c2_d_ad2 = c2_b_ad2_sizes[1];
  c2_j_loop_ub = c2_ad2_sizes[0] * c2_ad2_sizes[1] - 1;
  for (c2_i20 = 0; c2_i20 <= c2_j_loop_ub; c2_i20++) {
    c2_b_ad2_data[c2_i20] = c2_ad2_data[c2_i20];
  }

  c2_d2td = c2_polyval(chartInstance, c2_b_ad2_data, c2_b_ad2_sizes, c2_t);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -20);
  _SFD_SYMBOL_SCOPE_POP();
  *c2_b_td = c2_td;
  *c2_b_dtd = c2_dtd;
  *c2_b_d2td = c2_d2td;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_wheel2(SFc2_wheel2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber)
{
  (void)c2_machineNumber;
  (void)c2_chartNumber;
  (void)c2_instanceNumber;
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_wheel2InstanceStruct *chartInstance;
  chartInstance = (SFc2_wheel2InstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static real_T c2_emlrt_marshallIn(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_d2td, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_d2td), &c2_thisId);
  sf_mex_destroy(&c2_d2td);
  return c2_y;
}

static real_T c2_b_emlrt_marshallIn(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_d2td;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_wheel2InstanceStruct *chartInstance;
  chartInstance = (SFc2_wheel2InstanceStruct *)chartInstanceVoid;
  c2_d2td = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_d2td), &c2_thisId);
  sf_mex_destroy(&c2_d2td);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i21;
  real_T c2_b_inData[8];
  int32_T c2_i22;
  real_T c2_u[8];
  const mxArray *c2_y = NULL;
  SFc2_wheel2InstanceStruct *chartInstance;
  chartInstance = (SFc2_wheel2InstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i21 = 0; c2_i21 < 8; c2_i21++) {
    c2_b_inData[c2_i21] = (*(real_T (*)[8])c2_inData)[c2_i21];
  }

  for (c2_i22 = 0; c2_i22 < 8; c2_i22++) {
    c2_u[c2_i22] = c2_b_inData[c2_i22];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 8), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i23;
  int32_T c2_i24;
  int32_T c2_i25;
  real_T c2_b_inData[32];
  int32_T c2_i26;
  int32_T c2_i27;
  int32_T c2_i28;
  real_T c2_u[32];
  const mxArray *c2_y = NULL;
  SFc2_wheel2InstanceStruct *chartInstance;
  chartInstance = (SFc2_wheel2InstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i23 = 0;
  for (c2_i24 = 0; c2_i24 < 4; c2_i24++) {
    for (c2_i25 = 0; c2_i25 < 8; c2_i25++) {
      c2_b_inData[c2_i25 + c2_i23] = (*(real_T (*)[32])c2_inData)[c2_i25 +
        c2_i23];
    }

    c2_i23 += 8;
  }

  c2_i26 = 0;
  for (c2_i27 = 0; c2_i27 < 4; c2_i27++) {
    for (c2_i28 = 0; c2_i28 < 8; c2_i28++) {
      c2_u[c2_i28 + c2_i26] = c2_b_inData[c2_i28 + c2_i26];
    }

    c2_i26 += 8;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 8, 4), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, real_T
  c2_inData_data[], int32_T c2_inData_sizes[2])
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_b_inData_sizes[2];
  int32_T c2_loop_ub;
  int32_T c2_i29;
  real_T c2_b_inData_data[2];
  int32_T c2_u_sizes[2];
  int32_T c2_b_loop_ub;
  int32_T c2_i30;
  real_T c2_u_data[2];
  const mxArray *c2_y = NULL;
  SFc2_wheel2InstanceStruct *chartInstance;
  chartInstance = (SFc2_wheel2InstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_b_inData_sizes[0] = 1;
  c2_b_inData_sizes[1] = c2_inData_sizes[1];
  c2_loop_ub = c2_inData_sizes[1] - 1;
  for (c2_i29 = 0; c2_i29 <= c2_loop_ub; c2_i29++) {
    c2_b_inData_data[c2_b_inData_sizes[0] * c2_i29] =
      c2_inData_data[c2_inData_sizes[0] * c2_i29];
  }

  c2_u_sizes[0] = 1;
  c2_u_sizes[1] = c2_b_inData_sizes[1];
  c2_b_loop_ub = c2_b_inData_sizes[1] - 1;
  for (c2_i30 = 0; c2_i30 <= c2_b_loop_ub; c2_i30++) {
    c2_u_data[c2_u_sizes[0] * c2_i30] = c2_b_inData_data[c2_b_inData_sizes[0] *
      c2_i30];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u_data, 0, 0U, 1U, 0U, 2,
    c2_u_sizes[0], c2_u_sizes[1]), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_c_emlrt_marshallIn(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y_data[],
  int32_T c2_y_sizes[2])
{
  int32_T c2_i31;
  uint32_T c2_uv0[2];
  int32_T c2_i32;
  static boolean_T c2_bv0[2] = { false, true };

  boolean_T c2_bv1[2];
  int32_T c2_tmp_sizes[2];
  real_T c2_tmp_data[2];
  int32_T c2_y;
  int32_T c2_b_y;
  int32_T c2_loop_ub;
  int32_T c2_i33;
  (void)chartInstance;
  for (c2_i31 = 0; c2_i31 < 2; c2_i31++) {
    c2_uv0[c2_i31] = 1U + (uint32_T)c2_i31;
  }

  for (c2_i32 = 0; c2_i32 < 2; c2_i32++) {
    c2_bv1[c2_i32] = c2_bv0[c2_i32];
  }

  sf_mex_import_vs(c2_parentId, sf_mex_dup(c2_u), c2_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c2_bv1, c2_uv0, c2_tmp_sizes);
  c2_y_sizes[0] = 1;
  c2_y_sizes[1] = c2_tmp_sizes[1];
  c2_y = c2_y_sizes[0];
  c2_b_y = c2_y_sizes[1];
  c2_loop_ub = c2_tmp_sizes[0] * c2_tmp_sizes[1] - 1;
  for (c2_i33 = 0; c2_i33 <= c2_loop_ub; c2_i33++) {
    c2_y_data[c2_i33] = c2_tmp_data[c2_i33];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real_T c2_outData_data[], int32_T
  c2_outData_sizes[2])
{
  const mxArray *c2_ad2;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y_sizes[2];
  real_T c2_y_data[2];
  int32_T c2_loop_ub;
  int32_T c2_i34;
  SFc2_wheel2InstanceStruct *chartInstance;
  chartInstance = (SFc2_wheel2InstanceStruct *)chartInstanceVoid;
  c2_ad2 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_ad2), &c2_thisId, c2_y_data,
                        c2_y_sizes);
  sf_mex_destroy(&c2_ad2);
  c2_outData_sizes[0] = 1;
  c2_outData_sizes[1] = c2_y_sizes[1];
  c2_loop_ub = c2_y_sizes[1] - 1;
  for (c2_i34 = 0; c2_i34 <= c2_loop_ub; c2_i34++) {
    c2_outData_data[c2_outData_sizes[0] * c2_i34] = c2_y_data[c2_y_sizes[0] *
      c2_i34];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, real_T
  c2_inData_data[], int32_T c2_inData_sizes[2])
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_b_inData_sizes[2];
  int32_T c2_loop_ub;
  int32_T c2_i35;
  real_T c2_b_inData_data[3];
  int32_T c2_u_sizes[2];
  int32_T c2_b_loop_ub;
  int32_T c2_i36;
  real_T c2_u_data[3];
  const mxArray *c2_y = NULL;
  SFc2_wheel2InstanceStruct *chartInstance;
  chartInstance = (SFc2_wheel2InstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_b_inData_sizes[0] = 1;
  c2_b_inData_sizes[1] = c2_inData_sizes[1];
  c2_loop_ub = c2_inData_sizes[1] - 1;
  for (c2_i35 = 0; c2_i35 <= c2_loop_ub; c2_i35++) {
    c2_b_inData_data[c2_b_inData_sizes[0] * c2_i35] =
      c2_inData_data[c2_inData_sizes[0] * c2_i35];
  }

  c2_u_sizes[0] = 1;
  c2_u_sizes[1] = c2_b_inData_sizes[1];
  c2_b_loop_ub = c2_b_inData_sizes[1] - 1;
  for (c2_i36 = 0; c2_i36 <= c2_b_loop_ub; c2_i36++) {
    c2_u_data[c2_u_sizes[0] * c2_i36] = c2_b_inData_data[c2_b_inData_sizes[0] *
      c2_i36];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u_data, 0, 0U, 1U, 0U, 2,
    c2_u_sizes[0], c2_u_sizes[1]), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_d_emlrt_marshallIn(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y_data[],
  int32_T c2_y_sizes[2])
{
  int32_T c2_i37;
  uint32_T c2_uv1[2];
  int32_T c2_i38;
  static boolean_T c2_bv2[2] = { false, true };

  boolean_T c2_bv3[2];
  int32_T c2_tmp_sizes[2];
  real_T c2_tmp_data[3];
  int32_T c2_y;
  int32_T c2_b_y;
  int32_T c2_loop_ub;
  int32_T c2_i39;
  (void)chartInstance;
  for (c2_i37 = 0; c2_i37 < 2; c2_i37++) {
    c2_uv1[c2_i37] = 1U + ((uint32_T)c2_i37 << 1);
  }

  for (c2_i38 = 0; c2_i38 < 2; c2_i38++) {
    c2_bv3[c2_i38] = c2_bv2[c2_i38];
  }

  sf_mex_import_vs(c2_parentId, sf_mex_dup(c2_u), c2_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c2_bv3, c2_uv1, c2_tmp_sizes);
  c2_y_sizes[0] = 1;
  c2_y_sizes[1] = c2_tmp_sizes[1];
  c2_y = c2_y_sizes[0];
  c2_b_y = c2_y_sizes[1];
  c2_loop_ub = c2_tmp_sizes[0] * c2_tmp_sizes[1] - 1;
  for (c2_i39 = 0; c2_i39 <= c2_loop_ub; c2_i39++) {
    c2_y_data[c2_i39] = c2_tmp_data[c2_i39];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real_T c2_outData_data[], int32_T
  c2_outData_sizes[2])
{
  const mxArray *c2_ad;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y_sizes[2];
  real_T c2_y_data[3];
  int32_T c2_loop_ub;
  int32_T c2_i40;
  SFc2_wheel2InstanceStruct *chartInstance;
  chartInstance = (SFc2_wheel2InstanceStruct *)chartInstanceVoid;
  c2_ad = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_ad), &c2_thisId, c2_y_data,
                        c2_y_sizes);
  sf_mex_destroy(&c2_ad);
  c2_outData_sizes[0] = 1;
  c2_outData_sizes[1] = c2_y_sizes[1];
  c2_loop_ub = c2_y_sizes[1] - 1;
  for (c2_i40 = 0; c2_i40 <= c2_loop_ub; c2_i40++) {
    c2_outData_data[c2_outData_sizes[0] * c2_i40] = c2_y_data[c2_y_sizes[0] *
      c2_i40];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i41;
  real_T c2_b_inData[4];
  int32_T c2_i42;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_wheel2InstanceStruct *chartInstance;
  chartInstance = (SFc2_wheel2InstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i41 = 0; c2_i41 < 4; c2_i41++) {
    c2_b_inData[c2_i41] = (*(real_T (*)[4])c2_inData)[c2_i41];
  }

  for (c2_i42 = 0; c2_i42 < 4; c2_i42++) {
    c2_u[c2_i42] = c2_b_inData[c2_i42];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 4), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_e_emlrt_marshallIn(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[4])
{
  real_T c2_dv0[4];
  int32_T c2_i43;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv0, 1, 0, 0U, 1, 0U, 2, 1, 4);
  for (c2_i43 = 0; c2_i43 < 4; c2_i43++) {
    c2_y[c2_i43] = c2_dv0[c2_i43];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_a2;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i44;
  SFc2_wheel2InstanceStruct *chartInstance;
  chartInstance = (SFc2_wheel2InstanceStruct *)chartInstanceVoid;
  c2_a2 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_a2), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_a2);
  for (c2_i44 = 0; c2_i44 < 4; c2_i44++) {
    (*(real_T (*)[4])c2_outData)[c2_i44] = c2_y[c2_i44];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i45;
  real_T c2_b_inData[2];
  int32_T c2_i46;
  real_T c2_u[2];
  const mxArray *c2_y = NULL;
  SFc2_wheel2InstanceStruct *chartInstance;
  chartInstance = (SFc2_wheel2InstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i45 = 0; c2_i45 < 2; c2_i45++) {
    c2_b_inData[c2_i45] = (*(real_T (*)[2])c2_inData)[c2_i45];
  }

  for (c2_i46 = 0; c2_i46 < 2; c2_i46++) {
    c2_u[c2_i46] = c2_b_inData[c2_i46];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 2), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, real_T
  c2_inData_data[], int32_T *c2_inData_sizes)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_b_inData_sizes;
  int32_T c2_loop_ub;
  int32_T c2_i47;
  real_T c2_b_inData_data[8];
  int32_T c2_u_sizes;
  int32_T c2_b_loop_ub;
  int32_T c2_i48;
  real_T c2_u_data[8];
  const mxArray *c2_y = NULL;
  SFc2_wheel2InstanceStruct *chartInstance;
  chartInstance = (SFc2_wheel2InstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_b_inData_sizes = *c2_inData_sizes;
  c2_loop_ub = *c2_inData_sizes - 1;
  for (c2_i47 = 0; c2_i47 <= c2_loop_ub; c2_i47++) {
    c2_b_inData_data[c2_i47] = c2_inData_data[c2_i47];
  }

  c2_u_sizes = c2_b_inData_sizes;
  c2_b_loop_ub = c2_b_inData_sizes - 1;
  for (c2_i48 = 0; c2_i48 <= c2_b_loop_ub; c2_i48++) {
    c2_u_data[c2_i48] = c2_b_inData_data[c2_i48];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u_data, 0, 0U, 1U, 0U, 1,
    c2_u_sizes), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_f_emlrt_marshallIn(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y_data[],
  int32_T *c2_y_sizes)
{
  static uint32_T c2_uv2[1] = { 8U };

  uint32_T c2_uv3[1];
  static boolean_T c2_bv4[1] = { true };

  boolean_T c2_bv5[1];
  int32_T c2_tmp_sizes;
  real_T c2_tmp_data[8];
  int32_T c2_loop_ub;
  int32_T c2_i49;
  (void)chartInstance;
  c2_uv3[0] = c2_uv2[0];
  c2_bv5[0] = c2_bv4[0];
  sf_mex_import_vs(c2_parentId, sf_mex_dup(c2_u), c2_tmp_data, 1, 0, 0U, 1, 0U,
                   1, c2_bv5, c2_uv3, &c2_tmp_sizes);
  *c2_y_sizes = c2_tmp_sizes;
  c2_loop_ub = c2_tmp_sizes - 1;
  for (c2_i49 = 0; c2_i49 <= c2_loop_ub; c2_i49++) {
    c2_y_data[c2_i49] = c2_tmp_data[c2_i49];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real_T c2_outData_data[], int32_T
  *c2_outData_sizes)
{
  const mxArray *c2_n;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y_sizes;
  real_T c2_y_data[8];
  int32_T c2_loop_ub;
  int32_T c2_i50;
  SFc2_wheel2InstanceStruct *chartInstance;
  chartInstance = (SFc2_wheel2InstanceStruct *)chartInstanceVoid;
  c2_n = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_n), &c2_thisId, c2_y_data,
                        &c2_y_sizes);
  sf_mex_destroy(&c2_n);
  *c2_outData_sizes = c2_y_sizes;
  c2_loop_ub = c2_y_sizes - 1;
  for (c2_i50 = 0; c2_i50 <= c2_loop_ub; c2_i50++) {
    c2_outData_data[c2_i50] = c2_y_data[c2_i50];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_wheel2_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  sf_mex_assign(&c2_nameCaptureInfo, sf_mex_createstruct("structure", 2, 24, 1),
                false);
  c2_info_helper(&c2_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(const mxArray **c2_info)
{
  const mxArray *c2_rhs0 = NULL;
  const mxArray *c2_lhs0 = NULL;
  const mxArray *c2_rhs1 = NULL;
  const mxArray *c2_lhs1 = NULL;
  const mxArray *c2_rhs2 = NULL;
  const mxArray *c2_lhs2 = NULL;
  const mxArray *c2_rhs3 = NULL;
  const mxArray *c2_lhs3 = NULL;
  const mxArray *c2_rhs4 = NULL;
  const mxArray *c2_lhs4 = NULL;
  const mxArray *c2_rhs5 = NULL;
  const mxArray *c2_lhs5 = NULL;
  const mxArray *c2_rhs6 = NULL;
  const mxArray *c2_lhs6 = NULL;
  const mxArray *c2_rhs7 = NULL;
  const mxArray *c2_lhs7 = NULL;
  const mxArray *c2_rhs8 = NULL;
  const mxArray *c2_lhs8 = NULL;
  const mxArray *c2_rhs9 = NULL;
  const mxArray *c2_lhs9 = NULL;
  const mxArray *c2_rhs10 = NULL;
  const mxArray *c2_lhs10 = NULL;
  const mxArray *c2_rhs11 = NULL;
  const mxArray *c2_lhs11 = NULL;
  const mxArray *c2_rhs12 = NULL;
  const mxArray *c2_lhs12 = NULL;
  const mxArray *c2_rhs13 = NULL;
  const mxArray *c2_lhs13 = NULL;
  const mxArray *c2_rhs14 = NULL;
  const mxArray *c2_lhs14 = NULL;
  const mxArray *c2_rhs15 = NULL;
  const mxArray *c2_lhs15 = NULL;
  const mxArray *c2_rhs16 = NULL;
  const mxArray *c2_lhs16 = NULL;
  const mxArray *c2_rhs17 = NULL;
  const mxArray *c2_lhs17 = NULL;
  const mxArray *c2_rhs18 = NULL;
  const mxArray *c2_lhs18 = NULL;
  const mxArray *c2_rhs19 = NULL;
  const mxArray *c2_lhs19 = NULL;
  const mxArray *c2_rhs20 = NULL;
  const mxArray *c2_lhs20 = NULL;
  const mxArray *c2_rhs21 = NULL;
  const mxArray *c2_lhs21 = NULL;
  const mxArray *c2_rhs22 = NULL;
  const mxArray *c2_lhs22 = NULL;
  const mxArray *c2_rhs23 = NULL;
  const mxArray *c2_lhs23 = NULL;
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("find"), "name", "name", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1303171406U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c2_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find"),
                  "context", "context", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c2_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find"),
                  "context", "context", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c2_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389333120U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c2_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find"),
                  "context", "context", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c2_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362287082U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c2_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1381875500U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c2_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find"),
                  "context", "context", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c2_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c2_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("length"), "name", "name", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1303171406U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c2_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("polyval"), "name", "name", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/polyfun/polyval.m"), "resolved",
                  "resolved", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1305343202U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c2_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/polyfun/polyval.m"), "context",
                  "context", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c2_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389333120U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c2_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("polyder"), "name", "name", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/polyfun/polyder.m"), "resolved",
                  "resolved", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1378321192U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c2_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/polyfun/polyder.m"), "context",
                  "context", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363736156U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c2_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/polyfun/polyder.m!trim_leading_zeros"),
                  "context", "context", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c2_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/polyfun/polyder.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c2_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/polyfun/polyder.m"), "context",
                  "context", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isfinite"), "name", "name", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "resolved",
                  "resolved", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363735456U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c2_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363736156U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c2_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isinf"), "name", "name", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363735456U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c2_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "context",
                  "context", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363736156U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c2_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isnan"), "name", "name", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363735458U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c2_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363736156U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c2_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/polyfun/polyder.m"), "context",
                  "context", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.nan"), "name",
                  "name", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/nan.m"),
                  "resolved", "resolved", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362287232U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c2_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs23), "lhs", "lhs",
                  23);
  sf_mex_destroy(&c2_rhs0);
  sf_mex_destroy(&c2_lhs0);
  sf_mex_destroy(&c2_rhs1);
  sf_mex_destroy(&c2_lhs1);
  sf_mex_destroy(&c2_rhs2);
  sf_mex_destroy(&c2_lhs2);
  sf_mex_destroy(&c2_rhs3);
  sf_mex_destroy(&c2_lhs3);
  sf_mex_destroy(&c2_rhs4);
  sf_mex_destroy(&c2_lhs4);
  sf_mex_destroy(&c2_rhs5);
  sf_mex_destroy(&c2_lhs5);
  sf_mex_destroy(&c2_rhs6);
  sf_mex_destroy(&c2_lhs6);
  sf_mex_destroy(&c2_rhs7);
  sf_mex_destroy(&c2_lhs7);
  sf_mex_destroy(&c2_rhs8);
  sf_mex_destroy(&c2_lhs8);
  sf_mex_destroy(&c2_rhs9);
  sf_mex_destroy(&c2_lhs9);
  sf_mex_destroy(&c2_rhs10);
  sf_mex_destroy(&c2_lhs10);
  sf_mex_destroy(&c2_rhs11);
  sf_mex_destroy(&c2_lhs11);
  sf_mex_destroy(&c2_rhs12);
  sf_mex_destroy(&c2_lhs12);
  sf_mex_destroy(&c2_rhs13);
  sf_mex_destroy(&c2_lhs13);
  sf_mex_destroy(&c2_rhs14);
  sf_mex_destroy(&c2_lhs14);
  sf_mex_destroy(&c2_rhs15);
  sf_mex_destroy(&c2_lhs15);
  sf_mex_destroy(&c2_rhs16);
  sf_mex_destroy(&c2_lhs16);
  sf_mex_destroy(&c2_rhs17);
  sf_mex_destroy(&c2_lhs17);
  sf_mex_destroy(&c2_rhs18);
  sf_mex_destroy(&c2_lhs18);
  sf_mex_destroy(&c2_rhs19);
  sf_mex_destroy(&c2_lhs19);
  sf_mex_destroy(&c2_rhs20);
  sf_mex_destroy(&c2_lhs20);
  sf_mex_destroy(&c2_rhs21);
  sf_mex_destroy(&c2_lhs21);
  sf_mex_destroy(&c2_rhs22);
  sf_mex_destroy(&c2_lhs22);
  sf_mex_destroy(&c2_rhs23);
  sf_mex_destroy(&c2_lhs23);
}

static const mxArray *c2_emlrt_marshallOut(const char * c2_u)
{
  const mxArray *c2_y = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c2_u)), false);
  return c2_y;
}

static const mxArray *c2_b_emlrt_marshallOut(const uint32_T c2_u)
{
  const mxArray *c2_y = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 7, 0U, 0U, 0U, 0), false);
  return c2_y;
}

static void c2_polyder(SFc2_wheel2InstanceStruct *chartInstance, real_T c2_u[4],
  real_T c2_a_data[], int32_T c2_a_sizes[2])
{
  int32_T c2_nlead0;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_ny;
  const mxArray *c2_y = NULL;
  int32_T c2_tmp_sizes[2];
  int32_T c2_iv1[2];
  int32_T c2_i51;
  int32_T c2_i52;
  int32_T c2_loop_ub;
  int32_T c2_i53;
  real_T c2_tmp_data[3];
  int32_T c2_i54;
  int32_T c2_b_ny;
  int32_T c2_c_k;
  int32_T c2_n;
  int32_T c2_i55;
  int32_T c2_d_k;
  int32_T c2_e_k;
  boolean_T exitg1;
  c2_nlead0 = 0;
  c2_k = 1;
  exitg1 = false;
  while ((exitg1 == false) && (c2_k < 3)) {
    c2_b_k = c2_k - 1;
    if (c2_u[c2_b_k] == 0.0) {
      c2_nlead0++;
      c2_k++;
    } else {
      exitg1 = true;
    }
  }

  c2_ny = 3 - c2_nlead0;
  if (c2_ny <= 3) {
  } else {
    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", "Assertion failed.", 15, 0U, 0U, 0U,
      2, 1, strlen("Assertion failed.")), false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14, c2_y);
  }

  c2_tmp_sizes[0] = 1;
  c2_iv1[0] = 1;
  c2_iv1[1] = (int32_T)_SFD_NON_NEGATIVE_CHECK("", (real_T)c2_ny);
  c2_tmp_sizes[1] = c2_iv1[1];
  c2_i51 = c2_tmp_sizes[0];
  c2_i52 = c2_tmp_sizes[1];
  c2_loop_ub = (int32_T)_SFD_NON_NEGATIVE_CHECK("", (real_T)c2_ny) - 1;
  for (c2_i53 = 0; c2_i53 <= c2_loop_ub; c2_i53++) {
    c2_tmp_data[c2_i53] = 0.0;
  }

  for (c2_i54 = 0; c2_i54 < 2; c2_i54++) {
    c2_a_sizes[c2_i54] = c2_tmp_sizes[c2_i54];
  }

  c2_b_ny = c2_ny;
  for (c2_c_k = 1; c2_c_k <= c2_b_ny; c2_c_k++) {
    c2_b_k = c2_c_k;
    c2_a_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_b_k, 1, c2_a_sizes[1], 1, 0) -
      1] = c2_u[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_b_k + c2_nlead0, 1, 4, 1, 0)
      - 1];
  }

  c2_n = c2_a_sizes[1] - 1;
  c2_i55 = c2_n;
  for (c2_d_k = 1; c2_d_k <= c2_i55; c2_d_k++) {
    c2_e_k = c2_d_k;
    c2_a_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_e_k, 1, c2_a_sizes[1], 1, 0) -
      1] = c2_a_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_e_k, 1, c2_a_sizes[1], 1,
      0) - 1] * (real_T)((c2_n - c2_e_k) + 2);
  }

  if (!c2_isfinite(chartInstance, c2_u[3])) {
    c2_a_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_a_sizes[1], 1, c2_a_sizes[1], 1,
      0) - 1] = rtNaN;
  }
}

static boolean_T c2_isfinite(SFc2_wheel2InstanceStruct *chartInstance, real_T
  c2_x)
{
  real_T c2_b_x;
  boolean_T c2_b_b;
  boolean_T c2_b4;
  real_T c2_c_x;
  boolean_T c2_c_b;
  boolean_T c2_b5;
  (void)chartInstance;
  c2_b_x = c2_x;
  c2_b_b = muDoubleScalarIsInf(c2_b_x);
  c2_b4 = !c2_b_b;
  c2_c_x = c2_x;
  c2_c_b = muDoubleScalarIsNaN(c2_c_x);
  c2_b5 = !c2_c_b;
  return c2_b4 && c2_b5;
}

static real_T c2_polyval(SFc2_wheel2InstanceStruct *chartInstance, real_T
  c2_p_data[], int32_T c2_p_sizes[2], real_T c2_x)
{
  real_T c2_y;
  real_T c2_nc;
  boolean_T c2_b6;
  boolean_T c2_b7;
  real_T c2_b_nc;
  int32_T c2_i56;
  int32_T c2_k;
  real_T c2_b_k;
  (void)chartInstance;
  c2_nc = (real_T)c2_p_sizes[1];
  c2_b6 = (c2_p_sizes[1] == 0);
  c2_b7 = c2_b6;
  if (!c2_b7) {
    (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", 1, 1, c2_p_sizes[1], 1, 0);
    c2_y = c2_p_data[0];
    c2_b_nc = c2_nc;
    c2_i56 = (int32_T)(c2_b_nc + -1.0) - 1;
    for (c2_k = 0; c2_k <= c2_i56; c2_k++) {
      c2_b_k = 2.0 + (real_T)c2_k;
      c2_y = c2_x * c2_y + c2_p_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        c2_b_k, 1, c2_p_sizes[1], 1, 0) - 1];
    }
  }

  return c2_y;
}

static void c2_trim_leading_zeros(SFc2_wheel2InstanceStruct *chartInstance,
  real_T c2_x_data[], int32_T c2_x_sizes[2], real_T c2_y_data[], int32_T
  c2_y_sizes[2])
{
  int32_T c2_nx;
  int32_T c2_nymax;
  int32_T c2_tmp_sizes[2];
  int32_T c2_iv2[2];
  int32_T c2_i57;
  int32_T c2_i58;
  int32_T c2_loop_ub;
  int32_T c2_i59;
  real_T c2_tmp_data[2];
  int32_T c2_i60;
  boolean_T c2_b8;
  boolean_T c2_b9;
  int32_T c2_nlead0;
  int32_T c2_i61;
  int32_T c2_k;
  int32_T c2_ny;
  const mxArray *c2_y = NULL;
  int32_T c2_iv3[2];
  int32_T c2_i62;
  int32_T c2_i63;
  int32_T c2_b_loop_ub;
  int32_T c2_i64;
  int32_T c2_i65;
  int32_T c2_b_ny;
  int32_T c2_b_k;
  int32_T c2_c_k;
  boolean_T guard1 = false;
  boolean_T exitg1;
  (void)chartInstance;
  c2_nx = c2_x_sizes[1];
  if (c2_nx < 2) {
    c2_nymax = 1;
  } else {
    c2_nymax = c2_nx - 1;
  }

  c2_tmp_sizes[0] = 1;
  c2_iv2[0] = 1;
  c2_iv2[1] = c2_nymax;
  c2_tmp_sizes[1] = c2_iv2[1];
  c2_i57 = c2_tmp_sizes[0];
  c2_i58 = c2_tmp_sizes[1];
  c2_loop_ub = c2_nymax - 1;
  for (c2_i59 = 0; c2_i59 <= c2_loop_ub; c2_i59++) {
    c2_tmp_data[c2_i59] = 0.0;
  }

  for (c2_i60 = 0; c2_i60 < 2; c2_i60++) {
    c2_y_sizes[c2_i60] = c2_tmp_sizes[c2_i60];
  }

  c2_b8 = (c2_x_sizes[1] == 0);
  guard1 = false;
  if (c2_b8) {
    guard1 = true;
  } else {
    c2_b9 = (c2_x_sizes[1] == 1);
    if (c2_b9) {
      guard1 = true;
    } else {
      c2_nlead0 = 0;
      c2_i61 = c2_nymax;
      c2_k = 1;
      exitg1 = false;
      while ((exitg1 == false) && (c2_k <= c2_i61 - 1)) {
        _SFD_EML_ARRAY_BOUNDS_CHECK("", 1, 1, c2_x_sizes[1], 1, 0);
        if (c2_x_data[0] == 0.0) {
          c2_nlead0++;
          c2_k = 2;
        } else {
          exitg1 = true;
        }
      }

      c2_ny = c2_nymax - c2_nlead0;
      if (c2_ny <= c2_nymax) {
      } else {
        c2_y = NULL;
        sf_mex_assign(&c2_y, sf_mex_create("y", "Assertion failed.", 15, 0U, 0U,
          0U, 2, 1, strlen("Assertion failed.")), false);
        sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14, c2_y);
      }

      c2_tmp_sizes[0] = 1;
      c2_iv3[0] = 1;
      c2_iv3[1] = (int32_T)_SFD_NON_NEGATIVE_CHECK("", (real_T)c2_ny);
      c2_tmp_sizes[1] = c2_iv3[1];
      c2_i62 = c2_tmp_sizes[0];
      c2_i63 = c2_tmp_sizes[1];
      c2_b_loop_ub = (int32_T)_SFD_NON_NEGATIVE_CHECK("", (real_T)c2_ny) - 1;
      for (c2_i64 = 0; c2_i64 <= c2_b_loop_ub; c2_i64++) {
        c2_tmp_data[c2_i64] = 0.0;
      }

      for (c2_i65 = 0; c2_i65 < 2; c2_i65++) {
        c2_y_sizes[c2_i65] = c2_tmp_sizes[c2_i65];
      }

      c2_b_ny = c2_ny;
      for (c2_b_k = 1; c2_b_k <= c2_b_ny; c2_b_k++) {
        c2_c_k = c2_b_k;
        c2_y_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_c_k, 1, c2_y_sizes[1], 1, 0)
          - 1] = c2_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_c_k + c2_nlead0, 1,
          c2_x_sizes[1], 1, 0) - 1];
      }
    }
  }

  if (guard1 == true) {
    c2_y_data[0] = 0.0;
  }
}

static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_wheel2InstanceStruct *chartInstance;
  chartInstance = (SFc2_wheel2InstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static int32_T c2_g_emlrt_marshallIn(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i66;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i66, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i66;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_wheel2InstanceStruct *chartInstance;
  chartInstance = (SFc2_wheel2InstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_h_emlrt_marshallIn(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_b_is_active_c2_wheel2, const char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_wheel2), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_wheel2);
  return c2_y;
}

static uint8_T c2_i_emlrt_marshallIn(SFc2_wheel2InstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void init_dsm_address_info(SFc2_wheel2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c2_wheel2_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3004856571U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(543726975U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1451939078U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2427162485U);
}

mxArray *sf_c2_wheel2_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("0Z44jyaJHCsYAnozbQGYoF");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(8);
      pr[1] = (double)(4);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(8);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_wheel2_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c2_wheel2_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c2_wheel2(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[8],T\"d2td\",},{M[1],M[7],T\"dtd\",},{M[1],M[5],T\"td\",},{M[8],M[0],T\"is_active_c2_wheel2\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_wheel2_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_wheel2InstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc2_wheel2InstanceStruct *) chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _wheel2MachineNumber_,
           2,
           1,
           1,
           0,
           6,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize ist own list of scripts */
        init_script_number_translation(_wheel2MachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_wheel2MachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _wheel2MachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"array");
          _SFD_SET_DATA_PROPS(1,1,1,0,"times");
          _SFD_SET_DATA_PROPS(2,2,0,1,"td");
          _SFD_SET_DATA_PROPS(3,1,1,0,"t");
          _SFD_SET_DATA_PROPS(4,2,0,1,"dtd");
          _SFD_SET_DATA_PROPS(5,2,0,1,"d2td");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,2,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,300);
        _SFD_CV_INIT_EML_IF(0,1,0,67,80,104,126);
        _SFD_CV_INIT_EML_IF(0,1,1,144,157,179,203);

        {
          unsigned int dimVector[2];
          dimVector[0]= 8;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 8;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);

        {
          real_T *c2_td;
          real_T *c2_t;
          real_T *c2_dtd;
          real_T *c2_d2td;
          real_T (*c2_array)[32];
          real_T (*c2_times)[8];
          c2_d2td = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
          c2_dtd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c2_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c2_td = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c2_times = (real_T (*)[8])ssGetInputPortSignal(chartInstance->S, 1);
          c2_array = (real_T (*)[32])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c2_array);
          _SFD_SET_DATA_VALUE_PTR(1U, *c2_times);
          _SFD_SET_DATA_VALUE_PTR(2U, c2_td);
          _SFD_SET_DATA_VALUE_PTR(3U, c2_t);
          _SFD_SET_DATA_VALUE_PTR(4U, c2_dtd);
          _SFD_SET_DATA_VALUE_PTR(5U, c2_d2td);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _wheel2MachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "DQWsaBocCTmkSJV3H1Wsz";
}

static void sf_opaque_initialize_c2_wheel2(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_wheel2InstanceStruct*) chartInstanceVar)->S,
    0);
  initialize_params_c2_wheel2((SFc2_wheel2InstanceStruct*) chartInstanceVar);
  initialize_c2_wheel2((SFc2_wheel2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_wheel2(void *chartInstanceVar)
{
  enable_c2_wheel2((SFc2_wheel2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_wheel2(void *chartInstanceVar)
{
  disable_c2_wheel2((SFc2_wheel2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_wheel2(void *chartInstanceVar)
{
  sf_gateway_c2_wheel2((SFc2_wheel2InstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_wheel2(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_wheel2((SFc2_wheel2InstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_wheel2();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c2_wheel2(SimStruct* S, const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c2_wheel2();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_wheel2((SFc2_wheel2InstanceStruct*)chartInfo->chartInstance,
    mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_wheel2(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_wheel2(S);
}

static void sf_opaque_set_sim_state_c2_wheel2(SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c2_wheel2(S, st);
}

static void sf_opaque_terminate_c2_wheel2(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_wheel2InstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_wheel2_optimization_info();
    }

    finalize_c2_wheel2((SFc2_wheel2InstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_wheel2((SFc2_wheel2InstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_wheel2(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c2_wheel2((SFc2_wheel2InstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_wheel2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_wheel2_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,2,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,3);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=3; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 3; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1066616918U));
  ssSetChecksum1(S,(2788887136U));
  ssSetChecksum2(S,(3397530671U));
  ssSetChecksum3(S,(2714262984U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_wheel2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_wheel2(SimStruct *S)
{
  SFc2_wheel2InstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc2_wheel2InstanceStruct *)utMalloc(sizeof
    (SFc2_wheel2InstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_wheel2InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c2_wheel2;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c2_wheel2;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c2_wheel2;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_wheel2;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_wheel2;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c2_wheel2;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c2_wheel2;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c2_wheel2;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_wheel2;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_wheel2;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_wheel2;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c2_wheel2_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_wheel2(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_wheel2(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_wheel2(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_wheel2_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
