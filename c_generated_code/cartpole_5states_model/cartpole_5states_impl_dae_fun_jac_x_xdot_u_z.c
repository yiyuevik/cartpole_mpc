/* This file was automatically generated by CasADi 3.6.7.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) cartpole_5states_impl_dae_fun_jac_x_xdot_u_z_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_s7 CASADI_PREFIX(s7)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s1[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s2[4] = {0, 1, 0, 0};
static const casadi_int casadi_s3[3] = {0, 0, 0};
static const casadi_int casadi_s4[16] = {5, 5, 0, 0, 1, 4, 8, 8, 0, 1, 3, 4, 1, 2, 3, 4};
static const casadi_int casadi_s5[13] = {5, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s6[6] = {5, 1, 0, 2, 1, 3};
static const casadi_int casadi_s7[3] = {5, 0, 0};

/* cartpole_5states_impl_dae_fun_jac_x_xdot_u_z:(i0[5],i1[5],i2,i3[0],i4[],i5[0])->(o0[5],o1[5x5,8nz],o2[5x5,5nz],o3[5x1,2nz],o4[5x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[1]? arg[1][0] : 0;
  a1=arg[0]? arg[0][1] : 0;
  a0=(a0-a1);
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[1]? arg[1][1] : 0;
  a1=9.8100000000000005e+00;
  a2=arg[0]? arg[0][2] : 0;
  a3=sin(a2);
  a4=(a1*a3);
  a5=cos(a2);
  a6=(a4*a5);
  a7=arg[0]? arg[0][3] : 0;
  a8=casadi_sq(a7);
  a9=(a3*a8);
  a6=(a6-a9);
  a9=arg[2]? arg[2][0] : 0;
  a6=(a6+a9);
  a10=3.;
  a10=(a10-a5);
  a11=casadi_sq(a10);
  a6=(a6/a11);
  a0=(a0-a6);
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[1]? arg[1][2] : 0;
  a0=(a0-a7);
  if (res[0]!=0) res[0][2]=a0;
  a0=arg[1]? arg[1][3] : 0;
  a12=(a3*a5);
  a13=(a12*a8);
  a14=2.9430000000000000e+01;
  a15=(a14*a3);
  a13=(a13+a15);
  a15=(a5*a9);
  a13=(a13+a15);
  a15=casadi_sq(a5);
  a16=(a14-a15);
  a13=(a13/a16);
  a0=(a0+a13);
  if (res[0]!=0) res[0][3]=a0;
  a0=arg[1]? arg[1][4] : 0;
  a17=-6.3661977236758138e-01;
  a18=3.1415926535897931e+00;
  a2=(a2-a18);
  a2=(a17*a2);
  a18=(a2*a7);
  a0=(a0-a18);
  if (res[0]!=0) res[0][4]=a0;
  a0=-1.;
  if (res[1]!=0) res[1][0]=a0;
  a1=(a1*a5);
  a1=(a5*a1);
  a4=(a4*a3);
  a1=(a1-a4);
  a4=(a8*a5);
  a1=(a1-a4);
  a1=(a1/a11);
  a6=(a6/a11);
  a10=(a10+a10);
  a10=(a10*a3);
  a6=(a6*a10);
  a1=(a1-a6);
  a1=(-a1);
  if (res[1]!=0) res[1][1]=a1;
  a1=casadi_sq(a3);
  a15=(a15-a1);
  a8=(a8*a15);
  a14=(a14*a5);
  a8=(a8+a14);
  a9=(a9*a3);
  a8=(a8-a9);
  a8=(a8/a16);
  a13=(a13/a16);
  a9=(a5+a5);
  a9=(a9*a3);
  a13=(a13*a9);
  a8=(a8-a13);
  if (res[1]!=0) res[1][2]=a8;
  a17=(a17*a7);
  a17=(-a17);
  if (res[1]!=0) res[1][3]=a17;
  a7=(a7+a7);
  a3=(a3*a7);
  a3=(a3/a11);
  if (res[1]!=0) res[1][4]=a3;
  if (res[1]!=0) res[1][5]=a0;
  a12=(a12*a7);
  a12=(a12/a16);
  if (res[1]!=0) res[1][6]=a12;
  a2=(-a2);
  if (res[1]!=0) res[1][7]=a2;
  a2=1.;
  if (res[2]!=0) res[2][0]=a2;
  if (res[2]!=0) res[2][1]=a2;
  if (res[2]!=0) res[2][2]=a2;
  if (res[2]!=0) res[2][3]=a2;
  if (res[2]!=0) res[2][4]=a2;
  a11=(1./a11);
  a11=(-a11);
  if (res[3]!=0) res[3][0]=a11;
  a5=(a5/a16);
  if (res[3]!=0) res[3][1]=a5;
  return 0;
}

CASADI_SYMBOL_EXPORT int cartpole_5states_impl_dae_fun_jac_x_xdot_u_z(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int cartpole_5states_impl_dae_fun_jac_x_xdot_u_z_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int cartpole_5states_impl_dae_fun_jac_x_xdot_u_z_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void cartpole_5states_impl_dae_fun_jac_x_xdot_u_z_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int cartpole_5states_impl_dae_fun_jac_x_xdot_u_z_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void cartpole_5states_impl_dae_fun_jac_x_xdot_u_z_release(int mem) {
}

CASADI_SYMBOL_EXPORT void cartpole_5states_impl_dae_fun_jac_x_xdot_u_z_incref(void) {
}

CASADI_SYMBOL_EXPORT void cartpole_5states_impl_dae_fun_jac_x_xdot_u_z_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int cartpole_5states_impl_dae_fun_jac_x_xdot_u_z_n_in(void) { return 6;}

CASADI_SYMBOL_EXPORT casadi_int cartpole_5states_impl_dae_fun_jac_x_xdot_u_z_n_out(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_real cartpole_5states_impl_dae_fun_jac_x_xdot_u_z_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* cartpole_5states_impl_dae_fun_jac_x_xdot_u_z_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    case 5: return "i5";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* cartpole_5states_impl_dae_fun_jac_x_xdot_u_z_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    case 4: return "o4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* cartpole_5states_impl_dae_fun_jac_x_xdot_u_z_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    case 4: return casadi_s3;
    case 5: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* cartpole_5states_impl_dae_fun_jac_x_xdot_u_z_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s4;
    case 2: return casadi_s5;
    case 3: return casadi_s6;
    case 4: return casadi_s7;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int cartpole_5states_impl_dae_fun_jac_x_xdot_u_z_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6;
  if (sz_res) *sz_res = 5;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT int cartpole_5states_impl_dae_fun_jac_x_xdot_u_z_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 5*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 0*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
