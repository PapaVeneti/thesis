/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) ntnu_leg_cost_y_fun_ ## ID
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

static const casadi_int casadi_s0[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[12] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};

/* ntnu_leg_cost_y_fun:(i0[10],i1[3],i2[],i3[])->(o0[8]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[1]? arg[1][0] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[1]? arg[1][1] : 0;
  a1=arg[1]? arg[1][2] : 0;
  a2=(a0+a1);
  a3=arg[0]? arg[0][0] : 0;
  a4=sin(a3);
  a4=(a2*a4);
  if (res[0]!=0) res[0][1]=a4;
  a4=cos(a3);
  a2=(a2*a4);
  a2=(-a2);
  if (res[0]!=0) res[0][2]=a2;
  if (res[0]!=0) res[0][3]=a0;
  if (res[0]!=0) res[0][4]=a1;
  a1=5.9999999999999998e-01;
  a0=1.0000000000000000e-03;
  a2=(a0+a3);
  a4=2.;
  a2=(a2+a4);
  a2=log(a2);
  a2=(a1*a2);
  a4=5.0000000000000000e-01;
  a5=arg[0]? arg[0][1] : 0;
  a6=(a0+a5);
  a7=1.2000000000000000e+00;
  a6=(a6+a7);
  a6=log(a6);
  a6=(a4*a6);
  a2=(a2+a6);
  a6=arg[0]? arg[0][2] : 0;
  a8=(a0+a6);
  a9=1.3230000000000000e+00;
  a8=(a8+a9);
  a8=log(a8);
  a8=(a1*a8);
  a2=(a2+a8);
  a8=arg[0]? arg[0][3] : 0;
  a10=(a0+a8);
  a11=1.6499999999999999e+00;
  a10=(a10+a11);
  a10=log(a10);
  a10=(a1*a10);
  a2=(a2+a10);
  a10=arg[0]? arg[0][4] : 0;
  a12=(a0+a10);
  a13=1.4199999999999999e+00;
  a12=(a12+a13);
  a12=log(a12);
  a12=(a4*a12);
  a2=(a2+a12);
  a3=(a0-a3);
  a12=1.3100000000000001e+00;
  a3=(a3+a12);
  a3=log(a3);
  a3=(a1*a3);
  a2=(a2+a3);
  a3=(a0-a5);
  a3=(a3+a11);
  a3=log(a3);
  a3=(a1*a3);
  a2=(a2+a3);
  a3=(a0-a6);
  a3=(a3+a13);
  a3=log(a3);
  a3=(a4*a3);
  a2=(a2+a3);
  a3=(a0-a8);
  a3=(a3+a7);
  a3=log(a3);
  a4=(a4*a3);
  a2=(a2+a4);
  a0=(a0-a10);
  a0=(a0+a9);
  a0=log(a0);
  a1=(a1*a0);
  a2=(a2+a1);
  if (res[0]!=0) res[0][5]=a2;
  a2=1.3308156638417232e-01;
  a1=sin(a5);
  a2=(a2*a1);
  a1=1.2083599892846777e-01;
  a0=cos(a8);
  a1=(a1*a0);
  a2=(a2-a1);
  a1=1.2119940877652538e-01;
  a0=cos(a5);
  a1=(a1*a0);
  a2=(a2-a1);
  a1=1.3341162378892907e-01;
  a0=sin(a8);
  a1=(a1*a0);
  a2=(a2-a1);
  a1=2.9976999999999998e-01;
  a0=6.7333004875847435e-01;
  a9=cos(a5);
  a9=(a0*a9);
  a4=7.3934203546762400e-01;
  a3=sin(a5);
  a3=(a4*a3);
  a9=(a9-a3);
  a9=(a1*a9);
  a3=2.4187360515245046e-01;
  a7=cos(a6);
  a7=(a3*a7);
  a13=9.7030776516039308e-01;
  a11=sin(a6);
  a11=(a13*a11);
  a7=(a7-a11);
  a9=(a9*a7);
  a2=(a2-a9);
  a9=cos(a5);
  a4=(a4*a9);
  a9=sin(a5);
  a0=(a0*a9);
  a4=(a4+a0);
  a4=(a1*a4);
  a0=cos(a6);
  a0=(a13*a0);
  a9=sin(a6);
  a9=(a3*a9);
  a0=(a0+a9);
  a4=(a4*a0);
  a2=(a2+a4);
  a4=2.9929000000000000e-01;
  a0=6.7131110515815429e-01;
  a9=cos(a8);
  a9=(a0*a9);
  a7=7.4117568771627262e-01;
  a11=sin(a8);
  a11=(a7*a11);
  a9=(a9+a11);
  a9=(a4*a9);
  a11=2.4497734630999077e-01;
  a12=cos(a10);
  a12=(a11*a12);
  a14=9.6952880297333865e-01;
  a15=sin(a10);
  a15=(a14*a15);
  a12=(a12+a15);
  a9=(a9*a12);
  a2=(a2-a9);
  a9=cos(a8);
  a7=(a7*a9);
  a9=sin(a8);
  a0=(a0*a9);
  a7=(a7-a0);
  a7=(a4*a7);
  a0=cos(a10);
  a0=(a14*a0);
  a9=sin(a10);
  a9=(a11*a9);
  a0=(a0-a9);
  a7=(a7*a0);
  a2=(a2+a7);
  a7=8.9999999999999997e-02;
  a2=(a2-a7);
  a2=casadi_sq(a2);
  if (res[0]!=0) res[0][6]=a2;
  a2=1.3308156638776350e-01;
  a7=cos(a5);
  a2=(a2*a7);
  a7=1.3341162379252916e-01;
  a0=cos(a8);
  a7=(a7*a0);
  a2=(a2-a7);
  a7=1.2119940877325484e-01;
  a0=sin(a5);
  a7=(a7*a0);
  a2=(a2+a7);
  a7=1.2083599892520702e-01;
  a0=sin(a8);
  a7=(a7*a0);
  a2=(a2+a7);
  a7=6.7333004874030467e-01;
  a0=cos(a5);
  a0=(a7*a0);
  a9=7.3934203548757504e-01;
  a12=sin(a5);
  a12=(a9*a12);
  a0=(a0-a12);
  a0=(a1*a0);
  a12=cos(a6);
  a12=(a13*a12);
  a15=sin(a6);
  a15=(a3*a15);
  a12=(a12+a15);
  a0=(a0*a12);
  a2=(a2+a0);
  a0=cos(a5);
  a9=(a9*a0);
  a5=sin(a5);
  a7=(a7*a5);
  a9=(a9+a7);
  a1=(a1*a9);
  a9=cos(a6);
  a3=(a3*a9);
  a6=sin(a6);
  a13=(a13*a6);
  a3=(a3-a13);
  a1=(a1*a3);
  a2=(a2+a1);
  a1=6.7131110514003900e-01;
  a3=cos(a8);
  a3=(a1*a3);
  a13=7.4117568773627318e-01;
  a6=sin(a8);
  a6=(a13*a6);
  a3=(a3+a6);
  a3=(a4*a3);
  a6=cos(a10);
  a6=(a14*a6);
  a9=sin(a10);
  a9=(a11*a9);
  a6=(a6-a9);
  a3=(a3*a6);
  a2=(a2-a3);
  a3=cos(a8);
  a13=(a13*a3);
  a8=sin(a8);
  a1=(a1*a8);
  a13=(a13-a1);
  a4=(a4*a13);
  a13=cos(a10);
  a11=(a11*a13);
  a10=sin(a10);
  a14=(a14*a10);
  a11=(a11+a14);
  a4=(a4*a11);
  a2=(a2-a4);
  a2=casadi_sq(a2);
  if (res[0]!=0) res[0][7]=a2;
  return 0;
}

CASADI_SYMBOL_EXPORT int ntnu_leg_cost_y_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ntnu_leg_cost_y_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ntnu_leg_cost_y_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ntnu_leg_cost_y_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ntnu_leg_cost_y_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ntnu_leg_cost_y_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ntnu_leg_cost_y_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void ntnu_leg_cost_y_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ntnu_leg_cost_y_fun_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int ntnu_leg_cost_y_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real ntnu_leg_cost_y_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ntnu_leg_cost_y_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ntnu_leg_cost_y_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ntnu_leg_cost_y_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ntnu_leg_cost_y_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ntnu_leg_cost_y_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
