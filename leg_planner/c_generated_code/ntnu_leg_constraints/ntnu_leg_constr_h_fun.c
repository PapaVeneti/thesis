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
  #define CASADI_PREFIX(ID) ntnu_leg_constr_h_fun_ ## ID
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

static const casadi_int casadi_s0[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[6] = {2, 1, 0, 2, 0, 1};

/* ntnu_leg_constr_h_fun:(i0[10],i1[3],i2[],i3[])->(o0[2]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=1.0000000000000000e-02;
  a1=1.3308156638417232e-01;
  a2=arg[0]? arg[0][1] : 0;
  a3=sin(a2);
  a1=(a1*a3);
  a3=1.2083599892846777e-01;
  a4=arg[0]? arg[0][3] : 0;
  a5=cos(a4);
  a3=(a3*a5);
  a1=(a1-a3);
  a3=1.2119940877652538e-01;
  a5=cos(a2);
  a3=(a3*a5);
  a1=(a1-a3);
  a3=1.3341162378892907e-01;
  a5=sin(a4);
  a3=(a3*a5);
  a1=(a1-a3);
  a3=2.9976999999999998e-01;
  a5=6.7333004875847435e-01;
  a6=cos(a2);
  a6=(a5*a6);
  a7=7.3934203546762400e-01;
  a8=sin(a2);
  a8=(a7*a8);
  a6=(a6-a8);
  a6=(a3*a6);
  a8=2.4187360515245046e-01;
  a9=arg[0]? arg[0][2] : 0;
  a10=cos(a9);
  a10=(a8*a10);
  a11=9.7030776516039308e-01;
  a12=sin(a9);
  a12=(a11*a12);
  a10=(a10-a12);
  a6=(a6*a10);
  a1=(a1-a6);
  a6=cos(a2);
  a7=(a7*a6);
  a6=sin(a2);
  a5=(a5*a6);
  a7=(a7+a5);
  a7=(a3*a7);
  a5=cos(a9);
  a5=(a11*a5);
  a6=sin(a9);
  a6=(a8*a6);
  a5=(a5+a6);
  a7=(a7*a5);
  a1=(a1+a7);
  a7=2.9929000000000000e-01;
  a5=6.7131110515815429e-01;
  a6=cos(a4);
  a6=(a5*a6);
  a10=7.4117568771627262e-01;
  a12=sin(a4);
  a12=(a10*a12);
  a6=(a6+a12);
  a6=(a7*a6);
  a12=2.4497734630999077e-01;
  a13=arg[0]? arg[0][4] : 0;
  a14=cos(a13);
  a14=(a12*a14);
  a15=9.6952880297333865e-01;
  a16=sin(a13);
  a16=(a15*a16);
  a14=(a14+a16);
  a6=(a6*a14);
  a1=(a1-a6);
  a6=cos(a4);
  a10=(a10*a6);
  a6=sin(a4);
  a5=(a5*a6);
  a10=(a10-a5);
  a10=(a7*a10);
  a5=cos(a13);
  a5=(a15*a5);
  a6=sin(a13);
  a6=(a12*a6);
  a5=(a5-a6);
  a10=(a10*a5);
  a1=(a1+a10);
  a10=8.9999999999999997e-02;
  a1=(a1-a10);
  a1=(a0*a1);
  if (res[0]!=0) res[0][0]=a1;
  a1=1.3308156638776350e-01;
  a10=cos(a2);
  a1=(a1*a10);
  a10=1.3341162379252916e-01;
  a5=cos(a4);
  a10=(a10*a5);
  a1=(a1-a10);
  a10=1.2119940877325484e-01;
  a5=sin(a2);
  a10=(a10*a5);
  a1=(a1+a10);
  a10=1.2083599892520702e-01;
  a5=sin(a4);
  a10=(a10*a5);
  a1=(a1+a10);
  a10=6.7333004874030467e-01;
  a5=cos(a2);
  a5=(a10*a5);
  a6=7.3934203548757504e-01;
  a14=sin(a2);
  a14=(a6*a14);
  a5=(a5-a14);
  a5=(a3*a5);
  a14=cos(a9);
  a14=(a11*a14);
  a16=sin(a9);
  a16=(a8*a16);
  a14=(a14+a16);
  a5=(a5*a14);
  a1=(a1+a5);
  a5=cos(a2);
  a6=(a6*a5);
  a2=sin(a2);
  a10=(a10*a2);
  a6=(a6+a10);
  a3=(a3*a6);
  a6=cos(a9);
  a8=(a8*a6);
  a9=sin(a9);
  a11=(a11*a9);
  a8=(a8-a11);
  a3=(a3*a8);
  a1=(a1+a3);
  a3=6.7131110514003900e-01;
  a8=cos(a4);
  a8=(a3*a8);
  a11=7.4117568773627318e-01;
  a9=sin(a4);
  a9=(a11*a9);
  a8=(a8+a9);
  a8=(a7*a8);
  a9=cos(a13);
  a9=(a15*a9);
  a6=sin(a13);
  a6=(a12*a6);
  a9=(a9-a6);
  a8=(a8*a9);
  a1=(a1-a8);
  a8=cos(a4);
  a11=(a11*a8);
  a4=sin(a4);
  a3=(a3*a4);
  a11=(a11-a3);
  a7=(a7*a11);
  a11=cos(a13);
  a12=(a12*a11);
  a13=sin(a13);
  a15=(a15*a13);
  a12=(a12+a15);
  a7=(a7*a12);
  a1=(a1-a7);
  a0=(a0*a1);
  if (res[0]!=0) res[0][1]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int ntnu_leg_constr_h_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ntnu_leg_constr_h_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ntnu_leg_constr_h_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ntnu_leg_constr_h_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ntnu_leg_constr_h_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ntnu_leg_constr_h_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ntnu_leg_constr_h_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void ntnu_leg_constr_h_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ntnu_leg_constr_h_fun_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int ntnu_leg_constr_h_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real ntnu_leg_constr_h_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ntnu_leg_constr_h_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ntnu_leg_constr_h_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ntnu_leg_constr_h_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ntnu_leg_constr_h_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ntnu_leg_constr_h_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
