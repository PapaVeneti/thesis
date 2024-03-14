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
  #define CASADI_PREFIX(ID) ntnu_leg_cost_y_0_fun_jac_ut_xt_ ## ID
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
static const casadi_int casadi_s3[11] = {7, 1, 0, 7, 0, 1, 2, 3, 4, 5, 6};
static const casadi_int casadi_s4[27] = {13, 7, 0, 1, 4, 7, 8, 9, 13, 17, 0, 1, 2, 3, 1, 2, 3, 1, 2, 4, 5, 6, 7, 4, 5, 6, 7};
static const casadi_int casadi_s5[3] = {7, 0, 0};

/* ntnu_leg_cost_y_0_fun_jac_ut_xt:(i0[10],i1[3],i2[],i3[])->(o0[7],o1[13x7,17nz],o2[7x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a5, a6, a7, a8, a9;
  a0=arg[1]? arg[1][0] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[1]? arg[1][1] : 0;
  a1=arg[1]? arg[1][2] : 0;
  a2=(a0+a1);
  a3=arg[0]? arg[0][0] : 0;
  a4=sin(a3);
  a5=(a2*a4);
  if (res[0]!=0) res[0][1]=a5;
  a5=cos(a3);
  a6=(a2*a5);
  a6=(-a6);
  if (res[0]!=0) res[0][2]=a6;
  if (res[0]!=0) res[0][3]=a0;
  if (res[0]!=0) res[0][4]=a1;
  a1=1.3308156638417232e-01;
  a0=arg[0]? arg[0][1] : 0;
  a6=sin(a0);
  a6=(a1*a6);
  a7=1.2083599892846777e-01;
  a8=arg[0]? arg[0][3] : 0;
  a9=cos(a8);
  a9=(a7*a9);
  a6=(a6-a9);
  a9=1.2119940877652538e-01;
  a10=cos(a0);
  a10=(a9*a10);
  a6=(a6-a10);
  a10=1.3341162378892907e-01;
  a11=sin(a8);
  a11=(a10*a11);
  a6=(a6-a11);
  a11=2.9976999999999998e-01;
  a12=6.7333004875847435e-01;
  a13=cos(a0);
  a13=(a12*a13);
  a14=7.3934203546762400e-01;
  a15=sin(a0);
  a15=(a14*a15);
  a13=(a13-a15);
  a13=(a11*a13);
  a15=2.4187360515245046e-01;
  a16=arg[0]? arg[0][2] : 0;
  a17=cos(a16);
  a17=(a15*a17);
  a18=9.7030776516039308e-01;
  a19=sin(a16);
  a19=(a18*a19);
  a17=(a17-a19);
  a19=(a13*a17);
  a6=(a6-a19);
  a19=cos(a0);
  a19=(a14*a19);
  a20=sin(a0);
  a20=(a12*a20);
  a19=(a19+a20);
  a19=(a11*a19);
  a20=cos(a16);
  a20=(a18*a20);
  a21=sin(a16);
  a21=(a15*a21);
  a20=(a20+a21);
  a21=(a19*a20);
  a6=(a6+a21);
  a21=2.9929000000000000e-01;
  a22=6.7131110515815429e-01;
  a23=cos(a8);
  a23=(a22*a23);
  a24=7.4117568771627262e-01;
  a25=sin(a8);
  a25=(a24*a25);
  a23=(a23+a25);
  a23=(a21*a23);
  a25=2.4497734630999077e-01;
  a26=arg[0]? arg[0][4] : 0;
  a27=cos(a26);
  a27=(a25*a27);
  a28=9.6952880297333865e-01;
  a29=sin(a26);
  a29=(a28*a29);
  a27=(a27+a29);
  a29=(a23*a27);
  a6=(a6-a29);
  a29=cos(a8);
  a29=(a24*a29);
  a30=sin(a8);
  a30=(a22*a30);
  a29=(a29-a30);
  a29=(a21*a29);
  a30=cos(a26);
  a30=(a28*a30);
  a31=sin(a26);
  a31=(a25*a31);
  a30=(a30-a31);
  a31=(a29*a30);
  a6=(a6+a31);
  a31=8.9999999999999997e-02;
  a6=(a6-a31);
  if (res[0]!=0) res[0][5]=a6;
  a6=1.3308156638776350e-01;
  a31=cos(a0);
  a31=(a6*a31);
  a32=1.3341162379252916e-01;
  a33=cos(a8);
  a33=(a32*a33);
  a31=(a31-a33);
  a33=1.2119940877325484e-01;
  a34=sin(a0);
  a34=(a33*a34);
  a31=(a31+a34);
  a34=1.2083599892520702e-01;
  a35=sin(a8);
  a35=(a34*a35);
  a31=(a31+a35);
  a35=6.7333004874030467e-01;
  a36=cos(a0);
  a36=(a35*a36);
  a37=7.3934203548757504e-01;
  a38=sin(a0);
  a38=(a37*a38);
  a36=(a36-a38);
  a36=(a11*a36);
  a38=cos(a16);
  a38=(a18*a38);
  a39=sin(a16);
  a39=(a15*a39);
  a38=(a38+a39);
  a39=(a36*a38);
  a31=(a31+a39);
  a39=cos(a0);
  a39=(a37*a39);
  a40=sin(a0);
  a40=(a35*a40);
  a39=(a39+a40);
  a39=(a11*a39);
  a40=cos(a16);
  a40=(a15*a40);
  a41=sin(a16);
  a41=(a18*a41);
  a40=(a40-a41);
  a41=(a39*a40);
  a31=(a31+a41);
  a41=6.7131110514003900e-01;
  a42=cos(a8);
  a42=(a41*a42);
  a43=7.4117568773627318e-01;
  a44=sin(a8);
  a44=(a43*a44);
  a42=(a42+a44);
  a42=(a21*a42);
  a44=cos(a26);
  a44=(a28*a44);
  a45=sin(a26);
  a45=(a25*a45);
  a44=(a44-a45);
  a45=(a42*a44);
  a31=(a31-a45);
  a45=cos(a8);
  a45=(a43*a45);
  a46=sin(a8);
  a46=(a41*a46);
  a45=(a45-a46);
  a45=(a21*a45);
  a46=cos(a26);
  a46=(a25*a46);
  a47=sin(a26);
  a47=(a28*a47);
  a46=(a46+a47);
  a47=(a45*a46);
  a31=(a31-a47);
  if (res[0]!=0) res[0][6]=a31;
  a31=1.;
  if (res[1]!=0) res[1][0]=a31;
  if (res[1]!=0) res[1][1]=a4;
  if (res[1]!=0) res[1][2]=a4;
  a4=cos(a3);
  a4=(a2*a4);
  if (res[1]!=0) res[1][3]=a4;
  a4=(-a5);
  if (res[1]!=0) res[1][4]=a4;
  a5=(-a5);
  if (res[1]!=0) res[1][5]=a5;
  a3=sin(a3);
  a2=(a2*a3);
  if (res[1]!=0) res[1][6]=a2;
  if (res[1]!=0) res[1][7]=a31;
  if (res[1]!=0) res[1][8]=a31;
  a31=cos(a0);
  a1=(a1*a31);
  a31=sin(a0);
  a9=(a9*a31);
  a1=(a1+a9);
  a9=sin(a0);
  a9=(a12*a9);
  a31=cos(a0);
  a31=(a14*a31);
  a9=(a9+a31);
  a9=(a11*a9);
  a17=(a17*a9);
  a1=(a1+a17);
  a17=cos(a0);
  a12=(a12*a17);
  a17=sin(a0);
  a14=(a14*a17);
  a12=(a12-a14);
  a12=(a11*a12);
  a20=(a20*a12);
  a1=(a1+a20);
  if (res[1]!=0) res[1][9]=a1;
  a1=sin(a16);
  a1=(a15*a1);
  a20=cos(a16);
  a20=(a18*a20);
  a1=(a1+a20);
  a13=(a13*a1);
  a1=cos(a16);
  a1=(a15*a1);
  a20=sin(a16);
  a20=(a18*a20);
  a1=(a1-a20);
  a19=(a19*a1);
  a13=(a13+a19);
  if (res[1]!=0) res[1][10]=a13;
  a13=sin(a8);
  a7=(a7*a13);
  a13=cos(a8);
  a10=(a10*a13);
  a7=(a7-a10);
  a10=cos(a8);
  a10=(a24*a10);
  a13=sin(a8);
  a13=(a22*a13);
  a10=(a10-a13);
  a10=(a21*a10);
  a27=(a27*a10);
  a7=(a7-a27);
  a27=sin(a8);
  a24=(a24*a27);
  a27=cos(a8);
  a22=(a22*a27);
  a24=(a24+a22);
  a24=(a21*a24);
  a30=(a30*a24);
  a7=(a7-a30);
  if (res[1]!=0) res[1][11]=a7;
  a7=cos(a26);
  a7=(a28*a7);
  a30=sin(a26);
  a30=(a25*a30);
  a7=(a7-a30);
  a23=(a23*a7);
  a7=sin(a26);
  a7=(a28*a7);
  a30=cos(a26);
  a30=(a25*a30);
  a7=(a7+a30);
  a29=(a29*a7);
  a23=(a23+a29);
  a23=(-a23);
  if (res[1]!=0) res[1][12]=a23;
  a23=cos(a0);
  a33=(a33*a23);
  a23=sin(a0);
  a6=(a6*a23);
  a33=(a33-a6);
  a6=sin(a0);
  a6=(a35*a6);
  a23=cos(a0);
  a23=(a37*a23);
  a6=(a6+a23);
  a6=(a11*a6);
  a38=(a38*a6);
  a33=(a33-a38);
  a38=cos(a0);
  a35=(a35*a38);
  a0=sin(a0);
  a37=(a37*a0);
  a35=(a35-a37);
  a11=(a11*a35);
  a40=(a40*a11);
  a33=(a33+a40);
  if (res[1]!=0) res[1][13]=a33;
  a33=cos(a16);
  a33=(a15*a33);
  a40=sin(a16);
  a40=(a18*a40);
  a33=(a33-a40);
  a36=(a36*a33);
  a33=sin(a16);
  a15=(a15*a33);
  a16=cos(a16);
  a18=(a18*a16);
  a15=(a15+a18);
  a39=(a39*a15);
  a36=(a36-a39);
  if (res[1]!=0) res[1][14]=a36;
  a36=sin(a8);
  a32=(a32*a36);
  a36=cos(a8);
  a34=(a34*a36);
  a32=(a32+a34);
  a34=cos(a8);
  a34=(a43*a34);
  a36=sin(a8);
  a36=(a41*a36);
  a34=(a34-a36);
  a34=(a21*a34);
  a44=(a44*a34);
  a32=(a32-a44);
  a44=sin(a8);
  a43=(a43*a44);
  a8=cos(a8);
  a41=(a41*a8);
  a43=(a43+a41);
  a21=(a21*a43);
  a46=(a46*a21);
  a32=(a32+a46);
  if (res[1]!=0) res[1][15]=a32;
  a32=sin(a26);
  a32=(a28*a32);
  a46=cos(a26);
  a46=(a25*a46);
  a32=(a32+a46);
  a42=(a42*a32);
  a32=cos(a26);
  a28=(a28*a32);
  a26=sin(a26);
  a25=(a25*a26);
  a28=(a28-a25);
  a45=(a45*a28);
  a42=(a42-a45);
  if (res[1]!=0) res[1][16]=a42;
  return 0;
}

CASADI_SYMBOL_EXPORT int ntnu_leg_cost_y_0_fun_jac_ut_xt(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ntnu_leg_cost_y_0_fun_jac_ut_xt_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ntnu_leg_cost_y_0_fun_jac_ut_xt_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ntnu_leg_cost_y_0_fun_jac_ut_xt_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ntnu_leg_cost_y_0_fun_jac_ut_xt_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ntnu_leg_cost_y_0_fun_jac_ut_xt_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ntnu_leg_cost_y_0_fun_jac_ut_xt_incref(void) {
}

CASADI_SYMBOL_EXPORT void ntnu_leg_cost_y_0_fun_jac_ut_xt_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ntnu_leg_cost_y_0_fun_jac_ut_xt_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int ntnu_leg_cost_y_0_fun_jac_ut_xt_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real ntnu_leg_cost_y_0_fun_jac_ut_xt_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ntnu_leg_cost_y_0_fun_jac_ut_xt_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ntnu_leg_cost_y_0_fun_jac_ut_xt_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ntnu_leg_cost_y_0_fun_jac_ut_xt_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ntnu_leg_cost_y_0_fun_jac_ut_xt_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s4;
    case 2: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ntnu_leg_cost_y_0_fun_jac_ut_xt_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
