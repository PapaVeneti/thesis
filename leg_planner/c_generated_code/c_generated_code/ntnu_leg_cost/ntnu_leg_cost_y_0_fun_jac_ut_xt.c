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
static const casadi_int casadi_s4[33] = {13, 8, 0, 1, 4, 7, 8, 9, 14, 18, 22, 0, 1, 2, 3, 1, 2, 3, 1, 2, 3, 4, 5, 6, 7, 4, 5, 6, 7, 4, 5, 6, 7};
static const casadi_int casadi_s5[3] = {8, 0, 0};

/* ntnu_leg_cost_y_0_fun_jac_ut_xt:(i0[10],i1[3],i2[],i3[])->(o0[8],o1[13x8,22nz],o2[8x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a6, a7, a8, a9;
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
  a1=1.0000000000000000e-03;
  a0=(a1+a3);
  a6=2.;
  a0=(a0+a6);
  a6=log(a0);
  a7=arg[0]? arg[0][1] : 0;
  a8=(a1+a7);
  a9=1.2000000000000000e+00;
  a8=(a8+a9);
  a10=log(a8);
  a6=(a6+a10);
  a10=arg[0]? arg[0][2] : 0;
  a11=(a1+a10);
  a12=1.3230000000000000e+00;
  a11=(a11+a12);
  a13=log(a11);
  a6=(a6+a13);
  a13=arg[0]? arg[0][3] : 0;
  a14=(a1+a13);
  a15=1.6499999999999999e+00;
  a14=(a14+a15);
  a16=log(a14);
  a6=(a6+a16);
  a16=arg[0]? arg[0][4] : 0;
  a17=(a1+a16);
  a18=1.4199999999999999e+00;
  a17=(a17+a18);
  a19=log(a17);
  a6=(a6+a19);
  a19=(a1-a3);
  a20=1.3100000000000001e+00;
  a19=(a19+a20);
  a20=log(a19);
  a6=(a6+a20);
  a20=(a1-a7);
  a20=(a20+a15);
  a15=log(a20);
  a6=(a6+a15);
  a15=(a1-a10);
  a15=(a15+a18);
  a18=log(a15);
  a6=(a6+a18);
  a18=(a1-a13);
  a18=(a18+a9);
  a9=log(a18);
  a6=(a6+a9);
  a1=(a1-a16);
  a1=(a1+a12);
  a12=log(a1);
  a6=(a6+a12);
  a6=(-a6);
  if (res[0]!=0) res[0][5]=a6;
  a6=1.3308156638417232e-01;
  a12=sin(a7);
  a12=(a6*a12);
  a9=1.2083599892846777e-01;
  a21=cos(a13);
  a21=(a9*a21);
  a12=(a12-a21);
  a21=1.2119940877652538e-01;
  a22=cos(a7);
  a22=(a21*a22);
  a12=(a12-a22);
  a22=1.3341162378892907e-01;
  a23=sin(a13);
  a23=(a22*a23);
  a12=(a12-a23);
  a23=2.9976999999999998e-01;
  a24=6.7333004875847435e-01;
  a25=cos(a7);
  a25=(a24*a25);
  a26=7.3934203546762400e-01;
  a27=sin(a7);
  a27=(a26*a27);
  a25=(a25-a27);
  a25=(a23*a25);
  a27=2.4187360515245046e-01;
  a28=cos(a10);
  a28=(a27*a28);
  a29=9.7030776516039308e-01;
  a30=sin(a10);
  a30=(a29*a30);
  a28=(a28-a30);
  a30=(a25*a28);
  a12=(a12-a30);
  a30=cos(a7);
  a30=(a26*a30);
  a31=sin(a7);
  a31=(a24*a31);
  a30=(a30+a31);
  a30=(a23*a30);
  a31=cos(a10);
  a31=(a29*a31);
  a32=sin(a10);
  a32=(a27*a32);
  a31=(a31+a32);
  a32=(a30*a31);
  a12=(a12+a32);
  a32=2.9929000000000000e-01;
  a33=6.7131110515815429e-01;
  a34=cos(a13);
  a34=(a33*a34);
  a35=7.4117568771627262e-01;
  a36=sin(a13);
  a36=(a35*a36);
  a34=(a34+a36);
  a34=(a32*a34);
  a36=2.4497734630999077e-01;
  a37=cos(a16);
  a37=(a36*a37);
  a38=9.6952880297333865e-01;
  a39=sin(a16);
  a39=(a38*a39);
  a37=(a37+a39);
  a39=(a34*a37);
  a12=(a12-a39);
  a39=cos(a13);
  a39=(a35*a39);
  a40=sin(a13);
  a40=(a33*a40);
  a39=(a39-a40);
  a39=(a32*a39);
  a40=cos(a16);
  a40=(a38*a40);
  a41=sin(a16);
  a41=(a36*a41);
  a40=(a40-a41);
  a41=(a39*a40);
  a12=(a12+a41);
  a41=8.9999999999999997e-02;
  a12=(a12-a41);
  a41=casadi_sq(a12);
  if (res[0]!=0) res[0][6]=a41;
  a41=1.3308156638776350e-01;
  a42=cos(a7);
  a42=(a41*a42);
  a43=1.3341162379252916e-01;
  a44=cos(a13);
  a44=(a43*a44);
  a42=(a42-a44);
  a44=1.2119940877325484e-01;
  a45=sin(a7);
  a45=(a44*a45);
  a42=(a42+a45);
  a45=1.2083599892520702e-01;
  a46=sin(a13);
  a46=(a45*a46);
  a42=(a42+a46);
  a46=6.7333004874030467e-01;
  a47=cos(a7);
  a47=(a46*a47);
  a48=7.3934203548757504e-01;
  a49=sin(a7);
  a49=(a48*a49);
  a47=(a47-a49);
  a47=(a23*a47);
  a49=cos(a10);
  a49=(a29*a49);
  a50=sin(a10);
  a50=(a27*a50);
  a49=(a49+a50);
  a50=(a47*a49);
  a42=(a42+a50);
  a50=cos(a7);
  a50=(a48*a50);
  a51=sin(a7);
  a51=(a46*a51);
  a50=(a50+a51);
  a50=(a23*a50);
  a51=cos(a10);
  a51=(a27*a51);
  a52=sin(a10);
  a52=(a29*a52);
  a51=(a51-a52);
  a52=(a50*a51);
  a42=(a42+a52);
  a52=6.7131110514003900e-01;
  a53=cos(a13);
  a53=(a52*a53);
  a54=7.4117568773627318e-01;
  a55=sin(a13);
  a55=(a54*a55);
  a53=(a53+a55);
  a53=(a32*a53);
  a55=cos(a16);
  a55=(a38*a55);
  a56=sin(a16);
  a56=(a36*a56);
  a55=(a55-a56);
  a56=(a53*a55);
  a42=(a42-a56);
  a56=cos(a13);
  a56=(a54*a56);
  a57=sin(a13);
  a57=(a52*a57);
  a56=(a56-a57);
  a56=(a32*a56);
  a57=cos(a16);
  a57=(a36*a57);
  a58=sin(a16);
  a58=(a38*a58);
  a57=(a57+a58);
  a58=(a56*a57);
  a42=(a42-a58);
  a58=casadi_sq(a42);
  if (res[0]!=0) res[0][7]=a58;
  a58=1.;
  if (res[1]!=0) res[1][0]=a58;
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
  if (res[1]!=0) res[1][7]=a58;
  if (res[1]!=0) res[1][8]=a58;
  a0=(1./a0);
  a19=(1./a19);
  a0=(a0-a19);
  a0=(-a0);
  if (res[1]!=0) res[1][9]=a0;
  a8=(1./a8);
  a20=(1./a20);
  a8=(a8-a20);
  a8=(-a8);
  if (res[1]!=0) res[1][10]=a8;
  a11=(1./a11);
  a15=(1./a15);
  a11=(a11-a15);
  a11=(-a11);
  if (res[1]!=0) res[1][11]=a11;
  a14=(1./a14);
  a18=(1./a18);
  a14=(a14-a18);
  a14=(-a14);
  if (res[1]!=0) res[1][12]=a14;
  a17=(1./a17);
  a1=(1./a1);
  a17=(a17-a1);
  a17=(-a17);
  if (res[1]!=0) res[1][13]=a17;
  a12=(a12+a12);
  a17=cos(a7);
  a6=(a6*a17);
  a17=sin(a7);
  a21=(a21*a17);
  a6=(a6+a21);
  a21=sin(a7);
  a21=(a24*a21);
  a17=cos(a7);
  a17=(a26*a17);
  a21=(a21+a17);
  a21=(a23*a21);
  a28=(a28*a21);
  a6=(a6+a28);
  a28=cos(a7);
  a24=(a24*a28);
  a28=sin(a7);
  a26=(a26*a28);
  a24=(a24-a26);
  a24=(a23*a24);
  a31=(a31*a24);
  a6=(a6+a31);
  a6=(a12*a6);
  if (res[1]!=0) res[1][14]=a6;
  a6=sin(a10);
  a6=(a27*a6);
  a31=cos(a10);
  a31=(a29*a31);
  a6=(a6+a31);
  a25=(a25*a6);
  a6=cos(a10);
  a6=(a27*a6);
  a31=sin(a10);
  a31=(a29*a31);
  a6=(a6-a31);
  a30=(a30*a6);
  a25=(a25+a30);
  a25=(a12*a25);
  if (res[1]!=0) res[1][15]=a25;
  a25=sin(a13);
  a9=(a9*a25);
  a25=cos(a13);
  a22=(a22*a25);
  a9=(a9-a22);
  a22=cos(a13);
  a22=(a35*a22);
  a25=sin(a13);
  a25=(a33*a25);
  a22=(a22-a25);
  a22=(a32*a22);
  a37=(a37*a22);
  a9=(a9-a37);
  a37=sin(a13);
  a35=(a35*a37);
  a37=cos(a13);
  a33=(a33*a37);
  a35=(a35+a33);
  a35=(a32*a35);
  a40=(a40*a35);
  a9=(a9-a40);
  a9=(a12*a9);
  if (res[1]!=0) res[1][16]=a9;
  a9=cos(a16);
  a9=(a38*a9);
  a40=sin(a16);
  a40=(a36*a40);
  a9=(a9-a40);
  a34=(a34*a9);
  a9=sin(a16);
  a9=(a38*a9);
  a40=cos(a16);
  a40=(a36*a40);
  a9=(a9+a40);
  a39=(a39*a9);
  a34=(a34+a39);
  a12=(a12*a34);
  a12=(-a12);
  if (res[1]!=0) res[1][17]=a12;
  a42=(a42+a42);
  a12=cos(a7);
  a44=(a44*a12);
  a12=sin(a7);
  a41=(a41*a12);
  a44=(a44-a41);
  a41=sin(a7);
  a41=(a46*a41);
  a12=cos(a7);
  a12=(a48*a12);
  a41=(a41+a12);
  a41=(a23*a41);
  a49=(a49*a41);
  a44=(a44-a49);
  a49=cos(a7);
  a46=(a46*a49);
  a7=sin(a7);
  a48=(a48*a7);
  a46=(a46-a48);
  a23=(a23*a46);
  a51=(a51*a23);
  a44=(a44+a51);
  a44=(a42*a44);
  if (res[1]!=0) res[1][18]=a44;
  a44=cos(a10);
  a44=(a27*a44);
  a51=sin(a10);
  a51=(a29*a51);
  a44=(a44-a51);
  a47=(a47*a44);
  a44=sin(a10);
  a27=(a27*a44);
  a10=cos(a10);
  a29=(a29*a10);
  a27=(a27+a29);
  a50=(a50*a27);
  a47=(a47-a50);
  a47=(a42*a47);
  if (res[1]!=0) res[1][19]=a47;
  a47=sin(a13);
  a43=(a43*a47);
  a47=cos(a13);
  a45=(a45*a47);
  a43=(a43+a45);
  a45=cos(a13);
  a45=(a54*a45);
  a47=sin(a13);
  a47=(a52*a47);
  a45=(a45-a47);
  a45=(a32*a45);
  a55=(a55*a45);
  a43=(a43-a55);
  a55=sin(a13);
  a54=(a54*a55);
  a13=cos(a13);
  a52=(a52*a13);
  a54=(a54+a52);
  a32=(a32*a54);
  a57=(a57*a32);
  a43=(a43+a57);
  a43=(a42*a43);
  if (res[1]!=0) res[1][20]=a43;
  a43=sin(a16);
  a43=(a38*a43);
  a57=cos(a16);
  a57=(a36*a57);
  a43=(a43+a57);
  a53=(a53*a43);
  a43=cos(a16);
  a38=(a38*a43);
  a16=sin(a16);
  a36=(a36*a16);
  a38=(a38-a36);
  a56=(a56*a38);
  a53=(a53-a56);
  a42=(a42*a53);
  if (res[1]!=0) res[1][21]=a42;
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
