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
  #define CASADI_PREFIX(ID) ntnu_leg_constr_h_fun_jac_uxt_zt_ ## ID
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
static const casadi_int casadi_s3[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s4[13] = {13, 2, 0, 4, 8, 4, 5, 6, 7, 4, 5, 6, 7};
static const casadi_int casadi_s5[5] = {0, 2, 0, 0, 0};

/* ntnu_leg_constr_h_fun_jac_uxt_zt:(i0[10],i1[3],i2[],i3[])->(o0[2],o1[13x2,8nz],o2[0x2]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a5, a6, a7, a8, a9;
  a0=1.0000000000000000e-02;
  a1=1.3308156638417232e-01;
  a2=arg[0]? arg[0][1] : 0;
  a3=sin(a2);
  a3=(a1*a3);
  a4=1.2083599892846777e-01;
  a5=arg[0]? arg[0][3] : 0;
  a6=cos(a5);
  a6=(a4*a6);
  a3=(a3-a6);
  a6=1.2119940877652538e-01;
  a7=cos(a2);
  a7=(a6*a7);
  a3=(a3-a7);
  a7=1.3341162378892907e-01;
  a8=sin(a5);
  a8=(a7*a8);
  a3=(a3-a8);
  a8=2.9976999999999998e-01;
  a9=6.7333004875847435e-01;
  a10=cos(a2);
  a10=(a9*a10);
  a11=7.3934203546762400e-01;
  a12=sin(a2);
  a12=(a11*a12);
  a10=(a10-a12);
  a10=(a8*a10);
  a12=2.4187360515245046e-01;
  a13=arg[0]? arg[0][2] : 0;
  a14=cos(a13);
  a14=(a12*a14);
  a15=9.7030776516039308e-01;
  a16=sin(a13);
  a16=(a15*a16);
  a14=(a14-a16);
  a16=(a10*a14);
  a3=(a3-a16);
  a16=cos(a2);
  a16=(a11*a16);
  a17=sin(a2);
  a17=(a9*a17);
  a16=(a16+a17);
  a16=(a8*a16);
  a17=cos(a13);
  a17=(a15*a17);
  a18=sin(a13);
  a18=(a12*a18);
  a17=(a17+a18);
  a18=(a16*a17);
  a3=(a3+a18);
  a18=2.9929000000000000e-01;
  a19=6.7131110515815429e-01;
  a20=cos(a5);
  a20=(a19*a20);
  a21=7.4117568771627262e-01;
  a22=sin(a5);
  a22=(a21*a22);
  a20=(a20+a22);
  a20=(a18*a20);
  a22=2.4497734630999077e-01;
  a23=arg[0]? arg[0][4] : 0;
  a24=cos(a23);
  a24=(a22*a24);
  a25=9.6952880297333865e-01;
  a26=sin(a23);
  a26=(a25*a26);
  a24=(a24+a26);
  a26=(a20*a24);
  a3=(a3-a26);
  a26=cos(a5);
  a26=(a21*a26);
  a27=sin(a5);
  a27=(a19*a27);
  a26=(a26-a27);
  a26=(a18*a26);
  a27=cos(a23);
  a27=(a25*a27);
  a28=sin(a23);
  a28=(a22*a28);
  a27=(a27-a28);
  a28=(a26*a27);
  a3=(a3+a28);
  a28=8.9999999999999997e-02;
  a3=(a3-a28);
  a3=(a0*a3);
  if (res[0]!=0) res[0][0]=a3;
  a3=1.3308156638776350e-01;
  a28=cos(a2);
  a28=(a3*a28);
  a29=1.3341162379252916e-01;
  a30=cos(a5);
  a30=(a29*a30);
  a28=(a28-a30);
  a30=1.2119940877325484e-01;
  a31=sin(a2);
  a31=(a30*a31);
  a28=(a28+a31);
  a31=1.2083599892520702e-01;
  a32=sin(a5);
  a32=(a31*a32);
  a28=(a28+a32);
  a32=6.7333004874030467e-01;
  a33=cos(a2);
  a33=(a32*a33);
  a34=7.3934203548757504e-01;
  a35=sin(a2);
  a35=(a34*a35);
  a33=(a33-a35);
  a33=(a8*a33);
  a35=cos(a13);
  a35=(a15*a35);
  a36=sin(a13);
  a36=(a12*a36);
  a35=(a35+a36);
  a36=(a33*a35);
  a28=(a28+a36);
  a36=cos(a2);
  a36=(a34*a36);
  a37=sin(a2);
  a37=(a32*a37);
  a36=(a36+a37);
  a36=(a8*a36);
  a37=cos(a13);
  a37=(a12*a37);
  a38=sin(a13);
  a38=(a15*a38);
  a37=(a37-a38);
  a38=(a36*a37);
  a28=(a28+a38);
  a38=6.7131110514003900e-01;
  a39=cos(a5);
  a39=(a38*a39);
  a40=7.4117568773627318e-01;
  a41=sin(a5);
  a41=(a40*a41);
  a39=(a39+a41);
  a39=(a18*a39);
  a41=cos(a23);
  a41=(a25*a41);
  a42=sin(a23);
  a42=(a22*a42);
  a41=(a41-a42);
  a42=(a39*a41);
  a28=(a28-a42);
  a42=cos(a5);
  a42=(a40*a42);
  a43=sin(a5);
  a43=(a38*a43);
  a42=(a42-a43);
  a42=(a18*a42);
  a43=cos(a23);
  a43=(a22*a43);
  a44=sin(a23);
  a44=(a25*a44);
  a43=(a43+a44);
  a44=(a42*a43);
  a28=(a28-a44);
  a28=(a0*a28);
  if (res[0]!=0) res[0][1]=a28;
  a28=cos(a2);
  a1=(a1*a28);
  a28=sin(a2);
  a6=(a6*a28);
  a1=(a1+a6);
  a6=sin(a2);
  a6=(a9*a6);
  a28=cos(a2);
  a28=(a11*a28);
  a6=(a6+a28);
  a6=(a8*a6);
  a14=(a14*a6);
  a1=(a1+a14);
  a14=cos(a2);
  a9=(a9*a14);
  a14=sin(a2);
  a11=(a11*a14);
  a9=(a9-a11);
  a9=(a8*a9);
  a17=(a17*a9);
  a1=(a1+a17);
  a1=(a0*a1);
  if (res[1]!=0) res[1][0]=a1;
  a1=sin(a13);
  a1=(a12*a1);
  a17=cos(a13);
  a17=(a15*a17);
  a1=(a1+a17);
  a10=(a10*a1);
  a1=cos(a13);
  a1=(a12*a1);
  a17=sin(a13);
  a17=(a15*a17);
  a1=(a1-a17);
  a16=(a16*a1);
  a10=(a10+a16);
  a10=(a0*a10);
  if (res[1]!=0) res[1][1]=a10;
  a10=sin(a5);
  a4=(a4*a10);
  a10=cos(a5);
  a7=(a7*a10);
  a4=(a4-a7);
  a7=cos(a5);
  a7=(a21*a7);
  a10=sin(a5);
  a10=(a19*a10);
  a7=(a7-a10);
  a7=(a18*a7);
  a24=(a24*a7);
  a4=(a4-a24);
  a24=sin(a5);
  a21=(a21*a24);
  a24=cos(a5);
  a19=(a19*a24);
  a21=(a21+a19);
  a21=(a18*a21);
  a27=(a27*a21);
  a4=(a4-a27);
  a4=(a0*a4);
  if (res[1]!=0) res[1][2]=a4;
  a4=cos(a23);
  a4=(a25*a4);
  a27=sin(a23);
  a27=(a22*a27);
  a4=(a4-a27);
  a20=(a20*a4);
  a4=sin(a23);
  a4=(a25*a4);
  a27=cos(a23);
  a27=(a22*a27);
  a4=(a4+a27);
  a26=(a26*a4);
  a20=(a20+a26);
  a20=(a0*a20);
  a20=(-a20);
  if (res[1]!=0) res[1][3]=a20;
  a20=cos(a2);
  a30=(a30*a20);
  a20=sin(a2);
  a3=(a3*a20);
  a30=(a30-a3);
  a3=sin(a2);
  a3=(a32*a3);
  a20=cos(a2);
  a20=(a34*a20);
  a3=(a3+a20);
  a3=(a8*a3);
  a35=(a35*a3);
  a30=(a30-a35);
  a35=cos(a2);
  a32=(a32*a35);
  a2=sin(a2);
  a34=(a34*a2);
  a32=(a32-a34);
  a8=(a8*a32);
  a37=(a37*a8);
  a30=(a30+a37);
  a30=(a0*a30);
  if (res[1]!=0) res[1][4]=a30;
  a30=cos(a13);
  a30=(a12*a30);
  a37=sin(a13);
  a37=(a15*a37);
  a30=(a30-a37);
  a33=(a33*a30);
  a30=sin(a13);
  a12=(a12*a30);
  a13=cos(a13);
  a15=(a15*a13);
  a12=(a12+a15);
  a36=(a36*a12);
  a33=(a33-a36);
  a33=(a0*a33);
  if (res[1]!=0) res[1][5]=a33;
  a33=sin(a5);
  a29=(a29*a33);
  a33=cos(a5);
  a31=(a31*a33);
  a29=(a29+a31);
  a31=cos(a5);
  a31=(a40*a31);
  a33=sin(a5);
  a33=(a38*a33);
  a31=(a31-a33);
  a31=(a18*a31);
  a41=(a41*a31);
  a29=(a29-a41);
  a41=sin(a5);
  a40=(a40*a41);
  a5=cos(a5);
  a38=(a38*a5);
  a40=(a40+a38);
  a18=(a18*a40);
  a43=(a43*a18);
  a29=(a29+a43);
  a29=(a0*a29);
  if (res[1]!=0) res[1][6]=a29;
  a29=sin(a23);
  a29=(a25*a29);
  a43=cos(a23);
  a43=(a22*a43);
  a29=(a29+a43);
  a39=(a39*a29);
  a29=cos(a23);
  a25=(a25*a29);
  a23=sin(a23);
  a22=(a22*a23);
  a25=(a25-a22);
  a42=(a42*a25);
  a39=(a39-a42);
  a0=(a0*a39);
  if (res[1]!=0) res[1][7]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int ntnu_leg_constr_h_fun_jac_uxt_zt(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ntnu_leg_constr_h_fun_jac_uxt_zt_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ntnu_leg_constr_h_fun_jac_uxt_zt_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ntnu_leg_constr_h_fun_jac_uxt_zt_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ntnu_leg_constr_h_fun_jac_uxt_zt_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ntnu_leg_constr_h_fun_jac_uxt_zt_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ntnu_leg_constr_h_fun_jac_uxt_zt_incref(void) {
}

CASADI_SYMBOL_EXPORT void ntnu_leg_constr_h_fun_jac_uxt_zt_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ntnu_leg_constr_h_fun_jac_uxt_zt_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int ntnu_leg_constr_h_fun_jac_uxt_zt_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real ntnu_leg_constr_h_fun_jac_uxt_zt_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ntnu_leg_constr_h_fun_jac_uxt_zt_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ntnu_leg_constr_h_fun_jac_uxt_zt_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ntnu_leg_constr_h_fun_jac_uxt_zt_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ntnu_leg_constr_h_fun_jac_uxt_zt_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s4;
    case 2: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ntnu_leg_constr_h_fun_jac_uxt_zt_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif