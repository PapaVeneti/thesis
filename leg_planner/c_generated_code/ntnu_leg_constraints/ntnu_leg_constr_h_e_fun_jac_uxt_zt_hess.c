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
  #define CASADI_PREFIX(ID) ntnu_leg_constr_h_e_fun_jac_uxt_zt_hess_ ## ID
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
static const casadi_int casadi_s1[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[13] = {10, 2, 0, 4, 8, 1, 2, 3, 4, 1, 2, 3, 4};
static const casadi_int casadi_s4[21] = {10, 10, 0, 0, 2, 4, 6, 8, 8, 8, 8, 8, 8, 1, 2, 1, 2, 3, 4, 3, 4};

/* ntnu_leg_constr_h_e_fun_jac_uxt_zt_hess:(i0[10],i1[2],i2[])->(o0[2],o1[10x2,8nz],o2[10x10,8nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a5, a6, a7, a8, a9;
  a0=1.3308156638417232e-01;
  a1=arg[0]? arg[0][1] : 0;
  a2=sin(a1);
  a2=(a0*a2);
  a3=1.2083599892846777e-01;
  a4=arg[0]? arg[0][3] : 0;
  a5=cos(a4);
  a5=(a3*a5);
  a2=(a2-a5);
  a5=1.2119940877652538e-01;
  a6=cos(a1);
  a6=(a5*a6);
  a2=(a2-a6);
  a6=1.3341162378892907e-01;
  a7=sin(a4);
  a7=(a6*a7);
  a2=(a2-a7);
  a7=2.9976999999999998e-01;
  a8=6.7333004875847435e-01;
  a9=cos(a1);
  a9=(a8*a9);
  a10=7.3934203546762400e-01;
  a11=sin(a1);
  a11=(a10*a11);
  a9=(a9-a11);
  a9=(a7*a9);
  a11=2.4187360515245046e-01;
  a12=arg[0]? arg[0][2] : 0;
  a13=cos(a12);
  a13=(a11*a13);
  a14=9.7030776516039308e-01;
  a15=sin(a12);
  a15=(a14*a15);
  a13=(a13-a15);
  a15=(a9*a13);
  a2=(a2-a15);
  a15=cos(a1);
  a15=(a10*a15);
  a16=sin(a1);
  a16=(a8*a16);
  a15=(a15+a16);
  a15=(a7*a15);
  a16=cos(a12);
  a16=(a14*a16);
  a17=sin(a12);
  a17=(a11*a17);
  a16=(a16+a17);
  a17=(a15*a16);
  a2=(a2+a17);
  a17=2.9929000000000000e-01;
  a18=6.7131110515815429e-01;
  a19=cos(a4);
  a19=(a18*a19);
  a20=7.4117568771627262e-01;
  a21=sin(a4);
  a21=(a20*a21);
  a19=(a19+a21);
  a19=(a17*a19);
  a21=2.4497734630999077e-01;
  a22=arg[0]? arg[0][4] : 0;
  a23=cos(a22);
  a23=(a21*a23);
  a24=9.6952880297333865e-01;
  a25=sin(a22);
  a25=(a24*a25);
  a23=(a23+a25);
  a25=(a19*a23);
  a2=(a2-a25);
  a25=cos(a4);
  a25=(a20*a25);
  a26=sin(a4);
  a26=(a18*a26);
  a25=(a25-a26);
  a25=(a17*a25);
  a26=cos(a22);
  a26=(a24*a26);
  a27=sin(a22);
  a27=(a21*a27);
  a26=(a26-a27);
  a27=(a25*a26);
  a2=(a2+a27);
  a27=8.9999999999999997e-02;
  a2=(a2-a27);
  if (res[0]!=0) res[0][0]=a2;
  a2=1.3308156638776350e-01;
  a27=cos(a1);
  a27=(a2*a27);
  a28=1.3341162379252916e-01;
  a29=cos(a4);
  a29=(a28*a29);
  a27=(a27-a29);
  a29=1.2119940877325484e-01;
  a30=sin(a1);
  a30=(a29*a30);
  a27=(a27+a30);
  a30=1.2083599892520702e-01;
  a31=sin(a4);
  a31=(a30*a31);
  a27=(a27+a31);
  a31=6.7333004874030467e-01;
  a32=cos(a1);
  a32=(a31*a32);
  a33=7.3934203548757504e-01;
  a34=sin(a1);
  a34=(a33*a34);
  a32=(a32-a34);
  a32=(a7*a32);
  a34=cos(a12);
  a34=(a14*a34);
  a35=sin(a12);
  a35=(a11*a35);
  a34=(a34+a35);
  a35=(a32*a34);
  a27=(a27+a35);
  a35=cos(a1);
  a35=(a33*a35);
  a36=sin(a1);
  a36=(a31*a36);
  a35=(a35+a36);
  a35=(a7*a35);
  a36=cos(a12);
  a36=(a11*a36);
  a37=sin(a12);
  a37=(a14*a37);
  a36=(a36-a37);
  a37=(a35*a36);
  a27=(a27+a37);
  a37=6.7131110514003900e-01;
  a38=cos(a4);
  a38=(a37*a38);
  a39=7.4117568773627318e-01;
  a40=sin(a4);
  a40=(a39*a40);
  a38=(a38+a40);
  a38=(a17*a38);
  a40=cos(a22);
  a40=(a24*a40);
  a41=sin(a22);
  a41=(a21*a41);
  a40=(a40-a41);
  a41=(a38*a40);
  a27=(a27-a41);
  a41=cos(a4);
  a41=(a39*a41);
  a42=sin(a4);
  a42=(a37*a42);
  a41=(a41-a42);
  a41=(a17*a41);
  a42=cos(a22);
  a42=(a21*a42);
  a43=sin(a22);
  a43=(a24*a43);
  a42=(a42+a43);
  a43=(a41*a42);
  a27=(a27-a43);
  if (res[0]!=0) res[0][1]=a27;
  a27=cos(a1);
  a27=(a0*a27);
  a43=sin(a1);
  a43=(a5*a43);
  a27=(a27+a43);
  a43=sin(a1);
  a43=(a8*a43);
  a44=cos(a1);
  a44=(a10*a44);
  a43=(a43+a44);
  a43=(a7*a43);
  a43=(a13*a43);
  a27=(a27+a43);
  a43=cos(a1);
  a43=(a8*a43);
  a44=sin(a1);
  a44=(a10*a44);
  a43=(a43-a44);
  a43=(a7*a43);
  a43=(a16*a43);
  a27=(a27+a43);
  if (res[1]!=0) res[1][0]=a27;
  a27=sin(a12);
  a27=(a11*a27);
  a43=cos(a12);
  a43=(a14*a43);
  a27=(a27+a43);
  a27=(a9*a27);
  a43=cos(a12);
  a43=(a11*a43);
  a44=sin(a12);
  a44=(a14*a44);
  a43=(a43-a44);
  a43=(a15*a43);
  a27=(a27+a43);
  if (res[1]!=0) res[1][1]=a27;
  a27=sin(a4);
  a27=(a3*a27);
  a43=cos(a4);
  a43=(a6*a43);
  a27=(a27-a43);
  a43=cos(a4);
  a43=(a20*a43);
  a44=sin(a4);
  a44=(a18*a44);
  a43=(a43-a44);
  a43=(a17*a43);
  a43=(a23*a43);
  a27=(a27-a43);
  a43=sin(a4);
  a43=(a20*a43);
  a44=cos(a4);
  a44=(a18*a44);
  a43=(a43+a44);
  a43=(a17*a43);
  a43=(a26*a43);
  a27=(a27-a43);
  if (res[1]!=0) res[1][2]=a27;
  a27=cos(a22);
  a27=(a24*a27);
  a43=sin(a22);
  a43=(a21*a43);
  a27=(a27-a43);
  a27=(a19*a27);
  a43=sin(a22);
  a43=(a24*a43);
  a44=cos(a22);
  a44=(a21*a44);
  a43=(a43+a44);
  a43=(a25*a43);
  a27=(a27+a43);
  a27=(-a27);
  if (res[1]!=0) res[1][3]=a27;
  a27=cos(a1);
  a27=(a29*a27);
  a43=sin(a1);
  a43=(a2*a43);
  a27=(a27-a43);
  a43=sin(a1);
  a43=(a31*a43);
  a44=cos(a1);
  a44=(a33*a44);
  a43=(a43+a44);
  a43=(a7*a43);
  a43=(a34*a43);
  a27=(a27-a43);
  a43=cos(a1);
  a43=(a31*a43);
  a44=sin(a1);
  a44=(a33*a44);
  a43=(a43-a44);
  a43=(a7*a43);
  a43=(a36*a43);
  a27=(a27+a43);
  if (res[1]!=0) res[1][4]=a27;
  a27=cos(a12);
  a27=(a11*a27);
  a43=sin(a12);
  a43=(a14*a43);
  a27=(a27-a43);
  a27=(a32*a27);
  a43=sin(a12);
  a43=(a11*a43);
  a44=cos(a12);
  a44=(a14*a44);
  a43=(a43+a44);
  a43=(a35*a43);
  a27=(a27-a43);
  if (res[1]!=0) res[1][5]=a27;
  a27=sin(a4);
  a27=(a28*a27);
  a43=cos(a4);
  a43=(a30*a43);
  a27=(a27+a43);
  a43=cos(a4);
  a43=(a39*a43);
  a44=sin(a4);
  a44=(a37*a44);
  a43=(a43-a44);
  a43=(a17*a43);
  a43=(a40*a43);
  a27=(a27-a43);
  a43=sin(a4);
  a43=(a39*a43);
  a44=cos(a4);
  a44=(a37*a44);
  a43=(a43+a44);
  a43=(a17*a43);
  a43=(a42*a43);
  a27=(a27+a43);
  if (res[1]!=0) res[1][6]=a27;
  a27=sin(a22);
  a27=(a24*a27);
  a43=cos(a22);
  a43=(a21*a43);
  a27=(a27+a43);
  a27=(a38*a27);
  a43=cos(a22);
  a43=(a24*a43);
  a44=sin(a22);
  a44=(a21*a44);
  a43=(a43-a44);
  a43=(a41*a43);
  a27=(a27-a43);
  if (res[1]!=0) res[1][7]=a27;
  a27=arg[1]? arg[1][1] : 0;
  a34=(a34*a27);
  a34=(a7*a34);
  a43=(a33*a34);
  a44=sin(a1);
  a43=(a43*a44);
  a36=(a36*a27);
  a36=(a7*a36);
  a44=(a31*a36);
  a45=sin(a1);
  a44=(a44*a45);
  a36=(a33*a36);
  a45=cos(a1);
  a36=(a36*a45);
  a44=(a44+a36);
  a43=(a43-a44);
  a34=(a31*a34);
  a44=cos(a1);
  a34=(a34*a44);
  a43=(a43-a34);
  a29=(a29*a27);
  a34=sin(a1);
  a29=(a29*a34);
  a43=(a43-a29);
  a2=(a2*a27);
  a29=cos(a1);
  a2=(a2*a29);
  a43=(a43-a2);
  a2=arg[1]? arg[1][0] : 0;
  a16=(a16*a2);
  a16=(a7*a16);
  a29=(a8*a16);
  a34=sin(a1);
  a29=(a29*a34);
  a43=(a43-a29);
  a16=(a10*a16);
  a29=cos(a1);
  a16=(a16*a29);
  a43=(a43-a16);
  a13=(a13*a2);
  a13=(a7*a13);
  a16=(a10*a13);
  a29=sin(a1);
  a16=(a16*a29);
  a43=(a43-a16);
  a13=(a8*a13);
  a16=cos(a1);
  a13=(a13*a16);
  a43=(a43+a13);
  a5=(a5*a2);
  a13=cos(a1);
  a5=(a5*a13);
  a43=(a43+a5);
  a0=(a0*a2);
  a5=sin(a1);
  a0=(a0*a5);
  a43=(a43-a0);
  if (res[2]!=0) res[2][0]=a43;
  a43=sin(a12);
  a0=sin(a1);
  a0=(a31*a0);
  a5=cos(a1);
  a5=(a33*a5);
  a0=(a0+a5);
  a0=(a7*a0);
  a0=(a27*a0);
  a5=(a14*a0);
  a43=(a43*a5);
  a5=cos(a12);
  a0=(a11*a0);
  a5=(a5*a0);
  a0=cos(a12);
  a13=cos(a1);
  a13=(a31*a13);
  a16=sin(a1);
  a16=(a33*a16);
  a13=(a13-a16);
  a13=(a7*a13);
  a13=(a27*a13);
  a16=(a14*a13);
  a0=(a0*a16);
  a16=sin(a12);
  a13=(a11*a13);
  a16=(a16*a13);
  a0=(a0+a16);
  a5=(a5+a0);
  a43=(a43-a5);
  a5=cos(a12);
  a0=cos(a1);
  a0=(a8*a0);
  a16=sin(a1);
  a16=(a10*a16);
  a0=(a0-a16);
  a0=(a7*a0);
  a0=(a2*a0);
  a16=(a11*a0);
  a5=(a5*a16);
  a43=(a43+a5);
  a5=sin(a12);
  a0=(a14*a0);
  a5=(a5*a0);
  a43=(a43-a5);
  a5=cos(a12);
  a0=sin(a1);
  a0=(a8*a0);
  a16=cos(a1);
  a16=(a10*a16);
  a0=(a0+a16);
  a0=(a7*a0);
  a0=(a2*a0);
  a16=(a14*a0);
  a5=(a5*a16);
  a43=(a43-a5);
  a5=sin(a12);
  a0=(a11*a0);
  a5=(a5*a0);
  a43=(a43-a5);
  if (res[2]!=0) res[2][1]=a43;
  a43=sin(a1);
  a5=sin(a12);
  a5=(a11*a5);
  a0=cos(a12);
  a0=(a14*a0);
  a5=(a5+a0);
  a5=(a27*a5);
  a5=(a7*a5);
  a0=(a33*a5);
  a43=(a43*a0);
  a0=cos(a1);
  a5=(a31*a5);
  a0=(a0*a5);
  a43=(a43-a0);
  a0=cos(a1);
  a5=cos(a12);
  a5=(a11*a5);
  a16=sin(a12);
  a16=(a14*a16);
  a5=(a5-a16);
  a5=(a27*a5);
  a5=(a7*a5);
  a33=(a33*a5);
  a0=(a0*a33);
  a43=(a43-a0);
  a0=sin(a1);
  a31=(a31*a5);
  a0=(a0*a31);
  a43=(a43-a0);
  a0=cos(a1);
  a31=cos(a12);
  a31=(a11*a31);
  a5=sin(a12);
  a5=(a14*a5);
  a31=(a31-a5);
  a31=(a2*a31);
  a31=(a7*a31);
  a5=(a8*a31);
  a0=(a0*a5);
  a43=(a43+a0);
  a0=sin(a1);
  a31=(a10*a31);
  a0=(a0*a31);
  a43=(a43-a0);
  a0=cos(a1);
  a31=sin(a12);
  a31=(a11*a31);
  a5=cos(a12);
  a5=(a14*a5);
  a31=(a31+a5);
  a31=(a2*a31);
  a7=(a7*a31);
  a10=(a10*a7);
  a0=(a0*a10);
  a43=(a43-a0);
  a1=sin(a1);
  a8=(a8*a7);
  a1=(a1*a8);
  a43=(a43-a1);
  if (res[2]!=0) res[2][2]=a43;
  a9=(a9*a2);
  a43=(a11*a9);
  a1=cos(a12);
  a43=(a43*a1);
  a32=(a32*a27);
  a1=(a11*a32);
  a8=sin(a12);
  a1=(a1*a8);
  a35=(a35*a27);
  a8=(a11*a35);
  a7=cos(a12);
  a8=(a8*a7);
  a35=(a14*a35);
  a7=sin(a12);
  a35=(a35*a7);
  a8=(a8-a35);
  a1=(a1+a8);
  a32=(a14*a32);
  a8=cos(a12);
  a32=(a32*a8);
  a1=(a1+a32);
  a15=(a15*a2);
  a11=(a11*a15);
  a32=sin(a12);
  a11=(a11*a32);
  a1=(a1+a11);
  a15=(a14*a15);
  a11=cos(a12);
  a15=(a15*a11);
  a1=(a1+a15);
  a14=(a14*a9);
  a12=sin(a12);
  a14=(a14*a12);
  a1=(a1+a14);
  a43=(a43-a1);
  if (res[2]!=0) res[2][3]=a43;
  a42=(a42*a27);
  a42=(a17*a42);
  a43=(a39*a42);
  a1=cos(a4);
  a43=(a43*a1);
  a42=(a37*a42);
  a1=sin(a4);
  a42=(a42*a1);
  a43=(a43-a42);
  a40=(a40*a27);
  a40=(a17*a40);
  a42=(a39*a40);
  a1=sin(a4);
  a42=(a42*a1);
  a43=(a43+a42);
  a40=(a37*a40);
  a42=cos(a4);
  a40=(a40*a42);
  a43=(a43+a40);
  a30=(a30*a27);
  a40=sin(a4);
  a30=(a30*a40);
  a43=(a43-a30);
  a28=(a28*a27);
  a30=cos(a4);
  a28=(a28*a30);
  a43=(a43+a28);
  a26=(a26*a2);
  a26=(a17*a26);
  a28=(a18*a26);
  a30=sin(a4);
  a28=(a28*a30);
  a43=(a43+a28);
  a26=(a20*a26);
  a28=cos(a4);
  a26=(a26*a28);
  a43=(a43-a26);
  a23=(a23*a2);
  a23=(a17*a23);
  a26=(a20*a23);
  a28=sin(a4);
  a26=(a26*a28);
  a43=(a43+a26);
  a23=(a18*a23);
  a26=cos(a4);
  a23=(a23*a26);
  a43=(a43+a23);
  a6=(a6*a2);
  a23=sin(a4);
  a6=(a6*a23);
  a43=(a43+a6);
  a3=(a3*a2);
  a6=cos(a4);
  a3=(a3*a6);
  a43=(a43+a3);
  if (res[2]!=0) res[2][4]=a43;
  a43=cos(a22);
  a3=sin(a4);
  a3=(a39*a3);
  a6=cos(a4);
  a6=(a37*a6);
  a3=(a3+a6);
  a3=(a17*a3);
  a3=(a27*a3);
  a6=(a24*a3);
  a43=(a43*a6);
  a6=sin(a22);
  a3=(a21*a3);
  a6=(a6*a3);
  a43=(a43-a6);
  a6=cos(a22);
  a3=cos(a4);
  a3=(a39*a3);
  a23=sin(a4);
  a23=(a37*a23);
  a3=(a3-a23);
  a3=(a17*a3);
  a3=(a27*a3);
  a23=(a21*a3);
  a6=(a6*a23);
  a43=(a43+a6);
  a6=sin(a22);
  a3=(a24*a3);
  a6=(a6*a3);
  a43=(a43+a6);
  a6=cos(a22);
  a3=sin(a4);
  a3=(a20*a3);
  a23=cos(a4);
  a23=(a18*a23);
  a3=(a3+a23);
  a3=(a17*a3);
  a3=(a2*a3);
  a23=(a21*a3);
  a6=(a6*a23);
  a43=(a43+a6);
  a6=sin(a22);
  a3=(a24*a3);
  a6=(a6*a3);
  a43=(a43+a6);
  a6=cos(a22);
  a3=cos(a4);
  a3=(a20*a3);
  a23=sin(a4);
  a23=(a18*a23);
  a3=(a3-a23);
  a3=(a17*a3);
  a3=(a2*a3);
  a23=(a24*a3);
  a6=(a6*a23);
  a43=(a43-a6);
  a6=sin(a22);
  a3=(a21*a3);
  a6=(a6*a3);
  a43=(a43+a6);
  if (res[2]!=0) res[2][5]=a43;
  a43=cos(a4);
  a6=cos(a22);
  a6=(a24*a6);
  a3=sin(a22);
  a3=(a21*a3);
  a6=(a6-a3);
  a6=(a27*a6);
  a6=(a17*a6);
  a3=(a37*a6);
  a43=(a43*a3);
  a3=sin(a4);
  a6=(a39*a6);
  a3=(a3*a6);
  a43=(a43+a3);
  a3=cos(a4);
  a6=sin(a22);
  a6=(a24*a6);
  a23=cos(a22);
  a23=(a21*a23);
  a6=(a6+a23);
  a6=(a27*a6);
  a6=(a17*a6);
  a39=(a39*a6);
  a3=(a3*a39);
  a43=(a43+a3);
  a3=sin(a4);
  a37=(a37*a6);
  a3=(a3*a37);
  a43=(a43-a3);
  a3=cos(a4);
  a37=sin(a22);
  a37=(a24*a37);
  a6=cos(a22);
  a6=(a21*a6);
  a37=(a37+a6);
  a37=(a2*a37);
  a37=(a17*a37);
  a6=(a18*a37);
  a3=(a3*a6);
  a43=(a43+a3);
  a3=sin(a4);
  a37=(a20*a37);
  a3=(a3*a37);
  a43=(a43+a3);
  a3=cos(a4);
  a37=cos(a22);
  a37=(a24*a37);
  a6=sin(a22);
  a6=(a21*a6);
  a37=(a37-a6);
  a37=(a2*a37);
  a17=(a17*a37);
  a20=(a20*a17);
  a3=(a3*a20);
  a43=(a43-a3);
  a4=sin(a4);
  a18=(a18*a17);
  a4=(a4*a18);
  a43=(a43+a4);
  if (res[2]!=0) res[2][6]=a43;
  a41=(a41*a27);
  a43=(a21*a41);
  a4=cos(a22);
  a43=(a43*a4);
  a41=(a24*a41);
  a4=sin(a22);
  a41=(a41*a4);
  a43=(a43+a41);
  a38=(a38*a27);
  a27=(a21*a38);
  a41=sin(a22);
  a27=(a27*a41);
  a43=(a43-a27);
  a38=(a24*a38);
  a27=cos(a22);
  a38=(a38*a27);
  a43=(a43+a38);
  a25=(a25*a2);
  a38=(a21*a25);
  a27=sin(a22);
  a38=(a38*a27);
  a43=(a43+a38);
  a25=(a24*a25);
  a38=cos(a22);
  a25=(a25*a38);
  a43=(a43-a25);
  a19=(a19*a2);
  a24=(a24*a19);
  a2=sin(a22);
  a24=(a24*a2);
  a43=(a43+a24);
  a21=(a21*a19);
  a22=cos(a22);
  a21=(a21*a22);
  a43=(a43+a21);
  if (res[2]!=0) res[2][7]=a43;
  return 0;
}

CASADI_SYMBOL_EXPORT int ntnu_leg_constr_h_e_fun_jac_uxt_zt_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ntnu_leg_constr_h_e_fun_jac_uxt_zt_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ntnu_leg_constr_h_e_fun_jac_uxt_zt_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ntnu_leg_constr_h_e_fun_jac_uxt_zt_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ntnu_leg_constr_h_e_fun_jac_uxt_zt_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ntnu_leg_constr_h_e_fun_jac_uxt_zt_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ntnu_leg_constr_h_e_fun_jac_uxt_zt_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void ntnu_leg_constr_h_e_fun_jac_uxt_zt_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ntnu_leg_constr_h_e_fun_jac_uxt_zt_hess_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int ntnu_leg_constr_h_e_fun_jac_uxt_zt_hess_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real ntnu_leg_constr_h_e_fun_jac_uxt_zt_hess_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ntnu_leg_constr_h_e_fun_jac_uxt_zt_hess_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ntnu_leg_constr_h_e_fun_jac_uxt_zt_hess_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ntnu_leg_constr_h_e_fun_jac_uxt_zt_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ntnu_leg_constr_h_e_fun_jac_uxt_zt_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    case 1: return casadi_s3;
    case 2: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ntnu_leg_constr_h_e_fun_jac_uxt_zt_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
