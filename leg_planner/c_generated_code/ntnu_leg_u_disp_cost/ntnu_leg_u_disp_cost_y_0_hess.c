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
  #define CASADI_PREFIX(ID) ntnu_leg_u_disp_cost_y_0_hess_ ## ID
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

static const casadi_int casadi_s0[17] = {13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[21] = {17, 1, 0, 17, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
static const casadi_int casadi_s4[32] = {13, 16, 0, 0, 0, 0, 3, 5, 7, 9, 11, 11, 11, 11, 11, 11, 11, 12, 13, 0, 11, 12, 1, 2, 1, 2, 3, 4, 3, 4, 0, 0};

/* ntnu_leg_u_disp_cost_y_0_hess:(i0[13],i1[3],i2[],i3[17],i4[])->(o0[13x16,13nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a4, a5, a6, a7, a8, a9;
  a0=2.;
  a1=arg[0]? arg[0][0] : 0;
  a2=1.3100000000000001e+00;
  a2=(a1-a2);
  a3=casadi_sq(a2);
  a4=(a3+a3);
  a5=casadi_sq(a3);
  a5=(a5+a5);
  a6=1.0000000000000000e-04;
  a7=arg[3]? arg[3][7] : 0;
  a8=(a6*a7);
  a5=(a5*a8);
  a9=(a4*a5);
  a9=(a0*a9);
  a10=(a2+a2);
  a2=(a2+a2);
  a11=(a2+a2);
  a5=(a5*a11);
  a3=(a3+a3);
  a3=(a3*a2);
  a3=(a3+a3);
  a8=(a8*a3);
  a4=(a4*a8);
  a5=(a5+a4);
  a10=(a10*a5);
  a9=(a9+a10);
  a10=(a1+a0);
  a5=casadi_sq(a10);
  a4=(a5+a5);
  a8=casadi_sq(a5);
  a8=(a8+a8);
  a6=(a6*a7);
  a8=(a8*a6);
  a7=(a4*a8);
  a7=(a0*a7);
  a3=(a10+a10);
  a10=(a10+a10);
  a2=(a10+a10);
  a8=(a8*a2);
  a5=(a5+a5);
  a5=(a5*a10);
  a5=(a5+a5);
  a6=(a6*a5);
  a4=(a4*a6);
  a8=(a8+a4);
  a3=(a3*a8);
  a7=(a7+a3);
  a9=(a9+a7);
  a7=arg[0]? arg[0][11] : 0;
  a3=arg[0]? arg[0][12] : 0;
  a7=(a7+a3);
  a3=arg[3]? arg[3][2] : 0;
  a8=(a7*a3);
  a4=cos(a1);
  a8=(a8*a4);
  a9=(a9+a8);
  a8=arg[3]? arg[3][1] : 0;
  a7=(a7*a8);
  a4=sin(a1);
  a7=(a7*a4);
  a9=(a9-a7);
  if (res[0]!=0) res[0][0]=a9;
  a9=cos(a1);
  a9=(a8*a9);
  a7=sin(a1);
  a7=(a3*a7);
  a9=(a9+a7);
  if (res[0]!=0) res[0][1]=a9;
  if (res[0]!=0) res[0][2]=a9;
  a9=arg[0]? arg[0][1] : 0;
  a7=1.6499999999999999e+00;
  a4=(a9-a7);
  a6=casadi_sq(a4);
  a5=(a6+a6);
  a10=casadi_sq(a6);
  a10=(a10+a10);
  a2=4.0000000000000002e-04;
  a11=arg[3]? arg[3][8] : 0;
  a12=(a2*a11);
  a10=(a10*a12);
  a13=(a5*a10);
  a13=(a0*a13);
  a14=(a4+a4);
  a4=(a4+a4);
  a15=(a4+a4);
  a10=(a10*a15);
  a6=(a6+a6);
  a6=(a6*a4);
  a6=(a6+a6);
  a12=(a12*a6);
  a5=(a5*a12);
  a10=(a10+a5);
  a14=(a14*a10);
  a13=(a13+a14);
  a14=1.2000000000000000e+00;
  a10=(a9+a14);
  a5=casadi_sq(a10);
  a12=(a5+a5);
  a6=casadi_sq(a5);
  a6=(a6+a6);
  a4=5.9999999999999995e-04;
  a11=(a4*a11);
  a6=(a6*a11);
  a15=(a12*a6);
  a15=(a0*a15);
  a16=(a10+a10);
  a10=(a10+a10);
  a17=(a10+a10);
  a6=(a6*a17);
  a5=(a5+a5);
  a5=(a5*a10);
  a5=(a5+a5);
  a11=(a11*a5);
  a12=(a12*a11);
  a6=(a6+a12);
  a16=(a16*a6);
  a15=(a15+a16);
  a13=(a13+a15);
  a15=6.7333004874030467e-01;
  a16=2.9976999999999998e-01;
  a6=2.4187360515245046e-01;
  a12=arg[0]? arg[0][2] : 0;
  a11=cos(a12);
  a11=(a6*a11);
  a5=9.7030776516039308e-01;
  a10=sin(a12);
  a10=(a5*a10);
  a11=(a11-a10);
  a10=arg[3]? arg[3][6] : 0;
  a11=(a11*a10);
  a11=(a16*a11);
  a17=(a15*a11);
  a18=sin(a9);
  a17=(a17*a18);
  a13=(a13-a17);
  a17=7.3934203548757504e-01;
  a11=(a17*a11);
  a18=cos(a9);
  a11=(a11*a18);
  a13=(a13-a11);
  a11=cos(a12);
  a11=(a5*a11);
  a18=sin(a12);
  a18=(a6*a18);
  a11=(a11+a18);
  a11=(a11*a10);
  a11=(a16*a11);
  a18=(a17*a11);
  a19=sin(a9);
  a18=(a18*a19);
  a13=(a13+a18);
  a11=(a15*a11);
  a18=cos(a9);
  a11=(a11*a18);
  a13=(a13-a11);
  a11=1.2119940877325484e-01;
  a11=(a11*a10);
  a18=sin(a9);
  a11=(a11*a18);
  a13=(a13-a11);
  a11=1.3308156638776350e-01;
  a11=(a11*a10);
  a18=cos(a9);
  a11=(a11*a18);
  a13=(a13-a11);
  a11=6.7333004875847435e-01;
  a18=cos(a12);
  a18=(a5*a18);
  a19=sin(a12);
  a19=(a6*a19);
  a18=(a18+a19);
  a19=arg[3]? arg[3][5] : 0;
  a18=(a18*a19);
  a18=(a16*a18);
  a20=(a11*a18);
  a21=sin(a9);
  a20=(a20*a21);
  a13=(a13-a20);
  a20=7.3934203546762400e-01;
  a18=(a20*a18);
  a21=cos(a9);
  a18=(a18*a21);
  a13=(a13-a18);
  a18=cos(a12);
  a18=(a6*a18);
  a21=sin(a12);
  a21=(a5*a21);
  a18=(a18-a21);
  a18=(a18*a19);
  a18=(a16*a18);
  a21=(a20*a18);
  a22=sin(a9);
  a21=(a21*a22);
  a13=(a13-a21);
  a18=(a11*a18);
  a21=cos(a9);
  a18=(a18*a21);
  a13=(a13+a18);
  a18=1.2119940877652538e-01;
  a18=(a18*a19);
  a21=cos(a9);
  a18=(a18*a21);
  a13=(a13+a18);
  a18=1.3308156638417232e-01;
  a18=(a18*a19);
  a21=sin(a9);
  a18=(a18*a21);
  a13=(a13-a18);
  if (res[0]!=0) res[0][3]=a13;
  a13=sin(a12);
  a18=sin(a9);
  a18=(a15*a18);
  a21=cos(a9);
  a21=(a17*a21);
  a18=(a18+a21);
  a18=(a16*a18);
  a18=(a10*a18);
  a21=(a5*a18);
  a13=(a13*a21);
  a21=cos(a12);
  a22=cos(a9);
  a22=(a15*a22);
  a23=sin(a9);
  a23=(a17*a23);
  a22=(a22-a23);
  a22=(a16*a22);
  a22=(a10*a22);
  a23=(a5*a22);
  a21=(a21*a23);
  a23=sin(a12);
  a22=(a6*a22);
  a23=(a23*a22);
  a21=(a21+a23);
  a23=cos(a12);
  a18=(a6*a18);
  a23=(a23*a18);
  a21=(a21+a23);
  a13=(a13-a21);
  a21=cos(a12);
  a23=cos(a9);
  a23=(a11*a23);
  a18=sin(a9);
  a18=(a20*a18);
  a23=(a23-a18);
  a23=(a16*a23);
  a23=(a19*a23);
  a18=(a6*a23);
  a21=(a21*a18);
  a13=(a13+a21);
  a21=sin(a12);
  a23=(a5*a23);
  a21=(a21*a23);
  a13=(a13-a21);
  a21=cos(a12);
  a23=sin(a9);
  a23=(a11*a23);
  a18=cos(a9);
  a18=(a20*a18);
  a23=(a23+a18);
  a23=(a16*a23);
  a23=(a19*a23);
  a18=(a5*a23);
  a21=(a21*a18);
  a13=(a13-a21);
  a21=sin(a12);
  a23=(a6*a23);
  a21=(a21*a23);
  a13=(a13-a21);
  if (res[0]!=0) res[0][4]=a13;
  a13=sin(a9);
  a21=sin(a12);
  a21=(a6*a21);
  a23=cos(a12);
  a23=(a5*a23);
  a21=(a21+a23);
  a21=(a10*a21);
  a21=(a16*a21);
  a23=(a17*a21);
  a13=(a13*a23);
  a23=cos(a9);
  a21=(a15*a21);
  a23=(a23*a21);
  a13=(a13-a23);
  a23=cos(a9);
  a21=cos(a12);
  a21=(a6*a21);
  a18=sin(a12);
  a18=(a5*a18);
  a21=(a21-a18);
  a21=(a10*a21);
  a21=(a16*a21);
  a18=(a17*a21);
  a23=(a23*a18);
  a13=(a13-a23);
  a23=sin(a9);
  a21=(a15*a21);
  a23=(a23*a21);
  a13=(a13-a23);
  a23=cos(a9);
  a21=cos(a12);
  a21=(a6*a21);
  a18=sin(a12);
  a18=(a5*a18);
  a21=(a21-a18);
  a21=(a19*a21);
  a21=(a16*a21);
  a18=(a11*a21);
  a23=(a23*a18);
  a13=(a13+a23);
  a23=sin(a9);
  a21=(a20*a21);
  a23=(a23*a21);
  a13=(a13-a23);
  a23=cos(a9);
  a21=sin(a12);
  a21=(a6*a21);
  a18=cos(a12);
  a18=(a5*a18);
  a21=(a21+a18);
  a21=(a19*a21);
  a21=(a16*a21);
  a18=(a20*a21);
  a23=(a23*a18);
  a13=(a13-a23);
  a23=sin(a9);
  a21=(a11*a21);
  a23=(a23*a21);
  a13=(a13-a23);
  if (res[0]!=0) res[0][5]=a13;
  a13=1.4199999999999999e+00;
  a23=(a12-a13);
  a21=casadi_sq(a23);
  a18=(a21+a21);
  a22=casadi_sq(a21);
  a22=(a22+a22);
  a24=arg[3]? arg[3][9] : 0;
  a25=(a2*a24);
  a22=(a22*a25);
  a26=(a18*a22);
  a26=(a0*a26);
  a27=(a23+a23);
  a23=(a23+a23);
  a28=(a23+a23);
  a22=(a22*a28);
  a21=(a21+a21);
  a21=(a21*a23);
  a21=(a21+a21);
  a25=(a25*a21);
  a18=(a18*a25);
  a22=(a22+a18);
  a27=(a27*a22);
  a26=(a26+a27);
  a27=1.3230000000000000e+00;
  a22=(a12+a27);
  a18=casadi_sq(a22);
  a25=(a18+a18);
  a21=casadi_sq(a18);
  a21=(a21+a21);
  a24=(a4*a24);
  a21=(a21*a24);
  a23=(a25*a21);
  a23=(a0*a23);
  a28=(a22+a22);
  a22=(a22+a22);
  a29=(a22+a22);
  a21=(a21*a29);
  a18=(a18+a18);
  a18=(a18*a22);
  a18=(a18+a18);
  a24=(a24*a18);
  a25=(a25*a24);
  a21=(a21+a25);
  a28=(a28*a21);
  a23=(a23+a28);
  a26=(a26+a23);
  a23=cos(a9);
  a23=(a17*a23);
  a28=sin(a9);
  a28=(a15*a28);
  a23=(a23+a28);
  a23=(a16*a23);
  a23=(a23*a10);
  a28=(a5*a23);
  a21=sin(a12);
  a28=(a28*a21);
  a26=(a26+a28);
  a23=(a6*a23);
  a28=cos(a12);
  a23=(a23*a28);
  a26=(a26-a23);
  a23=cos(a9);
  a15=(a15*a23);
  a23=sin(a9);
  a17=(a17*a23);
  a15=(a15-a17);
  a15=(a16*a15);
  a15=(a15*a10);
  a17=(a6*a15);
  a23=sin(a12);
  a17=(a17*a23);
  a26=(a26-a17);
  a15=(a5*a15);
  a17=cos(a12);
  a15=(a15*a17);
  a26=(a26-a15);
  a15=cos(a9);
  a15=(a20*a15);
  a17=sin(a9);
  a17=(a11*a17);
  a15=(a15+a17);
  a15=(a16*a15);
  a15=(a15*a19);
  a17=(a6*a15);
  a23=sin(a12);
  a17=(a17*a23);
  a26=(a26-a17);
  a15=(a5*a15);
  a17=cos(a12);
  a15=(a15*a17);
  a26=(a26-a15);
  a15=cos(a9);
  a11=(a11*a15);
  a9=sin(a9);
  a20=(a20*a9);
  a11=(a11-a20);
  a16=(a16*a11);
  a16=(a16*a19);
  a5=(a5*a16);
  a11=sin(a12);
  a5=(a5*a11);
  a26=(a26-a5);
  a6=(a6*a16);
  a12=cos(a12);
  a6=(a6*a12);
  a26=(a26+a6);
  if (res[0]!=0) res[0][6]=a26;
  a26=arg[0]? arg[0][3] : 0;
  a14=(a26-a14);
  a6=casadi_sq(a14);
  a12=(a6+a6);
  a16=casadi_sq(a6);
  a16=(a16+a16);
  a5=arg[3]? arg[3][10] : 0;
  a11=(a4*a5);
  a16=(a16*a11);
  a20=(a12*a16);
  a20=(a0*a20);
  a9=(a14+a14);
  a14=(a14+a14);
  a15=(a14+a14);
  a16=(a16*a15);
  a6=(a6+a6);
  a6=(a6*a14);
  a6=(a6+a6);
  a11=(a11*a6);
  a12=(a12*a11);
  a16=(a16+a12);
  a9=(a9*a16);
  a20=(a20+a9);
  a7=(a26+a7);
  a9=casadi_sq(a7);
  a16=(a9+a9);
  a12=casadi_sq(a9);
  a12=(a12+a12);
  a5=(a2*a5);
  a12=(a12*a5);
  a11=(a16*a12);
  a11=(a0*a11);
  a6=(a7+a7);
  a7=(a7+a7);
  a14=(a7+a7);
  a12=(a12*a14);
  a9=(a9+a9);
  a9=(a9*a7);
  a9=(a9+a9);
  a5=(a5*a9);
  a16=(a16*a5);
  a12=(a12+a16);
  a6=(a6*a12);
  a11=(a11+a6);
  a20=(a20+a11);
  a11=6.7131110514003900e-01;
  a6=2.9929000000000000e-01;
  a12=2.4497734630999077e-01;
  a16=arg[0]? arg[0][4] : 0;
  a5=cos(a16);
  a5=(a12*a5);
  a9=9.6952880297333865e-01;
  a7=sin(a16);
  a7=(a9*a7);
  a5=(a5+a7);
  a5=(a5*a10);
  a5=(a6*a5);
  a7=(a11*a5);
  a14=sin(a26);
  a7=(a7*a14);
  a20=(a20-a7);
  a7=7.4117568773627318e-01;
  a5=(a7*a5);
  a14=cos(a26);
  a5=(a5*a14);
  a20=(a20+a5);
  a5=cos(a16);
  a5=(a9*a5);
  a14=sin(a16);
  a14=(a12*a14);
  a5=(a5-a14);
  a5=(a5*a10);
  a5=(a6*a5);
  a14=(a7*a5);
  a15=sin(a26);
  a14=(a14*a15);
  a20=(a20+a14);
  a5=(a11*a5);
  a14=cos(a26);
  a5=(a5*a14);
  a20=(a20+a5);
  a5=1.2083599892520702e-01;
  a5=(a5*a10);
  a14=sin(a26);
  a5=(a5*a14);
  a20=(a20-a5);
  a5=1.3341162379252916e-01;
  a5=(a5*a10);
  a14=cos(a26);
  a5=(a5*a14);
  a20=(a20+a5);
  a5=6.7131110515815429e-01;
  a14=cos(a16);
  a14=(a9*a14);
  a15=sin(a16);
  a15=(a12*a15);
  a14=(a14-a15);
  a14=(a14*a19);
  a14=(a6*a14);
  a15=(a5*a14);
  a17=sin(a26);
  a15=(a15*a17);
  a20=(a20+a15);
  a15=7.4117568771627262e-01;
  a14=(a15*a14);
  a17=cos(a26);
  a14=(a14*a17);
  a20=(a20-a14);
  a14=cos(a16);
  a14=(a12*a14);
  a17=sin(a16);
  a17=(a9*a17);
  a14=(a14+a17);
  a14=(a14*a19);
  a14=(a6*a14);
  a17=(a15*a14);
  a23=sin(a26);
  a17=(a17*a23);
  a20=(a20+a17);
  a14=(a5*a14);
  a17=cos(a26);
  a14=(a14*a17);
  a20=(a20+a14);
  a14=1.3341162378892907e-01;
  a14=(a14*a19);
  a17=sin(a26);
  a14=(a14*a17);
  a20=(a20+a14);
  a14=1.2083599892846777e-01;
  a14=(a14*a19);
  a17=cos(a26);
  a14=(a14*a17);
  a20=(a20+a14);
  if (res[0]!=0) res[0][7]=a20;
  a20=cos(a16);
  a14=sin(a26);
  a14=(a7*a14);
  a17=cos(a26);
  a17=(a11*a17);
  a14=(a14+a17);
  a14=(a6*a14);
  a14=(a10*a14);
  a17=(a9*a14);
  a20=(a20*a17);
  a17=sin(a16);
  a14=(a12*a14);
  a17=(a17*a14);
  a20=(a20-a17);
  a17=cos(a16);
  a14=cos(a26);
  a14=(a7*a14);
  a23=sin(a26);
  a23=(a11*a23);
  a14=(a14-a23);
  a14=(a6*a14);
  a14=(a10*a14);
  a23=(a12*a14);
  a17=(a17*a23);
  a20=(a20+a17);
  a17=sin(a16);
  a14=(a9*a14);
  a17=(a17*a14);
  a20=(a20+a17);
  a17=cos(a16);
  a14=sin(a26);
  a14=(a15*a14);
  a23=cos(a26);
  a23=(a5*a23);
  a14=(a14+a23);
  a14=(a6*a14);
  a14=(a19*a14);
  a23=(a12*a14);
  a17=(a17*a23);
  a20=(a20+a17);
  a17=sin(a16);
  a14=(a9*a14);
  a17=(a17*a14);
  a20=(a20+a17);
  a17=cos(a16);
  a14=cos(a26);
  a14=(a15*a14);
  a23=sin(a26);
  a23=(a5*a23);
  a14=(a14-a23);
  a14=(a6*a14);
  a14=(a19*a14);
  a23=(a9*a14);
  a17=(a17*a23);
  a20=(a20-a17);
  a17=sin(a16);
  a14=(a12*a14);
  a17=(a17*a14);
  a20=(a20+a17);
  if (res[0]!=0) res[0][8]=a20;
  a20=cos(a26);
  a17=cos(a16);
  a17=(a9*a17);
  a14=sin(a16);
  a14=(a12*a14);
  a17=(a17-a14);
  a17=(a10*a17);
  a17=(a6*a17);
  a14=(a11*a17);
  a20=(a20*a14);
  a14=sin(a26);
  a17=(a7*a17);
  a14=(a14*a17);
  a20=(a20+a14);
  a14=cos(a26);
  a17=sin(a16);
  a17=(a9*a17);
  a23=cos(a16);
  a23=(a12*a23);
  a17=(a17+a23);
  a17=(a10*a17);
  a17=(a6*a17);
  a23=(a7*a17);
  a14=(a14*a23);
  a20=(a20+a14);
  a14=sin(a26);
  a17=(a11*a17);
  a14=(a14*a17);
  a20=(a20-a14);
  a14=cos(a26);
  a17=sin(a16);
  a17=(a9*a17);
  a23=cos(a16);
  a23=(a12*a23);
  a17=(a17+a23);
  a17=(a19*a17);
  a17=(a6*a17);
  a23=(a5*a17);
  a14=(a14*a23);
  a20=(a20+a14);
  a14=sin(a26);
  a17=(a15*a17);
  a14=(a14*a17);
  a20=(a20+a14);
  a14=cos(a26);
  a17=cos(a16);
  a17=(a9*a17);
  a23=sin(a16);
  a23=(a12*a23);
  a17=(a17-a23);
  a17=(a19*a17);
  a17=(a6*a17);
  a23=(a15*a17);
  a14=(a14*a23);
  a20=(a20-a14);
  a14=sin(a26);
  a17=(a5*a17);
  a14=(a14*a17);
  a20=(a20+a14);
  if (res[0]!=0) res[0][9]=a20;
  a27=(a16-a27);
  a20=casadi_sq(a27);
  a14=(a20+a20);
  a17=casadi_sq(a20);
  a17=(a17+a17);
  a23=arg[3]? arg[3][11] : 0;
  a4=(a4*a23);
  a17=(a17*a4);
  a28=(a14*a17);
  a28=(a0*a28);
  a21=(a27+a27);
  a27=(a27+a27);
  a25=(a27+a27);
  a17=(a17*a25);
  a20=(a20+a20);
  a20=(a20*a27);
  a20=(a20+a20);
  a4=(a4*a20);
  a14=(a14*a4);
  a17=(a17+a14);
  a21=(a21*a17);
  a28=(a28+a21);
  a13=(a16+a13);
  a21=casadi_sq(a13);
  a17=(a21+a21);
  a14=casadi_sq(a21);
  a14=(a14+a14);
  a2=(a2*a23);
  a14=(a14*a2);
  a23=(a17*a14);
  a0=(a0*a23);
  a23=(a13+a13);
  a13=(a13+a13);
  a4=(a13+a13);
  a14=(a14*a4);
  a21=(a21+a21);
  a21=(a21*a13);
  a21=(a21+a21);
  a2=(a2*a21);
  a17=(a17*a2);
  a14=(a14+a17);
  a23=(a23*a14);
  a0=(a0+a23);
  a28=(a28+a0);
  a0=cos(a26);
  a0=(a7*a0);
  a23=sin(a26);
  a23=(a11*a23);
  a0=(a0-a23);
  a0=(a6*a0);
  a0=(a0*a10);
  a23=(a9*a0);
  a14=sin(a16);
  a23=(a23*a14);
  a28=(a28+a23);
  a0=(a12*a0);
  a23=cos(a16);
  a0=(a0*a23);
  a28=(a28+a0);
  a0=cos(a26);
  a11=(a11*a0);
  a0=sin(a26);
  a7=(a7*a0);
  a11=(a11+a7);
  a11=(a6*a11);
  a11=(a11*a10);
  a10=(a12*a11);
  a7=sin(a16);
  a10=(a10*a7);
  a28=(a28-a10);
  a11=(a9*a11);
  a10=cos(a16);
  a11=(a11*a10);
  a28=(a28+a11);
  a11=cos(a26);
  a11=(a15*a11);
  a10=sin(a26);
  a10=(a5*a10);
  a11=(a11-a10);
  a11=(a6*a11);
  a11=(a11*a19);
  a10=(a12*a11);
  a7=sin(a16);
  a10=(a10*a7);
  a28=(a28+a10);
  a11=(a9*a11);
  a10=cos(a16);
  a11=(a11*a10);
  a28=(a28-a11);
  a11=cos(a26);
  a5=(a5*a11);
  a26=sin(a26);
  a15=(a15*a26);
  a5=(a5+a15);
  a6=(a6*a5);
  a6=(a6*a19);
  a9=(a9*a6);
  a19=sin(a16);
  a9=(a9*a19);
  a28=(a28+a9);
  a12=(a12*a6);
  a16=cos(a16);
  a12=(a12*a16);
  a28=(a28+a12);
  if (res[0]!=0) res[0][10]=a28;
  a28=sin(a1);
  a12=(a28*a3);
  a1=cos(a1);
  a16=(a1*a8);
  a12=(a12+a16);
  if (res[0]!=0) res[0][11]=a12;
  a28=(a28*a3);
  a1=(a1*a8);
  a28=(a28+a1);
  if (res[0]!=0) res[0][12]=a28;
  return 0;
}

CASADI_SYMBOL_EXPORT int ntnu_leg_u_disp_cost_y_0_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ntnu_leg_u_disp_cost_y_0_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ntnu_leg_u_disp_cost_y_0_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ntnu_leg_u_disp_cost_y_0_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ntnu_leg_u_disp_cost_y_0_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ntnu_leg_u_disp_cost_y_0_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ntnu_leg_u_disp_cost_y_0_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void ntnu_leg_u_disp_cost_y_0_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ntnu_leg_u_disp_cost_y_0_hess_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int ntnu_leg_u_disp_cost_y_0_hess_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real ntnu_leg_u_disp_cost_y_0_hess_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ntnu_leg_u_disp_cost_y_0_hess_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ntnu_leg_u_disp_cost_y_0_hess_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ntnu_leg_u_disp_cost_y_0_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    case 4: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ntnu_leg_u_disp_cost_y_0_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ntnu_leg_u_disp_cost_y_0_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
