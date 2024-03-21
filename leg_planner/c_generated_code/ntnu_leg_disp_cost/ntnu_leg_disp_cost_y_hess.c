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
  #define CASADI_PREFIX(ID) ntnu_leg_disp_cost_y_hess_ ## ID
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

static const casadi_int casadi_s0[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[21] = {17, 1, 0, 17, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
static const casadi_int casadi_s4[29] = {13, 13, 0, 0, 1, 2, 5, 7, 9, 11, 13, 13, 13, 13, 13, 13, 3, 3, 1, 2, 3, 4, 5, 4, 5, 6, 7, 6, 7};

/* ntnu_leg_disp_cost_y_hess:(i0[10],i1[3],i2[],i3[17],i4[])->(o0[13x13,13nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][0] : 0;
  a1=sin(a0);
  a2=arg[3]? arg[3][2] : 0;
  a3=(a1*a2);
  a4=cos(a0);
  a5=arg[3]? arg[3][1] : 0;
  a6=(a4*a5);
  a3=(a3+a6);
  if (res[0]!=0) res[0][0]=a3;
  a1=(a1*a2);
  a4=(a4*a5);
  a1=(a1+a4);
  if (res[0]!=0) res[0][1]=a1;
  a1=cos(a0);
  a1=(a5*a1);
  a4=sin(a0);
  a4=(a2*a4);
  a1=(a1+a4);
  if (res[0]!=0) res[0][2]=a1;
  if (res[0]!=0) res[0][3]=a1;
  a1=2.;
  a4=1.3100000000000001e+00;
  a4=(a0-a4);
  a3=casadi_sq(a4);
  a6=(a3+a3);
  a7=casadi_sq(a3);
  a7=(a7+a7);
  a8=1.0000000000000000e-04;
  a9=arg[3]? arg[3][7] : 0;
  a10=(a8*a9);
  a7=(a7*a10);
  a11=(a6*a7);
  a11=(a1*a11);
  a12=(a4+a4);
  a4=(a4+a4);
  a13=(a4+a4);
  a7=(a7*a13);
  a3=(a3+a3);
  a3=(a3*a4);
  a3=(a3+a3);
  a10=(a10*a3);
  a6=(a6*a10);
  a7=(a7+a6);
  a12=(a12*a7);
  a11=(a11+a12);
  a12=(a0+a1);
  a7=casadi_sq(a12);
  a6=(a7+a7);
  a10=casadi_sq(a7);
  a10=(a10+a10);
  a8=(a8*a9);
  a10=(a10*a8);
  a9=(a6*a10);
  a9=(a1*a9);
  a3=(a12+a12);
  a12=(a12+a12);
  a4=(a12+a12);
  a10=(a10*a4);
  a7=(a7+a7);
  a7=(a7*a12);
  a7=(a7+a7);
  a8=(a8*a7);
  a6=(a6*a8);
  a10=(a10+a6);
  a3=(a3*a10);
  a9=(a9+a3);
  a11=(a11+a9);
  a9=arg[1]? arg[1][1] : 0;
  a3=arg[1]? arg[1][2] : 0;
  a9=(a9+a3);
  a2=(a9*a2);
  a3=cos(a0);
  a2=(a2*a3);
  a11=(a11+a2);
  a9=(a9*a5);
  a0=sin(a0);
  a9=(a9*a0);
  a11=(a11-a9);
  if (res[0]!=0) res[0][4]=a11;
  a11=arg[0]? arg[0][1] : 0;
  a9=1.6499999999999999e+00;
  a0=(a11-a9);
  a5=casadi_sq(a0);
  a2=(a5+a5);
  a3=casadi_sq(a5);
  a3=(a3+a3);
  a10=4.0000000000000002e-04;
  a6=arg[3]? arg[3][8] : 0;
  a8=(a10*a6);
  a3=(a3*a8);
  a7=(a2*a3);
  a7=(a1*a7);
  a12=(a0+a0);
  a0=(a0+a0);
  a4=(a0+a0);
  a3=(a3*a4);
  a5=(a5+a5);
  a5=(a5*a0);
  a5=(a5+a5);
  a8=(a8*a5);
  a2=(a2*a8);
  a3=(a3+a2);
  a12=(a12*a3);
  a7=(a7+a12);
  a12=1.2000000000000000e+00;
  a3=(a11+a12);
  a2=casadi_sq(a3);
  a8=(a2+a2);
  a5=casadi_sq(a2);
  a5=(a5+a5);
  a0=5.9999999999999995e-04;
  a6=(a0*a6);
  a5=(a5*a6);
  a4=(a8*a5);
  a4=(a1*a4);
  a13=(a3+a3);
  a3=(a3+a3);
  a14=(a3+a3);
  a5=(a5*a14);
  a2=(a2+a2);
  a2=(a2*a3);
  a2=(a2+a2);
  a6=(a6*a2);
  a8=(a8*a6);
  a5=(a5+a8);
  a13=(a13*a5);
  a4=(a4+a13);
  a7=(a7+a4);
  a4=6.7333004874030467e-01;
  a13=2.9976999999999998e-01;
  a5=2.4187360515245046e-01;
  a8=arg[0]? arg[0][2] : 0;
  a6=cos(a8);
  a6=(a5*a6);
  a2=9.7030776516039308e-01;
  a3=sin(a8);
  a3=(a2*a3);
  a6=(a6-a3);
  a3=arg[3]? arg[3][6] : 0;
  a6=(a6*a3);
  a6=(a13*a6);
  a14=(a4*a6);
  a15=sin(a11);
  a14=(a14*a15);
  a7=(a7-a14);
  a14=7.3934203548757504e-01;
  a6=(a14*a6);
  a15=cos(a11);
  a6=(a6*a15);
  a7=(a7-a6);
  a6=cos(a8);
  a6=(a2*a6);
  a15=sin(a8);
  a15=(a5*a15);
  a6=(a6+a15);
  a6=(a6*a3);
  a6=(a13*a6);
  a15=(a14*a6);
  a16=sin(a11);
  a15=(a15*a16);
  a7=(a7+a15);
  a6=(a4*a6);
  a15=cos(a11);
  a6=(a6*a15);
  a7=(a7-a6);
  a6=1.2119940877325484e-01;
  a6=(a6*a3);
  a15=sin(a11);
  a6=(a6*a15);
  a7=(a7-a6);
  a6=1.3308156638776350e-01;
  a6=(a6*a3);
  a15=cos(a11);
  a6=(a6*a15);
  a7=(a7-a6);
  a6=6.7333004875847435e-01;
  a15=cos(a8);
  a15=(a2*a15);
  a16=sin(a8);
  a16=(a5*a16);
  a15=(a15+a16);
  a16=arg[3]? arg[3][5] : 0;
  a15=(a15*a16);
  a15=(a13*a15);
  a17=(a6*a15);
  a18=sin(a11);
  a17=(a17*a18);
  a7=(a7-a17);
  a17=7.3934203546762400e-01;
  a15=(a17*a15);
  a18=cos(a11);
  a15=(a15*a18);
  a7=(a7-a15);
  a15=cos(a8);
  a15=(a5*a15);
  a18=sin(a8);
  a18=(a2*a18);
  a15=(a15-a18);
  a15=(a15*a16);
  a15=(a13*a15);
  a18=(a17*a15);
  a19=sin(a11);
  a18=(a18*a19);
  a7=(a7-a18);
  a15=(a6*a15);
  a18=cos(a11);
  a15=(a15*a18);
  a7=(a7+a15);
  a15=1.2119940877652538e-01;
  a15=(a15*a16);
  a18=cos(a11);
  a15=(a15*a18);
  a7=(a7+a15);
  a15=1.3308156638417232e-01;
  a15=(a15*a16);
  a18=sin(a11);
  a15=(a15*a18);
  a7=(a7-a15);
  if (res[0]!=0) res[0][5]=a7;
  a7=sin(a8);
  a15=sin(a11);
  a15=(a4*a15);
  a18=cos(a11);
  a18=(a14*a18);
  a15=(a15+a18);
  a15=(a13*a15);
  a15=(a3*a15);
  a18=(a2*a15);
  a7=(a7*a18);
  a18=cos(a8);
  a19=cos(a11);
  a19=(a4*a19);
  a20=sin(a11);
  a20=(a14*a20);
  a19=(a19-a20);
  a19=(a13*a19);
  a19=(a3*a19);
  a20=(a2*a19);
  a18=(a18*a20);
  a20=sin(a8);
  a19=(a5*a19);
  a20=(a20*a19);
  a18=(a18+a20);
  a20=cos(a8);
  a15=(a5*a15);
  a20=(a20*a15);
  a18=(a18+a20);
  a7=(a7-a18);
  a18=cos(a8);
  a20=cos(a11);
  a20=(a6*a20);
  a15=sin(a11);
  a15=(a17*a15);
  a20=(a20-a15);
  a20=(a13*a20);
  a20=(a16*a20);
  a15=(a5*a20);
  a18=(a18*a15);
  a7=(a7+a18);
  a18=sin(a8);
  a20=(a2*a20);
  a18=(a18*a20);
  a7=(a7-a18);
  a18=cos(a8);
  a20=sin(a11);
  a20=(a6*a20);
  a15=cos(a11);
  a15=(a17*a15);
  a20=(a20+a15);
  a20=(a13*a20);
  a20=(a16*a20);
  a15=(a2*a20);
  a18=(a18*a15);
  a7=(a7-a18);
  a18=sin(a8);
  a20=(a5*a20);
  a18=(a18*a20);
  a7=(a7-a18);
  if (res[0]!=0) res[0][6]=a7;
  a7=sin(a11);
  a18=sin(a8);
  a18=(a5*a18);
  a20=cos(a8);
  a20=(a2*a20);
  a18=(a18+a20);
  a18=(a3*a18);
  a18=(a13*a18);
  a20=(a14*a18);
  a7=(a7*a20);
  a20=cos(a11);
  a18=(a4*a18);
  a20=(a20*a18);
  a7=(a7-a20);
  a20=cos(a11);
  a18=cos(a8);
  a18=(a5*a18);
  a15=sin(a8);
  a15=(a2*a15);
  a18=(a18-a15);
  a18=(a3*a18);
  a18=(a13*a18);
  a15=(a14*a18);
  a20=(a20*a15);
  a7=(a7-a20);
  a20=sin(a11);
  a18=(a4*a18);
  a20=(a20*a18);
  a7=(a7-a20);
  a20=cos(a11);
  a18=cos(a8);
  a18=(a5*a18);
  a15=sin(a8);
  a15=(a2*a15);
  a18=(a18-a15);
  a18=(a16*a18);
  a18=(a13*a18);
  a15=(a6*a18);
  a20=(a20*a15);
  a7=(a7+a20);
  a20=sin(a11);
  a18=(a17*a18);
  a20=(a20*a18);
  a7=(a7-a20);
  a20=cos(a11);
  a18=sin(a8);
  a18=(a5*a18);
  a15=cos(a8);
  a15=(a2*a15);
  a18=(a18+a15);
  a18=(a16*a18);
  a18=(a13*a18);
  a15=(a17*a18);
  a20=(a20*a15);
  a7=(a7-a20);
  a20=sin(a11);
  a18=(a6*a18);
  a20=(a20*a18);
  a7=(a7-a20);
  if (res[0]!=0) res[0][7]=a7;
  a7=1.4199999999999999e+00;
  a20=(a8-a7);
  a18=casadi_sq(a20);
  a15=(a18+a18);
  a19=casadi_sq(a18);
  a19=(a19+a19);
  a21=arg[3]? arg[3][9] : 0;
  a22=(a10*a21);
  a19=(a19*a22);
  a23=(a15*a19);
  a23=(a1*a23);
  a24=(a20+a20);
  a20=(a20+a20);
  a25=(a20+a20);
  a19=(a19*a25);
  a18=(a18+a18);
  a18=(a18*a20);
  a18=(a18+a18);
  a22=(a22*a18);
  a15=(a15*a22);
  a19=(a19+a15);
  a24=(a24*a19);
  a23=(a23+a24);
  a24=1.3230000000000000e+00;
  a19=(a8+a24);
  a15=casadi_sq(a19);
  a22=(a15+a15);
  a18=casadi_sq(a15);
  a18=(a18+a18);
  a21=(a0*a21);
  a18=(a18*a21);
  a20=(a22*a18);
  a20=(a1*a20);
  a25=(a19+a19);
  a19=(a19+a19);
  a26=(a19+a19);
  a18=(a18*a26);
  a15=(a15+a15);
  a15=(a15*a19);
  a15=(a15+a15);
  a21=(a21*a15);
  a22=(a22*a21);
  a18=(a18+a22);
  a25=(a25*a18);
  a20=(a20+a25);
  a23=(a23+a20);
  a20=cos(a11);
  a20=(a14*a20);
  a25=sin(a11);
  a25=(a4*a25);
  a20=(a20+a25);
  a20=(a13*a20);
  a20=(a20*a3);
  a25=(a2*a20);
  a18=sin(a8);
  a25=(a25*a18);
  a23=(a23+a25);
  a20=(a5*a20);
  a25=cos(a8);
  a20=(a20*a25);
  a23=(a23-a20);
  a20=cos(a11);
  a4=(a4*a20);
  a20=sin(a11);
  a14=(a14*a20);
  a4=(a4-a14);
  a4=(a13*a4);
  a4=(a4*a3);
  a14=(a5*a4);
  a20=sin(a8);
  a14=(a14*a20);
  a23=(a23-a14);
  a4=(a2*a4);
  a14=cos(a8);
  a4=(a4*a14);
  a23=(a23-a4);
  a4=cos(a11);
  a4=(a17*a4);
  a14=sin(a11);
  a14=(a6*a14);
  a4=(a4+a14);
  a4=(a13*a4);
  a4=(a4*a16);
  a14=(a5*a4);
  a20=sin(a8);
  a14=(a14*a20);
  a23=(a23-a14);
  a4=(a2*a4);
  a14=cos(a8);
  a4=(a4*a14);
  a23=(a23-a4);
  a4=cos(a11);
  a6=(a6*a4);
  a11=sin(a11);
  a17=(a17*a11);
  a6=(a6-a17);
  a13=(a13*a6);
  a13=(a13*a16);
  a2=(a2*a13);
  a6=sin(a8);
  a2=(a2*a6);
  a23=(a23-a2);
  a5=(a5*a13);
  a8=cos(a8);
  a5=(a5*a8);
  a23=(a23+a5);
  if (res[0]!=0) res[0][8]=a23;
  a23=arg[0]? arg[0][3] : 0;
  a12=(a23-a12);
  a5=casadi_sq(a12);
  a8=(a5+a5);
  a13=casadi_sq(a5);
  a13=(a13+a13);
  a2=arg[3]? arg[3][10] : 0;
  a6=(a0*a2);
  a13=(a13*a6);
  a17=(a8*a13);
  a17=(a1*a17);
  a11=(a12+a12);
  a12=(a12+a12);
  a4=(a12+a12);
  a13=(a13*a4);
  a5=(a5+a5);
  a5=(a5*a12);
  a5=(a5+a5);
  a6=(a6*a5);
  a8=(a8*a6);
  a13=(a13+a8);
  a11=(a11*a13);
  a17=(a17+a11);
  a9=(a23+a9);
  a11=casadi_sq(a9);
  a13=(a11+a11);
  a8=casadi_sq(a11);
  a8=(a8+a8);
  a2=(a10*a2);
  a8=(a8*a2);
  a6=(a13*a8);
  a6=(a1*a6);
  a5=(a9+a9);
  a9=(a9+a9);
  a12=(a9+a9);
  a8=(a8*a12);
  a11=(a11+a11);
  a11=(a11*a9);
  a11=(a11+a11);
  a2=(a2*a11);
  a13=(a13*a2);
  a8=(a8+a13);
  a5=(a5*a8);
  a6=(a6+a5);
  a17=(a17+a6);
  a6=6.7131110514003900e-01;
  a5=2.9929000000000000e-01;
  a8=2.4497734630999077e-01;
  a13=arg[0]? arg[0][4] : 0;
  a2=cos(a13);
  a2=(a8*a2);
  a11=9.6952880297333865e-01;
  a9=sin(a13);
  a9=(a11*a9);
  a2=(a2+a9);
  a2=(a2*a3);
  a2=(a5*a2);
  a9=(a6*a2);
  a12=sin(a23);
  a9=(a9*a12);
  a17=(a17-a9);
  a9=7.4117568773627318e-01;
  a2=(a9*a2);
  a12=cos(a23);
  a2=(a2*a12);
  a17=(a17+a2);
  a2=cos(a13);
  a2=(a11*a2);
  a12=sin(a13);
  a12=(a8*a12);
  a2=(a2-a12);
  a2=(a2*a3);
  a2=(a5*a2);
  a12=(a9*a2);
  a4=sin(a23);
  a12=(a12*a4);
  a17=(a17+a12);
  a2=(a6*a2);
  a12=cos(a23);
  a2=(a2*a12);
  a17=(a17+a2);
  a2=1.2083599892520702e-01;
  a2=(a2*a3);
  a12=sin(a23);
  a2=(a2*a12);
  a17=(a17-a2);
  a2=1.3341162379252916e-01;
  a2=(a2*a3);
  a12=cos(a23);
  a2=(a2*a12);
  a17=(a17+a2);
  a2=6.7131110515815429e-01;
  a12=cos(a13);
  a12=(a11*a12);
  a4=sin(a13);
  a4=(a8*a4);
  a12=(a12-a4);
  a12=(a12*a16);
  a12=(a5*a12);
  a4=(a2*a12);
  a14=sin(a23);
  a4=(a4*a14);
  a17=(a17+a4);
  a4=7.4117568771627262e-01;
  a12=(a4*a12);
  a14=cos(a23);
  a12=(a12*a14);
  a17=(a17-a12);
  a12=cos(a13);
  a12=(a8*a12);
  a14=sin(a13);
  a14=(a11*a14);
  a12=(a12+a14);
  a12=(a12*a16);
  a12=(a5*a12);
  a14=(a4*a12);
  a20=sin(a23);
  a14=(a14*a20);
  a17=(a17+a14);
  a12=(a2*a12);
  a14=cos(a23);
  a12=(a12*a14);
  a17=(a17+a12);
  a12=1.3341162378892907e-01;
  a12=(a12*a16);
  a14=sin(a23);
  a12=(a12*a14);
  a17=(a17+a12);
  a12=1.2083599892846777e-01;
  a12=(a12*a16);
  a14=cos(a23);
  a12=(a12*a14);
  a17=(a17+a12);
  if (res[0]!=0) res[0][9]=a17;
  a17=cos(a13);
  a12=sin(a23);
  a12=(a9*a12);
  a14=cos(a23);
  a14=(a6*a14);
  a12=(a12+a14);
  a12=(a5*a12);
  a12=(a3*a12);
  a14=(a11*a12);
  a17=(a17*a14);
  a14=sin(a13);
  a12=(a8*a12);
  a14=(a14*a12);
  a17=(a17-a14);
  a14=cos(a13);
  a12=cos(a23);
  a12=(a9*a12);
  a20=sin(a23);
  a20=(a6*a20);
  a12=(a12-a20);
  a12=(a5*a12);
  a12=(a3*a12);
  a20=(a8*a12);
  a14=(a14*a20);
  a17=(a17+a14);
  a14=sin(a13);
  a12=(a11*a12);
  a14=(a14*a12);
  a17=(a17+a14);
  a14=cos(a13);
  a12=sin(a23);
  a12=(a4*a12);
  a20=cos(a23);
  a20=(a2*a20);
  a12=(a12+a20);
  a12=(a5*a12);
  a12=(a16*a12);
  a20=(a8*a12);
  a14=(a14*a20);
  a17=(a17+a14);
  a14=sin(a13);
  a12=(a11*a12);
  a14=(a14*a12);
  a17=(a17+a14);
  a14=cos(a13);
  a12=cos(a23);
  a12=(a4*a12);
  a20=sin(a23);
  a20=(a2*a20);
  a12=(a12-a20);
  a12=(a5*a12);
  a12=(a16*a12);
  a20=(a11*a12);
  a14=(a14*a20);
  a17=(a17-a14);
  a14=sin(a13);
  a12=(a8*a12);
  a14=(a14*a12);
  a17=(a17+a14);
  if (res[0]!=0) res[0][10]=a17;
  a17=cos(a23);
  a14=cos(a13);
  a14=(a11*a14);
  a12=sin(a13);
  a12=(a8*a12);
  a14=(a14-a12);
  a14=(a3*a14);
  a14=(a5*a14);
  a12=(a6*a14);
  a17=(a17*a12);
  a12=sin(a23);
  a14=(a9*a14);
  a12=(a12*a14);
  a17=(a17+a12);
  a12=cos(a23);
  a14=sin(a13);
  a14=(a11*a14);
  a20=cos(a13);
  a20=(a8*a20);
  a14=(a14+a20);
  a14=(a3*a14);
  a14=(a5*a14);
  a20=(a9*a14);
  a12=(a12*a20);
  a17=(a17+a12);
  a12=sin(a23);
  a14=(a6*a14);
  a12=(a12*a14);
  a17=(a17-a12);
  a12=cos(a23);
  a14=sin(a13);
  a14=(a11*a14);
  a20=cos(a13);
  a20=(a8*a20);
  a14=(a14+a20);
  a14=(a16*a14);
  a14=(a5*a14);
  a20=(a2*a14);
  a12=(a12*a20);
  a17=(a17+a12);
  a12=sin(a23);
  a14=(a4*a14);
  a12=(a12*a14);
  a17=(a17+a12);
  a12=cos(a23);
  a14=cos(a13);
  a14=(a11*a14);
  a20=sin(a13);
  a20=(a8*a20);
  a14=(a14-a20);
  a14=(a16*a14);
  a14=(a5*a14);
  a20=(a4*a14);
  a12=(a12*a20);
  a17=(a17-a12);
  a12=sin(a23);
  a14=(a2*a14);
  a12=(a12*a14);
  a17=(a17+a12);
  if (res[0]!=0) res[0][11]=a17;
  a24=(a13-a24);
  a17=casadi_sq(a24);
  a12=(a17+a17);
  a14=casadi_sq(a17);
  a14=(a14+a14);
  a20=arg[3]? arg[3][11] : 0;
  a0=(a0*a20);
  a14=(a14*a0);
  a25=(a12*a14);
  a25=(a1*a25);
  a18=(a24+a24);
  a24=(a24+a24);
  a22=(a24+a24);
  a14=(a14*a22);
  a17=(a17+a17);
  a17=(a17*a24);
  a17=(a17+a17);
  a0=(a0*a17);
  a12=(a12*a0);
  a14=(a14+a12);
  a18=(a18*a14);
  a25=(a25+a18);
  a7=(a13+a7);
  a18=casadi_sq(a7);
  a14=(a18+a18);
  a12=casadi_sq(a18);
  a12=(a12+a12);
  a10=(a10*a20);
  a12=(a12*a10);
  a20=(a14*a12);
  a1=(a1*a20);
  a20=(a7+a7);
  a7=(a7+a7);
  a0=(a7+a7);
  a12=(a12*a0);
  a18=(a18+a18);
  a18=(a18*a7);
  a18=(a18+a18);
  a10=(a10*a18);
  a14=(a14*a10);
  a12=(a12+a14);
  a20=(a20*a12);
  a1=(a1+a20);
  a25=(a25+a1);
  a1=cos(a23);
  a1=(a9*a1);
  a20=sin(a23);
  a20=(a6*a20);
  a1=(a1-a20);
  a1=(a5*a1);
  a1=(a1*a3);
  a20=(a11*a1);
  a12=sin(a13);
  a20=(a20*a12);
  a25=(a25+a20);
  a1=(a8*a1);
  a20=cos(a13);
  a1=(a1*a20);
  a25=(a25+a1);
  a1=cos(a23);
  a6=(a6*a1);
  a1=sin(a23);
  a9=(a9*a1);
  a6=(a6+a9);
  a6=(a5*a6);
  a6=(a6*a3);
  a3=(a8*a6);
  a9=sin(a13);
  a3=(a3*a9);
  a25=(a25-a3);
  a6=(a11*a6);
  a3=cos(a13);
  a6=(a6*a3);
  a25=(a25+a6);
  a6=cos(a23);
  a6=(a4*a6);
  a3=sin(a23);
  a3=(a2*a3);
  a6=(a6-a3);
  a6=(a5*a6);
  a6=(a6*a16);
  a3=(a8*a6);
  a9=sin(a13);
  a3=(a3*a9);
  a25=(a25+a3);
  a6=(a11*a6);
  a3=cos(a13);
  a6=(a6*a3);
  a25=(a25-a6);
  a6=cos(a23);
  a2=(a2*a6);
  a23=sin(a23);
  a4=(a4*a23);
  a2=(a2+a4);
  a5=(a5*a2);
  a5=(a5*a16);
  a11=(a11*a5);
  a16=sin(a13);
  a11=(a11*a16);
  a25=(a25+a11);
  a8=(a8*a5);
  a13=cos(a13);
  a8=(a8*a13);
  a25=(a25+a8);
  if (res[0]!=0) res[0][12]=a25;
  return 0;
}

CASADI_SYMBOL_EXPORT int ntnu_leg_disp_cost_y_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ntnu_leg_disp_cost_y_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ntnu_leg_disp_cost_y_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ntnu_leg_disp_cost_y_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ntnu_leg_disp_cost_y_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ntnu_leg_disp_cost_y_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ntnu_leg_disp_cost_y_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void ntnu_leg_disp_cost_y_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ntnu_leg_disp_cost_y_hess_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int ntnu_leg_disp_cost_y_hess_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real ntnu_leg_disp_cost_y_hess_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ntnu_leg_disp_cost_y_hess_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ntnu_leg_disp_cost_y_hess_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ntnu_leg_disp_cost_y_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    case 4: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ntnu_leg_disp_cost_y_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ntnu_leg_disp_cost_y_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
