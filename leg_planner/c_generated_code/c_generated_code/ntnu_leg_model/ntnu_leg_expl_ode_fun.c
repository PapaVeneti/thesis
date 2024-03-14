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
  #define CASADI_PREFIX(ID) ntnu_leg_expl_ode_fun_ ## ID
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

/* ntnu_leg_expl_ode_fun:(i0[10],i1[3],i2[])->(o0[10]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a100, a101, a102, a103, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99;
  a0=arg[0]? arg[0][5] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a1=arg[0]? arg[0][6] : 0;
  if (res[0]!=0) res[0][1]=a1;
  a2=arg[0]? arg[0][7] : 0;
  if (res[0]!=0) res[0][2]=a2;
  a3=arg[0]? arg[0][8] : 0;
  if (res[0]!=0) res[0][3]=a3;
  a4=arg[0]? arg[0][9] : 0;
  if (res[0]!=0) res[0][4]=a4;
  a5=2.9009083883279074e-03;
  a6=2.;
  a7=arg[0]? arg[0][3] : 0;
  a8=(a6*a7);
  a9=1.4760149642585605e+00;
  a8=(a8+a9);
  a8=cos(a8);
  a5=(a5*a8);
  a8=3.9981650244275187e-03;
  a9=arg[0]? arg[0][1] : 0;
  a10=(a6*a9);
  a11=1.4803293471180217e+00;
  a10=(a10-a11);
  a10=cos(a10);
  a8=(a8*a10);
  a5=(a5+a8);
  a8=5.2012147170254165e-03;
  a10=(a6*a9);
  a11=arg[0]? arg[0][2] : 0;
  a12=(a6*a11);
  a10=(a10+a12);
  a12=9.8948941097606768e-01;
  a10=(a10+a12);
  a10=cos(a10);
  a8=(a8*a10);
  a5=(a5+a8);
  a8=7.4599841052066613e-03;
  a10=(a6*a9);
  a10=(a10+a11);
  a12=2.5318414183723831e-01;
  a10=(a10-a12);
  a10=cos(a10);
  a8=(a8*a10);
  a5=(a5+a8);
  a8=4.4756298428977456e-03;
  a10=(a6*a7);
  a12=arg[0]? arg[0][4] : 0;
  a10=(a10+a12);
  a13=1.7115435197523857e-01;
  a10=(a10+a13);
  a10=cos(a10);
  a8=(a8*a10);
  a5=(a5+a8);
  a8=7.4599841056468197e-03;
  a10=1.2242232045998356e+00;
  a13=(a11+a10);
  a13=cos(a13);
  a8=(a8*a13);
  a5=(a5+a8);
  a8=4.4756298431631314e-03;
  a13=1.3007983016414078e+00;
  a14=(a12-a13);
  a14=cos(a14);
  a8=(a8*a14);
  a5=(a5+a8);
  a8=3.0934333237104942e-03;
  a14=(a6*a7);
  a15=(a6*a12);
  a14=(a14+a15);
  a15=1.1577396279565728e+00;
  a14=(a14-a15);
  a14=cos(a14);
  a8=(a8*a14);
  a5=(a5+a8);
  a8=1.9468084116755956e-02;
  a5=(a5+a8);
  a8=casadi_sq(a5);
  a14=2.7568906440328294e-03;
  a15=8.3005746783315937e-01;
  a16=(a9+a15);
  a16=cos(a16);
  a16=(a14*a16);
  a17=2.4729321105027451e-03;
  a18=(a9+a11);
  a19=1.0852757698124977e+00;
  a18=(a18-a19);
  a18=cos(a18);
  a18=(a17*a18);
  a16=(a16-a18);
  a18=casadi_sq(a16);
  a8=(a8+a18);
  a18=-2.4729321105027451e-03;
  a20=(a9+a11);
  a20=(a20-a19);
  a20=cos(a20);
  a20=(a18*a20);
  a21=casadi_sq(a20);
  a8=(a8+a21);
  a21=1.4844172298633827e-03;
  a22=(a7+a12);
  a23=1.0059743509008685e+00;
  a22=(a22+a23);
  a22=cos(a22);
  a22=(a21*a22);
  a24=2.0308398734360934e-03;
  a25=8.3203853073985323e-01;
  a26=(a7-a25);
  a26=cos(a26);
  a26=(a24*a26);
  a22=(a22-a26);
  a26=casadi_sq(a22);
  a8=(a8+a26);
  a26=(a7+a12);
  a26=(a26+a23);
  a26=cos(a26);
  a26=(a21*a26);
  a27=casadi_sq(a26);
  a8=(a8+a27);
  a8=sqrt(a8);
  a5=(a5/a8);
  a27=(a7+a12);
  a27=(a27+a23);
  a27=cos(a27);
  a27=(a21*a27);
  a28=(a27*a5);
  a29=4.4756298430304389e-03;
  a30=(a12-a13);
  a30=cos(a30);
  a30=(a29*a30);
  a31=6.2093779111864643e-03;
  a30=(a30+a31);
  a22=(a22/a8);
  a32=(a30*a22);
  a28=(a28+a32);
  a26=(a26/a8);
  a32=(a31*a26);
  a28=(a28+a32);
  a32=(a28*a5);
  a27=(a27-a32);
  a15=(a9+a15);
  a15=cos(a15);
  a14=(a14*a15);
  a15=(a9+a11);
  a15=(a15-a19);
  a15=cos(a15);
  a17=(a17*a15);
  a14=(a14-a17);
  a17=(a14*a5);
  a15=1.4919968210853481e-02;
  a32=(a11+a10);
  a32=cos(a32);
  a15=(a15*a32);
  a32=1.8458484811692932e-02;
  a15=(a15+a32);
  a16=(a16/a8);
  a32=(a15*a16);
  a17=(a17+a32);
  a32=7.4599841054267405e-03;
  a33=(a11+a10);
  a33=cos(a33);
  a33=(a32*a33);
  a34=1.0447124665524521e-02;
  a33=(a33+a34);
  a20=(a20/a8);
  a35=(a33*a20);
  a17=(a17+a35);
  a35=(a17*a5);
  a14=(a14-a35);
  a35=casadi_sq(a14);
  a36=(a17*a16);
  a15=(a15-a36);
  a36=casadi_sq(a15);
  a35=(a35+a36);
  a36=(a17*a20);
  a33=(a33-a36);
  a36=casadi_sq(a33);
  a35=(a35+a36);
  a36=(a17*a22);
  a37=casadi_sq(a36);
  a35=(a35+a37);
  a37=(a17*a26);
  a38=casadi_sq(a37);
  a35=(a35+a38);
  a35=sqrt(a35);
  a14=(a14/a35);
  a38=(a27*a14);
  a39=(a28*a16);
  a15=(a15/a35);
  a40=(a39*a15);
  a38=(a38-a40);
  a40=(a28*a20);
  a33=(a33/a35);
  a41=(a40*a33);
  a38=(a38-a41);
  a41=(a28*a22);
  a30=(a30-a41);
  a36=(a36/a35);
  a41=(a30*a36);
  a38=(a38-a41);
  a41=(a28*a26);
  a41=(a31-a41);
  a37=(a37/a35);
  a42=(a41*a37);
  a38=(a38-a42);
  a42=(a38*a14);
  a27=(a27-a42);
  a42=(a9+a11);
  a42=(a42-a19);
  a42=cos(a42);
  a42=(a18*a42);
  a19=(a42*a5);
  a10=(a11+a10);
  a10=cos(a10);
  a32=(a32*a10);
  a32=(a32+a34);
  a10=(a32*a16);
  a19=(a19+a10);
  a10=(a34*a20);
  a19=(a19+a10);
  a10=(a19*a5);
  a42=(a42-a10);
  a10=(a42*a14);
  a43=(a19*a16);
  a32=(a32-a43);
  a43=(a32*a15);
  a10=(a10+a43);
  a43=(a19*a20);
  a34=(a34-a43);
  a43=(a34*a33);
  a10=(a10+a43);
  a43=(a19*a22);
  a44=(a43*a36);
  a10=(a10+a44);
  a44=(a19*a26);
  a45=(a44*a37);
  a10=(a10+a45);
  a45=(a10*a14);
  a42=(a42-a45);
  a45=casadi_sq(a42);
  a46=(a10*a15);
  a32=(a32-a46);
  a46=casadi_sq(a32);
  a45=(a45+a46);
  a46=(a10*a33);
  a34=(a34-a46);
  a46=casadi_sq(a34);
  a45=(a45+a46);
  a46=(a10*a36);
  a46=(a46-a43);
  a43=casadi_sq(a46);
  a45=(a45+a43);
  a43=(a10*a37);
  a43=(a43-a44);
  a44=casadi_sq(a43);
  a45=(a45+a44);
  a45=sqrt(a45);
  a42=(a42/a45);
  a44=(a27*a42);
  a47=(a38*a15);
  a39=(a39+a47);
  a32=(a32/a45);
  a47=(a39*a32);
  a44=(a44-a47);
  a47=(a38*a33);
  a40=(a40+a47);
  a34=(a34/a45);
  a47=(a40*a34);
  a44=(a44-a47);
  a47=(a38*a36);
  a30=(a30+a47);
  a46=(a46/a45);
  a47=(a30*a46);
  a44=(a44+a47);
  a47=(a38*a37);
  a41=(a41+a47);
  a43=(a43/a45);
  a47=(a41*a43);
  a44=(a44+a47);
  a47=(a44*a42);
  a27=(a27-a47);
  a47=(a7+a12);
  a47=(a47+a23);
  a47=cos(a47);
  a21=(a21*a47);
  a25=(a7-a25);
  a25=cos(a25);
  a24=(a24*a25);
  a21=(a21-a24);
  a24=(a21*a5);
  a25=2.3875653193042665e-03;
  a47=cos(a12);
  a47=(a25*a47);
  a23=8.6269682863306160e-03;
  a48=sin(a12);
  a48=(a23*a48);
  a47=(a47+a48);
  a48=1.2026282375593944e-02;
  a47=(a47+a48);
  a48=(a47*a22);
  a24=(a24+a48);
  a13=(a12-a13);
  a13=cos(a13);
  a13=(a29*a13);
  a13=(a13+a31);
  a31=(a13*a26);
  a24=(a24+a31);
  a31=(a24*a5);
  a21=(a21-a31);
  a31=(a21*a14);
  a48=(a24*a16);
  a49=(a48*a15);
  a31=(a31-a49);
  a49=(a24*a20);
  a50=(a49*a33);
  a31=(a31-a50);
  a50=(a24*a22);
  a47=(a47-a50);
  a50=(a47*a36);
  a31=(a31-a50);
  a50=(a24*a26);
  a13=(a13-a50);
  a50=(a13*a37);
  a31=(a31-a50);
  a50=(a31*a14);
  a21=(a21-a50);
  a50=(a21*a42);
  a51=(a31*a15);
  a48=(a48+a51);
  a51=(a48*a32);
  a50=(a50-a51);
  a51=(a31*a33);
  a49=(a49+a51);
  a51=(a49*a34);
  a50=(a50-a51);
  a51=(a31*a36);
  a47=(a47+a51);
  a51=(a47*a46);
  a50=(a50+a51);
  a51=(a31*a37);
  a13=(a13+a51);
  a51=(a13*a43);
  a50=(a50+a51);
  a51=(a50*a42);
  a21=(a21-a51);
  a51=casadi_sq(a21);
  a52=(a50*a32);
  a48=(a48+a52);
  a52=casadi_sq(a48);
  a51=(a51+a52);
  a52=(a50*a34);
  a49=(a49+a52);
  a52=casadi_sq(a49);
  a51=(a51+a52);
  a52=(a50*a46);
  a47=(a47-a52);
  a52=casadi_sq(a47);
  a51=(a51+a52);
  a52=(a50*a43);
  a13=(a13-a52);
  a52=casadi_sq(a13);
  a51=(a51+a52);
  a51=sqrt(a51);
  a21=(a21/a51);
  a52=(a27*a21);
  a53=(a44*a32);
  a39=(a39+a53);
  a48=(a48/a51);
  a53=(a39*a48);
  a52=(a52+a53);
  a53=(a44*a34);
  a40=(a40+a53);
  a49=(a49/a51);
  a53=(a40*a49);
  a52=(a52+a53);
  a53=(a44*a46);
  a30=(a30-a53);
  a47=(a47/a51);
  a53=(a30*a47);
  a52=(a52+a53);
  a53=(a44*a43);
  a41=(a41-a53);
  a13=(a13/a51);
  a53=(a41*a13);
  a52=(a52+a53);
  a53=(a52*a21);
  a27=(a27-a53);
  a53=casadi_sq(a27);
  a54=(a52*a48);
  a54=(a54-a39);
  a39=casadi_sq(a54);
  a53=(a53+a39);
  a39=(a52*a49);
  a39=(a39-a40);
  a40=casadi_sq(a39);
  a53=(a53+a40);
  a40=(a52*a47);
  a30=(a30-a40);
  a40=casadi_sq(a30);
  a53=(a53+a40);
  a40=(a52*a13);
  a41=(a41-a40);
  a40=casadi_sq(a41);
  a53=(a53+a40);
  a53=sqrt(a53);
  a27=(a27/a53);
  a27=(a27/a53);
  a40=(a28*a27);
  a5=(a5-a40);
  a40=(a52*a27);
  a21=(a21-a40);
  a21=(a21/a51);
  a40=(a24*a21);
  a5=(a5-a40);
  a40=(a44*a27);
  a42=(a42-a40);
  a40=(a50*a21);
  a42=(a42-a40);
  a42=(a42/a45);
  a40=(a19*a42);
  a5=(a5-a40);
  a40=(a38*a27);
  a14=(a14-a40);
  a40=(a31*a21);
  a14=(a14-a40);
  a40=(a10*a42);
  a14=(a14-a40);
  a14=(a14/a35);
  a40=(a17*a14);
  a5=(a5-a40);
  a5=(a5/a8);
  a40=arg[1]? arg[1][0] : 0;
  a55=7.9636302802704242e-03;
  a55=(a55*a1);
  a56=(a6*a9);
  a56=cos(a56);
  a55=(a55*a56);
  a56=5.7757759503467097e-03;
  a56=(a56*a3);
  a57=(a6*a7);
  a57=cos(a57);
  a56=(a56*a57);
  a55=(a55-a56);
  a56=8.6937858211019962e-03;
  a57=(a56*a1);
  a58=(a6*a9);
  a59=(a6*a11);
  a58=(a58+a59);
  a58=cos(a58);
  a57=(a57*a58);
  a55=(a55-a57);
  a56=(a56*a2);
  a57=(a6*a9);
  a58=(a6*a11);
  a57=(a57+a58);
  a57=cos(a57);
  a56=(a56*a57);
  a55=(a55-a56);
  a56=5.6665394786645697e-03;
  a57=(a56*a3);
  a58=(a6*a7);
  a59=(a6*a12);
  a58=(a58+a59);
  a58=cos(a58);
  a57=(a57*a58);
  a55=(a55+a57);
  a56=(a56*a4);
  a57=(a6*a7);
  a58=(a6*a12);
  a57=(a57+a58);
  a57=cos(a57);
  a56=(a56*a57);
  a55=(a55+a56);
  a56=5.7121472518846207e-03;
  a57=(a56*a1);
  a58=(a6*a9);
  a59=(a6*a11);
  a58=(a58+a59);
  a58=sin(a58);
  a57=(a57*a58);
  a55=(a55-a57);
  a56=(a56*a2);
  a57=(a6*a9);
  a58=(a6*a11);
  a57=(a57+a58);
  a57=sin(a57);
  a56=(a56*a57);
  a55=(a55-a56);
  a56=2.4834752363786690e-03;
  a57=(a56*a3);
  a58=(a6*a7);
  a59=(a6*a12);
  a58=(a58+a59);
  a58=sin(a58);
  a57=(a57*a58);
  a55=(a55-a57);
  a56=(a56*a4);
  a57=(a6*a7);
  a58=(a6*a12);
  a57=(a57+a58);
  a57=sin(a57);
  a56=(a56*a57);
  a55=(a55-a56);
  a56=7.0164303377484296e-03;
  a56=(a56*a2);
  a57=cos(a11);
  a56=(a56*a57);
  a55=(a55-a56);
  a56=4.3134841432931927e-03;
  a56=(a56*a4);
  a57=cos(a12);
  a56=(a56*a57);
  a55=(a55+a56);
  a56=2.5339826700328167e-03;
  a56=(a56*a2);
  a57=sin(a11);
  a56=(a56*a57);
  a55=(a55-a56);
  a56=1.1937826596875264e-03;
  a56=(a56*a4);
  a57=sin(a12);
  a56=(a56*a57);
  a55=(a55-a56);
  a56=3.7372708252844120e-03;
  a56=(a56*a1);
  a57=(a6*a9);
  a57=(a57+a11);
  a57=cos(a57);
  a56=(a56*a57);
  a55=(a55+a56);
  a56=1.8686354126422060e-03;
  a57=(a56*a2);
  a58=(a6*a9);
  a58=(a58+a11);
  a58=cos(a58);
  a57=(a57*a58);
  a55=(a55+a57);
  a57=1.5245780825778276e-03;
  a57=(a57*a3);
  a58=(a6*a7);
  a58=(a58+a12);
  a58=cos(a58);
  a57=(a57*a58);
  a55=(a55-a57);
  a57=1.4444315773972200e-02;
  a57=(a57*a1);
  a58=(a6*a9);
  a58=(a58+a11);
  a58=sin(a58);
  a57=(a57*a58);
  a55=(a55-a57);
  a57=7.2221578869860999e-03;
  a58=(a57*a2);
  a59=(a6*a9);
  a59=(a59+a11);
  a59=sin(a59);
  a58=(a58*a59);
  a55=(a55-a58);
  a58=8.8204711684054006e-03;
  a58=(a58*a3);
  a59=(a6*a7);
  a59=(a59+a12);
  a59=sin(a59);
  a58=(a58*a59);
  a55=(a55-a58);
  a58=4.4102355842027003e-03;
  a59=(a58*a4);
  a60=(a6*a7);
  a60=(a60+a12);
  a60=sin(a60);
  a59=(a59*a60);
  a55=(a55-a59);
  a59=(a55*a0);
  a60=1.1540403099071681e-03;
  a60=(a60*a1);
  a61=(a9+a11);
  a61=sin(a61);
  a60=(a60*a61);
  a61=4.3742812832578241e-03;
  a61=(a61*a2);
  a62=(a9+a11);
  a62=cos(a62);
  a61=(a61*a62);
  a60=(a60-a61);
  a61=2.1871406416289121e-03;
  a61=(a61*a1);
  a62=(a9+a11);
  a62=cos(a62);
  a61=(a61*a62);
  a60=(a60-a61);
  a61=2.3080806198143363e-03;
  a61=(a61*a2);
  a62=(a9+a11);
  a62=sin(a62);
  a61=(a61*a62);
  a60=(a60+a61);
  a61=2.0345030118671288e-03;
  a61=(a61*a1);
  a62=cos(a9);
  a61=(a61*a62);
  a60=(a60-a61);
  a61=1.8604417534175395e-03;
  a61=(a61*a1);
  a62=sin(a9);
  a61=(a61*a62);
  a60=(a60-a61);
  a61=(a60*a1);
  a59=(a59+a61);
  a18=(a18*a2);
  a61=(a9+a11);
  a62=4.8552055698239882e-01;
  a61=(a61+a62);
  a61=cos(a61);
  a18=(a18*a61);
  a61=(a18*a2);
  a59=(a59+a61);
  a61=1.3675067736613601e-03;
  a61=(a61*a3);
  a62=sin(a7);
  a61=(a61*a62);
  a62=2.5077266816566084e-03;
  a62=(a62*a4);
  a63=(a7+a12);
  a63=cos(a63);
  a62=(a62*a63);
  a61=(a61-a62);
  a62=1.5891145142401322e-03;
  a62=(a62*a4);
  a63=(a7+a12);
  a63=sin(a63);
  a62=(a62*a63);
  a61=(a61-a62);
  a62=1.5014112746107328e-03;
  a62=(a62*a3);
  a63=cos(a7);
  a62=(a62*a63);
  a61=(a61-a62);
  a62=1.2538633408283042e-03;
  a62=(a62*a3);
  a63=(a7+a12);
  a63=cos(a63);
  a62=(a62*a63);
  a61=(a61-a62);
  a62=(a61*a3);
  a59=(a59+a62);
  a62=-1.4844172298633827e-03;
  a62=(a62*a4);
  a63=(a7+a12);
  a64=5.6482197589402805e-01;
  a63=(a63-a64);
  a63=cos(a63);
  a62=(a62*a63);
  a63=(a62*a4);
  a59=(a59+a63);
  a59=(a40-a59);
  a63=5.0000000000000001e-03;
  a64=(a63*a0);
  a59=(a59-a64);
  a5=(a5*a59);
  a54=(a54/a53);
  a54=(a54/a53);
  a64=(a28*a54);
  a16=(a16-a64);
  a64=(a52*a54);
  a48=(a48+a64);
  a48=(a48/a51);
  a64=(a24*a48);
  a16=(a16+a64);
  a64=(a44*a54);
  a32=(a32-a64);
  a64=(a50*a48);
  a32=(a32+a64);
  a32=(a32/a45);
  a64=(a19*a32);
  a16=(a16-a64);
  a64=(a38*a54);
  a15=(a15-a64);
  a64=(a31*a48);
  a15=(a15+a64);
  a64=(a10*a32);
  a15=(a15-a64);
  a15=(a15/a35);
  a64=(a17*a15);
  a16=(a16-a64);
  a16=(a16/a8);
  a64=arg[1]? arg[1][1] : 0;
  a65=4.3468929105509981e-03;
  a66=(a65*a0);
  a67=(a6*a9);
  a68=(a6*a11);
  a67=(a67+a68);
  a67=cos(a67);
  a66=(a66*a67);
  a67=3.9818151401352121e-03;
  a67=(a67*a0);
  a68=(a6*a9);
  a68=cos(a68);
  a67=(a67*a68);
  a66=(a66-a67);
  a67=2.8560736259423103e-03;
  a68=(a67*a0);
  a69=(a6*a9);
  a70=(a6*a11);
  a69=(a69+a70);
  a69=sin(a69);
  a68=(a68*a69);
  a66=(a66+a68);
  a56=(a56*a0);
  a68=(a6*a9);
  a68=(a68+a11);
  a68=cos(a68);
  a56=(a56*a68);
  a66=(a66-a56);
  a57=(a57*a0);
  a56=(a6*a9);
  a56=(a56+a11);
  a56=sin(a56);
  a57=(a57*a56);
  a66=(a66+a57);
  a57=(a66*a0);
  a56=-1.4032860675082871e-02;
  a56=(a56*a2);
  a68=cos(a11);
  a56=(a56*a68);
  a68=5.0679653399161220e-03;
  a68=(a68*a2);
  a69=sin(a11);
  a68=(a68*a69);
  a56=(a56-a68);
  a68=(a56*a1);
  a57=(a57+a68);
  a68=-7.4599841054267405e-03;
  a68=(a68*a2);
  a69=3.4657312219506103e-01;
  a69=(a11-a69);
  a69=cos(a69);
  a68=(a68*a69);
  a69=(a68*a2);
  a57=(a57+a69);
  a57=(a64-a57);
  a69=1.3308156638417232e-01;
  a70=cos(a9);
  a70=(a69*a70);
  a71=1.2119940877652538e-01;
  a72=sin(a9);
  a72=(a71*a72);
  a70=(a70+a72);
  a72=2.9976999999999998e-01;
  a73=6.7333004875847435e-01;
  a74=cos(a9);
  a74=(a73*a74);
  a75=7.3934203546762400e-01;
  a76=sin(a9);
  a76=(a75*a76);
  a74=(a74-a76);
  a74=(a72*a74);
  a76=9.7030776516039308e-01;
  a77=cos(a11);
  a77=(a76*a77);
  a78=2.4187360515245046e-01;
  a79=sin(a11);
  a79=(a78*a79);
  a77=(a77+a79);
  a74=(a74*a77);
  a70=(a70+a74);
  a74=cos(a9);
  a74=(a75*a74);
  a77=sin(a9);
  a77=(a73*a77);
  a74=(a74+a77);
  a74=(a72*a74);
  a77=cos(a11);
  a77=(a78*a77);
  a79=sin(a11);
  a79=(a76*a79);
  a77=(a77-a79);
  a74=(a74*a77);
  a70=(a70+a74);
  a74=(a70*a15);
  a77=cos(a9);
  a77=(a73*a77);
  a79=sin(a9);
  a79=(a75*a79);
  a77=(a77-a79);
  a77=(a72*a77);
  a79=cos(a11);
  a79=(a76*a79);
  a80=sin(a11);
  a80=(a78*a80);
  a79=(a79+a80);
  a77=(a77*a79);
  a79=cos(a9);
  a79=(a75*a79);
  a80=sin(a9);
  a80=(a73*a80);
  a79=(a79+a80);
  a79=(a72*a79);
  a80=cos(a11);
  a80=(a78*a80);
  a81=sin(a11);
  a81=(a76*a81);
  a80=(a80-a81);
  a79=(a79*a80);
  a77=(a77+a79);
  a79=(a77*a32);
  a74=(a74+a79);
  a79=1.2083599892846777e-01;
  a80=sin(a7);
  a80=(a79*a80);
  a81=1.3341162378892907e-01;
  a82=cos(a7);
  a82=(a81*a82);
  a80=(a80-a82);
  a82=2.9929000000000000e-01;
  a83=6.7131110515815429e-01;
  a84=cos(a7);
  a84=(a83*a84);
  a85=7.4117568771627262e-01;
  a86=sin(a7);
  a86=(a85*a86);
  a84=(a84+a86);
  a84=(a82*a84);
  a86=9.6952880297333865e-01;
  a87=cos(a12);
  a87=(a86*a87);
  a88=2.4497734630999077e-01;
  a89=sin(a12);
  a89=(a88*a89);
  a87=(a87-a89);
  a84=(a84*a87);
  a80=(a80-a84);
  a84=cos(a7);
  a84=(a85*a84);
  a87=sin(a7);
  a87=(a83*a87);
  a84=(a84-a87);
  a84=(a82*a84);
  a87=cos(a12);
  a87=(a88*a87);
  a89=sin(a12);
  a89=(a86*a89);
  a87=(a87+a89);
  a84=(a84*a87);
  a80=(a80-a84);
  a84=(a80*a48);
  a74=(a74-a84);
  a84=-2.9929000000000000e-01;
  a87=cos(a7);
  a87=(a83*a87);
  a89=sin(a7);
  a89=(a85*a89);
  a87=(a87+a89);
  a84=(a84*a87);
  a87=cos(a12);
  a87=(a86*a87);
  a89=sin(a12);
  a89=(a88*a89);
  a87=(a87-a89);
  a84=(a84*a87);
  a87=cos(a7);
  a87=(a85*a87);
  a89=sin(a7);
  a89=(a83*a89);
  a87=(a87-a89);
  a87=(a82*a87);
  a89=cos(a12);
  a89=(a88*a89);
  a90=sin(a12);
  a90=(a86*a90);
  a89=(a89+a90);
  a87=(a87*a89);
  a84=(a84-a87);
  a87=(a84*a54);
  a74=(a74+a87);
  a87=1.2119940877325484e-01;
  a89=cos(a9);
  a89=(a87*a89);
  a90=1.3308156638776350e-01;
  a91=sin(a9);
  a90=(a90*a91);
  a89=(a89-a90);
  a90=6.7333004874030467e-01;
  a91=cos(a9);
  a91=(a90*a91);
  a92=7.3934203548757504e-01;
  a93=sin(a9);
  a93=(a92*a93);
  a91=(a91-a93);
  a91=(a72*a91);
  a93=cos(a11);
  a93=(a78*a93);
  a94=sin(a11);
  a94=(a76*a94);
  a93=(a93-a94);
  a91=(a91*a93);
  a89=(a89+a91);
  a91=cos(a9);
  a91=(a92*a91);
  a93=sin(a9);
  a93=(a90*a93);
  a91=(a91+a93);
  a91=(a72*a91);
  a93=cos(a11);
  a93=(a76*a93);
  a94=sin(a11);
  a94=(a78*a94);
  a93=(a93+a94);
  a91=(a91*a93);
  a89=(a89-a91);
  a91=(a74*a89);
  a39=(a39/a53);
  a39=(a39/a53);
  a93=(a38*a39);
  a33=(a33-a93);
  a93=(a52*a39);
  a49=(a49+a93);
  a49=(a49/a51);
  a93=(a31*a49);
  a33=(a33+a93);
  a93=(a44*a39);
  a34=(a34-a93);
  a93=(a50*a49);
  a34=(a34+a93);
  a34=(a34/a45);
  a93=(a10*a34);
  a33=(a33-a93);
  a33=(a33/a35);
  a93=(a70*a33);
  a94=(a77*a34);
  a93=(a93+a94);
  a94=(a80*a49);
  a93=(a93-a94);
  a94=(a84*a39);
  a93=(a93+a94);
  a94=cos(a9);
  a94=(a90*a94);
  a95=sin(a9);
  a95=(a92*a95);
  a94=(a94-a95);
  a94=(a72*a94);
  a95=cos(a11);
  a95=(a78*a95);
  a96=sin(a11);
  a96=(a76*a96);
  a95=(a95-a96);
  a94=(a94*a95);
  a95=cos(a9);
  a95=(a92*a95);
  a96=sin(a9);
  a96=(a90*a96);
  a95=(a95+a96);
  a95=(a72*a95);
  a96=cos(a11);
  a96=(a76*a96);
  a97=sin(a11);
  a97=(a78*a97);
  a96=(a96+a97);
  a95=(a95*a96);
  a94=(a94-a95);
  a95=(a93*a94);
  a91=(a91+a95);
  a30=(a30/a53);
  a30=(a30/a53);
  a95=(a44*a30);
  a46=(a46-a95);
  a95=(a52*a30);
  a47=(a47-a95);
  a47=(a47/a51);
  a95=(a50*a47);
  a46=(a46-a95);
  a46=(a46/a45);
  a95=(a77*a46);
  a96=(a38*a30);
  a36=(a36+a96);
  a96=(a31*a47);
  a36=(a36+a96);
  a96=(a10*a46);
  a36=(a36+a96);
  a36=(a36/a35);
  a96=(a70*a36);
  a95=(a95-a96);
  a96=(a80*a47);
  a95=(a95+a96);
  a96=(a84*a30);
  a95=(a95+a96);
  a96=1.2083599892520702e-01;
  a97=cos(a7);
  a97=(a96*a97);
  a98=1.3341162379252916e-01;
  a99=sin(a7);
  a99=(a98*a99);
  a97=(a97+a99);
  a99=6.7131110514003900e-01;
  a100=cos(a7);
  a100=(a99*a100);
  a101=7.4117568773627318e-01;
  a102=sin(a7);
  a102=(a101*a102);
  a100=(a100+a102);
  a100=(a82*a100);
  a102=cos(a12);
  a102=(a88*a102);
  a103=sin(a12);
  a103=(a86*a103);
  a102=(a102+a103);
  a100=(a100*a102);
  a97=(a97+a100);
  a100=cos(a7);
  a100=(a101*a100);
  a102=sin(a7);
  a102=(a99*a102);
  a100=(a100-a102);
  a100=(a82*a100);
  a102=cos(a12);
  a102=(a86*a102);
  a103=sin(a12);
  a103=(a88*a103);
  a102=(a102-a103);
  a100=(a100*a102);
  a97=(a97-a100);
  a100=(a95*a97);
  a91=(a91+a100);
  a41=(a41/a53);
  a41=(a41/a53);
  a44=(a44*a41);
  a43=(a43-a44);
  a52=(a52*a41);
  a13=(a13-a52);
  a13=(a13/a51);
  a50=(a50*a13);
  a43=(a43-a50);
  a43=(a43/a45);
  a45=(a77*a43);
  a38=(a38*a41);
  a37=(a37+a38);
  a31=(a31*a13);
  a37=(a37+a31);
  a10=(a10*a43);
  a37=(a37+a10);
  a37=(a37/a35);
  a35=(a70*a37);
  a45=(a45-a35);
  a35=(a80*a13);
  a45=(a45+a35);
  a35=(a84*a41);
  a45=(a45+a35);
  a35=cos(a7);
  a35=(a99*a35);
  a10=sin(a7);
  a10=(a101*a10);
  a35=(a35+a10);
  a35=(a82*a35);
  a10=cos(a12);
  a10=(a88*a10);
  a31=sin(a12);
  a31=(a86*a31);
  a10=(a10+a31);
  a35=(a35*a10);
  a10=cos(a7);
  a10=(a101*a10);
  a31=sin(a7);
  a31=(a99*a31);
  a10=(a10-a31);
  a10=(a82*a10);
  a31=cos(a12);
  a31=(a86*a31);
  a38=sin(a12);
  a38=(a88*a38);
  a31=(a31-a38);
  a10=(a10*a31);
  a35=(a35-a10);
  a10=(a45*a35);
  a91=(a91+a10);
  a74=(a74*a70);
  a93=(a93*a77);
  a74=(a74+a93);
  a95=(a95*a80);
  a74=(a74+a95);
  a45=(a45*a84);
  a74=(a74+a45);
  a45=(a89*a15);
  a95=(a94*a32);
  a45=(a45+a95);
  a95=(a97*a48);
  a45=(a45-a95);
  a95=(a35*a54);
  a45=(a45+a95);
  a95=(a45*a89);
  a93=(a89*a33);
  a10=(a94*a34);
  a93=(a93+a10);
  a10=(a97*a49);
  a93=(a93-a10);
  a10=(a35*a39);
  a93=(a93+a10);
  a10=(a93*a94);
  a95=(a95+a10);
  a10=(a94*a46);
  a31=(a89*a36);
  a10=(a10-a31);
  a31=(a97*a47);
  a10=(a10+a31);
  a31=(a35*a30);
  a10=(a10+a31);
  a31=(a10*a97);
  a95=(a95+a31);
  a31=(a94*a43);
  a38=(a89*a37);
  a31=(a31-a38);
  a38=(a97*a13);
  a31=(a31+a38);
  a38=(a35*a41);
  a31=(a31+a38);
  a38=(a31*a35);
  a95=(a95+a38);
  a38=(a74*a95);
  a45=(a45*a70);
  a93=(a93*a77);
  a45=(a45+a93);
  a10=(a10*a80);
  a45=(a45+a10);
  a31=(a31*a84);
  a45=(a45+a31);
  a31=(a91*a45);
  a38=(a38-a31);
  a91=(a91/a38);
  a55=(a55*a0);
  a60=(a60*a1);
  a55=(a55+a60);
  a18=(a18*a2);
  a55=(a55+a18);
  a61=(a61*a3);
  a55=(a55+a61);
  a62=(a62*a4);
  a55=(a55+a62);
  a40=(a40-a55);
  a55=(a63*a0);
  a40=(a40-a55);
  a55=(a14*a40);
  a66=(a66*a0);
  a56=(a56*a1);
  a66=(a66+a56);
  a68=(a68*a2);
  a66=(a66+a68);
  a64=(a64-a66);
  a66=(a63*a1);
  a64=(a64-a66);
  a66=(a15*a64);
  a55=(a55+a66);
  a65=(a65*a0);
  a66=(a6*a9);
  a68=(a6*a11);
  a66=(a66+a68);
  a66=cos(a66);
  a65=(a65*a66);
  a67=(a67*a0);
  a66=(a6*a9);
  a68=(a6*a11);
  a66=(a66+a68);
  a66=sin(a66);
  a67=(a67*a66);
  a65=(a65+a67);
  a67=3.5082151688742148e-03;
  a67=(a67*a0);
  a66=cos(a11);
  a67=(a67*a66);
  a65=(a65+a67);
  a67=1.2669913350164084e-03;
  a67=(a67*a0);
  a66=sin(a11);
  a67=(a67*a66);
  a65=(a65+a67);
  a67=3.6110789434930500e-03;
  a67=(a67*a0);
  a66=(a6*a9);
  a66=(a66+a11);
  a66=sin(a66);
  a67=(a67*a66);
  a65=(a65+a67);
  a67=(a65*a0);
  a66=7.0164303375414355e-03;
  a66=(a66*a1);
  a68=cos(a11);
  a66=(a66*a68);
  a68=2.5339826699580610e-03;
  a68=(a68*a1);
  a56=sin(a11);
  a68=(a68*a56);
  a66=(a66+a68);
  a68=(a66*a1);
  a67=(a67+a68);
  a68=(a63*a2);
  a67=(a67+a68);
  a68=(a33*a67);
  a55=(a55-a68);
  a68=arg[1]? arg[1][2] : 0;
  a56=2.8878879751733548e-03;
  a56=(a56*a0);
  a62=(a6*a7);
  a62=cos(a62);
  a56=(a56*a62);
  a62=2.8332697393322849e-03;
  a61=(a62*a0);
  a18=(a6*a7);
  a60=(a6*a12);
  a18=(a18+a60);
  a18=cos(a18);
  a61=(a61*a18);
  a56=(a56-a61);
  a61=1.2417376181893345e-03;
  a18=(a61*a0);
  a60=(a6*a7);
  a31=(a6*a12);
  a60=(a60+a31);
  a60=sin(a60);
  a18=(a18*a60);
  a56=(a56+a18);
  a58=(a58*a0);
  a18=(a6*a7);
  a18=(a18+a12);
  a18=sin(a18);
  a58=(a58*a18);
  a56=(a56+a58);
  a58=(a56*a0);
  a23=(a23*a4);
  a18=cos(a12);
  a23=(a23*a18);
  a25=(a25*a4);
  a18=sin(a12);
  a25=(a25*a18);
  a23=(a23-a25);
  a25=(a23*a3);
  a58=(a58+a25);
  a29=(a29*a4);
  a25=2.6999802515348881e-01;
  a25=(a12+a25);
  a25=cos(a25);
  a29=(a29*a25);
  a25=(a29*a4);
  a58=(a58+a25);
  a58=(a68-a58);
  a25=(a63*a3);
  a58=(a58-a25);
  a25=(a36*a58);
  a55=(a55-a25);
  a61=(a61*a0);
  a25=(a6*a7);
  a18=(a6*a12);
  a25=(a25+a18);
  a25=sin(a25);
  a61=(a61*a25);
  a62=(a62*a0);
  a25=(a6*a7);
  a18=(a6*a12);
  a25=(a25+a18);
  a25=cos(a25);
  a62=(a62*a25);
  a61=(a61-a62);
  a62=2.1567420716465963e-03;
  a62=(a62*a0);
  a25=cos(a12);
  a62=(a62*a25);
  a61=(a61-a62);
  a62=2.2051177921013502e-03;
  a62=(a62*a0);
  a6=(a6*a7);
  a6=(a6+a12);
  a6=sin(a6);
  a62=(a62*a6);
  a61=(a61+a62);
  a62=(a61*a0);
  a6=4.3134841431653080e-03;
  a25=cos(a12);
  a6=(a6*a25);
  a25=1.1937826596521333e-03;
  a18=sin(a12);
  a25=(a25*a18);
  a6=(a6-a25);
  a6=(a3*a6);
  a25=(a6*a3);
  a62=(a62-a25);
  a25=(a63*a4);
  a62=(a62+a25);
  a25=(a37*a62);
  a55=(a55+a25);
  a25=(a89*a55);
  a18=(a42*a40);
  a60=(a32*a64);
  a18=(a18+a60);
  a60=(a34*a67);
  a18=(a18-a60);
  a60=(a46*a58);
  a18=(a18+a60);
  a60=(a43*a62);
  a18=(a18-a60);
  a60=(a94*a18);
  a25=(a25+a60);
  a60=(a21*a40);
  a31=(a48*a64);
  a60=(a60-a31);
  a31=(a49*a67);
  a60=(a60+a31);
  a31=(a47*a58);
  a60=(a60+a31);
  a31=(a13*a62);
  a60=(a60-a31);
  a31=(a97*a60);
  a25=(a25+a31);
  a40=(a27*a40);
  a64=(a54*a64);
  a40=(a40+a64);
  a67=(a39*a67);
  a40=(a40-a67);
  a58=(a30*a58);
  a40=(a40+a58);
  a62=(a41*a62);
  a40=(a40-a62);
  a62=(a35*a40);
  a25=(a25+a62);
  a62=-1.3308156638776350e-01;
  a62=(a62*a1);
  a58=cos(a9);
  a62=(a62*a58);
  a87=(a87*a1);
  a58=sin(a9);
  a87=(a87*a58);
  a62=(a62-a87);
  a87=cos(a11);
  a87=(a78*a87);
  a58=sin(a11);
  a58=(a76*a58);
  a87=(a87-a58);
  a87=(a72*a87);
  a58=(a92*a1);
  a67=cos(a9);
  a58=(a58*a67);
  a67=(a90*a1);
  a64=sin(a9);
  a67=(a67*a64);
  a58=(a58+a67);
  a87=(a87*a58);
  a62=(a62-a87);
  a87=cos(a11);
  a87=(a76*a87);
  a58=sin(a11);
  a58=(a78*a58);
  a87=(a87+a58);
  a87=(a72*a87);
  a58=(a90*a1);
  a67=cos(a9);
  a58=(a58*a67);
  a67=(a92*a1);
  a64=sin(a9);
  a67=(a67*a64);
  a58=(a58-a67);
  a87=(a87*a58);
  a62=(a62-a87);
  a87=(a78*a2);
  a58=cos(a11);
  a87=(a87*a58);
  a58=(a76*a2);
  a67=sin(a11);
  a58=(a58*a67);
  a87=(a87-a58);
  a87=(a72*a87);
  a58=cos(a9);
  a58=(a92*a58);
  a67=sin(a9);
  a67=(a90*a67);
  a58=(a58+a67);
  a87=(a87*a58);
  a62=(a62-a87);
  a87=(a76*a2);
  a58=cos(a11);
  a87=(a87*a58);
  a58=(a78*a2);
  a67=sin(a11);
  a58=(a58*a67);
  a87=(a87+a58);
  a87=(a72*a87);
  a58=cos(a9);
  a58=(a90*a58);
  a67=sin(a9);
  a67=(a92*a67);
  a58=(a58-a67);
  a87=(a87*a58);
  a62=(a62-a87);
  a62=(a62*a1);
  a87=-2.9976999999999998e-01;
  a58=cos(a11);
  a58=(a78*a58);
  a67=sin(a11);
  a67=(a76*a67);
  a58=(a58-a67);
  a87=(a87*a58);
  a58=(a92*a1);
  a67=cos(a9);
  a58=(a58*a67);
  a67=(a90*a1);
  a64=sin(a9);
  a67=(a67*a64);
  a58=(a58+a67);
  a87=(a87*a58);
  a58=cos(a11);
  a58=(a76*a58);
  a67=sin(a11);
  a67=(a78*a67);
  a58=(a58+a67);
  a58=(a72*a58);
  a67=(a90*a1);
  a64=cos(a9);
  a67=(a67*a64);
  a64=(a92*a1);
  a31=sin(a9);
  a64=(a64*a31);
  a67=(a67-a64);
  a58=(a58*a67);
  a87=(a87-a58);
  a58=(a78*a2);
  a67=cos(a11);
  a58=(a58*a67);
  a67=(a76*a2);
  a64=sin(a11);
  a67=(a67*a64);
  a58=(a58-a67);
  a58=(a72*a58);
  a67=cos(a9);
  a67=(a92*a67);
  a64=sin(a9);
  a64=(a90*a64);
  a67=(a67+a64);
  a58=(a58*a67);
  a87=(a87-a58);
  a58=(a76*a2);
  a67=cos(a11);
  a58=(a58*a67);
  a67=(a78*a2);
  a64=sin(a11);
  a67=(a67*a64);
  a58=(a58+a67);
  a58=(a72*a58);
  a67=cos(a9);
  a90=(a90*a67);
  a67=sin(a9);
  a92=(a92*a67);
  a90=(a90-a92);
  a58=(a58*a90);
  a87=(a87-a58);
  a87=(a87*a2);
  a62=(a62+a87);
  a98=(a98*a3);
  a87=cos(a7);
  a98=(a98*a87);
  a96=(a96*a3);
  a87=sin(a7);
  a96=(a96*a87);
  a98=(a98-a96);
  a96=cos(a12);
  a96=(a86*a96);
  a87=sin(a12);
  a87=(a88*a87);
  a96=(a96-a87);
  a96=(a82*a96);
  a87=(a99*a3);
  a58=cos(a7);
  a87=(a87*a58);
  a58=(a101*a3);
  a90=sin(a7);
  a58=(a58*a90);
  a87=(a87+a58);
  a96=(a96*a87);
  a98=(a98+a96);
  a96=cos(a12);
  a96=(a88*a96);
  a87=sin(a12);
  a87=(a86*a87);
  a96=(a96+a87);
  a96=(a82*a96);
  a87=(a101*a3);
  a58=cos(a7);
  a87=(a87*a58);
  a58=(a99*a3);
  a90=sin(a7);
  a58=(a58*a90);
  a87=(a87-a58);
  a96=(a96*a87);
  a98=(a98+a96);
  a96=(a86*a4);
  a87=cos(a12);
  a96=(a96*a87);
  a87=(a88*a4);
  a58=sin(a12);
  a87=(a87*a58);
  a96=(a96-a87);
  a96=(a82*a96);
  a87=cos(a7);
  a87=(a99*a87);
  a58=sin(a7);
  a58=(a101*a58);
  a87=(a87+a58);
  a96=(a96*a87);
  a98=(a98+a96);
  a96=(a88*a4);
  a87=cos(a12);
  a96=(a96*a87);
  a87=(a86*a4);
  a58=sin(a12);
  a87=(a87*a58);
  a96=(a96+a87);
  a96=(a82*a96);
  a87=cos(a7);
  a87=(a101*a87);
  a58=sin(a7);
  a58=(a99*a58);
  a87=(a87-a58);
  a96=(a96*a87);
  a98=(a98+a96);
  a98=(a98*a3);
  a62=(a62+a98);
  a98=cos(a12);
  a98=(a86*a98);
  a96=sin(a12);
  a96=(a88*a96);
  a98=(a98-a96);
  a98=(a82*a98);
  a96=(a99*a3);
  a87=cos(a7);
  a96=(a96*a87);
  a87=(a101*a3);
  a58=sin(a7);
  a87=(a87*a58);
  a96=(a96+a87);
  a98=(a98*a96);
  a96=cos(a12);
  a96=(a88*a96);
  a87=sin(a12);
  a87=(a86*a87);
  a96=(a96+a87);
  a96=(a82*a96);
  a87=(a101*a3);
  a58=cos(a7);
  a87=(a87*a58);
  a58=(a99*a3);
  a90=sin(a7);
  a58=(a58*a90);
  a87=(a87-a58);
  a96=(a96*a87);
  a98=(a98+a96);
  a96=(a86*a4);
  a87=cos(a12);
  a96=(a96*a87);
  a87=(a88*a4);
  a58=sin(a12);
  a87=(a87*a58);
  a96=(a96-a87);
  a96=(a82*a96);
  a87=cos(a7);
  a87=(a99*a87);
  a58=sin(a7);
  a58=(a101*a58);
  a87=(a87+a58);
  a96=(a96*a87);
  a98=(a98+a96);
  a96=(a88*a4);
  a87=cos(a12);
  a96=(a96*a87);
  a87=(a86*a4);
  a58=sin(a12);
  a87=(a87*a58);
  a96=(a96+a87);
  a96=(a82*a96);
  a87=cos(a7);
  a101=(a101*a87);
  a87=sin(a7);
  a99=(a99*a87);
  a101=(a101-a99);
  a96=(a96*a101);
  a98=(a98+a96);
  a98=(a98*a4);
  a62=(a62+a98);
  a25=(a25+a62);
  a91=(a91*a25);
  a95=(a95/a38);
  a55=(a70*a55);
  a18=(a77*a18);
  a55=(a55+a18);
  a60=(a80*a60);
  a55=(a55+a60);
  a40=(a84*a40);
  a55=(a55+a40);
  a71=(a71*a1);
  a40=cos(a9);
  a71=(a71*a40);
  a69=(a69*a1);
  a40=sin(a9);
  a69=(a69*a40);
  a71=(a71-a69);
  a69=cos(a11);
  a69=(a78*a69);
  a40=sin(a11);
  a40=(a76*a40);
  a69=(a69-a40);
  a69=(a72*a69);
  a40=(a73*a1);
  a60=cos(a9);
  a40=(a40*a60);
  a60=(a75*a1);
  a18=sin(a9);
  a60=(a60*a18);
  a40=(a40-a60);
  a69=(a69*a40);
  a71=(a71+a69);
  a69=cos(a11);
  a69=(a76*a69);
  a40=sin(a11);
  a40=(a78*a40);
  a69=(a69+a40);
  a69=(a72*a69);
  a40=(a75*a1);
  a60=cos(a9);
  a40=(a40*a60);
  a60=(a73*a1);
  a18=sin(a9);
  a60=(a60*a18);
  a40=(a40+a60);
  a69=(a69*a40);
  a71=(a71-a69);
  a69=(a78*a2);
  a40=cos(a11);
  a69=(a69*a40);
  a40=(a76*a2);
  a60=sin(a11);
  a40=(a40*a60);
  a69=(a69-a40);
  a69=(a72*a69);
  a40=cos(a9);
  a40=(a73*a40);
  a60=sin(a9);
  a60=(a75*a60);
  a40=(a40-a60);
  a69=(a69*a40);
  a71=(a71+a69);
  a69=(a76*a2);
  a40=cos(a11);
  a69=(a69*a40);
  a40=(a78*a2);
  a60=sin(a11);
  a40=(a40*a60);
  a69=(a69+a40);
  a69=(a72*a69);
  a40=cos(a9);
  a40=(a75*a40);
  a60=sin(a9);
  a60=(a73*a60);
  a40=(a40+a60);
  a69=(a69*a40);
  a71=(a71-a69);
  a71=(a71*a1);
  a69=cos(a11);
  a69=(a78*a69);
  a40=sin(a11);
  a40=(a76*a40);
  a69=(a69-a40);
  a69=(a72*a69);
  a40=(a73*a1);
  a60=cos(a9);
  a40=(a40*a60);
  a60=(a75*a1);
  a18=sin(a9);
  a60=(a60*a18);
  a40=(a40-a60);
  a69=(a69*a40);
  a40=cos(a11);
  a40=(a76*a40);
  a60=sin(a11);
  a60=(a78*a60);
  a40=(a40+a60);
  a40=(a72*a40);
  a60=(a75*a1);
  a18=cos(a9);
  a60=(a60*a18);
  a18=(a73*a1);
  a62=sin(a9);
  a18=(a18*a62);
  a60=(a60+a18);
  a40=(a40*a60);
  a69=(a69-a40);
  a40=(a78*a2);
  a60=cos(a11);
  a40=(a40*a60);
  a60=(a76*a2);
  a18=sin(a11);
  a60=(a60*a18);
  a40=(a40-a60);
  a40=(a72*a40);
  a60=cos(a9);
  a60=(a73*a60);
  a18=sin(a9);
  a18=(a75*a18);
  a60=(a60-a18);
  a40=(a40*a60);
  a69=(a69+a40);
  a76=(a76*a2);
  a40=cos(a11);
  a76=(a76*a40);
  a78=(a78*a2);
  a11=sin(a11);
  a78=(a78*a11);
  a76=(a76+a78);
  a72=(a72*a76);
  a76=cos(a9);
  a75=(a75*a76);
  a9=sin(a9);
  a73=(a73*a9);
  a75=(a75+a73);
  a72=(a72*a75);
  a69=(a69-a72);
  a69=(a69*a2);
  a71=(a71+a69);
  a79=(a79*a3);
  a69=cos(a7);
  a79=(a79*a69);
  a81=(a81*a3);
  a69=sin(a7);
  a81=(a81*a69);
  a79=(a79+a81);
  a81=cos(a12);
  a81=(a86*a81);
  a69=sin(a12);
  a69=(a88*a69);
  a81=(a81-a69);
  a81=(a82*a81);
  a69=(a85*a3);
  a72=cos(a7);
  a69=(a69*a72);
  a72=(a83*a3);
  a75=sin(a7);
  a72=(a72*a75);
  a69=(a69-a72);
  a81=(a81*a69);
  a79=(a79-a81);
  a81=cos(a12);
  a81=(a88*a81);
  a69=sin(a12);
  a69=(a86*a69);
  a81=(a81+a69);
  a81=(a82*a81);
  a69=(a83*a3);
  a72=cos(a7);
  a69=(a69*a72);
  a72=(a85*a3);
  a75=sin(a7);
  a72=(a72*a75);
  a69=(a69+a72);
  a81=(a81*a69);
  a79=(a79+a81);
  a81=(a86*a4);
  a69=cos(a12);
  a81=(a81*a69);
  a69=(a88*a4);
  a72=sin(a12);
  a69=(a69*a72);
  a81=(a81-a69);
  a81=(a82*a81);
  a69=cos(a7);
  a69=(a85*a69);
  a72=sin(a7);
  a72=(a83*a72);
  a69=(a69-a72);
  a81=(a81*a69);
  a79=(a79-a81);
  a81=(a88*a4);
  a69=cos(a12);
  a81=(a81*a69);
  a69=(a86*a4);
  a72=sin(a12);
  a69=(a69*a72);
  a81=(a81+a69);
  a81=(a82*a81);
  a69=cos(a7);
  a69=(a83*a69);
  a72=sin(a7);
  a72=(a85*a72);
  a69=(a69+a72);
  a81=(a81*a69);
  a79=(a79+a81);
  a79=(a79*a3);
  a71=(a71+a79);
  a79=cos(a12);
  a79=(a88*a79);
  a81=sin(a12);
  a81=(a86*a81);
  a79=(a79+a81);
  a79=(a82*a79);
  a81=(a83*a3);
  a69=cos(a7);
  a81=(a81*a69);
  a69=(a85*a3);
  a72=sin(a7);
  a69=(a69*a72);
  a81=(a81+a69);
  a79=(a79*a81);
  a81=cos(a12);
  a81=(a86*a81);
  a69=sin(a12);
  a69=(a88*a69);
  a81=(a81-a69);
  a81=(a82*a81);
  a69=(a85*a3);
  a72=cos(a7);
  a69=(a69*a72);
  a72=(a83*a3);
  a75=sin(a7);
  a72=(a72*a75);
  a69=(a69-a72);
  a81=(a81*a69);
  a79=(a79-a81);
  a81=(a86*a4);
  a69=cos(a12);
  a81=(a81*a69);
  a69=(a88*a4);
  a72=sin(a12);
  a69=(a69*a72);
  a81=(a81-a69);
  a81=(a82*a81);
  a69=cos(a7);
  a69=(a85*a69);
  a72=sin(a7);
  a72=(a83*a72);
  a69=(a69-a72);
  a81=(a81*a69);
  a79=(a79-a81);
  a88=(a88*a4);
  a81=cos(a12);
  a88=(a88*a81);
  a86=(a86*a4);
  a12=sin(a12);
  a86=(a86*a12);
  a88=(a88+a86);
  a82=(a82*a88);
  a88=cos(a7);
  a83=(a83*a88);
  a7=sin(a7);
  a85=(a85*a7);
  a83=(a83+a85);
  a82=(a82*a83);
  a79=(a79+a82);
  a79=(a79*a4);
  a71=(a71+a79);
  a55=(a55+a71);
  a95=(a95*a55);
  a91=(a91-a95);
  a70=(a70*a91);
  a45=(a45/a38);
  a45=(a45*a55);
  a74=(a74/a38);
  a74=(a74*a25);
  a45=(a45-a74);
  a89=(a89*a45);
  a70=(a70+a89);
  a57=(a57+a70);
  a70=(a63*a1);
  a57=(a57-a70);
  a16=(a16*a57);
  a5=(a5+a16);
  a16=(a28*a39);
  a20=(a20-a16);
  a16=(a24*a49);
  a20=(a20+a16);
  a16=(a19*a34);
  a20=(a20-a16);
  a16=(a17*a33);
  a20=(a20-a16);
  a20=(a20/a8);
  a77=(a77*a91);
  a94=(a94*a45);
  a77=(a77+a94);
  a65=(a65*a0);
  a66=(a66*a1);
  a65=(a65+a66);
  a77=(a77-a65);
  a2=(a63*a2);
  a77=(a77-a2);
  a20=(a20*a77);
  a5=(a5+a20);
  a20=(a28*a30);
  a22=(a22-a20);
  a20=(a24*a47);
  a22=(a22-a20);
  a20=(a19*a46);
  a22=(a22-a20);
  a20=(a17*a36);
  a22=(a22+a20);
  a22=(a22/a8);
  a56=(a56*a0);
  a23=(a23*a3);
  a56=(a56+a23);
  a29=(a29*a4);
  a56=(a56+a29);
  a68=(a68-a56);
  a80=(a80*a91);
  a97=(a97*a45);
  a80=(a80+a97);
  a68=(a68+a80);
  a80=(a63*a3);
  a68=(a68-a80);
  a22=(a22*a68);
  a5=(a5+a22);
  a28=(a28*a41);
  a26=(a26-a28);
  a24=(a24*a13);
  a26=(a26-a24);
  a19=(a19*a43);
  a26=(a26-a19);
  a17=(a17*a37);
  a26=(a26+a17);
  a26=(a26/a8);
  a84=(a84*a91);
  a35=(a35*a45);
  a84=(a84+a35);
  a61=(a61*a0);
  a6=(a6*a3);
  a61=(a61-a6);
  a84=(a84-a61);
  a63=(a63*a4);
  a84=(a84-a63);
  a26=(a26*a84);
  a5=(a5+a26);
  if (res[0]!=0) res[0][5]=a5;
  a14=(a14*a59);
  a15=(a15*a57);
  a14=(a14+a15);
  a33=(a33*a77);
  a14=(a14+a33);
  a36=(a36*a68);
  a14=(a14-a36);
  a37=(a37*a84);
  a14=(a14-a37);
  if (res[0]!=0) res[0][6]=a14;
  a42=(a42*a59);
  a32=(a32*a57);
  a42=(a42+a32);
  a34=(a34*a77);
  a42=(a42+a34);
  a46=(a46*a68);
  a42=(a42+a46);
  a43=(a43*a84);
  a42=(a42+a43);
  if (res[0]!=0) res[0][7]=a42;
  a21=(a21*a59);
  a48=(a48*a57);
  a21=(a21-a48);
  a49=(a49*a77);
  a21=(a21-a49);
  a47=(a47*a68);
  a21=(a21+a47);
  a13=(a13*a84);
  a21=(a21+a13);
  if (res[0]!=0) res[0][8]=a21;
  a27=(a27*a59);
  a54=(a54*a57);
  a27=(a27+a54);
  a39=(a39*a77);
  a27=(a27+a39);
  a30=(a30*a68);
  a27=(a27+a30);
  a41=(a41*a84);
  a27=(a27+a41);
  if (res[0]!=0) res[0][9]=a27;
  return 0;
}

CASADI_SYMBOL_EXPORT int ntnu_leg_expl_ode_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ntnu_leg_expl_ode_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ntnu_leg_expl_ode_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ntnu_leg_expl_ode_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ntnu_leg_expl_ode_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ntnu_leg_expl_ode_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ntnu_leg_expl_ode_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void ntnu_leg_expl_ode_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ntnu_leg_expl_ode_fun_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int ntnu_leg_expl_ode_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real ntnu_leg_expl_ode_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ntnu_leg_expl_ode_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ntnu_leg_expl_ode_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ntnu_leg_expl_ode_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ntnu_leg_expl_ode_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ntnu_leg_expl_ode_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
