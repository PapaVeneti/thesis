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
  #define CASADI_PREFIX(ID) ntnu_leg_cost_y_hess_ ## ID
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
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[12] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s4[37] = {13, 13, 0, 0, 1, 2, 5, 9, 13, 17, 21, 21, 21, 21, 21, 21, 3, 3, 1, 2, 3, 4, 5, 6, 7, 4, 5, 6, 7, 4, 5, 6, 7, 4, 5, 6, 7};

/* ntnu_leg_cost_y_hess:(i0[10],i1[3],i2[],i3[8],i4[])->(o0[13x13,21nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a100, a101, a102, a103, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99;
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
  a1=arg[1]? arg[1][1] : 0;
  a4=arg[1]? arg[1][2] : 0;
  a1=(a1+a4);
  a2=(a1*a2);
  a4=cos(a0);
  a2=(a2*a4);
  a4=5.9999999999999998e-01;
  a3=arg[3]? arg[3][5] : 0;
  a6=(a4*a3);
  a7=1.0000000000000000e-03;
  a8=(a7+a0);
  a9=2.;
  a8=(a8+a9);
  a6=(a6/a8);
  a6=(a6/a8);
  a8=(a4*a3);
  a9=(a7-a0);
  a10=1.3100000000000001e+00;
  a9=(a9+a10);
  a8=(a8/a9);
  a8=(a8/a9);
  a6=(a6+a8);
  a2=(a2-a6);
  a1=(a1*a5);
  a0=sin(a0);
  a1=(a1*a0);
  a2=(a2-a1);
  if (res[0]!=0) res[0][4]=a2;
  a2=arg[0]? arg[0][1] : 0;
  a1=cos(a2);
  a0=6.7333004874030467e-01;
  a5=2.9976999999999998e-01;
  a6=2.4187360515245046e-01;
  a8=arg[0]? arg[0][2] : 0;
  a9=cos(a8);
  a9=(a6*a9);
  a10=9.7030776516039308e-01;
  a11=sin(a8);
  a11=(a10*a11);
  a9=(a9-a11);
  a11=arg[3]? arg[3][7] : 0;
  a12=1.2119940877325484e-01;
  a13=cos(a2);
  a13=(a12*a13);
  a14=1.3308156638776350e-01;
  a15=sin(a2);
  a15=(a14*a15);
  a13=(a13-a15);
  a15=cos(a8);
  a15=(a10*a15);
  a16=sin(a8);
  a16=(a6*a16);
  a15=(a15+a16);
  a16=sin(a2);
  a16=(a0*a16);
  a17=7.3934203548757504e-01;
  a18=cos(a2);
  a18=(a17*a18);
  a16=(a16+a18);
  a16=(a5*a16);
  a18=(a15*a16);
  a13=(a13-a18);
  a18=cos(a2);
  a18=(a0*a18);
  a19=sin(a2);
  a19=(a17*a19);
  a18=(a18-a19);
  a18=(a5*a18);
  a19=(a9*a18);
  a13=(a13+a19);
  a13=(a13+a13);
  a13=(a11*a13);
  a19=(a9*a13);
  a19=(a5*a19);
  a20=(a0*a19);
  a20=(a1*a20);
  a21=cos(a2);
  a21=(a14*a21);
  a22=1.3341162379252916e-01;
  a23=arg[0]? arg[0][3] : 0;
  a24=cos(a23);
  a24=(a22*a24);
  a21=(a21-a24);
  a24=sin(a2);
  a24=(a12*a24);
  a21=(a21+a24);
  a24=1.2083599892520702e-01;
  a25=sin(a23);
  a25=(a24*a25);
  a21=(a21+a25);
  a25=cos(a2);
  a25=(a0*a25);
  a26=sin(a2);
  a26=(a17*a26);
  a25=(a25-a26);
  a25=(a5*a25);
  a26=(a25*a15);
  a21=(a21+a26);
  a26=cos(a2);
  a26=(a17*a26);
  a27=sin(a2);
  a27=(a0*a27);
  a26=(a26+a27);
  a26=(a5*a26);
  a27=(a26*a9);
  a21=(a21+a27);
  a27=2.9929000000000000e-01;
  a28=6.7131110514003900e-01;
  a29=cos(a23);
  a29=(a28*a29);
  a30=7.4117568773627318e-01;
  a31=sin(a23);
  a31=(a30*a31);
  a29=(a29+a31);
  a29=(a27*a29);
  a31=9.6952880297333865e-01;
  a32=arg[0]? arg[0][4] : 0;
  a33=cos(a32);
  a33=(a31*a33);
  a34=2.4497734630999077e-01;
  a35=sin(a32);
  a35=(a34*a35);
  a33=(a33-a35);
  a35=(a29*a33);
  a21=(a21-a35);
  a35=cos(a23);
  a35=(a30*a35);
  a36=sin(a23);
  a36=(a28*a36);
  a35=(a35-a36);
  a35=(a27*a35);
  a36=cos(a32);
  a36=(a34*a36);
  a37=sin(a32);
  a37=(a31*a37);
  a36=(a36+a37);
  a37=(a35*a36);
  a21=(a21-a37);
  a21=(a21+a21);
  a21=(a21*a11);
  a37=(a9*a21);
  a37=(a5*a37);
  a38=(a0*a37);
  a39=sin(a2);
  a38=(a38*a39);
  a20=(a20-a38);
  a37=(a17*a37);
  a38=cos(a2);
  a37=(a37*a38);
  a38=sin(a2);
  a19=(a17*a19);
  a19=(a38*a19);
  a37=(a37+a19);
  a20=(a20-a37);
  a37=cos(a2);
  a19=(a15*a13);
  a19=(a5*a19);
  a39=(a17*a19);
  a39=(a37*a39);
  a40=(a15*a21);
  a40=(a5*a40);
  a41=(a17*a40);
  a42=sin(a2);
  a41=(a41*a42);
  a39=(a39-a41);
  a20=(a20-a39);
  a40=(a0*a40);
  a39=cos(a2);
  a40=(a40*a39);
  a39=sin(a2);
  a19=(a0*a19);
  a19=(a39*a19);
  a40=(a40+a19);
  a20=(a20-a40);
  a40=cos(a2);
  a19=(a12*a13);
  a19=(a40*a19);
  a41=(a12*a21);
  a42=sin(a2);
  a41=(a41*a42);
  a19=(a19-a41);
  a20=(a20+a19);
  a19=(a14*a21);
  a41=cos(a2);
  a19=(a19*a41);
  a41=sin(a2);
  a42=(a14*a13);
  a42=(a41*a42);
  a19=(a19+a42);
  a20=(a20-a19);
  a19=cos(a2);
  a42=6.7333004875847435e-01;
  a43=cos(a8);
  a43=(a10*a43);
  a44=sin(a8);
  a44=(a6*a44);
  a43=(a43+a44);
  a44=arg[3]? arg[3][6] : 0;
  a45=1.3308156638417232e-01;
  a46=cos(a2);
  a46=(a45*a46);
  a47=1.2119940877652538e-01;
  a48=sin(a2);
  a48=(a47*a48);
  a46=(a46+a48);
  a48=cos(a8);
  a48=(a6*a48);
  a49=sin(a8);
  a49=(a10*a49);
  a48=(a48-a49);
  a49=sin(a2);
  a49=(a42*a49);
  a50=7.3934203546762400e-01;
  a51=cos(a2);
  a51=(a50*a51);
  a49=(a49+a51);
  a49=(a5*a49);
  a51=(a48*a49);
  a46=(a46+a51);
  a51=cos(a2);
  a51=(a42*a51);
  a52=sin(a2);
  a52=(a50*a52);
  a51=(a51-a52);
  a51=(a5*a51);
  a52=(a43*a51);
  a46=(a46+a52);
  a46=(a46+a46);
  a46=(a44*a46);
  a52=(a43*a46);
  a52=(a5*a52);
  a53=(a42*a52);
  a53=(a19*a53);
  a54=sin(a2);
  a54=(a45*a54);
  a55=1.2083599892846777e-01;
  a56=cos(a23);
  a56=(a55*a56);
  a54=(a54-a56);
  a56=cos(a2);
  a56=(a47*a56);
  a54=(a54-a56);
  a56=1.3341162378892907e-01;
  a57=sin(a23);
  a57=(a56*a57);
  a54=(a54-a57);
  a57=cos(a2);
  a57=(a42*a57);
  a58=sin(a2);
  a58=(a50*a58);
  a57=(a57-a58);
  a57=(a5*a57);
  a58=(a57*a48);
  a54=(a54-a58);
  a58=cos(a2);
  a58=(a50*a58);
  a59=sin(a2);
  a59=(a42*a59);
  a58=(a58+a59);
  a58=(a5*a58);
  a59=(a58*a43);
  a54=(a54+a59);
  a59=6.7131110515815429e-01;
  a60=cos(a23);
  a60=(a59*a60);
  a61=7.4117568771627262e-01;
  a62=sin(a23);
  a62=(a61*a62);
  a60=(a60+a62);
  a60=(a27*a60);
  a62=cos(a32);
  a62=(a34*a62);
  a63=sin(a32);
  a63=(a31*a63);
  a62=(a62+a63);
  a63=(a60*a62);
  a54=(a54-a63);
  a63=cos(a23);
  a63=(a61*a63);
  a64=sin(a23);
  a64=(a59*a64);
  a63=(a63-a64);
  a63=(a27*a63);
  a64=cos(a32);
  a64=(a31*a64);
  a65=sin(a32);
  a65=(a34*a65);
  a64=(a64-a65);
  a65=(a63*a64);
  a54=(a54+a65);
  a65=8.9999999999999997e-02;
  a54=(a54-a65);
  a54=(a54+a54);
  a54=(a54*a44);
  a65=(a43*a54);
  a65=(a5*a65);
  a66=(a42*a65);
  a67=sin(a2);
  a66=(a66*a67);
  a53=(a53-a66);
  a20=(a20+a53);
  a65=(a50*a65);
  a53=cos(a2);
  a65=(a65*a53);
  a53=sin(a2);
  a52=(a50*a52);
  a52=(a53*a52);
  a65=(a65+a52);
  a20=(a20-a65);
  a65=cos(a2);
  a52=(a48*a46);
  a52=(a5*a52);
  a66=(a50*a52);
  a66=(a65*a66);
  a67=(a48*a54);
  a67=(a5*a67);
  a68=(a50*a67);
  a69=sin(a2);
  a68=(a68*a69);
  a66=(a66-a68);
  a20=(a20+a66);
  a67=(a42*a67);
  a66=cos(a2);
  a67=(a67*a66);
  a66=sin(a2);
  a52=(a42*a52);
  a52=(a66*a52);
  a67=(a67+a52);
  a20=(a20+a67);
  a67=(a47*a54);
  a52=cos(a2);
  a67=(a67*a52);
  a52=sin(a2);
  a68=(a47*a46);
  a68=(a52*a68);
  a67=(a67+a68);
  a20=(a20+a67);
  a67=cos(a2);
  a68=(a45*a46);
  a68=(a67*a68);
  a69=(a45*a54);
  a70=sin(a2);
  a69=(a69*a70);
  a68=(a68-a69);
  a20=(a20+a68);
  a68=(a4*a3);
  a69=(a7-a2);
  a70=1.6499999999999999e+00;
  a69=(a69+a70);
  a68=(a68/a69);
  a68=(a68/a69);
  a20=(a20-a68);
  a68=5.0000000000000000e-01;
  a69=(a68*a3);
  a2=(a7+a2);
  a71=1.2000000000000000e+00;
  a2=(a2+a71);
  a69=(a69/a2);
  a69=(a69/a2);
  a20=(a20-a69);
  if (res[0]!=0) res[0][5]=a20;
  a20=cos(a8);
  a69=(a25*a13);
  a16=(a21*a16);
  a69=(a69-a16);
  a16=(a6*a69);
  a16=(a20*a16);
  a2=cos(a8);
  a18=(a21*a18);
  a72=(a26*a13);
  a18=(a18+a72);
  a72=(a10*a18);
  a72=(a2*a72);
  a73=sin(a8);
  a18=(a6*a18);
  a18=(a73*a18);
  a72=(a72+a18);
  a16=(a16-a72);
  a72=sin(a8);
  a69=(a10*a69);
  a69=(a72*a69);
  a16=(a16-a69);
  a69=cos(a8);
  a51=(a54*a51);
  a18=(a58*a46);
  a51=(a51+a18);
  a18=(a6*a51);
  a18=(a69*a18);
  a16=(a16+a18);
  a18=sin(a8);
  a51=(a10*a51);
  a51=(a18*a51);
  a16=(a16-a51);
  a51=cos(a8);
  a74=(a57*a46);
  a49=(a54*a49);
  a74=(a74-a49);
  a49=(a10*a74);
  a49=(a51*a49);
  a16=(a16+a49);
  a49=sin(a8);
  a74=(a6*a74);
  a74=(a49*a74);
  a16=(a16+a74);
  if (res[0]!=0) res[0][6]=a16;
  a16=cos(a23);
  a74=(a36*a13);
  a74=(a27*a74);
  a75=(a28*a74);
  a75=(a16*a75);
  a76=sin(a23);
  a74=(a30*a74);
  a74=(a76*a74);
  a75=(a75+a74);
  a74=cos(a23);
  a77=(a33*a13);
  a77=(a27*a77);
  a78=(a30*a77);
  a78=(a74*a78);
  a75=(a75-a78);
  a78=sin(a23);
  a77=(a28*a77);
  a77=(a78*a77);
  a75=(a75+a77);
  a77=cos(a23);
  a79=(a24*a13);
  a79=(a77*a79);
  a75=(a75+a79);
  a79=sin(a23);
  a80=(a22*a13);
  a80=(a79*a80);
  a75=(a75+a80);
  a80=cos(a23);
  a81=(a64*a46);
  a81=(a27*a81);
  a82=(a59*a81);
  a82=(a80*a82);
  a75=(a75-a82);
  a82=sin(a23);
  a81=(a61*a81);
  a81=(a82*a81);
  a75=(a75-a81);
  a81=cos(a23);
  a83=(a62*a46);
  a83=(a27*a83);
  a84=(a61*a83);
  a84=(a81*a84);
  a75=(a75-a84);
  a84=sin(a23);
  a83=(a59*a83);
  a83=(a84*a83);
  a75=(a75+a83);
  a83=cos(a23);
  a85=(a56*a46);
  a85=(a83*a85);
  a75=(a75-a85);
  a85=sin(a23);
  a86=(a55*a46);
  a86=(a85*a86);
  a75=(a75+a86);
  if (res[0]!=0) res[0][7]=a75;
  a75=sin(a32);
  a86=(a35*a13);
  a87=(a34*a86);
  a87=(a75*a87);
  a88=cos(a32);
  a86=(a31*a86);
  a86=(a88*a86);
  a87=(a87-a86);
  a86=cos(a32);
  a13=(a29*a13);
  a89=(a34*a13);
  a89=(a86*a89);
  a87=(a87+a89);
  a89=sin(a32);
  a13=(a31*a13);
  a13=(a89*a13);
  a87=(a87+a13);
  a13=cos(a32);
  a90=(a63*a46);
  a91=(a34*a90);
  a91=(a13*a91);
  a87=(a87-a91);
  a91=sin(a32);
  a90=(a31*a90);
  a90=(a91*a90);
  a87=(a87-a90);
  a90=cos(a32);
  a46=(a60*a46);
  a92=(a31*a46);
  a92=(a90*a92);
  a87=(a87-a92);
  a92=sin(a32);
  a46=(a34*a46);
  a46=(a92*a46);
  a87=(a87+a46);
  if (res[0]!=0) res[0][8]=a87;
  a87=cos(a8);
  a87=(a6*a87);
  a46=sin(a8);
  a46=(a10*a46);
  a87=(a87-a46);
  a46=(a25*a87);
  a93=sin(a8);
  a93=(a6*a93);
  a94=cos(a8);
  a94=(a10*a94);
  a93=(a93+a94);
  a94=(a26*a93);
  a46=(a46-a94);
  a46=(a46+a46);
  a46=(a11*a46);
  a94=(a9*a46);
  a93=(a21*a93);
  a94=(a94-a93);
  a94=(a5*a94);
  a93=(a0*a94);
  a93=(a1*a93);
  a94=(a17*a94);
  a94=(a38*a94);
  a93=(a93-a94);
  a87=(a21*a87);
  a94=(a15*a46);
  a87=(a87+a94);
  a87=(a5*a87);
  a94=(a17*a87);
  a94=(a37*a94);
  a93=(a93-a94);
  a87=(a0*a87);
  a87=(a39*a87);
  a93=(a93-a87);
  a87=(a12*a46);
  a87=(a40*a87);
  a93=(a93+a87);
  a87=(a14*a46);
  a87=(a41*a87);
  a93=(a93-a87);
  a87=cos(a8);
  a87=(a6*a87);
  a94=sin(a8);
  a94=(a10*a94);
  a87=(a87-a94);
  a94=(a54*a87);
  a95=sin(a8);
  a95=(a6*a95);
  a96=cos(a8);
  a96=(a10*a96);
  a95=(a95+a96);
  a96=(a57*a95);
  a87=(a58*a87);
  a96=(a96+a87);
  a96=(a96+a96);
  a96=(a44*a96);
  a87=(a43*a96);
  a94=(a94+a87);
  a94=(a5*a94);
  a87=(a42*a94);
  a87=(a19*a87);
  a93=(a93+a87);
  a94=(a50*a94);
  a94=(a53*a94);
  a93=(a93-a94);
  a94=(a48*a96);
  a95=(a54*a95);
  a94=(a94-a95);
  a94=(a5*a94);
  a95=(a50*a94);
  a95=(a65*a95);
  a93=(a93+a95);
  a94=(a42*a94);
  a94=(a66*a94);
  a93=(a93+a94);
  a94=(a47*a96);
  a94=(a52*a94);
  a93=(a93+a94);
  a94=(a45*a96);
  a94=(a67*a94);
  a93=(a93+a94);
  if (res[0]!=0) res[0][9]=a93;
  a93=(a25*a46);
  a94=(a6*a93);
  a94=(a20*a94);
  a95=(a25*a21);
  a87=(a6*a95);
  a97=sin(a8);
  a87=(a87*a97);
  a94=(a94-a87);
  a87=(a26*a46);
  a97=(a10*a87);
  a97=(a2*a97);
  a98=(a26*a21);
  a99=(a10*a98);
  a100=sin(a8);
  a99=(a99*a100);
  a97=(a97-a99);
  a98=(a6*a98);
  a99=cos(a8);
  a98=(a98*a99);
  a87=(a6*a87);
  a87=(a73*a87);
  a98=(a98+a87);
  a97=(a97+a98);
  a94=(a94-a97);
  a95=(a10*a95);
  a97=cos(a8);
  a95=(a95*a97);
  a93=(a10*a93);
  a93=(a72*a93);
  a95=(a95+a93);
  a94=(a94-a95);
  a95=(a58*a96);
  a93=(a6*a95);
  a93=(a69*a93);
  a97=(a58*a54);
  a98=(a6*a97);
  a87=sin(a8);
  a98=(a98*a87);
  a93=(a93-a98);
  a94=(a94+a93);
  a97=(a10*a97);
  a93=cos(a8);
  a97=(a97*a93);
  a95=(a10*a95);
  a95=(a18*a95);
  a97=(a97+a95);
  a94=(a94-a97);
  a97=(a57*a96);
  a95=(a10*a97);
  a95=(a51*a95);
  a93=(a57*a54);
  a98=(a10*a93);
  a87=sin(a8);
  a98=(a98*a87);
  a95=(a95-a98);
  a94=(a94+a95);
  a93=(a6*a93);
  a95=cos(a8);
  a93=(a93*a95);
  a97=(a6*a97);
  a97=(a49*a97);
  a93=(a93+a97);
  a94=(a94+a93);
  a93=(a68*a3);
  a97=(a7-a8);
  a95=1.4199999999999999e+00;
  a97=(a97+a95);
  a93=(a93/a97);
  a93=(a93/a97);
  a94=(a94-a93);
  a93=(a4*a3);
  a8=(a7+a8);
  a97=1.3230000000000000e+00;
  a8=(a8+a97);
  a93=(a93/a8);
  a93=(a93/a8);
  a94=(a94-a93);
  if (res[0]!=0) res[0][10]=a94;
  a94=(a36*a46);
  a94=(a27*a94);
  a93=(a28*a94);
  a93=(a16*a93);
  a94=(a30*a94);
  a94=(a76*a94);
  a93=(a93+a94);
  a94=(a33*a46);
  a94=(a27*a94);
  a8=(a30*a94);
  a8=(a74*a8);
  a93=(a93-a8);
  a94=(a28*a94);
  a94=(a78*a94);
  a93=(a93+a94);
  a94=(a24*a46);
  a94=(a77*a94);
  a93=(a93+a94);
  a94=(a22*a46);
  a94=(a79*a94);
  a93=(a93+a94);
  a94=(a64*a96);
  a94=(a27*a94);
  a8=(a59*a94);
  a8=(a80*a8);
  a93=(a93-a8);
  a94=(a61*a94);
  a94=(a82*a94);
  a93=(a93-a94);
  a94=(a62*a96);
  a94=(a27*a94);
  a8=(a61*a94);
  a8=(a81*a8);
  a93=(a93-a8);
  a94=(a59*a94);
  a94=(a84*a94);
  a93=(a93+a94);
  a94=(a56*a96);
  a94=(a83*a94);
  a93=(a93-a94);
  a94=(a55*a96);
  a94=(a85*a94);
  a93=(a93+a94);
  if (res[0]!=0) res[0][11]=a93;
  a93=(a35*a46);
  a94=(a34*a93);
  a94=(a75*a94);
  a93=(a31*a93);
  a93=(a88*a93);
  a94=(a94-a93);
  a46=(a29*a46);
  a93=(a34*a46);
  a93=(a86*a93);
  a94=(a94+a93);
  a46=(a31*a46);
  a46=(a89*a46);
  a94=(a94+a46);
  a46=(a63*a96);
  a93=(a34*a46);
  a93=(a13*a93);
  a94=(a94-a93);
  a46=(a31*a46);
  a46=(a91*a46);
  a94=(a94-a46);
  a96=(a60*a96);
  a46=(a31*a96);
  a46=(a90*a46);
  a94=(a94-a46);
  a96=(a34*a96);
  a96=(a92*a96);
  a94=(a94+a96);
  if (res[0]!=0) res[0][12]=a94;
  a94=sin(a23);
  a94=(a22*a94);
  a96=cos(a23);
  a96=(a24*a96);
  a94=(a94+a96);
  a96=cos(a23);
  a96=(a30*a96);
  a46=sin(a23);
  a46=(a28*a46);
  a96=(a96-a46);
  a96=(a27*a96);
  a46=(a33*a96);
  a94=(a94-a46);
  a46=sin(a23);
  a46=(a30*a46);
  a93=cos(a23);
  a93=(a28*a93);
  a46=(a46+a93);
  a46=(a27*a46);
  a93=(a36*a46);
  a94=(a94+a93);
  a94=(a94+a94);
  a94=(a11*a94);
  a93=(a9*a94);
  a93=(a5*a93);
  a8=(a0*a93);
  a8=(a1*a8);
  a93=(a17*a93);
  a93=(a38*a93);
  a8=(a8-a93);
  a93=(a15*a94);
  a93=(a5*a93);
  a98=(a17*a93);
  a98=(a37*a98);
  a8=(a8-a98);
  a93=(a0*a93);
  a93=(a39*a93);
  a8=(a8-a93);
  a93=(a12*a94);
  a93=(a40*a93);
  a8=(a8+a93);
  a93=(a14*a94);
  a93=(a41*a93);
  a8=(a8-a93);
  a93=sin(a23);
  a93=(a55*a93);
  a98=cos(a23);
  a98=(a56*a98);
  a93=(a93-a98);
  a98=cos(a23);
  a98=(a61*a98);
  a87=sin(a23);
  a87=(a59*a87);
  a98=(a98-a87);
  a98=(a27*a98);
  a87=(a62*a98);
  a93=(a93-a87);
  a87=sin(a23);
  a87=(a61*a87);
  a99=cos(a23);
  a99=(a59*a99);
  a87=(a87+a99);
  a87=(a27*a87);
  a99=(a64*a87);
  a93=(a93-a99);
  a93=(a93+a93);
  a93=(a44*a93);
  a99=(a43*a93);
  a99=(a5*a99);
  a100=(a42*a99);
  a100=(a19*a100);
  a8=(a8+a100);
  a99=(a50*a99);
  a99=(a53*a99);
  a8=(a8-a99);
  a99=(a48*a93);
  a99=(a5*a99);
  a100=(a50*a99);
  a100=(a65*a100);
  a8=(a8+a100);
  a99=(a42*a99);
  a99=(a66*a99);
  a8=(a8+a99);
  a99=(a47*a93);
  a99=(a52*a99);
  a8=(a8+a99);
  a99=(a45*a93);
  a99=(a67*a99);
  a8=(a8+a99);
  if (res[0]!=0) res[0][13]=a8;
  a8=(a25*a94);
  a99=(a6*a8);
  a99=(a20*a99);
  a100=(a26*a94);
  a101=(a10*a100);
  a101=(a2*a101);
  a100=(a6*a100);
  a100=(a73*a100);
  a101=(a101+a100);
  a99=(a99-a101);
  a8=(a10*a8);
  a8=(a72*a8);
  a99=(a99-a8);
  a8=(a58*a93);
  a101=(a6*a8);
  a101=(a69*a101);
  a99=(a99+a101);
  a8=(a10*a8);
  a8=(a18*a8);
  a99=(a99-a8);
  a8=(a57*a93);
  a101=(a10*a8);
  a101=(a51*a101);
  a99=(a99+a101);
  a8=(a6*a8);
  a8=(a49*a8);
  a99=(a99+a8);
  if (res[0]!=0) res[0][14]=a99;
  a99=(a36*a94);
  a99=(a27*a99);
  a8=(a28*a99);
  a8=(a16*a8);
  a101=(a36*a21);
  a101=(a27*a101);
  a100=(a28*a101);
  a102=sin(a23);
  a100=(a100*a102);
  a8=(a8-a100);
  a101=(a30*a101);
  a100=cos(a23);
  a101=(a101*a100);
  a99=(a30*a99);
  a99=(a76*a99);
  a101=(a101+a99);
  a8=(a8+a101);
  a101=(a33*a94);
  a101=(a27*a101);
  a99=(a30*a101);
  a99=(a74*a99);
  a100=(a33*a21);
  a100=(a27*a100);
  a102=(a30*a100);
  a103=sin(a23);
  a102=(a102*a103);
  a99=(a99-a102);
  a8=(a8-a99);
  a100=(a28*a100);
  a99=cos(a23);
  a100=(a100*a99);
  a101=(a28*a101);
  a101=(a78*a101);
  a100=(a100+a101);
  a8=(a8+a100);
  a100=(a24*a94);
  a100=(a77*a100);
  a101=(a24*a21);
  a99=sin(a23);
  a101=(a101*a99);
  a100=(a100-a101);
  a8=(a8+a100);
  a100=(a22*a21);
  a101=cos(a23);
  a100=(a100*a101);
  a101=(a22*a94);
  a101=(a79*a101);
  a100=(a100+a101);
  a8=(a8+a100);
  a100=(a64*a93);
  a100=(a27*a100);
  a101=(a59*a100);
  a101=(a80*a101);
  a99=(a64*a54);
  a99=(a27*a99);
  a102=(a59*a99);
  a103=sin(a23);
  a102=(a102*a103);
  a101=(a101-a102);
  a8=(a8-a101);
  a99=(a61*a99);
  a101=cos(a23);
  a99=(a99*a101);
  a100=(a61*a100);
  a100=(a82*a100);
  a99=(a99+a100);
  a8=(a8-a99);
  a99=(a62*a93);
  a99=(a27*a99);
  a100=(a61*a99);
  a100=(a81*a100);
  a101=(a62*a54);
  a101=(a27*a101);
  a102=(a61*a101);
  a103=sin(a23);
  a102=(a102*a103);
  a100=(a100-a102);
  a8=(a8-a100);
  a101=(a59*a101);
  a100=cos(a23);
  a101=(a101*a100);
  a99=(a59*a99);
  a99=(a84*a99);
  a101=(a101+a99);
  a8=(a8+a101);
  a101=(a56*a93);
  a101=(a83*a101);
  a99=(a56*a54);
  a100=sin(a23);
  a99=(a99*a100);
  a101=(a101-a99);
  a8=(a8-a101);
  a101=(a55*a54);
  a99=cos(a23);
  a101=(a101*a99);
  a99=(a55*a93);
  a99=(a85*a99);
  a101=(a101+a99);
  a8=(a8+a101);
  a101=(a68*a3);
  a99=(a7-a23);
  a99=(a99+a71);
  a101=(a101/a99);
  a101=(a101/a99);
  a8=(a8-a101);
  a101=(a4*a3);
  a23=(a7+a23);
  a23=(a23+a70);
  a101=(a101/a23);
  a101=(a101/a23);
  a8=(a8-a101);
  if (res[0]!=0) res[0][15]=a8;
  a8=(a35*a94);
  a46=(a21*a46);
  a8=(a8-a46);
  a46=(a34*a8);
  a46=(a75*a46);
  a8=(a31*a8);
  a8=(a88*a8);
  a46=(a46-a8);
  a96=(a21*a96);
  a94=(a29*a94);
  a96=(a96+a94);
  a94=(a34*a96);
  a94=(a86*a94);
  a46=(a46+a94);
  a96=(a31*a96);
  a96=(a89*a96);
  a46=(a46+a96);
  a96=(a63*a93);
  a87=(a54*a87);
  a96=(a96-a87);
  a87=(a34*a96);
  a87=(a13*a87);
  a46=(a46-a87);
  a96=(a31*a96);
  a96=(a91*a96);
  a46=(a46-a96);
  a98=(a54*a98);
  a93=(a60*a93);
  a98=(a98+a93);
  a93=(a31*a98);
  a93=(a90*a93);
  a46=(a46-a93);
  a98=(a34*a98);
  a98=(a92*a98);
  a46=(a46+a98);
  if (res[0]!=0) res[0][16]=a46;
  a46=sin(a32);
  a46=(a31*a46);
  a98=cos(a32);
  a98=(a34*a98);
  a46=(a46+a98);
  a98=(a29*a46);
  a93=cos(a32);
  a93=(a31*a93);
  a96=sin(a32);
  a96=(a34*a96);
  a93=(a93-a96);
  a96=(a35*a93);
  a98=(a98-a96);
  a98=(a98+a98);
  a11=(a11*a98);
  a9=(a9*a11);
  a9=(a5*a9);
  a98=(a0*a9);
  a1=(a1*a98);
  a9=(a17*a9);
  a38=(a38*a9);
  a1=(a1-a38);
  a15=(a15*a11);
  a15=(a5*a15);
  a17=(a17*a15);
  a37=(a37*a17);
  a1=(a1-a37);
  a0=(a0*a15);
  a39=(a39*a0);
  a1=(a1-a39);
  a12=(a12*a11);
  a40=(a40*a12);
  a1=(a1+a40);
  a14=(a14*a11);
  a41=(a41*a14);
  a1=(a1-a41);
  a41=cos(a32);
  a41=(a31*a41);
  a14=sin(a32);
  a14=(a34*a14);
  a41=(a41-a14);
  a14=(a60*a41);
  a40=sin(a32);
  a40=(a31*a40);
  a12=cos(a32);
  a12=(a34*a12);
  a40=(a40+a12);
  a12=(a63*a40);
  a14=(a14+a12);
  a14=(a14+a14);
  a44=(a44*a14);
  a43=(a43*a44);
  a43=(a5*a43);
  a14=(a42*a43);
  a19=(a19*a14);
  a1=(a1-a19);
  a43=(a50*a43);
  a53=(a53*a43);
  a1=(a1+a53);
  a48=(a48*a44);
  a5=(a5*a48);
  a50=(a50*a5);
  a65=(a65*a50);
  a1=(a1-a65);
  a42=(a42*a5);
  a66=(a66*a42);
  a1=(a1-a66);
  a47=(a47*a44);
  a52=(a52*a47);
  a1=(a1-a52);
  a45=(a45*a44);
  a67=(a67*a45);
  a1=(a1-a67);
  if (res[0]!=0) res[0][17]=a1;
  a25=(a25*a11);
  a1=(a6*a25);
  a20=(a20*a1);
  a26=(a26*a11);
  a1=(a10*a26);
  a2=(a2*a1);
  a26=(a6*a26);
  a73=(a73*a26);
  a2=(a2+a73);
  a20=(a20-a2);
  a25=(a10*a25);
  a72=(a72*a25);
  a20=(a20-a72);
  a58=(a58*a44);
  a72=(a6*a58);
  a69=(a69*a72);
  a20=(a20-a69);
  a58=(a10*a58);
  a18=(a18*a58);
  a20=(a20+a18);
  a57=(a57*a44);
  a10=(a10*a57);
  a51=(a51*a10);
  a20=(a20-a51);
  a6=(a6*a57);
  a49=(a49*a6);
  a20=(a20-a49);
  if (res[0]!=0) res[0][18]=a20;
  a93=(a21*a93);
  a36=(a36*a11);
  a93=(a93+a36);
  a93=(a27*a93);
  a36=(a28*a93);
  a16=(a16*a36);
  a93=(a30*a93);
  a76=(a76*a93);
  a16=(a16+a76);
  a33=(a33*a11);
  a46=(a21*a46);
  a33=(a33-a46);
  a33=(a27*a33);
  a30=(a30*a33);
  a74=(a74*a30);
  a16=(a16-a74);
  a28=(a28*a33);
  a78=(a78*a28);
  a16=(a16+a78);
  a24=(a24*a11);
  a77=(a77*a24);
  a16=(a16+a77);
  a22=(a22*a11);
  a79=(a79*a22);
  a16=(a16+a79);
  a40=(a54*a40);
  a64=(a64*a44);
  a40=(a40+a64);
  a40=(a27*a40);
  a64=(a59*a40);
  a80=(a80*a64);
  a16=(a16+a80);
  a40=(a61*a40);
  a82=(a82*a40);
  a16=(a16+a82);
  a41=(a54*a41);
  a62=(a62*a44);
  a41=(a41-a62);
  a27=(a27*a41);
  a61=(a61*a27);
  a81=(a81*a61);
  a16=(a16-a81);
  a59=(a59*a27);
  a84=(a84*a59);
  a16=(a16+a84);
  a56=(a56*a44);
  a83=(a83*a56);
  a16=(a16+a83);
  a55=(a55*a44);
  a85=(a85*a55);
  a16=(a16-a85);
  if (res[0]!=0) res[0][19]=a16;
  a16=(a35*a21);
  a85=(a34*a16);
  a55=cos(a32);
  a85=(a85*a55);
  a35=(a35*a11);
  a55=(a34*a35);
  a75=(a75*a55);
  a85=(a85+a75);
  a35=(a31*a35);
  a88=(a88*a35);
  a16=(a31*a16);
  a35=sin(a32);
  a16=(a16*a35);
  a88=(a88-a16);
  a85=(a85-a88);
  a11=(a29*a11);
  a88=(a34*a11);
  a86=(a86*a88);
  a29=(a29*a21);
  a21=(a34*a29);
  a88=sin(a32);
  a21=(a21*a88);
  a86=(a86-a21);
  a85=(a85+a86);
  a29=(a31*a29);
  a86=cos(a32);
  a29=(a29*a86);
  a11=(a31*a11);
  a89=(a89*a11);
  a29=(a29+a89);
  a85=(a85+a29);
  a29=(a63*a54);
  a89=(a34*a29);
  a11=sin(a32);
  a89=(a89*a11);
  a63=(a63*a44);
  a11=(a34*a63);
  a13=(a13*a11);
  a89=(a89+a13);
  a85=(a85+a89);
  a29=(a31*a29);
  a89=cos(a32);
  a29=(a29*a89);
  a63=(a31*a63);
  a91=(a91*a63);
  a29=(a29-a91);
  a85=(a85-a29);
  a54=(a60*a54);
  a29=(a31*a54);
  a91=sin(a32);
  a29=(a29*a91);
  a60=(a60*a44);
  a31=(a31*a60);
  a90=(a90*a31);
  a29=(a29+a90);
  a85=(a85+a29);
  a54=(a34*a54);
  a29=cos(a32);
  a54=(a54*a29);
  a34=(a34*a60);
  a92=(a92*a34);
  a54=(a54-a92);
  a85=(a85+a54);
  a4=(a4*a3);
  a54=(a7-a32);
  a54=(a54+a97);
  a4=(a4/a54);
  a4=(a4/a54);
  a85=(a85-a4);
  a68=(a68*a3);
  a7=(a7+a32);
  a7=(a7+a95);
  a68=(a68/a7);
  a68=(a68/a7);
  a85=(a85-a68);
  if (res[0]!=0) res[0][20]=a85;
  return 0;
}

CASADI_SYMBOL_EXPORT int ntnu_leg_cost_y_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ntnu_leg_cost_y_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ntnu_leg_cost_y_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ntnu_leg_cost_y_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ntnu_leg_cost_y_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ntnu_leg_cost_y_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ntnu_leg_cost_y_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void ntnu_leg_cost_y_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ntnu_leg_cost_y_hess_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int ntnu_leg_cost_y_hess_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real ntnu_leg_cost_y_hess_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ntnu_leg_cost_y_hess_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ntnu_leg_cost_y_hess_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ntnu_leg_cost_y_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    case 4: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ntnu_leg_cost_y_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ntnu_leg_cost_y_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif