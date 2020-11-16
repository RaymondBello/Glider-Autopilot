#ifndef RTW_HEADER_Simulink_Simulation_h_
#define RTW_HEADER_Simulink_Simulation_h_
#include <string.h>
#include <stddef.h>
#include "rtw_modelmap.h"
#ifndef Simulink_Simulation_COMMON_INCLUDES_
#define Simulink_Simulation_COMMON_INCLUDES_
#include <stdlib.h>
#include "rtwtypes.h"
#include "simtarget/slSimTgtSigstreamRTW.h"
#include "simtarget/slSimTgtSlioCoreRTW.h"
#include "simtarget/slSimTgtSlioClientsRTW.h"
#include "simtarget/slSimTgtSlioSdiRTW.h"
#include "sigstream_rtw.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "raccel.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "rt_logging.h"
#include "dt_info.h"
#include "ext_work.h"
#endif
#include "Simulink_Simulation_types.h"
#include "multiword_types.h"
#include "rtsplntypes.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include "rtGetNaN.h"
#include "rt_defines.h"
#define MODEL_NAME Simulink_Simulation
#define NSAMPLE_TIMES (6) 
#define NINPUTS (0)       
#define NOUTPUTS (0)     
#define NBLOCKIO (125) 
#define NUM_ZC_EVENTS (0) 
#ifndef NCSTATES
#define NCSTATES (32)   
#elif NCSTATES != 32
#error Invalid specification of NCSTATES defined in compiler command
#endif
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm) (*rt_dataMapInfoPtr)
#endif
#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val) (rt_dataMapInfoPtr = &val)
#endif
#ifndef IN_RACCEL_MAIN
#endif
typedef struct { real_T cwtrsfu0ve ; } pgkaf0422g ; typedef struct { int_T
kaoqin3vcx ; int8_T jyd1ppenco ; boolean_T ltfrc3b41y ; } ettvb3gcku ;
typedef struct { real_T peioropukt ; } mluofao15w ; typedef struct { real_T
peioropukt ; } b3t4dgphal ; typedef struct { boolean_T peioropukt ; }
otn2c1vhrz ; typedef struct { real_T peioropukt ; } mtq3gtrjnc ; typedef
struct { real_T peioropukt ; } k1l1t3w5fx ; typedef struct { real_T
peioropukt ; } d04jqeiyh1 ; typedef struct { real_T anzwhaorff ; real_T
fkyjdvnvd5 ; real_T bz1j3sgcos ; } grkqfjgiva ; typedef struct { real_T
czibmh2e1r [ 3 ] ; real_T bsnqpkum4j [ 3660 ] ; real_T ng2ho4wdw5 [ 3660 ] ;
real_T emyp25k2ra [ 3660 ] ; real_T ke3jfhfxg1 [ 88 ] ; real_T c1uyo1uy5e [
322080 ] ; real_T l0xygmktrt [ 3 ] ; real_T ec4ih5lgh0 [ 3660 ] ; real_T
hf0jovm0ds [ 3660 ] ; real_T d2jzmsgxqy [ 3660 ] ; real_T iel1urtae2 [ 88 ] ;
real_T mv4eqkyxzg [ 322080 ] ; real_T o3sl5fyijx [ 3 ] ; real_T iownbpqjgp [
3660 ] ; real_T htq3wzf05k [ 3660 ] ; real_T mhvuogwz5g [ 3660 ] ; real_T
e4boznuo11 [ 88 ] ; real_T oltjiqpr2r [ 322080 ] ; real_T kbkjlm3ow2 [ 3 ] ;
real_T gpo4oh4yoc [ 3660 ] ; real_T lbrztkveec [ 3660 ] ; real_T hbisxcpsrg [
3660 ] ; real_T maz32p5ch2 [ 88 ] ; real_T otcphh42tj [ 322080 ] ; real_T
mucstbyucg [ 3 ] ; real_T lvndzs1soq [ 3660 ] ; real_T pcfohnqayd [ 3660 ] ;
real_T gvwl2w40zt [ 3660 ] ; real_T llvsxfyht1 [ 88 ] ; real_T c1emqgsxgf [
322080 ] ; void * js2t0wh1ap [ 3 ] ; void * dldelxht02 [ 6 ] ; void *
jvfpfqeupc [ 9 ] ; void * mxyuzcexca [ 3 ] ; void * eioxfitten [ 6 ] ; void *
npajdwg2lh [ 9 ] ; void * czje14zbes [ 3 ] ; void * ddpi0aehhj [ 6 ] ; void *
bjlmawntgi [ 9 ] ; void * czreq4mspp [ 3 ] ; void * eogjamfv2u [ 6 ] ; void *
cughybvon0 [ 9 ] ; void * hmckm540f5 [ 3 ] ; void * k5z3ljbn5f [ 6 ] ; void *
nzik33yqcn [ 9 ] ; uint32_T pkty1tadnq [ 3 ] ; uint32_T pr4m2g3lsg [ 3 ] ;
uint32_T fr1myblrol [ 3 ] ; uint32_T li4rk12cor [ 3 ] ; uint32_T bbfw2gdxcn [
3 ] ; int8_T jijajg5nkb ; int8_T gcr4x1ri1s ; int8_T biatop040u ; int8_T
meuzevkn2y ; int8_T ckxwrcsgi3 ; int8_T kglzzcbeka ; int8_T johe0vk44j ;
uint8_T dgmohvowou ; uint8_T nogzbokry4 ; uint8_T dhoi4kxfar ; uint8_T
czjauhqmhr ; uint8_T fg35qxl3vi ; } pg51jubuyw ; typedef struct { real_T
pkgvmqtajm [ 2 ] ; real_T cbt1v4nio1 [ 2 ] ; real_T chuqvl4sya ; real_T
hpaztoaur1 ; real_T pwdhurk11o ; real_T akzllltikr ; real_T pstvhythqv ;
real_T hjs5mprgdw ; real_T ckkqbbvet5 ; real_T lauuzxpsuv ; real_T kpfxhsrdzc
; real_T d2gfe2p42l ; real_T fdu3hdd1wq ; real_T gr2e1rit1v [ 2 ] ; real_T
jeyqu0v1xd ; real_T erkqk25euy ; real_T chm2baiq0o ; real_T hevlxlfvxw ;
real_T j1pkkc3ac5 ; real_T iiwmf2tmrv ; real_T onh5wf0mbq ; real_T a4jldatvmm
; real_T gectl3vjgg ; real_T nrzu1kxyjs [ 3 ] ; real_T bf5sj4v0nw ; real_T
nw4xrcz5qc [ 2 ] ; real_T gtmxqvxrzk [ 2 ] ; real_T nazotke0rm [ 2 ] ; real_T
pxggbjiorr ; real_T kjlhrll14x ; real_T jfhgj1iwfd ; real_T c12vlekqly ;
real_T nf1iua4byr ; real_T n0vzjwkja5 ; real_T cchzsu55nk ; real_T chmbb41gxy
; real_T dlbqs3jbq5 ; real_T a2xdtn2hjy ; real_T kbkk0nabqt [ 2 ] ; real_T
ohifllm5xd ; real_T ot3vk2b455 [ 3 ] ; real_T ddawuzzdzz [ 9 ] ; real_T
mzb2t2xbjn [ 3 ] ; real_T exldqi4iq0 ; real_T eyzfoa2rgd ; real_T knraoiqthj
; real_T oxryesp4l3 ; real_T ktgttcsket ; real_T dtrbet0r1l ; real_T
dojqnhucc1 ; real_T kkwizmpnnu ; real_T p1ud1zl0l5 ; real_T h1os02q0lx ;
real_T di0bxuhii0 [ 6 ] ; real_T fh1jzoqwae ; real_T netnloma2z ; real_T
bz2agq32f5 ; real_T gakkrw1mku ; real_T lldfszhlwc ; real_T iqcro5zqc5 ;
real_T khr4lzxr01 ; real_T otbeb0smep ; real_T mbiqcrghxm ; real_T gxcomph2oi
; real_T pcxo4cuead ; real_T euq0m2wkrk ; real_T lzfl1px3we ; real_T
c2uwt5jnat ; real_T e4lsqrzqjh ; real_T j2oyzznxjz ; real_T hkzk0mcpdz ;
real_T cjcp2dooxg ; real_T ivvuf1kwbw ; real_T dv0mxcbo4y ; real_T nts3iqoscx
; real_T lvp0utv5zh ; real_T bs4j1k4omq ; real_T nsbfkssthw [ 4 ] ; real_T
gsfqejjcfo ; real_T ewd50jnfc5 ; real_T pjsvxmsmf0 ; real_T pk4ybsmrx1 ;
real_T bav3d1hobe ; real_T o1nidekcl5 ; real_T pt0yyggmye ; real_T dp0qd0ktnr
[ 4 ] ; real_T fzxirwzcjx [ 3 ] ; real_T or3uazt4fw ; real_T ntt1gar3xf ;
real_T mb51iyxxjy ; real_T lxqebjchye [ 2 ] ; real_T ircgwytey3 [ 2 ] ;
real_T c5gx5xznq1 [ 2 ] ; real_T kxuhls1vu4 ; real_T ckuncjpeqo [ 2 ] ;
real_T gehuvldt5n [ 2 ] ; real_T k3touvbkw2 [ 2 ] ; real_T fosv245qat [ 2 ] ;
real_T leujf5c51t [ 2 ] ; real_T mv5ztdhgch [ 2 ] ; real_T agqbvy2aau [ 2 ] ;
real_T f3yq1g0dcx [ 2 ] ; real_T plmewtggpi ; real_T fffhvqsyn4 ; real_T
ohglmemv1w ; uint32_T prdmace1ry ; real32_T h1fm141rey ; real32_T hllvbobsss
; real32_T augghaqbk3 ; uint8_T fcdyl4g5vg [ 408 ] ; boolean_T oov0isvnf1 ;
boolean_T beyrovclvb ; boolean_T b5r5v125p5 ; boolean_T kulaji43pi ;
boolean_T drwselis2e ; boolean_T pxhra0n40a ; pgkaf0422g i5wp2ffbry ;
pgkaf0422g mirhnoymfvo ; } B ; typedef struct { real_T dh0tof3kep [ 2 ] ;
real_T glj000y2ul ; real_T gyluhpuobq [ 8 ] ; real_T hfu1jte3yo [ 8 ] ;
real_T hgabqtd4hg ; real_T lwgbxvspmn ; real_T btjxdj1nnm ; real_T foeszin0cg
; real_T e0khmaiz1x ; real_T csnixtyelx [ 8 ] ; real_T aqckczktpc [ 8 ] ;
real_T joib1qjnje [ 4 ] ; struct { void * LoggedData ; } jqnbqtb4sh ; void *
cygydsjrfb ; void * gu0lptjayg ; void * h4fprkvunp ; void * ib0jqzpceq ; void
* fodolp1u3u ; int32_T bu3swunyz2 ; int32_T eyrf2vyldh ; int32_T nzxqnu5jmv ;
int32_T osz5yvmvo1 ; int32_T otuccisgv4 ; int32_T kz3kayz3v2 ; int32_T
kjeb1ibj4p ; uint32_T mjfthrgmwy ; uint32_T ffudfy2unb ; uint32_T khqvqeq4d4
[ 4 ] ; uint32_T bxnoyp5430 ; int_T afe03te4gm ; int_T bp1ctbtl0z ; int_T
p0uqfawr0j ; int_T mff001ptpl ; int_T m5ksfibjla ; int_T li4wvvnvo0 ; int_T
bkbk0mdq0s ; int_T ogz3an5p1g ; int_T afsnxqrf1q ; int_T b02jv5hvxo ; int_T
le0andlo2k ; int_T gofb51vunu ; int_T bzco1krw01 ; int_T d0gx1hbrnk ; int_T
m2dc0faool ; int_T n4rcpokqpc ; int_T hueaazkach ; int8_T bh43qqaomf ; int8_T
cal4znigzs ; int8_T o3p0d0pvrk ; int8_T mxjlbo5rjs ; int8_T hmyyl45jap ;
int8_T mdzj2hr4jc ; int8_T fk0au12fsx ; int8_T pa1f5cjxua ; int8_T mewl43zgov
; int8_T jutuodwsau ; int8_T n12ashwpk5 ; int8_T poolnl3e1h ; int8_T
p4vcgbddhj ; int8_T l3k54bxdj1 ; int8_T dd1fatoipk ; int8_T dvxffgw44x ;
int8_T lfjjbagrjo ; int8_T bvueotiv4b ; int8_T hug50n4wps ; int8_T i1eyrq4uwi
; int8_T bfxedqpzaa ; int8_T fc1bretpi5 ; int8_T fg4worawde ; int8_T
mxamlbyck0 ; int8_T n4ytmke31n ; int8_T hudho2xp0v ; int8_T glvphpzltx ;
int8_T giro4eplfi ; int8_T fk3ir50wko ; int8_T ojqxnfreck ; int8_T g4vr3lujuz
; int8_T drdj3xeohi ; int8_T ngahvntjaq ; int8_T axi4q1he0b ; int8_T
f1edxk4pdy ; int8_T b3phozbfji ; int8_T faa1lc2tvb ; int8_T pdsijyzqws ;
int8_T dnxut4pgtq ; int8_T oo4gabx5lo ; int8_T jbrbgwy42h ; int8_T nb15cebgib
; int8_T fnjwuvlmmj ; int8_T ijpkeyczg1 ; int8_T e25tjl33um ; int8_T
auligz0bp0 ; int8_T cvvf5wjoub ; int8_T nv0y4bxbiz ; int8_T jwt5flmp0h ;
int8_T pkklmgxn3w ; int8_T ejm2q33rwd ; int8_T fvgndvljcd ; int8_T n5gzkaa0nd
; int8_T gylng33jck ; int8_T olplly2yli ; int8_T gkuxqeqcdi ; int8_T
o2i2c0ygga ; int8_T krzqaoxmqz ; int8_T jddl4uj35t ; int8_T adhcumf2n5 ;
int8_T npachunt01 ; int8_T byzsjh0px3 ; int8_T au2zmey5pc ; uint8_T
b0fymed3im [ 20 ] ; boolean_T es24y1hork ; boolean_T iuswyxudvk ; boolean_T
hk5aeqxoie ; boolean_T ex1irzrias ; boolean_T pldxv1lqbx ; boolean_T
guox3atybt ; boolean_T jvi410hnvz ; boolean_T en1aw5kkit ; boolean_T
kvd0keqbti ; boolean_T oq5swglxht ; boolean_T dvdytaltsj ; boolean_T
jteqln0zcu ; boolean_T ouupu23dos ; boolean_T nztae4vpib ; boolean_T
ahb01lfm54 ; boolean_T aovs2hjxyi ; boolean_T bdjnrykkzo ; boolean_T
auringonfr ; boolean_T ikp5xo5ucv ; boolean_T kk3mavtp0m ; pg51jubuyw
aqq0sq0gty ; pg51jubuyw cjigmwyptgo ; ettvb3gcku i5wp2ffbry ; ettvb3gcku
mirhnoymfvo ; } DW ; typedef struct { real_T nb321hhpjg [ 2 ] ; real_T
oyqwpsgdfy ; real_T nmbwexfepb [ 2 ] ; real_T c3n310npj5 ; real_T m2cgemps3b
[ 2 ] ; real_T fi3net4hji [ 2 ] ; real_T aldehcvxgf [ 2 ] ; real_T lwmusv4p0k
; real_T jcf02uzlnt [ 2 ] ; real_T go1vncxzci [ 2 ] ; real_T ds4avnfqd3 [ 2 ]
; real_T p11ukldqr3 [ 2 ] ; real_T galtq11byd [ 2 ] ; real_T efv1zergvx [ 2 ]
; real_T b5qj413fkr [ 2 ] ; real_T obrj2mgpdb [ 2 ] ; mluofao15w i5wp2ffbry ;
mluofao15w mirhnoymfvo ; real_T jxx3w5csvx ; } X ; typedef int_T PeriodicIndX
[ 1 ] ; typedef real_T PeriodicRngX [ 2 ] ; typedef struct { real_T
nb321hhpjg [ 2 ] ; real_T oyqwpsgdfy ; real_T nmbwexfepb [ 2 ] ; real_T
c3n310npj5 ; real_T m2cgemps3b [ 2 ] ; real_T fi3net4hji [ 2 ] ; real_T
aldehcvxgf [ 2 ] ; real_T lwmusv4p0k ; real_T jcf02uzlnt [ 2 ] ; real_T
go1vncxzci [ 2 ] ; real_T ds4avnfqd3 [ 2 ] ; real_T p11ukldqr3 [ 2 ] ; real_T
galtq11byd [ 2 ] ; real_T efv1zergvx [ 2 ] ; real_T b5qj413fkr [ 2 ] ; real_T
obrj2mgpdb [ 2 ] ; b3t4dgphal i5wp2ffbry ; b3t4dgphal mirhnoymfvo ; real_T
jxx3w5csvx ; } XDot ; typedef struct { boolean_T nb321hhpjg [ 2 ] ; boolean_T
oyqwpsgdfy ; boolean_T nmbwexfepb [ 2 ] ; boolean_T c3n310npj5 ; boolean_T
m2cgemps3b [ 2 ] ; boolean_T fi3net4hji [ 2 ] ; boolean_T aldehcvxgf [ 2 ] ;
boolean_T lwmusv4p0k ; boolean_T jcf02uzlnt [ 2 ] ; boolean_T go1vncxzci [ 2
] ; boolean_T ds4avnfqd3 [ 2 ] ; boolean_T p11ukldqr3 [ 2 ] ; boolean_T
galtq11byd [ 2 ] ; boolean_T efv1zergvx [ 2 ] ; boolean_T b5qj413fkr [ 2 ] ;
boolean_T obrj2mgpdb [ 2 ] ; otn2c1vhrz i5wp2ffbry ; otn2c1vhrz mirhnoymfvo ;
boolean_T jxx3w5csvx ; } XDis ; typedef struct { real_T nb321hhpjg [ 2 ] ;
real_T oyqwpsgdfy ; real_T nmbwexfepb [ 2 ] ; real_T c3n310npj5 ; real_T
m2cgemps3b [ 2 ] ; real_T fi3net4hji [ 2 ] ; real_T aldehcvxgf [ 2 ] ; real_T
lwmusv4p0k ; real_T jcf02uzlnt [ 2 ] ; real_T go1vncxzci [ 2 ] ; real_T
ds4avnfqd3 [ 2 ] ; real_T p11ukldqr3 [ 2 ] ; real_T galtq11byd [ 2 ] ; real_T
efv1zergvx [ 2 ] ; real_T b5qj413fkr [ 2 ] ; real_T obrj2mgpdb [ 2 ] ;
mtq3gtrjnc i5wp2ffbry ; mtq3gtrjnc mirhnoymfvo ; real_T jxx3w5csvx ; }
CStateAbsTol ; typedef struct { real_T nb321hhpjg [ 2 ] ; real_T oyqwpsgdfy ;
real_T nmbwexfepb [ 2 ] ; real_T c3n310npj5 ; real_T m2cgemps3b [ 2 ] ;
real_T fi3net4hji [ 2 ] ; real_T aldehcvxgf [ 2 ] ; real_T lwmusv4p0k ;
real_T jcf02uzlnt [ 2 ] ; real_T go1vncxzci [ 2 ] ; real_T ds4avnfqd3 [ 2 ] ;
real_T p11ukldqr3 [ 2 ] ; real_T galtq11byd [ 2 ] ; real_T efv1zergvx [ 2 ] ;
real_T b5qj413fkr [ 2 ] ; real_T obrj2mgpdb [ 2 ] ; k1l1t3w5fx i5wp2ffbry ;
k1l1t3w5fx mirhnoymfvo ; real_T jxx3w5csvx ; } CXPtMin ; typedef struct {
real_T nb321hhpjg [ 2 ] ; real_T oyqwpsgdfy ; real_T nmbwexfepb [ 2 ] ;
real_T c3n310npj5 ; real_T m2cgemps3b [ 2 ] ; real_T fi3net4hji [ 2 ] ;
real_T aldehcvxgf [ 2 ] ; real_T lwmusv4p0k ; real_T jcf02uzlnt [ 2 ] ;
real_T go1vncxzci [ 2 ] ; real_T ds4avnfqd3 [ 2 ] ; real_T p11ukldqr3 [ 2 ] ;
real_T galtq11byd [ 2 ] ; real_T efv1zergvx [ 2 ] ; real_T b5qj413fkr [ 2 ] ;
real_T obrj2mgpdb [ 2 ] ; d04jqeiyh1 i5wp2ffbry ; d04jqeiyh1 mirhnoymfvo ;
real_T jxx3w5csvx ; } CXPtMax ; typedef struct { real_T lzeqoaloix ; real_T
d1hk41evnx ; real_T hqcohimthh ; real_T gc52dxkvvu ; real_T kuqfbhlzzf ;
real_T oxin4qa23u ; real_T gn1kedvloc ; real_T gp0gnqirta ; real_T jlacqv5vtw
; real_T iwyhp5jkqq ; real_T cimcujnpcn ; real_T grqwdvhlin ; real_T
dmgrwrmwg5 ; real_T nk1g0dangh ; real_T elhkxuqxgq ; real_T kkznorvmx3 ;
real_T lrfqm10zu4 ; real_T ofizq1py51 ; real_T ajmmycxne1 ; real_T aiyxrxey2a
; real_T okdb0qcnrk ; real_T kjqit3l212 ; real_T a1kostazuj ; real_T
iasqm0qyqg ; real_T lrgoafpgt4 ; real_T gpzhpfpdtr ; real_T cufjvfhmpt ;
real_T kil22di02h ; real_T lebfshs02m ; real_T k1gnz04lfw ; real_T hvjawqgn5b
; real_T cbt5odztjp ; real_T ixjmldywkk ; real_T ja4ilf4myr ; real_T
ngnlqom0qz ; real_T hz5xr0iwvo ; real_T ojq25he1w4 ; real_T b5xpex1gz2 ;
real_T kt114whjjn ; real_T ghnvbi3vun ; real_T ovhuivd3z2 ; real_T k4agqastic
[ 2 ] ; real_T h2nqdokrmz [ 2 ] ; real_T porjn24fbf ; real_T f34qpd1zsm ;
grkqfjgiva i5wp2ffbry ; grkqfjgiva mirhnoymfvo ; real_T egav2gulw1 ; real_T
hubhp0n4tj ; real_T hnljyyxcdw ; } ZCV ; typedef struct {
rtwCAPI_ModelMappingInfo mmi ; } DataMapInfo ; struct flaan3t1j0_ { real_T
x_Y0 ; real_T DistanceintoGustxLimitedtogustlengthd_IC ; real_T
DistanceintoGustxLimitedtogustlengthd_LowerSat ; } ; struct jjovxzpofc_ {
uint32_T M0_maxIndex [ 3 ] ; uint32_T M0_dimSizes [ 3 ] ; uint32_T
M0_numYWorkElts [ 4 ] ; uint32_T M1_maxIndex [ 3 ] ; uint32_T M1_dimSizes [ 3
] ; uint32_T M1_numYWorkElts [ 4 ] ; uint32_T M2_maxIndex [ 3 ] ; uint32_T
M2_dimSizes [ 3 ] ; uint32_T M2_numYWorkElts [ 4 ] ; uint32_T M3_maxIndex [ 3
] ; uint32_T M3_dimSizes [ 3 ] ; uint32_T M3_numYWorkElts [ 4 ] ; uint32_T
M4_maxIndex [ 3 ] ; uint32_T M4_dimSizes [ 3 ] ; uint32_T M4_numYWorkElts [ 4
] ; } ; struct P_ { real_T LatLong0 [ 2 ] ; real_T Sref ; real_T V0 ; real_T
alpha0 ; real_T alt0 ; real_T bref ; real_T cbar ; real_T heading0 ; real_T
mass ; real_T maxdef_aileron ; real_T maxdef_elevator ; real_T maxdef_rudder
; real_T mindef_aileron ; real_T mindef_elevator ; real_T mindef_rudder ;
real_T theta0 ; real_T wn_act ; real_T wy0 ; real_T z_act ; real_T
DiscreteWindGustModel_Gx ; real_T DiscreteWindGustModel_Gy ; real_T
DiscreteWindGustModel_Gz ; real_T uDOFBodyAxes_Iyy ; real_T TAS2CAS_LUTM0 [
322080 ] ; real_T CAS2TAS_LUTM0 [ 322080 ] ; real_T TAS2CAS_LUTM1 [ 322080 ]
; real_T CAS2TAS_LUTM1 [ 322080 ] ; real_T TAS2CAS_LUTM2 [ 322080 ] ; real_T
CAS2TAS_LUTM2 [ 322080 ] ; real_T TAS2CAS_LUTM3 [ 322080 ] ; real_T
CAS2TAS_LUTM3 [ 322080 ] ; real_T TAS2CAS_LUTM4 [ 322080 ] ; real_T
CAS2TAS_LUTM4 [ 322080 ] ; real_T
DrydenWindTurbulenceModelContinuousqr_L_high ; real_T TAS2CAS_P_bp [ 88 ] ;
real_T CAS2TAS_P_bp [ 88 ] ; real_T AerodynamicForcesandMoments_S ; real_T
AerodynamicForcesandMoments_S_miuqcjxiwz ; real_T TAS2CAS_SOS_bp [ 61 ] ;
real_T CAS2TAS_SOS_bp [ 61 ] ; real_T
DrydenWindTurbulenceModelContinuousqr_Seed [ 4 ] ; real_T
DrydenWindTurbulenceModelContinuousqr_T_on ; real_T WhiteNoise_Ts ; real_T
DrydenWindTurbulenceModelContinuousqr_TurbProb ; real_T TAS2CAS_V_bp_M0 [ 60
] ; real_T CAS2TAS_V_bp_M0 [ 60 ] ; real_T TAS2CAS_V_bp_M1 [ 60 ] ; real_T
CAS2TAS_V_bp_M1 [ 60 ] ; real_T TAS2CAS_V_bp_M2 [ 60 ] ; real_T
CAS2TAS_V_bp_M2 [ 60 ] ; real_T TAS2CAS_V_bp_M3 [ 60 ] ; real_T
CAS2TAS_V_bp_M3 [ 60 ] ; real_T TAS2CAS_V_bp_M4 [ 60 ] ; real_T
CAS2TAS_V_bp_M4 [ 60 ] ; real_T DrydenWindTurbulenceModelContinuousqr_W20 ;
real_T WindShearModel_W_20 ; real_T
DrydenWindTurbulenceModelContinuousqr_Wdeg ; real_T WindShearModel_Wdeg ;
real_T DrydenWindTurbulenceModelContinuousqr_Wingspan ; real_T
COESAAtmosphereModel_action ; real_T AerodynamicForcesandMoments_b ; real_T
AerodynamicForcesandMoments_b_blpao4glfe ; real_T
AerodynamicForcesandMoments_cbar ; real_T
AerodynamicForcesandMoments_cbar_ect3p4btxl ; real_T CompareToConstant_const
; real_T CompareToConstant_const_j4cxad0q0g ; real_T
CompareToConstant_const_omj1fbfglm ; real_T
CompareToConstant_const_n1mct5qu1a ; real_T
CompareToConstant_const_d4qgtkvaqb ; real_T
CompareToConstant_const_fkrbb1l4we ; real_T
CompareToConstant_const_o4jsn2vuck ; real_T
CompareToConstant_const_n1tmrxoekt ; real_T Distanceintogustx_d_m ; real_T
Distanceintogusty_d_m ; real_T Distanceintogustz_d_m ; real_T
DiscreteWindGustModel_d_m [ 3 ] ; real_T
NonlinearSecondOrderActuator1_fin_act_0 ; real_T
NonlinearSecondOrderActuator_fin_act_0 ; real_T
NonlinearSecondOrderActuator2_fin_act_0 ; real_T
NonlinearSecondOrderActuator1_fin_act_vel ; real_T
NonlinearSecondOrderActuator_fin_act_vel ; real_T
NonlinearSecondOrderActuator2_fin_act_vel ; real_T
NonlinearSecondOrderActuator1_fin_maxrate ; real_T
NonlinearSecondOrderActuator_fin_maxrate ; real_T
NonlinearSecondOrderActuator2_fin_maxrate ; real_T uDOFBodyAxes_g ; real_T
uDOFBodyAxes_pos_ini [ 2 ] ; real_T WhiteNoise_pwr [ 4 ] ; real_T
DiscreteWindGustModel_t_0 ; real_T DiscreteWindGustModel_v_m [ 3 ] ; int32_T
FlightGearPreconfigured6DoFAnimation_DestinationPort ; int32_T
CompareToConstant1_const ; int32_T CompareToConstant1_const_gvieufb20s ;
real_T x_Y0 ; real_T DistanceintoGustxLimitedtogustlengthd_IC ; real_T
DistanceintoGustxLimitedtogustlengthd_LowerSat ; real_T pgw_Y0 ; real_T
pgw_p_IC ; real_T Constant1_Value ; real_T Constant2_Value ; real_T
Constant3_Value ; real_T qgw_Y0 ; real_T qgw_p_IC ; real_T pi4_Gain ; real_T
rgw_Y0 ; real_T rgw_p_IC ; real_T pi3_Gain ; real_T ugw_Y0 ; real_T upi_Gain
; real_T ug_p_IC ; real_T vgw_Y0 ; real_T upi_Gain_dmjomzxo4q ; real_T
vg_p1_IC ; real_T vgw_p2_IC ; real_T sqrt3_Gain ; real_T wgw_Y0 ; real_T
upi_Gain_ii5rsz2pui ; real_T wg_p1_IC ; real_T wg_p2_IC ; real_T
Constant_Value ; real_T Gain_Gain ; real_T max_height_low_Value ; real_T
min_height_high_Value ; real_T Gain_Gain_bzpcr2c2vd ; real_T
max_height_low_Value_bkbuta4yqd ; real_T min_height_high_Value_okjndayw2f ;
real_T GravityinEarthAxes_Gain [ 3 ] ; real_T
LimitFunction10ftto1000ft_UpperSat ; real_T
LimitFunction10ftto1000ft_LowerSat ; real_T Lw_Gain ; real_T
PreLookUpIndexSearchaltitude_BreakpointsData [ 12 ] ; real_T
MediumHighAltitudeIntensity_Table [ 84 ] ; real_T WhiteNoise_Mean ; real_T
WhiteNoise_StdDev ; real_T LimitHeighth1000ft_UpperSat ; real_T
LimitHeighth1000ft_LowerSat ; real_T Lv_Gain ; real_T uftinf_UpperSat ;
real_T uftinf_LowerSat ; real_T hz0_Gain ; real_T Longitude_Value ; real_T
u_Value ; real_T sigma_wg_Gain ; real_T
PreLookUpIndexSearchprobofexceed_BreakpointsData [ 7 ] ; real_T Wdeg1_Value ;
real_T ref_heightz0_Value ; real_T Step_Time ; real_T Step_Y0 ; real_T
Step_YFinal ; real_T StateSpace_A ; real_T StateSpace_B ; real_T StateSpace_C
; real_T StateSpace_InitialCondition ; real_T RateLimiter_RisingLim ; real_T
RateLimiter_FallingLim ; real_T throttle_Value ; real_T asOut_Y0 ; real_T
Memory_InitialCondition ; real_T Bias6_Bias ; real_T Bias5_Bias ; real_T
Bias4_Bias ; real_T Constant1_Value_iv5ptljryc ; real_T
Constant3_Value_h0acrtutky ; real_T Constant4_Value ; real_T Bias1_Bias ;
real_T Bias3_Bias ; real_T Bias1_Bias_cjyekcjkqd ; real_T Bias_Bias ; real_T
Bias2_Bias ; real_T Bias5_Bias_czj0v4rf55 ; real_T Bias4_Bias_cy2cy0rkru ;
real_T Bias6_Bias_cgt3io1in0 ; real_T Constant1_Value_jb5j0tvdpj ; real_T
Constant_Value_nv1uo0d3mg ; real_T Constant2_Value_pdjiwz3ftx ; real_T
Constant_Value_bfoige4wh4 ; real_T asOut_Y0_fruk2gwjsm ; real_T
Memory_InitialCondition_lgdo1d41mj ; real_T Bias6_Bias_hm1r0zlf2u ; real_T
Bias5_Bias_mcem25dje2 ; real_T Bias4_Bias_pl5eddn2p1 ; real_T
Constant1_Value_pb5yn2hc0w ; real_T Constant3_Value_kvwuhyt22j ; real_T
Constant4_Value_kox0vku1gy ; real_T Bias1_Bias_hfkjqppc2q ; real_T
Bias3_Bias_jd2w4riqmn ; real_T Bias1_Bias_ncd3jtrabd ; real_T
Bias_Bias_kruphzigt4 ; real_T Bias2_Bias_lotf5js1s3 ; real_T
Bias5_Bias_meya4vfioi ; real_T Bias4_Bias_odezqxp5ct ; real_T
Bias6_Bias_jskr1b1mi4 ; real_T Constant1_Value_ej01j13uct ; real_T
Constant_Value_lbccuktfh2 ; real_T Constant2_Value_bopb4ki4xd ; real_T
Constant_Value_kueaisvxhv ; real_T Merge1_1_InitialOutput ; real_T
Merge1_2_InitialOutput ; real_T Merge1_3_InitialOutput ; real_T
Merge1_4_InitialOutput ; real_T Merge1_5_InitialOutput ; real_T
Merge1_6_InitialOutput ; real_T Merge1_7_InitialOutput ; real_T
Merge1_8_InitialOutput ; real_T Merge1_9_InitialOutput ; real_T
Constant_Value_pmyhtxrqrr ; real_T Constant_Value_gun4ws4ai2 ; real_T
Constant_Value_mq1pkcil10 ; real_T Bias_Bias_k5qjjnbf1r ; real_T
Gain_Gain_opd2wvnqlj ; real_T Bias1_Bias_oluwwwumlb ; real_T
Bias_Bias_mahsvgbvzq ; real_T Bias1_Bias_fiyyfd4u5t ; real_T
Bias_Bias_du0tiht0fj ; real_T Bias1_Bias_nbxpttwzhh ; real_T
Packnet_fdmPacketforFlightGear_P8 ; real_T SimulationPace_P1 ; real_T
SimulationPace_P2 ; real_T SimulationPace_P3 ; real_T SimulationPace_P4 ;
real_T PacketSize_Value ; real_T Theta_WrappedStateUpperValue ; real_T
Theta_WrappedStateLowerValue ; real_T AltController_A [ 3 ] ; real_T
AltController_B ; real_T AltController_C [ 2 ] ; real_T AltController_D ;
real_T ThetaController_A ; real_T ThetaController_C ; real_T
ThetaController_D ; real_T Merge_InitialOutput ; real_T u2rhoV2_Gain ; real_T
alpha_BreakpointsData [ 10 ] ; real_T Mach_BreakpointsData [ 4 ] ; real_T
altitude_BreakpointsData [ 8 ] ; real_T CD_Table [ 320 ] ; real_T CYb_Table [
320 ] ; real_T CL_Table [ 320 ] ; real_T coefAdjust_Gain [ 3 ] ; real_T
Clb_Table [ 320 ] ; real_T Cm_Table [ 320 ] ; real_T Cnb_Table [ 320 ] ;
real_T CYp_Table [ 320 ] ; real_T CLad_Table [ 320 ] ; real_T CLq_Table [ 320
] ; real_T Clp_Table [ 320 ] ; real_T Clr_Table [ 320 ] ; real_T Cmq_Table [
320 ] ; real_T Cmad_Table [ 320 ] ; real_T Cnp_Table [ 320 ] ; real_T
Cnr_Table [ 320 ] ; real_T Gain1_Gain [ 6 ] ; real_T
coefAdjust_Gain_m5bwkewsju [ 3 ] ; real_T alpha_BreakpointsData_gcueg2tgd0 [
10 ] ; real_T Mach_BreakpointsData_jg33uk3jci [ 4 ] ; real_T
altitude_BreakpointsData_p0oyeupgfw [ 8 ] ; real_T delta_BreakpointsData [ 5
] ; real_T DCDI_Table [ 1600 ] ; real_T DCL_Table [ 160 ] ; real_T DCm_Table
[ 160 ] ; real_T coefAdjust_Gain_oacgeai0xk [ 3 ] ; real_T Xcp_Table [ 320 ]
; real_T Thrust_LMN_Value [ 3 ] ; real_T MatrixGain_Gain [ 4 ] ; real_T
ThrustX_tableData [ 7 ] ; real_T ThrustX_bp01Data [ 7 ] ; real_T
Thrust_YZ_Value [ 2 ] ; real_T Constant6_Value ; real_T gamma_Value ; real_T
one1_Value ; real_T const_Value ; real_T two_Value ; real_T one_Value ;
real_T Po_Value ; real_T Constant_Value_iw00e14laz ; real_T
Constant2_Value_nwrewbsrgx ; real_T seaLevelPstatic_Value ; real_T
seaLevelSOS_Value ; real_T Constant2_Value_gr5f00mfnm ; real_T
Constant_Value_n35gokrdfy ; real_T Constant1_Value_aypmrruauh ; real_T
Constant12_Value ; real_T Constant2_Value_dp3hb0zdw4 ; real_T
Constant3_Value_dde35bhip2 ; real_T Constant_Value_jdmivpanfl ; real_T
Constant_Value_fwsa41mimo ; real_T Constant10_Value ; real_T
Constant1_Value_n5qw4ehq0c ; real_T Constant1_Value_budxhsb1cr ; real_T
Constant1_Value_nyr0ia51bl ; real_T zero1_Value ; real_T
Constant_Value_olio5liftr [ 9 ] ; real_T u_Value_pyce5cxhso ; real_T
u_Value_nkmodb5shf ; real_T u_Value_a3c51jcqno ; real_T
Constant_Value_lr2cknagpa ; real_T Constant1_Value_idys3ziitq ; real_T
Constant2_Value_axatlcv2rs ; real_T Constant3_Value_ockepn4d1g ; real_T
Constant4_Value_aprubsstdi ; real_T Constant_Value_mbry10wmmx ; real_T
Constant1_Value_ishhczx4h3 ; real_T Constant2_Value_bed4d3ljnh ; real_T
Constant3_Value_l1ogwvmgck ; real_T Constant4_Value_env11uw4lz ; real_T
Constant_Value_jkivyeh0ov ; real_T Constant1_Value_dttko5tnn2 ; real_T
Constant2_Value_d2teymf5hj ; real_T Constant3_Value_goletwl4mi ; real_T
Constant4_Value_iqinklay2r ; real_T zero1_Value_jquxqq4qvz [ 3 ] ; real_T
zero3_Value [ 3 ] ; real_T Constant_Value_nm4nmql4xz ; real_T zero_Value ;
real_T Constant_Value_dk11ift0i1 ; real_T Constant1_Value_e0wp0jqtlj ; real_T
Constant2_Value_b5ilyky2la ; real_T Constant3_Value_d4mh0cif3h ; real_T
Constant4_Value_bqgwz3qflq ; real_T Constant_Value_pzjjbfwyj5 ; real_T
Constant1_Value_fvdtk4fzi5 ; real_T Constant2_Value_ms43idsvzx ; real_T
Constant3_Value_mgyvggblh0 ; real_T Constant4_Value_dv4oyrtdrl ; real_T
Constant_Value_aagndz5zvc ; real_T Constant1_Value_cviw4ezr5s ; real_T
Constant2_Value_faxwqrunex ; real_T Constant3_Value_hkckgaoniw ; real_T
Constant4_Value_ctnx5cdtqt ; real_T Constant_Value_klve0yes4x ; real_T
Constant_Value_gddg5y0bon ; real_T Constant1_Value_fdgj4x4yvk ; real_T
Constant2_Value_g4cw5n3yku ; real_T Constant2_Value_f3imtyqlu2 ; real_T
Constant_Value_iatzfmuax5 ; real_T Constant1_Value_jbkvuyvxyw ; real_T
Bias_Bias_epi214vgpp ; real_T Constant2_Value_oe5zwfuhbf ; real_T
Bias1_Bias_itbskgc1ah ; real_T Bias_Bias_aujzrf41ca ; real_T
Gain_Gain_fmc2fxjyzt ; real_T Bias1_Bias_jirwvifxga ; real_T
Bias_Bias_gall5umyqi ; real_T Constant2_Value_pkv2f4fmve ; real_T
Bias1_Bias_g0mi2jjfta ; real_T Constant_Value_bf4fbrpvxe ; real_T
Constant1_Value_hjtl2lfjek ; real_T Constant2_Value_dnwbrdkxjw ; real_T
Constant3_Value_hlef1jp2hn ; real_T Constant_Value_ltgnw1q5be ; real_T
Constant_Value_hdq4ys1wku ; real_T Constant_Value_ddl3nhb2x0 ; real_T f_Value
; uint32_T MediumHighAltitudeIntensity_maxIndex [ 2 ] ; uint32_T CD_dimSize [
3 ] ; uint32_T CYb_dimSize [ 3 ] ; uint32_T CL_dimSize [ 3 ] ; uint32_T
Clb_dimSize [ 3 ] ; uint32_T Cm_dimSize [ 3 ] ; uint32_T Cnb_dimSize [ 3 ] ;
uint32_T CYp_dimSize [ 3 ] ; uint32_T CLad_dimSize [ 3 ] ; uint32_T
CLq_dimSize [ 3 ] ; uint32_T Clp_dimSize [ 3 ] ; uint32_T Clr_dimSize [ 3 ] ;
uint32_T Cmq_dimSize [ 3 ] ; uint32_T Cmad_dimSize [ 3 ] ; uint32_T
Cnp_dimSize [ 3 ] ; uint32_T Cnr_dimSize [ 3 ] ; uint32_T DCDI_dimSize [ 4 ]
; uint32_T DCL_dimSize [ 3 ] ; uint32_T DCm_dimSize [ 3 ] ; uint32_T
Xcp_dimSize [ 3 ] ; uint16_T Packnet_fdmPacketforFlightGear_P1 [ 2 ] ;
uint16_T Packnet_fdmPacketforFlightGear_P2 [ 2 ] ; uint16_T
Packnet_fdmPacketforFlightGear_P3 [ 3 ] ; uint16_T
Packnet_fdmPacketforFlightGear_P4 [ 3 ] ; uint16_T
Packnet_fdmPacketforFlightGear_P5 [ 3 ] ; uint16_T
Packnet_fdmPacketforFlightGear_P6 [ 3 ] ; uint16_T
Packnet_fdmPacketforFlightGear_P7 [ 3 ] ; jjovxzpofc aqq0sq0gty ; jjovxzpofc
cjigmwyptgo ; flaan3t1j0 i5wp2ffbry ; flaan3t1j0 mirhnoymfvo ; } ; extern
const char * RT_MEMORY_ALLOCATION_ERROR ; extern B rtB ; extern X rtX ;
extern DW rtDW ; extern P rtP ; extern const rtwCAPI_ModelMappingStaticInfo *
Simulink_Simulation_GetCAPIStaticMap ( void ) ; extern SimStruct * const rtS
; extern const int_T gblNumToFiles ; extern const int_T gblNumFrFiles ;
extern const int_T gblNumFrWksBlocks ; extern rtInportTUtable *
gblInportTUtables ; extern const char * gblInportFileName ; extern const
int_T gblNumRootInportBlks ; extern const int_T gblNumModelInputs ; extern
const int_T gblInportDataTypeIdx [ ] ; extern const int_T gblInportDims [ ] ;
extern const int_T gblInportComplex [ ] ; extern const int_T
gblInportInterpoFlag [ ] ; extern const int_T gblInportContinuous [ ] ;
extern const int_T gblParameterTuningTid ; extern DataMapInfo *
rt_dataMapInfoPtr ; extern rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr ;
void MdlOutputs ( int_T tid ) ; void MdlOutputsParameterSampleTime ( int_T
tid ) ; void MdlUpdate ( int_T tid ) ; void MdlTerminate ( void ) ; void
MdlInitializeSizes ( void ) ; void MdlInitializeSampleTimes ( void ) ;
SimStruct * raccel_register_model ( void ) ;
#endif
