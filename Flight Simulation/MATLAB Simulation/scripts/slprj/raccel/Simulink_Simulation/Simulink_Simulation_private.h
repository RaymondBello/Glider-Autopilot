#ifndef RTW_HEADER_Simulink_Simulation_private_h_
#define RTW_HEADER_Simulink_Simulation_private_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#include "Simulink_Simulation.h"
#if !defined(rt_VALIDATE_MEMORY)
#define rt_VALIDATE_MEMORY(S, ptr)   if(!(ptr)) {\
  ssSetErrorStatus(rtS, RT_MEMORY_ALLOCATION_ERROR);\
  }
#endif
#if !defined(rt_FREE)
#if !defined(_WIN32)
#define rt_FREE(ptr)   if((ptr) != (NULL)) {\
  free((ptr));\
  (ptr) = (NULL);\
  }
#else
#define rt_FREE(ptr)   if((ptr) != (NULL)) {\
  free((void *)(ptr));\
  (ptr) = (NULL);\
  }
#endif
#endif
#ifndef ATMOS_TYPEDEF
typedef enum { COESA = 1 , MILHDBK310 , MILSTD210C } AtmosTypeIdx ; typedef
enum { PROFILE = 1 , ENVELOPE } ModelIdx ; typedef enum { HIGHTEMP = 1 ,
LOWTEMP , HIGHDENSITY , LOWDENSITY , HIGHPRESSURE , LOWPRESSURE } VarIdx ;
typedef enum { PP1 = 1 , PP10 } PPercentIdx ; typedef enum { K5 = 1 , K10 ,
K20 , K30 , K40 } PAltIdx ; typedef enum { EXTREME = 1 , P1 , P5 , P10 , P20
} EPercentIdx ;
#define ATMOS_TYPEDEF
#endif
#ifndef ATMOS_DEFINE
#define PRESSURE0   101325.0     
#define TEMPERATURE0   288.15    
#define GRAV_CONST       9.80665 
#define MOL_WT          28.9644  
#define R_HAT         8314.32    
#define GAMMA            1.4     
#define GMR       ( GRAV_CONST * MOL_WT / R_HAT )
#define ATMOS_DEFINE
#endif
#ifndef COESA76_DEFINE_DATA
#define NUM1976PTS 8
static real_T altitude76 [ NUM1976PTS ] = { 0.0 , 11000.0 , 20000.0 , 32000.0
, 47000.0 , 51000.0 , 71000.0 , 84852.0 } ; static real_T tempGradient76 [
NUM1976PTS ] = { ( - 0.0065 ) , 0.0 , 0.0010 , 0.0028 , 0.0 , - 0.0028 , -
0.0020 , - 0.0020 } ;
#define COESA76_DEFINE_DATA
#endif
extern real_T rt_urand_Upu32_Yd_f_pw_snf ( uint32_T * u ) ; extern real_T
rt_nrand_Upu32_Yd_f_pw_snf ( uint32_T * u ) ; real_T look_SplNLinSScd (
uint32_T numDims , const real_T * u , const rt_LUTSplineWork * const SWork )
; void rt_Spline2Derivd ( const real_T * x , const real_T * y , uint32_T n ,
real_T * u , real_T * y2 ) ; real_T intrp_NSplcd ( uint32_T numDims , const
rt_LUTSplineWork * const splWork , uint32_T extrapMethod ) ; void
InitCalcAtmosCOESA ( real_T * temperature76 , real_T * pressureRatio76 ) ;
void CalcAtmosCOESA ( const real_T * altitude , real_T * temp , real_T *
pressure , real_T * density , real_T * speedofsound , real_T * temperature76
, real_T * pressureRatio76 , int_T numPoints ) ; void CalcPAltCOESA ( const
real_T * pressure , real_T * altitude , real_T * temperature76 , real_T *
pressureRatio76 , int_T numPoints ) ; extern int32_T plook_s32dd_binxp (
real_T u , const real_T bp [ ] , uint32_T maxIndex , real_T * fraction ,
int32_T * prevIndex ) ; extern real_T intrp3d_s32dl_pw ( const int32_T
bpIndex [ ] , const real_T frac [ ] , const real_T table [ ] , const uint32_T
stride [ ] ) ; extern real_T intrp4d_s32dl_pw ( const int32_T bpIndex [ ] ,
const real_T frac [ ] , const real_T table [ ] , const uint32_T stride [ ] )
; extern real_T look1_plinlcpw ( real_T u0 , const real_T bp0 [ ] , const
real_T table [ ] , uint32_T prevIndex [ ] , uint32_T maxIndex ) ; extern
uint32_T plook_bincpa ( real_T u , const real_T bp [ ] , uint32_T maxIndex ,
real_T * fraction , uint32_T * prevIndex ) ; extern real_T intrp2d_la_pw (
const uint32_T bpIndex [ ] , const real_T frac [ ] , const real_T table [ ] ,
const uint32_T stride , const uint32_T maxIndex [ ] ) ; extern int32_T
binsearch_s32d_prevIdx ( real_T u , const real_T bp [ ] , uint32_T startIndex
, uint32_T maxIndex ) ; extern uint32_T binsearch_u32d_prevIdx ( real_T u ,
const real_T bp [ ] , uint32_T startIndex , uint32_T maxIndex ) ; extern
uint32_T linsearch_u32d ( real_T u , const real_T bp [ ] , uint32_T
startIndex ) ; extern uint32_T plook_linxp ( real_T u , const real_T bp [ ] ,
uint32_T maxIndex , real_T * fraction , uint32_T * prevIndex ) ; extern void
dccrmaip5g ( pgkaf0422g * localB , flaan3t1j0 * localP , mluofao15w * localX
) ; extern void h30xczttjx ( flaan3t1j0 * localP , mluofao15w * localX ) ;
extern void m4ya0h344x ( SimStruct * rtS_p , ettvb3gcku * localDW ,
otn2c1vhrz * localXdis ) ; extern void n5ulcqjymx ( real_T n5cmxniiio ,
ettvb3gcku * localDW , otn2c1vhrz * localXdis , b3t4dgphal * localXdot ) ;
extern void aeerk0ugtn ( real_T n5cmxniiio , ettvb3gcku * localDW ,
flaan3t1j0 * localP , mluofao15w * localX , grkqfjgiva * localZCSV , real_T
rtp_d_m ) ; extern void jlqimm0fxt ( ettvb3gcku * localDW ) ; extern void
ilmclpxlmx ( SimStruct * rtS_m , real_T n5cmxniiio , ettvb3gcku * localDW ,
flaan3t1j0 * localP , mluofao15w * localX , real_T rtp_d_m ) ; extern void
mirhnoymfv ( SimStruct * rtS_i , boolean_T amwham150v , pgkaf0422g * localB ,
ettvb3gcku * localDW , flaan3t1j0 * localP , mluofao15w * localX , real_T
rtp_d_m , otn2c1vhrz * localXdis ) ; extern void narklmqo2k ( SimStruct *
rtS_c ) ; extern void ki0yuzsymz ( pg51jubuyw * localDW , jjovxzpofc * localP
, const real_T rtp_LUTM0 [ 322080 ] , const real_T rtp_P_bp [ 88 ] , const
real_T rtp_V_bp_M0 [ 60 ] , const real_T rtp_SOS_bp [ 61 ] , const real_T
rtp_LUTM1 [ 322080 ] , const real_T rtp_V_bp_M1 [ 60 ] , const real_T
rtp_LUTM2 [ 322080 ] , const real_T rtp_V_bp_M2 [ 60 ] , const real_T
rtp_LUTM3 [ 322080 ] , const real_T rtp_V_bp_M3 [ 60 ] , const real_T
rtp_LUTM4 [ 322080 ] , const real_T rtp_V_bp_M4 [ 60 ] ) ; extern void
ljzopg4h4v ( SimStruct * rtS_m , pg51jubuyw * localDW ) ; extern void
cjigmwyptg ( SimStruct * rtS_k , real_T o3iupu2hjo , real_T hast0zswvx ,
real_T llfz1vqscn , real_T pn0etbvpux , real_T * is3kvqln30 , pg51jubuyw *
localDW ) ;
#if defined(MULTITASKING)
#error Models using the variable step solvers cannot define MULTITASKING
#endif
#endif
