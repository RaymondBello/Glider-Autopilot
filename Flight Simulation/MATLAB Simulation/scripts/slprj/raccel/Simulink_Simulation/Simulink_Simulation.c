#include "rt_logging_mmi.h"
#include "Simulink_Simulation_capi.h"
#include <math.h>
#include "Simulink_Simulation.h"
#include "Simulink_Simulation_private.h"
#include "Simulink_Simulation_dt.h"
extern void * CreateDiagnosticAsVoidPtr_wrapper ( const char * id , int nargs
, ... ) ; RTWExtModeInfo * gblRTWExtModeInfo = NULL ; extern boolean_T
gblExtModeStartPktReceived ; void raccelForceExtModeShutdown ( ) { if ( !
gblExtModeStartPktReceived ) { boolean_T stopRequested = false ;
rtExtModeWaitForStartPkt ( gblRTWExtModeInfo , 5 , & stopRequested ) ; }
rtExtModeShutdown ( 5 ) ; }
#include "slsv_diagnostic_codegen_c_api.h"
const int_T gblNumToFiles = 0 ; const int_T gblNumFrFiles = 0 ; const int_T
gblNumFrWksBlocks = 0 ;
#ifdef RSIM_WITH_SOLVER_MULTITASKING
boolean_T gbl_raccel_isMultitasking = 1 ;
#else
boolean_T gbl_raccel_isMultitasking = 0 ;
#endif
boolean_T gbl_raccel_tid01eq = 0 ; int_T gbl_raccel_NumST = 6 ; const char_T
* gbl_raccel_Version = "9.3 (R2020a) 18-Nov-2019" ; void
raccel_setup_MMIStateLog ( SimStruct * S ) {
#ifdef UseMMIDataLogging
rt_FillStateSigInfoFromMMI ( ssGetRTWLogInfo ( S ) , & ssGetErrorStatus ( S )
) ;
#else
UNUSED_PARAMETER ( S ) ;
#endif
} static DataMapInfo rt_dataMapInfo ; DataMapInfo * rt_dataMapInfoPtr = &
rt_dataMapInfo ; rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr = & (
rt_dataMapInfo . mmi ) ; const char * gblSlvrJacPatternFileName =
"slprj\\raccel\\Simulink_Simulation\\Simulink_Simulation_Jpattern.mat" ;
const int_T gblNumRootInportBlks = 0 ; const int_T gblNumModelInputs = 0 ;
extern rtInportTUtable * gblInportTUtables ; extern const char *
gblInportFileName ; extern void * gblAperiodicPartitionHitTimes ; const int_T
gblInportDataTypeIdx [ ] = { - 1 } ; const int_T gblInportDims [ ] = { - 1 }
; const int_T gblInportComplex [ ] = { - 1 } ; const int_T
gblInportInterpoFlag [ ] = { - 1 } ; const int_T gblInportContinuous [ ] = {
- 1 } ; int_T enableFcnCallFlag [ ] = { 1 , 1 , 1 , 1 , 1 , 1 } ; const char
* raccelLoadInputsAndAperiodicHitTimes ( const char * inportFileName , int *
matFileFormat ) { return rt_RapidReadInportsMatFile ( inportFileName ,
matFileFormat , 1 ) ; }
#include "simstruc.h"
#include "fixedpoint.h"
B rtB ; X rtX ; PeriodicIndX rtPeriodicIndX ; PeriodicRngX rtPeriodicRngX ;
DW rtDW ; static SimStruct model_S ; SimStruct * const rtS = & model_S ;
real_T look_SplNLinSScd ( uint32_T numDims , const real_T * u , const
rt_LUTSplineWork * const SWork ) { rt_LUTnWork * const TWork_look = SWork ->
m_TWork ; real_T * const fraction = ( real_T * ) TWork_look -> m_bpLambda ;
uint32_T * const bpIdx = TWork_look -> m_bpIndex ; const uint32_T * const
maxIndex = TWork_look -> m_maxIndex ; uint32_T k ; for ( k = 0U ; k < numDims
; k ++ ) { const real_T * const bpData = ( ( const real_T * const * )
TWork_look -> m_bpDataSet ) [ k ] ; bpIdx [ k ] = plook_linxp ( u [ k ] ,
bpData , maxIndex [ k ] , & fraction [ k ] , & bpIdx [ k ] ) ; } return (
intrp_NSplcd ( numDims , SWork , 3U ) ) ; } void rt_Spline2Derivd ( const
real_T * x , const real_T * y , uint32_T n , real_T * u , real_T * y2 ) {
real_T p , qn , sig , un ; uint32_T n1 , i , k ; n1 = n - 1U ; y2 [ 0U ] =
0.0 ; u [ 0U ] = 0.0 ; for ( i = 1U ; i < n1 ; i ++ ) { real_T dxm1 = x [ i ]
- x [ i - 1U ] ; real_T dxp1 = x [ i + 1U ] - x [ i ] ; real_T dxpm = dxp1 +
dxm1 ; sig = dxm1 / dxpm ; p = ( sig * y2 [ i - 1U ] ) + 2.0 ; y2 [ i ] = (
sig - 1.0 ) / p ; u [ i ] = ( ( y [ i + 1U ] - y [ i ] ) / dxp1 ) - ( ( y [ i
] - y [ i - 1U ] ) / dxm1 ) ; u [ i ] = ( ( ( 6.0 * u [ i ] ) / dxpm ) - (
sig * u [ i - 1U ] ) ) / p ; } qn = 0.0 ; un = 0.0 ; y2 [ n1 ] = ( un - ( qn
* u [ n1 - 1U ] ) ) / ( ( qn * y2 [ n1 - 1U ] ) + 1.0 ) ; for ( k = n1 ; k >
0U ; k -- ) { y2 [ k - 1U ] = ( y2 [ k - 1U ] * y2 [ k ] ) + u [ k - 1U ] ; }
return ; } real_T intrp_NSplcd ( uint32_T numDims , const rt_LUTSplineWork *
const splWork , uint32_T extrapMethod ) { uint32_T il ; uint32_T iu , k , i ;
real_T h , s , p , smsq , pmsq ; const rt_LUTnWork * TWork_interp = ( const
rt_LUTnWork * ) splWork -> m_TWork ; const real_T * fraction = ( real_T * )
TWork_interp -> m_bpLambda ; const real_T * yp = ( real_T * ) TWork_interp ->
m_tableData ; real_T * yyA = ( real_T * ) splWork -> m_yyA ; real_T * yyB = (
real_T * ) splWork -> m_yyB ; real_T * yy2 = ( real_T * ) splWork -> m_yy2 ;
real_T * up = ( real_T * ) splWork -> m_up ; real_T * y2 = ( real_T * )
splWork -> m_y2 ; uint8_T * reCalc = splWork -> m_reCalc ; const real_T * *
bpDataSet = ( const real_T * * ) TWork_interp -> m_bpDataSet ; const real_T *
xp = bpDataSet [ 0U ] ; real_T * yy = yyA ; uint32_T bufBank = 0U ; uint32_T
len = TWork_interp -> m_maxIndex [ 0U ] + 1U ; if ( * reCalc == 1 ) { for ( i
= 0U ; i < splWork -> m_numYWorkElts [ 0U ] ; i ++ ) { rt_Spline2Derivd ( xp
, yp , len , up , y2 ) ; yp = & yp [ len ] ; y2 = & y2 [ len ] ; } yp = (
const real_T * ) TWork_interp -> m_tableData ; y2 = ( real_T * ) splWork ->
m_y2 ; } * reCalc = 0 ; for ( k = 0U ; k < numDims ; k ++ ) { xp = bpDataSet
[ k ] ; len = TWork_interp -> m_maxIndex [ k ] + 1U ; il = TWork_interp ->
m_bpIndex [ k ] ; iu = il + 1U ; h = xp [ iu ] - xp [ il ] ; p = fraction [ k
] ; s = 1.0 - p ; pmsq = p * ( ( p * p ) - 1.0 ) ; smsq = s * ( ( s * s ) -
1.0 ) ; if ( ( p > 1.0 ) && ( extrapMethod == 2U ) ) { real_T slope ; for ( i
= 0U ; i < splWork -> m_numYWorkElts [ k ] ; i ++ ) { slope = ( yp [ iu ] -
yp [ il ] ) + ( ( y2 [ il ] * h * h ) * ( 1.0 / 6.0 ) ) ; yy [ i ] = yp [ iu
] + ( slope * ( p - 1.0 ) ) ; yp = & yp [ len ] ; y2 = & y2 [ len ] ; } }
else if ( ( p < 0.0 ) && ( extrapMethod == 2U ) ) { real_T slope ; for ( i =
0U ; i < splWork -> m_numYWorkElts [ k ] ; i ++ ) { slope = ( yp [ iu ] - yp
[ il ] ) - ( ( y2 [ iu ] * h * h ) * ( 1.0 / 6.0 ) ) ; yy [ i ] = yp [ il ] +
( slope * p ) ; yp = & yp [ len ] ; y2 = & y2 [ len ] ; } } else { for ( i =
0U ; i < splWork -> m_numYWorkElts [ k ] ; i ++ ) { yy [ i ] = yp [ il ] + p
* ( yp [ iu ] - yp [ il ] ) + ( ( smsq * y2 [ il ] + pmsq * y2 [ iu ] ) * h *
h ) * ( 1.0 / 6.0 ) ; yp = & yp [ len ] ; y2 = & y2 [ len ] ; } } yp = yy ;
y2 = yy2 ; if ( splWork -> m_numYWorkElts [ k + 1U ] > 0U ) { uint32_T
nextLen = TWork_interp -> m_maxIndex [ k + 1U ] + 1U ; const real_T * nextXp
= bpDataSet [ k + 1U ] ; for ( i = 0U ; i < splWork -> m_numYWorkElts [ k +
1U ] ; i ++ ) { rt_Spline2Derivd ( nextXp , yp , nextLen , up , y2 ) ; yp = &
yp [ nextLen ] ; y2 = & y2 [ nextLen ] ; } } yp = yy ; y2 = yy2 ; if (
bufBank == 0U ) { yy = yyA ; bufBank = 1U ; } else { yy = yyB ; bufBank = 0U
; } } return ( yp [ 0U ] ) ; } void InitCalcAtmosCOESA ( real_T *
temperature76 , real_T * pressureRatio76 ) { if ( temperature76 [ 0 ] !=
TEMPERATURE0 ) { int_T k ; temperature76 [ 0 ] = TEMPERATURE0 ;
pressureRatio76 [ 0 ] = 1.0 ; for ( k = 0 ; k < ( NUM1976PTS - 1 ) ; k ++ ) {
if ( tempGradient76 [ k ] != 0.0 ) { temperature76 [ k + 1 ] = temperature76
[ k ] + tempGradient76 [ k ] * ( altitude76 [ k + 1 ] - altitude76 [ k ] ) ;
pressureRatio76 [ k + 1 ] = pressureRatio76 [ k ] * muDoubleScalarExp (
muDoubleScalarLog ( temperature76 [ k ] / temperature76 [ k + 1 ] ) * GMR /
tempGradient76 [ k ] ) ; } else { temperature76 [ k + 1 ] = temperature76 [ k
] ; pressureRatio76 [ k + 1 ] = pressureRatio76 [ k ] * muDoubleScalarExp ( (
- GMR ) * ( altitude76 [ k + 1 ] - altitude76 [ k ] ) / temperature76 [ k ] )
; } } } } void CalcAtmosCOESA ( const real_T * altitude , real_T * temp ,
real_T * pressure , real_T * density , real_T * speedofsound , real_T *
temperature76 , real_T * pressureRatio76 , int_T numPoints ) { int_T i ; for
( i = 0 ; i < numPoints ; i ++ ) { int_T bottom = 0 ; int_T top = NUM1976PTS
- 1 ; int_T idx ; if ( altitude [ i ] <= altitude76 [ bottom ] ) { idx =
bottom ; } else if ( altitude [ i ] >= altitude76 [ top ] ) { idx =
NUM1976PTS - 2 ; } else { for ( ; ; ) { idx = ( bottom + top ) / 2 ; if (
altitude [ i ] < altitude76 [ idx ] ) { top = idx - 1 ; } else if ( altitude
[ i ] >= altitude76 [ idx + 1 ] ) { bottom = idx + 1 ; } else { break ; } } }
if ( tempGradient76 [ idx ] != 0.0 ) { temp [ i ] = temperature76 [ idx ] +
tempGradient76 [ idx ] * ( altitude [ i ] - altitude76 [ idx ] ) ; pressure [
i ] = PRESSURE0 * pressureRatio76 [ idx ] * muDoubleScalarPower (
temperature76 [ idx ] / temp [ i ] , GMR / tempGradient76 [ idx ] ) ; } else
{ temp [ i ] = temperature76 [ idx ] ; pressure [ i ] = PRESSURE0 *
pressureRatio76 [ idx ] * muDoubleScalarExp ( ( - GMR ) * ( altitude [ i ] -
altitude76 [ idx ] ) / temperature76 [ idx ] ) ; } density [ i ] = pressure [
i ] / ( ( R_HAT / MOL_WT ) * temp [ i ] ) ; speedofsound [ i ] =
muDoubleScalarSqrt ( GAMMA * temp [ i ] * ( R_HAT / MOL_WT ) ) ; } } void
CalcPAltCOESA ( const real_T * pressure , real_T * altitude , real_T *
temperature76 , real_T * pressureRatio76 , int_T numPoints ) { int_T i ;
real_T ptemp ; for ( i = 0 ; i < numPoints ; i ++ ) { int_T bottom = 0 ;
int_T top = NUM1976PTS - 1 ; int_T idx ; if ( pressure [ i ] >=
pressureRatio76 [ bottom ] * PRESSURE0 ) { idx = bottom ; } else if (
pressure [ i ] <= pressureRatio76 [ top ] * PRESSURE0 ) { idx = NUM1976PTS -
2 ; } else { for ( ; ; ) { idx = ( bottom + top ) / 2 ; if ( pressure [ i ] >
pressureRatio76 [ idx ] * PRESSURE0 ) { top = idx - 1 ; } else if ( pressure
[ i ] <= pressureRatio76 [ idx + 1 ] * PRESSURE0 ) { bottom = idx + 1 ; }
else { break ; } } } if ( pressure [ i ] == ( PRESSURE0 * pressureRatio76 [
idx ] ) ) { altitude [ i ] = altitude76 [ idx ] ; } else { if (
tempGradient76 [ idx ] != 0.0 ) { ptemp = muDoubleScalarPower ( pressure [ i
] / ( PRESSURE0 * pressureRatio76 [ idx ] ) , ( tempGradient76 [ idx ] / GMR
) ) ; altitude [ i ] = altitude76 [ idx ] + ( ( 1.0 - ptemp ) / (
tempGradient76 [ idx ] * ptemp ) ) * temperature76 [ idx ] ; } else {
altitude [ i ] = altitude76 [ idx ] - ( ( temperature76 [ idx ] / GMR ) *
muDoubleScalarLog ( pressure [ i ] / ( PRESSURE0 * pressureRatio76 [ idx ] )
) ) ; } } } } int32_T plook_s32dd_binxp ( real_T u , const real_T bp [ ] ,
uint32_T maxIndex , real_T * fraction , int32_T * prevIndex ) { int32_T
bpIndex ; if ( u <= bp [ 0U ] ) { bpIndex = 0 ; * fraction = ( u - bp [ 0U ]
) / ( bp [ 1U ] - bp [ 0U ] ) ; } else if ( u < bp [ maxIndex ] ) { bpIndex =
binsearch_s32d_prevIdx ( u , bp , ( uint32_T ) * prevIndex , maxIndex ) ; *
fraction = ( u - bp [ ( uint32_T ) bpIndex ] ) / ( bp [ bpIndex + 1U ] - bp [
( uint32_T ) bpIndex ] ) ; } else { bpIndex = ( int32_T ) ( maxIndex - 1U ) ;
* fraction = ( u - bp [ maxIndex - 1U ] ) / ( bp [ maxIndex ] - bp [ maxIndex
- 1U ] ) ; } * prevIndex = bpIndex ; return bpIndex ; } real_T
intrp3d_s32dl_pw ( const int32_T bpIndex [ ] , const real_T frac [ ] , const
real_T table [ ] , const uint32_T stride [ ] ) { real_T yL_2d ; uint32_T
offset_2d ; real_T yL_1d ; uint32_T offset_0d ; offset_2d = ( ( uint32_T ) (
bpIndex [ 2U ] * ( int32_T ) stride [ 2U ] ) + bpIndex [ 1U ] * ( int32_T )
stride [ 1U ] ) + bpIndex [ 0U ] ; yL_1d = ( table [ offset_2d + 1U ] - table
[ offset_2d ] ) * frac [ 0U ] + table [ offset_2d ] ; offset_0d = offset_2d +
stride [ 1U ] ; yL_2d = ( ( ( table [ offset_0d + 1U ] - table [ offset_0d ]
) * frac [ 0U ] + table [ offset_0d ] ) - yL_1d ) * frac [ 1U ] + yL_1d ;
offset_2d += stride [ 2U ] ; yL_1d = ( table [ offset_2d + 1U ] - table [
offset_2d ] ) * frac [ 0U ] + table [ offset_2d ] ; offset_0d = offset_2d +
stride [ 1U ] ; return ( ( ( ( ( table [ offset_0d + 1U ] - table [ offset_0d
] ) * frac [ 0U ] + table [ offset_0d ] ) - yL_1d ) * frac [ 1U ] + yL_1d ) -
yL_2d ) * frac [ 2U ] + yL_2d ; } real_T intrp4d_s32dl_pw ( const int32_T
bpIndex [ ] , const real_T frac [ ] , const real_T table [ ] , const uint32_T
stride [ ] ) { real_T yL_3d ; uint32_T offset_3d ; real_T yL_2d ; real_T
yL_1d ; uint32_T offset_0d ; uint32_T offset_1d ; offset_3d = ( ( ( uint32_T
) ( bpIndex [ 3U ] * ( int32_T ) stride [ 3U ] ) + bpIndex [ 2U ] * ( int32_T
) stride [ 2U ] ) + bpIndex [ 1U ] * ( int32_T ) stride [ 1U ] ) + bpIndex [
0U ] ; yL_1d = ( table [ offset_3d + 1U ] - table [ offset_3d ] ) * frac [ 0U
] + table [ offset_3d ] ; offset_0d = offset_3d + stride [ 1U ] ; yL_2d = ( (
( table [ offset_0d + 1U ] - table [ offset_0d ] ) * frac [ 0U ] + table [
offset_0d ] ) - yL_1d ) * frac [ 1U ] + yL_1d ; offset_1d = offset_3d +
stride [ 2U ] ; yL_1d = ( table [ offset_1d + 1U ] - table [ offset_1d ] ) *
frac [ 0U ] + table [ offset_1d ] ; offset_0d = offset_1d + stride [ 1U ] ;
yL_3d = ( ( ( ( ( table [ offset_0d + 1U ] - table [ offset_0d ] ) * frac [
0U ] + table [ offset_0d ] ) - yL_1d ) * frac [ 1U ] + yL_1d ) - yL_2d ) *
frac [ 2U ] + yL_2d ; offset_1d = offset_3d + stride [ 3U ] ; yL_1d = ( table
[ offset_1d + 1U ] - table [ offset_1d ] ) * frac [ 0U ] + table [ offset_1d
] ; offset_0d = offset_1d + stride [ 1U ] ; yL_2d = ( ( ( table [ offset_0d +
1U ] - table [ offset_0d ] ) * frac [ 0U ] + table [ offset_0d ] ) - yL_1d )
* frac [ 1U ] + yL_1d ; offset_1d += stride [ 2U ] ; yL_1d = ( table [
offset_1d + 1U ] - table [ offset_1d ] ) * frac [ 0U ] + table [ offset_1d ]
; offset_0d = offset_1d + stride [ 1U ] ; return ( ( ( ( ( ( ( table [
offset_0d + 1U ] - table [ offset_0d ] ) * frac [ 0U ] + table [ offset_0d ]
) - yL_1d ) * frac [ 1U ] + yL_1d ) - yL_2d ) * frac [ 2U ] + yL_2d ) - yL_3d
) * frac [ 3U ] + yL_3d ; } real_T look1_plinlcpw ( real_T u0 , const real_T
bp0 [ ] , const real_T table [ ] , uint32_T prevIndex [ ] , uint32_T maxIndex
) { real_T frac ; uint32_T bpIdx ; if ( u0 <= bp0 [ 0U ] ) { bpIdx = 0U ;
frac = 0.0 ; } else if ( u0 < bp0 [ maxIndex ] ) { for ( bpIdx = prevIndex [
0U ] ; u0 < bp0 [ bpIdx ] ; bpIdx -- ) { } while ( u0 >= bp0 [ bpIdx + 1U ] )
{ bpIdx ++ ; } frac = ( u0 - bp0 [ bpIdx ] ) / ( bp0 [ bpIdx + 1U ] - bp0 [
bpIdx ] ) ; } else { bpIdx = maxIndex - 1U ; frac = 1.0 ; } prevIndex [ 0U ]
= bpIdx ; return ( table [ bpIdx + 1U ] - table [ bpIdx ] ) * frac + table [
bpIdx ] ; } uint32_T plook_bincpa ( real_T u , const real_T bp [ ] , uint32_T
maxIndex , real_T * fraction , uint32_T * prevIndex ) { uint32_T bpIndex ; if
( u <= bp [ 0U ] ) { bpIndex = 0U ; * fraction = 0.0 ; } else if ( u < bp [
maxIndex ] ) { bpIndex = binsearch_u32d_prevIdx ( u , bp , * prevIndex ,
maxIndex ) ; * fraction = ( u - bp [ bpIndex ] ) / ( bp [ bpIndex + 1U ] - bp
[ bpIndex ] ) ; } else { bpIndex = maxIndex ; * fraction = 0.0 ; } *
prevIndex = bpIndex ; return bpIndex ; } real_T intrp2d_la_pw ( const
uint32_T bpIndex [ ] , const real_T frac [ ] , const real_T table [ ] , const
uint32_T stride , const uint32_T maxIndex [ ] ) { real_T y ; real_T yR_1d ;
uint32_T offset_1d ; offset_1d = bpIndex [ 1U ] * stride + bpIndex [ 0U ] ;
if ( bpIndex [ 0U ] == maxIndex [ 0U ] ) { y = table [ offset_1d ] ; } else {
y = ( table [ offset_1d + 1U ] - table [ offset_1d ] ) * frac [ 0U ] + table
[ offset_1d ] ; } if ( bpIndex [ 1U ] == maxIndex [ 1U ] ) { } else {
offset_1d += stride ; if ( bpIndex [ 0U ] == maxIndex [ 0U ] ) { yR_1d =
table [ offset_1d ] ; } else { yR_1d = ( table [ offset_1d + 1U ] - table [
offset_1d ] ) * frac [ 0U ] + table [ offset_1d ] ; } y += ( yR_1d - y ) *
frac [ 1U ] ; } return y ; } int32_T binsearch_s32d_prevIdx ( real_T u ,
const real_T bp [ ] , uint32_T startIndex , uint32_T maxIndex ) { uint32_T
iRght ; uint32_T iLeft ; uint32_T bpIdx ; uint32_T found ; bpIdx = startIndex
; iLeft = 0U ; iRght = maxIndex ; found = 0U ; while ( found == 0U ) { if ( u
< bp [ bpIdx ] ) { iRght = bpIdx - 1U ; bpIdx = ( iRght + iLeft ) >> 1U ; }
else if ( u < bp [ bpIdx + 1U ] ) { found = 1U ; } else { iLeft = bpIdx + 1U
; bpIdx = ( iRght + iLeft ) >> 1U ; } } return ( int32_T ) bpIdx ; } uint32_T
binsearch_u32d_prevIdx ( real_T u , const real_T bp [ ] , uint32_T startIndex
, uint32_T maxIndex ) { uint32_T bpIndex ; uint32_T iRght ; uint32_T iLeft ;
uint32_T found ; bpIndex = startIndex ; iLeft = 0U ; iRght = maxIndex ; found
= 0U ; while ( found == 0U ) { if ( u < bp [ bpIndex ] ) { iRght = bpIndex -
1U ; bpIndex = ( iRght + iLeft ) >> 1U ; } else if ( u < bp [ bpIndex + 1U ]
) { found = 1U ; } else { iLeft = bpIndex + 1U ; bpIndex = ( iRght + iLeft )
>> 1U ; } } return bpIndex ; } uint32_T linsearch_u32d ( real_T u , const
real_T bp [ ] , uint32_T startIndex ) { uint32_T bpIndex ; for ( bpIndex =
startIndex ; u < bp [ bpIndex ] ; bpIndex -- ) { } while ( u >= bp [ bpIndex
+ 1U ] ) { bpIndex ++ ; } return bpIndex ; } uint32_T plook_linxp ( real_T u
, const real_T bp [ ] , uint32_T maxIndex , real_T * fraction , uint32_T *
prevIndex ) { uint32_T bpIndex ; uint32_T startIndex ; if ( u <= bp [ 0U ] )
{ bpIndex = 0U ; * fraction = ( u - bp [ 0U ] ) / ( bp [ 1U ] - bp [ 0U ] ) ;
} else if ( u < bp [ maxIndex ] ) { startIndex = * prevIndex ; bpIndex =
linsearch_u32d ( u , bp , startIndex ) ; * fraction = ( u - bp [ bpIndex ] )
/ ( bp [ bpIndex + 1U ] - bp [ bpIndex ] ) ; } else { bpIndex = maxIndex - 1U
; * fraction = ( u - bp [ maxIndex - 1U ] ) / ( bp [ maxIndex ] - bp [
maxIndex - 1U ] ) ; } * prevIndex = bpIndex ; return bpIndex ; } void
dccrmaip5g ( pgkaf0422g * localB , flaan3t1j0 * localP , mluofao15w * localX
) { localX -> peioropukt = localP -> DistanceintoGustxLimitedtogustlengthd_IC
; localB -> cwtrsfu0ve = localP -> x_Y0 ; } void h30xczttjx ( flaan3t1j0 *
localP , mluofao15w * localX ) { localX -> peioropukt = localP ->
DistanceintoGustxLimitedtogustlengthd_IC ; } void jlqimm0fxt ( ettvb3gcku *
localDW ) { localDW -> ltfrc3b41y = false ; } void m4ya0h344x ( SimStruct *
rtS_p , ettvb3gcku * localDW , otn2c1vhrz * localXdis ) { localDW ->
ltfrc3b41y = false ; ssSetBlockStateForSolverChangedAtMajorStep ( rtS_p ) ;
localXdis -> peioropukt = 1 ; } void mirhnoymfv ( SimStruct * rtS_i ,
boolean_T amwham150v , pgkaf0422g * localB , ettvb3gcku * localDW ,
flaan3t1j0 * localP , mluofao15w * localX , real_T rtp_d_m , otn2c1vhrz *
localXdis ) { if ( ssIsSampleHit ( rtS_i , 1 , 0 ) && ssIsMajorTimeStep (
rtS_i ) ) { if ( amwham150v ) { if ( ! localDW -> ltfrc3b41y ) { if (
ssGetTaskTime ( rtS_i , 1 ) != ssGetTStart ( rtS_i ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS_i ) ; } localXdis ->
peioropukt = 0 ; h30xczttjx ( localP , localX ) ; localDW -> ltfrc3b41y =
true ; } } else { if ( localDW -> ltfrc3b41y ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS_i ) ; localXdis ->
peioropukt = 1 ; jlqimm0fxt ( localDW ) ; } } } if ( localDW -> ltfrc3b41y )
{ if ( ssIsMajorTimeStep ( rtS_i ) ) { if ( localX -> peioropukt >= rtp_d_m )
{ if ( localX -> peioropukt > rtp_d_m ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS_i ) ; } localX -> peioropukt
= rtp_d_m ; } else { if ( localX -> peioropukt <= localP ->
DistanceintoGustxLimitedtogustlengthd_LowerSat ) { if ( localX -> peioropukt
< localP -> DistanceintoGustxLimitedtogustlengthd_LowerSat ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS_i ) ; } localX -> peioropukt
= localP -> DistanceintoGustxLimitedtogustlengthd_LowerSat ; } } } localB ->
cwtrsfu0ve = localX -> peioropukt ; if ( ssIsMajorTimeStep ( rtS_i ) ) {
srUpdateBC ( localDW -> jyd1ppenco ) ; } } } void ilmclpxlmx ( SimStruct *
rtS_m , real_T n5cmxniiio , ettvb3gcku * localDW , flaan3t1j0 * localP ,
mluofao15w * localX , real_T rtp_d_m ) { if ( localDW -> ltfrc3b41y ) { if (
localX -> peioropukt == rtp_d_m ) { switch ( localDW -> kaoqin3vcx ) { case 3
: if ( n5cmxniiio < 0.0 ) { ssSetBlockStateForSolverChangedAtMajorStep (
rtS_m ) ; localDW -> kaoqin3vcx = 1 ; } break ; case 1 : if ( n5cmxniiio >=
0.0 ) { localDW -> kaoqin3vcx = 3 ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS_m ) ; } break ; default :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS_m ) ; if ( n5cmxniiio < 0.0
) { localDW -> kaoqin3vcx = 1 ; } else { localDW -> kaoqin3vcx = 3 ; } break
; } } else if ( localX -> peioropukt == localP ->
DistanceintoGustxLimitedtogustlengthd_LowerSat ) { switch ( localDW ->
kaoqin3vcx ) { case 4 : if ( n5cmxniiio > 0.0 ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS_m ) ; localDW -> kaoqin3vcx
= 2 ; } break ; case 2 : if ( n5cmxniiio <= 0.0 ) { localDW -> kaoqin3vcx = 4
; ssSetBlockStateForSolverChangedAtMajorStep ( rtS_m ) ; } break ; default :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS_m ) ; if ( n5cmxniiio > 0.0
) { localDW -> kaoqin3vcx = 2 ; } else { localDW -> kaoqin3vcx = 4 ; } break
; } } else { localDW -> kaoqin3vcx = 0 ; } } } void n5ulcqjymx ( real_T
n5cmxniiio , ettvb3gcku * localDW , otn2c1vhrz * localXdis , b3t4dgphal *
localXdot ) { if ( localDW -> ltfrc3b41y ) { if ( ( localDW -> kaoqin3vcx !=
3 ) && ( localDW -> kaoqin3vcx != 4 ) ) { localXdot -> peioropukt =
n5cmxniiio ; localXdis -> peioropukt = false ; } else { localXdot ->
peioropukt = 0.0 ; localXdis -> peioropukt = ( ( localDW -> kaoqin3vcx == 3 )
|| ( localDW -> kaoqin3vcx == 4 ) || localXdis -> peioropukt ) ; } } else {
localXdot -> peioropukt = 0.0 ; } } void aeerk0ugtn ( real_T n5cmxniiio ,
ettvb3gcku * localDW , flaan3t1j0 * localP , mluofao15w * localX , grkqfjgiva
* localZCSV , real_T rtp_d_m ) { if ( localDW -> ltfrc3b41y ) { if ( (
localDW -> kaoqin3vcx == 1 ) && ( localX -> peioropukt >= rtp_d_m ) ) {
localZCSV -> anzwhaorff = 0.0 ; } else { localZCSV -> anzwhaorff = localX ->
peioropukt - rtp_d_m ; } if ( ( localDW -> kaoqin3vcx == 2 ) && ( localX ->
peioropukt <= localP -> DistanceintoGustxLimitedtogustlengthd_LowerSat ) ) {
localZCSV -> fkyjdvnvd5 = 0.0 ; } else { localZCSV -> fkyjdvnvd5 = localX ->
peioropukt - localP -> DistanceintoGustxLimitedtogustlengthd_LowerSat ; } if
( ( localDW -> kaoqin3vcx == 3 ) || ( localDW -> kaoqin3vcx == 4 ) ) {
localZCSV -> bz1j3sgcos = n5cmxniiio ; } else { localZCSV -> bz1j3sgcos = 0.0
; } } else { { real_T * zcsv = & ( localZCSV -> anzwhaorff ) ; int_T i ; for
( i = 0 ; i < 3 ; i ++ ) { zcsv [ i ] = 0.0 ; } } } } void narklmqo2k (
SimStruct * rtS_c ) { if ( ssGetTaskTime ( rtS_c , 1 ) != ssGetTStart ( rtS_c
) ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS_c ) ; } } void
ljzopg4h4v ( SimStruct * rtS_m , pg51jubuyw * localDW ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS_m ) ; switch ( localDW ->
gcr4x1ri1s ) { case 0 : ssSetBlockStateForSolverChangedAtMajorStep ( rtS_m )
; break ; case 1 : ssSetBlockStateForSolverChangedAtMajorStep ( rtS_m ) ;
break ; case 2 : ssSetBlockStateForSolverChangedAtMajorStep ( rtS_m ) ; break
; case 3 : ssSetBlockStateForSolverChangedAtMajorStep ( rtS_m ) ; break ;
case 4 : ssSetBlockStateForSolverChangedAtMajorStep ( rtS_m ) ; break ; }
localDW -> gcr4x1ri1s = - 1 ; } void ki0yuzsymz ( pg51jubuyw * localDW ,
jjovxzpofc * localP , const real_T rtp_LUTM0 [ 322080 ] , const real_T
rtp_P_bp [ 88 ] , const real_T rtp_V_bp_M0 [ 60 ] , const real_T rtp_SOS_bp [
61 ] , const real_T rtp_LUTM1 [ 322080 ] , const real_T rtp_V_bp_M1 [ 60 ] ,
const real_T rtp_LUTM2 [ 322080 ] , const real_T rtp_V_bp_M2 [ 60 ] , const
real_T rtp_LUTM3 [ 322080 ] , const real_T rtp_V_bp_M3 [ 60 ] , const real_T
rtp_LUTM4 [ 322080 ] , const real_T rtp_V_bp_M4 [ 60 ] ) { localDW ->
gcr4x1ri1s = - 1 ; { rt_LUTnWork * TWork_start = ( rt_LUTnWork * ) & localDW
-> k5z3ljbn5f [ 0 ] ; void * * bpDataSet = ( void * * ) & localDW ->
hmckm540f5 [ 0 ] ; TWork_start -> m_dimSizes = ( const uint32_T * ) localP ->
M0_dimSizes ; TWork_start -> m_tableData = ( void * ) rtp_LUTM0 ; TWork_start
-> m_bpDataSet = bpDataSet ; TWork_start -> m_bpIndex = & localDW ->
bbfw2gdxcn [ 0 ] ; TWork_start -> m_bpLambda = & localDW -> mucstbyucg [ 0 ]
; TWork_start -> m_maxIndex = ( const uint32_T * ) localP -> M0_maxIndex ;
bpDataSet [ 0 ] = ( void * ) rtp_P_bp ; bpDataSet [ 1 ] = ( void * )
rtp_V_bp_M0 ; bpDataSet [ 2 ] = ( void * ) rtp_SOS_bp ; } { rt_LUTSplineWork
* rt_SplWk = ( rt_LUTSplineWork * ) & localDW -> nzik33yqcn [ 0 ] ; rt_SplWk
-> m_TWork = ( rt_LUTnWork * ) & localDW -> k5z3ljbn5f [ 0 ] ; rt_SplWk ->
m_yyA = & localDW -> lvndzs1soq [ 0 ] ; rt_SplWk -> m_yyB = & localDW ->
pcfohnqayd [ 0 ] ; rt_SplWk -> m_yy2 = & localDW -> gvwl2w40zt [ 0 ] ;
rt_SplWk -> m_up = & localDW -> llvsxfyht1 [ 0 ] ; rt_SplWk -> m_y2 = &
localDW -> c1emqgsxgf [ 0 ] ; rt_SplWk -> m_numYWorkElts = localP ->
M0_numYWorkElts ; rt_SplWk -> m_reCalc = & localDW -> fg35qxl3vi ; * rt_SplWk
-> m_reCalc = 1 ; } { rt_LUTnWork * TWork_start = ( rt_LUTnWork * ) & localDW
-> eogjamfv2u [ 0 ] ; void * * bpDataSet = ( void * * ) & localDW ->
czreq4mspp [ 0 ] ; TWork_start -> m_dimSizes = ( const uint32_T * ) localP ->
M1_dimSizes ; TWork_start -> m_tableData = ( void * ) rtp_LUTM1 ; TWork_start
-> m_bpDataSet = bpDataSet ; TWork_start -> m_bpIndex = & localDW ->
li4rk12cor [ 0 ] ; TWork_start -> m_bpLambda = & localDW -> kbkjlm3ow2 [ 0 ]
; TWork_start -> m_maxIndex = ( const uint32_T * ) localP -> M1_maxIndex ;
bpDataSet [ 0 ] = ( void * ) rtp_P_bp ; bpDataSet [ 1 ] = ( void * )
rtp_V_bp_M1 ; bpDataSet [ 2 ] = ( void * ) rtp_SOS_bp ; } { rt_LUTSplineWork
* rt_SplWk = ( rt_LUTSplineWork * ) & localDW -> cughybvon0 [ 0 ] ; rt_SplWk
-> m_TWork = ( rt_LUTnWork * ) & localDW -> eogjamfv2u [ 0 ] ; rt_SplWk ->
m_yyA = & localDW -> gpo4oh4yoc [ 0 ] ; rt_SplWk -> m_yyB = & localDW ->
lbrztkveec [ 0 ] ; rt_SplWk -> m_yy2 = & localDW -> hbisxcpsrg [ 0 ] ;
rt_SplWk -> m_up = & localDW -> maz32p5ch2 [ 0 ] ; rt_SplWk -> m_y2 = &
localDW -> otcphh42tj [ 0 ] ; rt_SplWk -> m_numYWorkElts = localP ->
M1_numYWorkElts ; rt_SplWk -> m_reCalc = & localDW -> czjauhqmhr ; * rt_SplWk
-> m_reCalc = 1 ; } { rt_LUTnWork * TWork_start = ( rt_LUTnWork * ) & localDW
-> ddpi0aehhj [ 0 ] ; void * * bpDataSet = ( void * * ) & localDW ->
czje14zbes [ 0 ] ; TWork_start -> m_dimSizes = ( const uint32_T * ) localP ->
M2_dimSizes ; TWork_start -> m_tableData = ( void * ) rtp_LUTM2 ; TWork_start
-> m_bpDataSet = bpDataSet ; TWork_start -> m_bpIndex = & localDW ->
fr1myblrol [ 0 ] ; TWork_start -> m_bpLambda = & localDW -> o3sl5fyijx [ 0 ]
; TWork_start -> m_maxIndex = ( const uint32_T * ) localP -> M2_maxIndex ;
bpDataSet [ 0 ] = ( void * ) rtp_P_bp ; bpDataSet [ 1 ] = ( void * )
rtp_V_bp_M2 ; bpDataSet [ 2 ] = ( void * ) rtp_SOS_bp ; } { rt_LUTSplineWork
* rt_SplWk = ( rt_LUTSplineWork * ) & localDW -> bjlmawntgi [ 0 ] ; rt_SplWk
-> m_TWork = ( rt_LUTnWork * ) & localDW -> ddpi0aehhj [ 0 ] ; rt_SplWk ->
m_yyA = & localDW -> iownbpqjgp [ 0 ] ; rt_SplWk -> m_yyB = & localDW ->
htq3wzf05k [ 0 ] ; rt_SplWk -> m_yy2 = & localDW -> mhvuogwz5g [ 0 ] ;
rt_SplWk -> m_up = & localDW -> e4boznuo11 [ 0 ] ; rt_SplWk -> m_y2 = &
localDW -> oltjiqpr2r [ 0 ] ; rt_SplWk -> m_numYWorkElts = localP ->
M2_numYWorkElts ; rt_SplWk -> m_reCalc = & localDW -> dhoi4kxfar ; * rt_SplWk
-> m_reCalc = 1 ; } { rt_LUTnWork * TWork_start = ( rt_LUTnWork * ) & localDW
-> eioxfitten [ 0 ] ; void * * bpDataSet = ( void * * ) & localDW ->
mxyuzcexca [ 0 ] ; TWork_start -> m_dimSizes = ( const uint32_T * ) localP ->
M3_dimSizes ; TWork_start -> m_tableData = ( void * ) rtp_LUTM3 ; TWork_start
-> m_bpDataSet = bpDataSet ; TWork_start -> m_bpIndex = & localDW ->
pr4m2g3lsg [ 0 ] ; TWork_start -> m_bpLambda = & localDW -> l0xygmktrt [ 0 ]
; TWork_start -> m_maxIndex = ( const uint32_T * ) localP -> M3_maxIndex ;
bpDataSet [ 0 ] = ( void * ) rtp_P_bp ; bpDataSet [ 1 ] = ( void * )
rtp_V_bp_M3 ; bpDataSet [ 2 ] = ( void * ) rtp_SOS_bp ; } { rt_LUTSplineWork
* rt_SplWk = ( rt_LUTSplineWork * ) & localDW -> npajdwg2lh [ 0 ] ; rt_SplWk
-> m_TWork = ( rt_LUTnWork * ) & localDW -> eioxfitten [ 0 ] ; rt_SplWk ->
m_yyA = & localDW -> ec4ih5lgh0 [ 0 ] ; rt_SplWk -> m_yyB = & localDW ->
hf0jovm0ds [ 0 ] ; rt_SplWk -> m_yy2 = & localDW -> d2jzmsgxqy [ 0 ] ;
rt_SplWk -> m_up = & localDW -> iel1urtae2 [ 0 ] ; rt_SplWk -> m_y2 = &
localDW -> mv4eqkyxzg [ 0 ] ; rt_SplWk -> m_numYWorkElts = localP ->
M3_numYWorkElts ; rt_SplWk -> m_reCalc = & localDW -> nogzbokry4 ; * rt_SplWk
-> m_reCalc = 1 ; } { rt_LUTnWork * TWork_start = ( rt_LUTnWork * ) & localDW
-> dldelxht02 [ 0 ] ; void * * bpDataSet = ( void * * ) & localDW ->
js2t0wh1ap [ 0 ] ; TWork_start -> m_dimSizes = ( const uint32_T * ) localP ->
M4_dimSizes ; TWork_start -> m_tableData = ( void * ) rtp_LUTM4 ; TWork_start
-> m_bpDataSet = bpDataSet ; TWork_start -> m_bpIndex = & localDW ->
pkty1tadnq [ 0 ] ; TWork_start -> m_bpLambda = & localDW -> czibmh2e1r [ 0 ]
; TWork_start -> m_maxIndex = ( const uint32_T * ) localP -> M4_maxIndex ;
bpDataSet [ 0 ] = ( void * ) rtp_P_bp ; bpDataSet [ 1 ] = ( void * )
rtp_V_bp_M4 ; bpDataSet [ 2 ] = ( void * ) rtp_SOS_bp ; } { rt_LUTSplineWork
* rt_SplWk = ( rt_LUTSplineWork * ) & localDW -> jvfpfqeupc [ 0 ] ; rt_SplWk
-> m_TWork = ( rt_LUTnWork * ) & localDW -> dldelxht02 [ 0 ] ; rt_SplWk ->
m_yyA = & localDW -> bsnqpkum4j [ 0 ] ; rt_SplWk -> m_yyB = & localDW ->
ng2ho4wdw5 [ 0 ] ; rt_SplWk -> m_yy2 = & localDW -> emyp25k2ra [ 0 ] ;
rt_SplWk -> m_up = & localDW -> ke3jfhfxg1 [ 0 ] ; rt_SplWk -> m_y2 = &
localDW -> c1uyo1uy5e [ 0 ] ; rt_SplWk -> m_numYWorkElts = localP ->
M4_numYWorkElts ; rt_SplWk -> m_reCalc = & localDW -> dgmohvowou ; * rt_SplWk
-> m_reCalc = 1 ; } } void cjigmwyptg ( SimStruct * rtS_k , real_T o3iupu2hjo
, real_T hast0zswvx , real_T llfz1vqscn , real_T pn0etbvpux , real_T *
is3kvqln30 , pg51jubuyw * localDW ) { int8_T rtPrevAction ; int8_T rtAction ;
rtPrevAction = localDW -> gcr4x1ri1s ; if ( o3iupu2hjo < 1.0 ) { rtAction = 0
; } else if ( o3iupu2hjo < 2.0 ) { rtAction = 1 ; } else if ( o3iupu2hjo <
3.0 ) { rtAction = 2 ; } else if ( o3iupu2hjo < 4.0 ) { rtAction = 3 ; } else
{ rtAction = 4 ; } localDW -> gcr4x1ri1s = rtAction ; if ( rtPrevAction !=
rtAction ) { switch ( rtPrevAction ) { case 0 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS_k ) ; break ; case 1 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS_k ) ; break ; case 2 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS_k ) ; break ; case 3 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS_k ) ; break ; case 4 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS_k ) ; break ; } } switch (
rtAction ) { case 0 : if ( rtAction != rtPrevAction ) { if ( ssGetTaskTime (
rtS_k , 1 ) != ssGetTStart ( rtS_k ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS_k ) ; } } { real_T
rt_LUTuVect [ 3 ] ; rt_LUTuVect [ 0 ] = hast0zswvx ; rt_LUTuVect [ 1 ] =
llfz1vqscn ; rt_LUTuVect [ 2 ] = pn0etbvpux ; ( * is3kvqln30 ) =
look_SplNLinSScd ( 3U , rt_LUTuVect , ( rt_LUTSplineWork * ) & localDW ->
nzik33yqcn [ 0 ] ) ; } srUpdateBC ( localDW -> johe0vk44j ) ; break ; case 1
: if ( rtAction != rtPrevAction ) { if ( ssGetTaskTime ( rtS_k , 1 ) !=
ssGetTStart ( rtS_k ) ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS_k
) ; } } { real_T rt_LUTuVect [ 3 ] ; rt_LUTuVect [ 0 ] = hast0zswvx ;
rt_LUTuVect [ 1 ] = llfz1vqscn ; rt_LUTuVect [ 2 ] = pn0etbvpux ; ( *
is3kvqln30 ) = look_SplNLinSScd ( 3U , rt_LUTuVect , ( rt_LUTSplineWork * ) &
localDW -> cughybvon0 [ 0 ] ) ; } srUpdateBC ( localDW -> kglzzcbeka ) ;
break ; case 2 : if ( rtAction != rtPrevAction ) { if ( ssGetTaskTime ( rtS_k
, 1 ) != ssGetTStart ( rtS_k ) ) { ssSetBlockStateForSolverChangedAtMajorStep
( rtS_k ) ; } } { real_T rt_LUTuVect [ 3 ] ; rt_LUTuVect [ 0 ] = hast0zswvx ;
rt_LUTuVect [ 1 ] = llfz1vqscn ; rt_LUTuVect [ 2 ] = pn0etbvpux ; ( *
is3kvqln30 ) = look_SplNLinSScd ( 3U , rt_LUTuVect , ( rt_LUTSplineWork * ) &
localDW -> bjlmawntgi [ 0 ] ) ; } srUpdateBC ( localDW -> ckxwrcsgi3 ) ;
break ; case 3 : if ( rtAction != rtPrevAction ) { if ( ssGetTaskTime ( rtS_k
, 1 ) != ssGetTStart ( rtS_k ) ) { ssSetBlockStateForSolverChangedAtMajorStep
( rtS_k ) ; } } { real_T rt_LUTuVect [ 3 ] ; rt_LUTuVect [ 0 ] = hast0zswvx ;
rt_LUTuVect [ 1 ] = llfz1vqscn ; rt_LUTuVect [ 2 ] = pn0etbvpux ; ( *
is3kvqln30 ) = look_SplNLinSScd ( 3U , rt_LUTuVect , ( rt_LUTSplineWork * ) &
localDW -> npajdwg2lh [ 0 ] ) ; } srUpdateBC ( localDW -> meuzevkn2y ) ;
break ; default : if ( rtAction != rtPrevAction ) { if ( ssGetTaskTime (
rtS_k , 1 ) != ssGetTStart ( rtS_k ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS_k ) ; } } { real_T
rt_LUTuVect [ 3 ] ; rt_LUTuVect [ 0 ] = hast0zswvx ; rt_LUTuVect [ 1 ] =
llfz1vqscn ; rt_LUTuVect [ 2 ] = pn0etbvpux ; ( * is3kvqln30 ) =
look_SplNLinSScd ( 3U , rt_LUTuVect , ( rt_LUTSplineWork * ) & localDW ->
jvfpfqeupc [ 0 ] ) ; } srUpdateBC ( localDW -> biatop040u ) ; break ; } }
real_T rt_urand_Upu32_Yd_f_pw_snf ( uint32_T * u ) { uint32_T lo ; uint32_T
hi ; lo = * u % 127773U * 16807U ; hi = * u / 127773U * 2836U ; if ( lo < hi
) { * u = 2147483647U - ( hi - lo ) ; } else { * u = lo - hi ; } return (
real_T ) * u * 4.6566128752457969E-10 ; } real_T rt_nrand_Upu32_Yd_f_pw_snf (
uint32_T * u ) { real_T y ; real_T sr ; real_T si ; do { sr = 2.0 *
rt_urand_Upu32_Yd_f_pw_snf ( u ) - 1.0 ; si = 2.0 *
rt_urand_Upu32_Yd_f_pw_snf ( u ) - 1.0 ; si = sr * sr + si * si ; } while (
si > 1.0 ) ; y = muDoubleScalarSqrt ( - 2.0 * muDoubleScalarLog ( si ) / si )
* sr ; return y ; } void MdlInitialize ( void ) { boolean_T tmp ; uint32_T
tseed ; int32_T r ; int32_T t ; real_T tmp_p ; rtX . nb321hhpjg [ 0 ] = rtP .
uDOFBodyAxes_pos_ini [ 0 ] ; rtX . nb321hhpjg [ 1 ] = rtP .
uDOFBodyAxes_pos_ini [ 1 ] ; rtX . oyqwpsgdfy = rtP . theta0 ; rtDW .
afe03te4gm = 1 ; if ( ssIsFirstInitCond ( rtS ) ) { rtX . nmbwexfepb [ 0 ] =
93.091200024974071 ; rtX . nmbwexfepb [ 1 ] = 1.5913069971780047 ; tmp =
slIsRapidAcceleratorSimulating ( ) ; if ( tmp ) { tmp =
ssGetGlobalInitialStatesAvailable ( rtS ) ; rtDW . afe03te4gm = ! tmp ; }
else { rtDW . afe03te4gm = 1 ; } } rtDW . hgabqtd4hg = rtP . alpha0 ; rtX .
c3n310npj5 = rtP . wy0 ; rtX . m2cgemps3b [ 0 ] = rtP .
NonlinearSecondOrderActuator1_fin_act_0 ; rtX . m2cgemps3b [ 1 ] = rtP .
NonlinearSecondOrderActuator1_fin_act_vel ; rtDW . li4wvvnvo0 = 0 ; rtX .
fi3net4hji [ 0 ] = rtP . NonlinearSecondOrderActuator_fin_act_0 ; rtX .
fi3net4hji [ 1 ] = rtP . NonlinearSecondOrderActuator_fin_act_vel ; rtDW .
bkbk0mdq0s = 0 ; rtX . aldehcvxgf [ 0 ] = rtP .
NonlinearSecondOrderActuator2_fin_act_0 ; rtX . aldehcvxgf [ 1 ] = rtP .
NonlinearSecondOrderActuator2_fin_act_vel ; rtDW . b02jv5hvxo = 0 ; tmp_p =
muDoubleScalarFloor ( rtP . DrydenWindTurbulenceModelContinuousqr_Seed [ 0 ]
) ; if ( muDoubleScalarIsNaN ( tmp_p ) || muDoubleScalarIsInf ( tmp_p ) ) {
tmp_p = 0.0 ; } else { tmp_p = muDoubleScalarRem ( tmp_p , 4.294967296E+9 ) ;
} tseed = tmp_p < 0.0 ? ( uint32_T ) - ( int32_T ) ( uint32_T ) - tmp_p : (
uint32_T ) tmp_p ; r = ( int32_T ) ( tseed >> 16U ) ; t = ( int32_T ) ( tseed
& 32768U ) ; tseed = ( ( ( ( tseed - ( ( uint32_T ) r << 16U ) ) + t ) << 16U
) + t ) + r ; if ( tseed < 1U ) { tseed = 1144108930U ; } else { if ( tseed >
2147483646U ) { tseed = 2147483646U ; } } rtDW . khqvqeq4d4 [ 0 ] = tseed ;
rtDW . joib1qjnje [ 0 ] = rt_nrand_Upu32_Yd_f_pw_snf ( & rtDW . khqvqeq4d4 [
0 ] ) * rtP . WhiteNoise_StdDev + rtP . WhiteNoise_Mean ; tmp_p =
muDoubleScalarFloor ( rtP . DrydenWindTurbulenceModelContinuousqr_Seed [ 1 ]
) ; if ( muDoubleScalarIsNaN ( tmp_p ) || muDoubleScalarIsInf ( tmp_p ) ) {
tmp_p = 0.0 ; } else { tmp_p = muDoubleScalarRem ( tmp_p , 4.294967296E+9 ) ;
} tseed = tmp_p < 0.0 ? ( uint32_T ) - ( int32_T ) ( uint32_T ) - tmp_p : (
uint32_T ) tmp_p ; r = ( int32_T ) ( tseed >> 16U ) ; t = ( int32_T ) ( tseed
& 32768U ) ; tseed = ( ( ( ( tseed - ( ( uint32_T ) r << 16U ) ) + t ) << 16U
) + t ) + r ; if ( tseed < 1U ) { tseed = 1144108930U ; } else { if ( tseed >
2147483646U ) { tseed = 2147483646U ; } } rtDW . khqvqeq4d4 [ 1 ] = tseed ;
rtDW . joib1qjnje [ 1 ] = rt_nrand_Upu32_Yd_f_pw_snf ( & rtDW . khqvqeq4d4 [
1 ] ) * rtP . WhiteNoise_StdDev + rtP . WhiteNoise_Mean ; tmp_p =
muDoubleScalarFloor ( rtP . DrydenWindTurbulenceModelContinuousqr_Seed [ 2 ]
) ; if ( muDoubleScalarIsNaN ( tmp_p ) || muDoubleScalarIsInf ( tmp_p ) ) {
tmp_p = 0.0 ; } else { tmp_p = muDoubleScalarRem ( tmp_p , 4.294967296E+9 ) ;
} tseed = tmp_p < 0.0 ? ( uint32_T ) - ( int32_T ) ( uint32_T ) - tmp_p : (
uint32_T ) tmp_p ; r = ( int32_T ) ( tseed >> 16U ) ; t = ( int32_T ) ( tseed
& 32768U ) ; tseed = ( ( ( ( tseed - ( ( uint32_T ) r << 16U ) ) + t ) << 16U
) + t ) + r ; if ( tseed < 1U ) { tseed = 1144108930U ; } else { if ( tseed >
2147483646U ) { tseed = 2147483646U ; } } rtDW . khqvqeq4d4 [ 2 ] = tseed ;
rtDW . joib1qjnje [ 2 ] = rt_nrand_Upu32_Yd_f_pw_snf ( & rtDW . khqvqeq4d4 [
2 ] ) * rtP . WhiteNoise_StdDev + rtP . WhiteNoise_Mean ; tmp_p =
muDoubleScalarFloor ( rtP . DrydenWindTurbulenceModelContinuousqr_Seed [ 3 ]
) ; if ( muDoubleScalarIsNaN ( tmp_p ) || muDoubleScalarIsInf ( tmp_p ) ) {
tmp_p = 0.0 ; } else { tmp_p = muDoubleScalarRem ( tmp_p , 4.294967296E+9 ) ;
} tseed = tmp_p < 0.0 ? ( uint32_T ) - ( int32_T ) ( uint32_T ) - tmp_p : (
uint32_T ) tmp_p ; r = ( int32_T ) ( tseed >> 16U ) ; t = ( int32_T ) ( tseed
& 32768U ) ; tseed = ( ( ( ( tseed - ( ( uint32_T ) r << 16U ) ) + t ) << 16U
) + t ) + r ; if ( tseed < 1U ) { tseed = 1144108930U ; } else { if ( tseed >
2147483646U ) { tseed = 2147483646U ; } } rtDW . khqvqeq4d4 [ 3 ] = tseed ;
rtDW . joib1qjnje [ 3 ] = rt_nrand_Upu32_Yd_f_pw_snf ( & rtDW . khqvqeq4d4 [
3 ] ) * rtP . WhiteNoise_StdDev + rtP . WhiteNoise_Mean ; rtX . jxx3w5csvx =
rtP . DistanceintoGustxLimitedtogustlengthd_IC ; rtB . ohglmemv1w = rtP .
x_Y0 ; dccrmaip5g ( & rtB . mirhnoymfvo , & rtP . mirhnoymfvo , & rtX .
mirhnoymfvo ) ; dccrmaip5g ( & rtB . i5wp2ffbry , & rtP . i5wp2ffbry , & rtX
. i5wp2ffbry ) ; rtX . obrj2mgpdb [ 0 ] = rtP . pgw_p_IC ; rtX . jcf02uzlnt [
0 ] = rtP . wg_p1_IC ; rtX . go1vncxzci [ 0 ] = rtP . wg_p2_IC ; rtB .
ircgwytey3 [ 0 ] = rtP . wgw_Y0 ; rtX . b5qj413fkr [ 0 ] = rtP . qgw_p_IC ;
rtB . agqbvy2aau [ 0 ] = rtP . qgw_Y0 ; rtX . ds4avnfqd3 [ 0 ] = rtP .
vg_p1_IC ; rtX . p11ukldqr3 [ 0 ] = rtP . vgw_p2_IC ; rtB . k3touvbkw2 [ 0 ]
= rtP . vgw_Y0 ; rtX . efv1zergvx [ 0 ] = rtP . rgw_p_IC ; rtB . mv5ztdhgch [
0 ] = rtP . rgw_Y0 ; rtX . galtq11byd [ 0 ] = rtP . ug_p_IC ; rtB .
leujf5c51t [ 0 ] = rtP . ugw_Y0 ; rtX . obrj2mgpdb [ 1 ] = rtP . pgw_p_IC ;
rtX . jcf02uzlnt [ 1 ] = rtP . wg_p1_IC ; rtX . go1vncxzci [ 1 ] = rtP .
wg_p2_IC ; rtB . ircgwytey3 [ 1 ] = rtP . wgw_Y0 ; rtX . b5qj413fkr [ 1 ] =
rtP . qgw_p_IC ; rtB . agqbvy2aau [ 1 ] = rtP . qgw_Y0 ; rtX . ds4avnfqd3 [ 1
] = rtP . vg_p1_IC ; rtX . p11ukldqr3 [ 1 ] = rtP . vgw_p2_IC ; rtB .
k3touvbkw2 [ 1 ] = rtP . vgw_Y0 ; rtX . efv1zergvx [ 1 ] = rtP . rgw_p_IC ;
rtB . mv5ztdhgch [ 1 ] = rtP . rgw_Y0 ; rtX . galtq11byd [ 1 ] = rtP .
ug_p_IC ; rtB . leujf5c51t [ 1 ] = rtP . ugw_Y0 ; rtX . lwmusv4p0k = rtP .
StateSpace_InitialCondition ; rtDW . e0khmaiz1x = ( rtInf ) ; rtB .
gakkrw1mku = rtP . Merge1_1_InitialOutput ; rtB . lldfszhlwc = rtP .
Merge1_3_InitialOutput ; rtB . iqcro5zqc5 = rtP . Merge1_4_InitialOutput ;
rtB . khr4lzxr01 = rtP . Merge1_5_InitialOutput ; rtB . otbeb0smep = rtP .
Merge1_6_InitialOutput ; rtB . mbiqcrghxm = rtP . Merge1_7_InitialOutput ;
rtB . gxcomph2oi = rtP . Merge1_8_InitialOutput ; rtDW . btjxdj1nnm = rtP .
Memory_InitialCondition ; rtB . lzfl1px3we = rtP . asOut_Y0 ; rtDW .
lwgbxvspmn = rtP . Memory_InitialCondition_lgdo1d41mj ; rtB . euq0m2wkrk =
rtP . asOut_Y0_fruk2gwjsm ; { int_T rootPeriodicContStateIndices [ 1 ] = { 2
} ; real_T rootPeriodicContStateRanges [ 2 ] = { - 3.1415926535897931 ,
3.1415926535897931 } ; ( void ) memcpy ( ( void * ) rtPeriodicIndX ,
rootPeriodicContStateIndices , 1 * sizeof ( int_T ) ) ; ( void ) memcpy ( (
void * ) rtPeriodicRngX , rootPeriodicContStateRanges , 2 * sizeof ( real_T )
) ; } } void MdlStart ( void ) { void * catalog ; void * catalog_p ; void *
catalog_e ; void * catalog_i ; void * catalog_m ; { void * *
slioCatalogueAddr = rt_slioCatalogueAddr ( ) ; void * r2 = ( NULL ) ; void *
* pOSigstreamManagerAddr = ( NULL ) ; const int maxErrorBufferSize = 16384 ;
char errMsgCreatingOSigstreamManager [ 16384 ] ; bool
errorCreatingOSigstreamManager = false ; const char *
errorAddingR2SharedResource = ( NULL ) ; * slioCatalogueAddr =
rtwGetNewSlioCatalogue ( rt_GetMatSigLogSelectorFileName ( ) ) ;
errorAddingR2SharedResource = rtwAddR2SharedResource (
rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) , 1 ) ; if (
errorAddingR2SharedResource != ( NULL ) ) { rtwTerminateSlioCatalogue (
slioCatalogueAddr ) ; * slioCatalogueAddr = ( NULL ) ; ssSetErrorStatus ( rtS
, errorAddingR2SharedResource ) ; return ; } r2 = rtwGetR2SharedResource (
rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) ) ;
pOSigstreamManagerAddr = rt_GetOSigstreamManagerAddr ( ) ;
errorCreatingOSigstreamManager = rtwOSigstreamManagerCreateInstance (
rt_GetMatSigLogSelectorFileName ( ) , r2 , pOSigstreamManagerAddr ,
errMsgCreatingOSigstreamManager , maxErrorBufferSize ) ; if (
errorCreatingOSigstreamManager ) { * pOSigstreamManagerAddr = ( NULL ) ;
ssSetErrorStatus ( rtS , errMsgCreatingOSigstreamManager ) ; return ; } } {
bool externalInputIsInDatasetFormat = false ; void * pISigstreamManager =
rt_GetISigstreamManager ( ) ; rtwISigstreamManagerGetInputIsInDatasetFormat (
pISigstreamManager , & externalInputIsInDatasetFormat ) ; if (
externalInputIsInDatasetFormat ) { } } rtDW . ikp5xo5ucv = false ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; ( void ) memset ( & ( (
( XDis * ) ssGetContStateDisabled ( rtS ) ) -> obrj2mgpdb ) , 1 , 2 * sizeof
( boolean_T ) ) ; rtDW . auringonfr = false ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; ( void ) memset ( & ( (
( XDis * ) ssGetContStateDisabled ( rtS ) ) -> b5qj413fkr ) , 1 , 2 * sizeof
( boolean_T ) ) ; rtDW . bdjnrykkzo = false ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; ( void ) memset ( & ( (
( XDis * ) ssGetContStateDisabled ( rtS ) ) -> efv1zergvx ) , 1 , 2 * sizeof
( boolean_T ) ) ; rtDW . aovs2hjxyi = false ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; ( void ) memset ( & ( (
( XDis * ) ssGetContStateDisabled ( rtS ) ) -> galtq11byd ) , 1 , 2 * sizeof
( boolean_T ) ) ; rtDW . jddl4uj35t = 0 ; rtDW . nztae4vpib = false ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; ( void ) memset ( & ( (
( XDis * ) ssGetContStateDisabled ( rtS ) ) -> jcf02uzlnt ) , 1 , 4 * sizeof
( boolean_T ) ) ; rtDW . olplly2yli = 0 ; rtDW . gylng33jck = 0 ; rtDW .
ijpkeyczg1 = 0 ; rtDW . e25tjl33um = 0 ; rtDW . auligz0bp0 = 0 ; { real_T *
temp_table = ( real_T * ) & rtDW . csnixtyelx [ 0 ] ; real_T * pres_table = (
real_T * ) & rtDW . aqckczktpc [ 0 ] ; InitCalcAtmosCOESA ( temp_table ,
pres_table ) ; } rtDW . kk3mavtp0m = false ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; ( ( XDis * )
ssGetContStateDisabled ( rtS ) ) -> jxx3w5csvx = 1 ; m4ya0h344x ( rtS , &
rtDW . mirhnoymfvo , & ( ( XDis * ) ssGetContStateDisabled ( rtS ) ) ->
mirhnoymfvo ) ; m4ya0h344x ( rtS , & rtDW . i5wp2ffbry , & ( ( XDis * )
ssGetContStateDisabled ( rtS ) ) -> i5wp2ffbry ) ; rtDW . oo4gabx5lo = 0 ;
rtDW . ahb01lfm54 = false ; ssSetBlockStateForSolverChangedAtMajorStep ( rtS
) ; ( void ) memset ( & ( ( ( XDis * ) ssGetContStateDisabled ( rtS ) ) ->
ds4avnfqd3 ) , 1 , 4 * sizeof ( boolean_T ) ) ; rtDW . o2i2c0ygga = 0 ; rtDW
. jbrbgwy42h = - 1 ; rtDW . nb15cebgib = - 1 ; rtDW . fnjwuvlmmj = 0 ;
catalog = rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) ; rtDW .
ib0jqzpceq = rt_SlioAccessorAddClientAssessmentSdi ( 1 , 4 , catalog , rtDW .
ib0jqzpceq , "Assertion" ,
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/Supersonic/Supersonic/Assertion"
) ; rtDW . n4ytmke31n = 0 ; rtDW . fg4worawde = 0 ; catalog_p =
rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) ; rtDW . fodolp1u3u =
rt_SlioAccessorAddClientAssessmentSdi ( 1 , 4 , catalog_p , rtDW . fodolp1u3u
, "Assertion" ,
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/Supersonic/Supersonic/Assertion"
) ; rtDW . fk3ir50wko = 0 ; rtDW . glvphpzltx = 0 ; rtDW . bfxedqpzaa = - 1 ;
rtDW . dvxffgw44x = - 1 ; rtDW . lfjjbagrjo = - 1 ; rtDW . bvueotiv4b = - 1 ;
rtDW . g4vr3lujuz = - 1 ; ki0yuzsymz ( & rtDW . cjigmwyptgo , & rtP .
cjigmwyptgo , rtP . TAS2CAS_LUTM0 , rtP . TAS2CAS_P_bp , rtP .
TAS2CAS_V_bp_M0 , rtP . TAS2CAS_SOS_bp , rtP . TAS2CAS_LUTM1 , rtP .
TAS2CAS_V_bp_M1 , rtP . TAS2CAS_LUTM2 , rtP . TAS2CAS_V_bp_M2 , rtP .
TAS2CAS_LUTM3 , rtP . TAS2CAS_V_bp_M3 , rtP . TAS2CAS_LUTM4 , rtP .
TAS2CAS_V_bp_M4 ) ; ki0yuzsymz ( & rtDW . aqq0sq0gty , & rtP . aqq0sq0gty ,
rtP . CAS2TAS_LUTM0 , rtP . CAS2TAS_P_bp , rtP . CAS2TAS_V_bp_M0 , rtP .
CAS2TAS_SOS_bp , rtP . CAS2TAS_LUTM1 , rtP . CAS2TAS_V_bp_M1 , rtP .
CAS2TAS_LUTM2 , rtP . CAS2TAS_V_bp_M2 , rtP . CAS2TAS_LUTM3 , rtP .
CAS2TAS_V_bp_M3 , rtP . CAS2TAS_LUTM4 , rtP . CAS2TAS_V_bp_M4 ) ; rtDW .
hug50n4wps = - 1 ; rtDW . jutuodwsau = 0 ; rtDW . n12ashwpk5 = 0 ; { real_T *
temp_table = ( real_T * ) & rtDW . gyluhpuobq [ 0 ] ; real_T * pres_table = (
real_T * ) & rtDW . hfu1jte3yo [ 0 ] ; InitCalcAtmosCOESA ( temp_table ,
pres_table ) ; } rtDW . bh43qqaomf = 0 ; rtDW . cal4znigzs = - 1 ; catalog_e
= rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) ; rtDW . h4fprkvunp =
rt_SlioAccessorAddClientAssessmentSdi ( 1 , 4 , catalog_e , rtDW . h4fprkvunp
, "Assertion" ,
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Validate Inputs/If Action Subsystem1/Assertion"
) ; rtDW . o3p0d0pvrk = - 1 ; catalog_i = rtwGetPointerFromUniquePtr (
rt_slioCatalogue ( ) ) ; rtDW . gu0lptjayg =
rt_SlioAccessorAddClientAssessmentSdi ( 1 , 4 , catalog_i , rtDW . gu0lptjayg
, "Assertion" ,
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Validate Inputs/If Action Subsystem2/Assertion"
) ; rtDW . mxjlbo5rjs = - 1 ; catalog_m = rtwGetPointerFromUniquePtr (
rt_slioCatalogue ( ) ) ; rtDW . cygydsjrfb =
rt_SlioAccessorAddClientAssessmentSdi ( 1 , 4 , catalog_m , rtDW . cygydsjrfb
, "Assertion" ,
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Validate Inputs/If Action Subsystem3/Assertion"
) ; rtDW . hmyyl45jap = 0 ; rtDW . mdzj2hr4jc = - 1 ; rtDW . fk0au12fsx = 0 ;
rtDW . pa1f5cjxua = 0 ; rtDW . mewl43zgov = 0 ; MdlInitialize ( ) ; } void
MdlOutputs ( int_T tid ) { real_T ij1arac1oi ; real_T lat ; real_T lon ;
real_T alt ; real_T fphi ; real_T slat ; real_T sinphi ; real_T cospsi ;
real_T slam ; real_T temp1 ; boolean_T rEQ0 ; real_T q ; real_T avxeaykvgw [
3 ] ; real_T daxlvr32qq [ 3 ] ; real_T kh2xrb5jub [ 9 ] ; real_T jjatrktc0c [
9 ] ; int32_T oxjy2hf2cl ; int32_T kkmrxg5n4s ; int32_T dsrahxycol ; real_T
e4kgnowjgg [ 3 ] ; real_T frac [ 2 ] ; uint32_T bpIndex [ 2 ] ; real_T frac_p
[ 3 ] ; int32_T bpIndex_p [ 3 ] ; real_T frac_e [ 3 ] ; int32_T bpIndex_e [ 3
] ; real_T frac_i [ 3 ] ; int32_T bpIndex_i [ 3 ] ; real_T frac_m [ 4 ] ;
int32_T bpIndex_m [ 4 ] ; real_T frac_g [ 3 ] ; int32_T bpIndex_g [ 3 ] ;
real_T frac_j [ 3 ] ; int32_T bpIndex_j [ 3 ] ; real_T frac_f [ 3 ] ; int32_T
bpIndex_f [ 3 ] ; real_T frac_c [ 3 ] ; int32_T bpIndex_c [ 3 ] ; real_T
frac_k [ 3 ] ; int32_T bpIndex_k [ 3 ] ; real_T frac_b [ 3 ] ; int32_T
bpIndex_b [ 3 ] ; real_T frac_n [ 3 ] ; int32_T bpIndex_n [ 3 ] ; real_T
frac_l [ 3 ] ; int32_T bpIndex_l [ 3 ] ; real_T frac_d [ 3 ] ; int32_T
bpIndex_d [ 3 ] ; real_T frac_o [ 3 ] ; int32_T bpIndex_o [ 3 ] ; real_T
frac_dz [ 3 ] ; int32_T bpIndex_dz [ 3 ] ; real_T frac_fs [ 3 ] ; int32_T
bpIndex_fs [ 3 ] ; real_T frac_ck [ 3 ] ; int32_T bpIndex_ck [ 3 ] ; real_T
frac_f2 [ 3 ] ; int32_T bpIndex_f2 [ 3 ] ; real_T frac_kt [ 3 ] ; int32_T
bpIndex_kt [ 3 ] ; real_T gzwrrirgjp [ 6 ] ; int32_T assessmentVar = - 1 ;
int32_T assessmentVar_p = - 1 ; real_T jb2eust2qa ; int32_T assessmentVar_e =
- 1 ; int32_T assessmentVar_i = - 1 ; void * assessmentPtrVar ; int32_T
assessmentVar_m = - 1 ; int8_T rtAction ; int8_T rtPrevAction ; real_T
o2mmi0bsgf ; real_T or3lxhab5k [ 3 ] ; int32_T i ; real_T gkpo2ijrbk_idx_2 ;
real_T gkpo2ijrbk_idx_1 ; real_T gkpo2ijrbk_idx_0 ; real_T elm2mlhmo3_idx_1 ;
real_T oyontdhyku_idx_1 ; real_T abmm43etbq_idx_1 ; real_T elm2mlhmo3_idx_0 ;
real_T oyontdhyku_idx_0 ; real_T abmm43etbq_idx_0 ; real_T ksjzavltcv_idx_1 ;
real_T hxkhshlt5m_idx_1 ; real_T ksjzavltcv_idx_0 ; real_T hxkhshlt5m_idx_0 ;
real_T mn3ckvav5d_idx_2 ; real_T mn3ckvav5d_idx_1 ; real_T mn3ckvav5d_idx_0 ;
real_T fzwvbhycmn_idx_1 ; real_T fzwvbhycmn_idx_0 ; real_T bkk2o2zacc_idx_1 ;
real_T bkk2o2zacc_idx_0 ; real_T mc31ssftz2_idx_1 ; SimStruct * S ; void *
diag ; srClearBC ( rtDW . au2zmey5pc ) ; srClearBC ( rtDW . mirhnoymfvo .
jyd1ppenco ) ; srClearBC ( rtDW . byzsjh0px3 ) ; srClearBC ( rtDW .
npachunt01 ) ; srClearBC ( rtDW . adhcumf2n5 ) ; srClearBC ( rtDW .
krzqaoxmqz ) ; srClearBC ( rtDW . gkuxqeqcdi ) ; srClearBC ( rtDW .
n5gzkaa0nd ) ; srClearBC ( rtDW . fvgndvljcd ) ; srClearBC ( rtDW .
ejm2q33rwd ) ; srClearBC ( rtDW . pkklmgxn3w ) ; srClearBC ( rtDW .
jwt5flmp0h ) ; srClearBC ( rtDW . nv0y4bxbiz ) ; srClearBC ( rtDW .
cvvf5wjoub ) ; srClearBC ( rtDW . dnxut4pgtq ) ; srClearBC ( rtDW .
pdsijyzqws ) ; srClearBC ( rtDW . faa1lc2tvb ) ; srClearBC ( rtDW .
b3phozbfji ) ; srClearBC ( rtDW . f1edxk4pdy ) ; srClearBC ( rtDW .
axi4q1he0b ) ; srClearBC ( rtDW . ngahvntjaq ) ; srClearBC ( rtDW .
drdj3xeohi ) ; srClearBC ( rtDW . cjigmwyptgo . johe0vk44j ) ; srClearBC (
rtDW . cjigmwyptgo . kglzzcbeka ) ; srClearBC ( rtDW . cjigmwyptgo .
ckxwrcsgi3 ) ; srClearBC ( rtDW . cjigmwyptgo . meuzevkn2y ) ; srClearBC (
rtDW . cjigmwyptgo . biatop040u ) ; srClearBC ( rtDW . cjigmwyptgo .
jijajg5nkb ) ; srClearBC ( rtDW . ojqxnfreck ) ; srClearBC ( rtDW .
giro4eplfi ) ; srClearBC ( rtDW . hudho2xp0v ) ; srClearBC ( rtDW .
mxamlbyck0 ) ; srClearBC ( rtDW . fc1bretpi5 ) ; srClearBC ( rtDW .
i1eyrq4uwi ) ; srClearBC ( rtDW . dd1fatoipk ) ; srClearBC ( rtDW .
l3k54bxdj1 ) ; srClearBC ( rtDW . p4vcgbddhj ) ; srClearBC ( rtDW .
poolnl3e1h ) ; rtB . pkgvmqtajm [ 0 ] = rtX . nb321hhpjg [ 0 ] ; rtB .
pkgvmqtajm [ 1 ] = rtX . nb321hhpjg [ 1 ] ; rtB . cbt1v4nio1 [ 0 ] = ( rtB .
pkgvmqtajm [ 0 ] * rtB . kkwizmpnnu - rtB . h1os02q0lx ) * rtB . ktgttcsket *
57.295779513082323 + rtB . knraoiqthj ; rtB . cbt1v4nio1 [ 1 ] = ( rtB .
pkgvmqtajm [ 0 ] * rtB . dojqnhucc1 + rtB . p1ud1zl0l5 ) * rtB . dtrbet0r1l *
57.295779513082323 + rtB . oxryesp4l3 ; if ( ssIsMajorTimeStep ( rtS ) ) {
rtDW . bp1ctbtl0z = ( rtB . cbt1v4nio1 [ 0 ] >= 0.0 ) ; } rtB . chuqvl4sya =
rtDW . bp1ctbtl0z > 0 ? rtB . cbt1v4nio1 [ 0 ] : - rtB . cbt1v4nio1 [ 0 ] ;
if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { if ( ssIsMajorTimeStep ( rtS ) ) {
rtDW . es24y1hork = ( rtB . chuqvl4sya > rtP .
CompareToConstant_const_n1mct5qu1a ) ; } rtB . oov0isvnf1 = rtDW . es24y1hork
; } if ( rtB . oov0isvnf1 ) { rtB . hpaztoaur1 = muDoubleScalarMod ( rtB .
cbt1v4nio1 [ 0 ] + rtP . Bias_Bias_mahsvgbvzq , rtP .
Constant2_Value_g4cw5n3yku ) + rtP . Bias1_Bias_fiyyfd4u5t ; } else { rtB .
hpaztoaur1 = rtB . cbt1v4nio1 [ 0 ] ; } if ( ssIsMajorTimeStep ( rtS ) ) {
rtDW . p0uqfawr0j = ( rtB . hpaztoaur1 >= 0.0 ) ; } rtB . pwdhurk11o = rtDW .
p0uqfawr0j > 0 ? rtB . hpaztoaur1 : - rtB . hpaztoaur1 ; if ( ssIsSampleHit (
rtS , 1 , 0 ) ) { if ( ssIsMajorTimeStep ( rtS ) ) { rtDW . iuswyxudvk = (
rtB . pwdhurk11o > rtP . CompareToConstant_const_omj1fbfglm ) ; } rtB .
beyrovclvb = rtDW . iuswyxudvk ; if ( rtB . beyrovclvb ) { rtB . akzllltikr =
rtP . Constant_Value_gddg5y0bon ; } else { rtB . akzllltikr = rtP .
Constant1_Value_fdgj4x4yvk ; } } rtB . pstvhythqv = rtB . akzllltikr + rtB .
cbt1v4nio1 [ 1 ] ; if ( ssIsMajorTimeStep ( rtS ) ) { rtDW . mff001ptpl = (
rtB . pstvhythqv >= 0.0 ) ; } rtB . hjs5mprgdw = rtDW . mff001ptpl > 0 ? rtB
. pstvhythqv : - rtB . pstvhythqv ; if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { if
( ssIsMajorTimeStep ( rtS ) ) { rtDW . hk5aeqxoie = ( rtB . hjs5mprgdw > rtP
. CompareToConstant_const_d4qgtkvaqb ) ; } rtB . b5r5v125p5 = rtDW .
hk5aeqxoie ; if ( rtB . hpaztoaur1 > 0.0 ) { rtDW . m5ksfibjla = 1 ; } else
if ( rtB . hpaztoaur1 < 0.0 ) { rtDW . m5ksfibjla = - 1 ; } else { rtDW .
m5ksfibjla = 0 ; } rtB . lauuzxpsuv = rtDW . m5ksfibjla ; } if ( rtB .
b5r5v125p5 ) { rtB . ckkqbbvet5 = muDoubleScalarMod ( rtB . pstvhythqv + rtP
. Bias_Bias_du0tiht0fj , rtP . Constant2_Value_f3imtyqlu2 ) + rtP .
Bias1_Bias_nbxpttwzhh ; } else { rtB . ckkqbbvet5 = rtB . pstvhythqv ; } if (
rtB . beyrovclvb ) { rtB . kpfxhsrdzc = ( ( rtB . pwdhurk11o + rtP .
Bias_Bias_k5qjjnbf1r ) * rtP . Gain_Gain_opd2wvnqlj + rtP .
Bias1_Bias_oluwwwumlb ) * rtB . lauuzxpsuv ; } else { rtB . kpfxhsrdzc = rtB
. hpaztoaur1 ; } rtB . d2gfe2p42l = - rtB . pkgvmqtajm [ 1 ] - rtP .
Constant_Value_klve0yes4x ; rtB . fdu3hdd1wq = rtX . oyqwpsgdfy ;
muDoubleScalarSinCos ( rtB . dlbqs3jbq5 , & e4kgnowjgg [ 0 ] , &
gkpo2ijrbk_idx_0 ) ; muDoubleScalarSinCos ( rtB . fdu3hdd1wq , & e4kgnowjgg [
1 ] , & gkpo2ijrbk_idx_1 ) ; muDoubleScalarSinCos ( rtB . a2xdtn2hjy , & slat
, & gkpo2ijrbk_idx_2 ) ; jjatrktc0c [ 0 ] = gkpo2ijrbk_idx_1 *
gkpo2ijrbk_idx_0 ; jjatrktc0c [ 1 ] = slat * e4kgnowjgg [ 1 ] *
gkpo2ijrbk_idx_0 - gkpo2ijrbk_idx_2 * e4kgnowjgg [ 0 ] ; jjatrktc0c [ 2 ] =
gkpo2ijrbk_idx_2 * e4kgnowjgg [ 1 ] * gkpo2ijrbk_idx_0 + slat * e4kgnowjgg [
0 ] ; jjatrktc0c [ 3 ] = gkpo2ijrbk_idx_1 * e4kgnowjgg [ 0 ] ; jjatrktc0c [ 4
] = slat * e4kgnowjgg [ 1 ] * e4kgnowjgg [ 0 ] + gkpo2ijrbk_idx_2 *
gkpo2ijrbk_idx_0 ; jjatrktc0c [ 5 ] = gkpo2ijrbk_idx_2 * e4kgnowjgg [ 1 ] *
e4kgnowjgg [ 0 ] - slat * gkpo2ijrbk_idx_0 ; jjatrktc0c [ 6 ] = - e4kgnowjgg
[ 1 ] ; jjatrktc0c [ 7 ] = slat * gkpo2ijrbk_idx_1 ; jjatrktc0c [ 8 ] =
gkpo2ijrbk_idx_2 * gkpo2ijrbk_idx_1 ; rtB . gr2e1rit1v [ 0 ] = rtP . V0 *
muDoubleScalarCos ( rtP . alpha0 ) ; rtB . gr2e1rit1v [ 1 ] = rtP . V0 *
muDoubleScalarSin ( rtP . alpha0 ) ; if ( rtDW . afe03te4gm != 0 ) { rtX .
nmbwexfepb [ 0 ] = rtB . gr2e1rit1v [ 0 ] ; rtX . nmbwexfepb [ 1 ] = rtB .
gr2e1rit1v [ 1 ] ; } abmm43etbq_idx_0 = rtX . nmbwexfepb [ 0 ] ;
abmm43etbq_idx_1 = rtX . nmbwexfepb [ 1 ] ; gkpo2ijrbk_idx_0 = rtX .
nmbwexfepb [ 0 ] ; gkpo2ijrbk_idx_1 = rtP . Constant12_Value ;
gkpo2ijrbk_idx_2 = rtX . nmbwexfepb [ 1 ] ; for ( oxjy2hf2cl = 0 ; oxjy2hf2cl
< 3 ; oxjy2hf2cl ++ ) { e4kgnowjgg [ oxjy2hf2cl ] = jjatrktc0c [ 3 *
oxjy2hf2cl + 2 ] * gkpo2ijrbk_idx_2 + ( jjatrktc0c [ 3 * oxjy2hf2cl + 1 ] *
rtP . Constant12_Value + jjatrktc0c [ 3 * oxjy2hf2cl ] * gkpo2ijrbk_idx_0 ) ;
} if ( ssIsMajorTimeStep ( rtS ) ) { rtDW . bzco1krw01 = ( rtB . pkgvmqtajm [
1 ] >= 0.0 ) ; } rtB . hkzk0mcpdz = rtDW . bzco1krw01 > 0 ? rtB . pkgvmqtajm
[ 1 ] : - rtB . pkgvmqtajm [ 1 ] ; { real_T * temp_table = ( real_T * ) &
rtDW . csnixtyelx [ 0 ] ; real_T * pres_table = ( real_T * ) & rtDW .
aqckczktpc [ 0 ] ; CalcAtmosCOESA ( & rtB . hkzk0mcpdz , & rtB . cjcp2dooxg ,
& rtB . dv0mxcbo4y , & rtB . nts3iqoscx , & rtB . ivvuf1kwbw , temp_table ,
pres_table , 1 ) ; } lat = rtB . kpfxhsrdzc * 0.017453292519943295 ; fphi =
muDoubleScalarAbs ( lat ) ; slat = 1.0 ; if ( fphi > 3.1415926535897931 ) {
if ( lat < - 3.1415926535897931 ) { slat = - 1.0 ; } if ( muDoubleScalarIsInf
( fphi + 3.1415926535897931 ) ) { cospsi = ( rtNaN ) ; } else { cospsi =
muDoubleScalarRem ( fphi + 3.1415926535897931 , 6.2831853071795862 ) ; rEQ0 =
( cospsi == 0.0 ) ; if ( ! rEQ0 ) { q = ( fphi + 3.1415926535897931 ) /
6.2831853071795862 ; rEQ0 = ! ( muDoubleScalarAbs ( q - muDoubleScalarFloor (
q + 0.5 ) ) > 2.2204460492503131E-16 * q ) ; } if ( rEQ0 ) { cospsi = 0.0 ; }
} lat = ( cospsi - 3.1415926535897931 ) * slat ; fphi = muDoubleScalarAbs (
lat ) ; } if ( fphi > 1.5707963267948966 ) { if ( lat > 1.5707963267948966 )
{ lat = 1.5707963267948966 - ( fphi - 1.5707963267948966 ) ; } if ( lat < -
1.5707963267948966 ) { lat = - ( 1.5707963267948966 - ( fphi -
1.5707963267948966 ) ) ; } } sinphi = muDoubleScalarSin ( lat ) ; slat =
sinphi * sinphi ; avxeaykvgw [ 0 ] = rtP . GravityinEarthAxes_Gain [ 0 ] *
0.0 ; avxeaykvgw [ 1 ] = rtP . GravityinEarthAxes_Gain [ 1 ] * 0.0 ;
mn3ckvav5d_idx_0 = ( ( 1.0 - ( 1.006802597171564 - 2.0 * slat / 298.257223563
) * 2.0 * rtB . hkzk0mcpdz / 6.378137E+6 ) + 3.0 * rtB . hkzk0mcpdz * rtB .
hkzk0mcpdz / 4.0680631590769E+13 ) * ( ( 0.00193185265241 * slat + 1.0 ) *
9.7803253359 / muDoubleScalarSqrt ( 1.0 - 0.00669437999014 * slat ) ) * rtP .
GravityinEarthAxes_Gain [ 2 ] ; for ( oxjy2hf2cl = 0 ; oxjy2hf2cl < 3 ;
oxjy2hf2cl ++ ) { or3lxhab5k [ oxjy2hf2cl ] = jjatrktc0c [ oxjy2hf2cl + 6 ] *
mn3ckvav5d_idx_0 + ( jjatrktc0c [ oxjy2hf2cl + 3 ] * avxeaykvgw [ 1 ] +
jjatrktc0c [ oxjy2hf2cl ] * avxeaykvgw [ 0 ] ) ; } rtB . lvp0utv5zh = ssGetT
( rtS ) ; if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { if ( ssIsMajorTimeStep ( rtS
) ) { rtDW . ouupu23dos = ( rtB . lvp0utv5zh >= rtP .
DiscreteWindGustModel_t_0 ) ; } o2mmi0bsgf = rtDW . ouupu23dos ; rtB .
kulaji43pi = ( rtDW . ouupu23dos && ( rtP . DiscreteWindGustModel_Gx != 0.0 )
) ; if ( ssIsMajorTimeStep ( rtS ) ) { if ( rtB . kulaji43pi ) { if ( ! rtDW
. kk3mavtp0m ) { if ( ssGetTaskTime ( rtS , 1 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } ( ( XDis * )
ssGetContStateDisabled ( rtS ) ) -> jxx3w5csvx = 0 ; rtX . jxx3w5csvx = rtP .
DistanceintoGustxLimitedtogustlengthd_IC ; rtDW . kk3mavtp0m = true ; } }
else { if ( rtDW . kk3mavtp0m ) { ssSetBlockStateForSolverChangedAtMajorStep
( rtS ) ; ( ( XDis * ) ssGetContStateDisabled ( rtS ) ) -> jxx3w5csvx = 1 ;
rtDW . kk3mavtp0m = false ; } } } } if ( rtDW . kk3mavtp0m ) { if (
ssIsMajorTimeStep ( rtS ) ) { if ( rtX . jxx3w5csvx >= rtP .
Distanceintogustx_d_m ) { if ( rtX . jxx3w5csvx > rtP . Distanceintogustx_d_m
) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } rtX . jxx3w5csvx =
rtP . Distanceintogustx_d_m ; } else { if ( rtX . jxx3w5csvx <= rtP .
DistanceintoGustxLimitedtogustlengthd_LowerSat ) { if ( rtX . jxx3w5csvx <
rtP . DistanceintoGustxLimitedtogustlengthd_LowerSat ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } rtX . jxx3w5csvx = rtP
. DistanceintoGustxLimitedtogustlengthd_LowerSat ; } } } rtB . ohglmemv1w =
rtX . jxx3w5csvx ; if ( ssIsMajorTimeStep ( rtS ) ) { srUpdateBC ( rtDW .
au2zmey5pc ) ; } } if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { rtB . drwselis2e =
( ( o2mmi0bsgf != 0.0 ) && ( rtP . DiscreteWindGustModel_Gy != 0.0 ) ) ; }
mirhnoymfv ( rtS , rtB . drwselis2e , & rtB . mirhnoymfvo , & rtDW .
mirhnoymfvo , & rtP . mirhnoymfvo , & rtX . mirhnoymfvo , rtP .
Distanceintogusty_d_m , & ( ( XDis * ) ssGetContStateDisabled ( rtS ) ) ->
mirhnoymfvo ) ; if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { rtB . pxhra0n40a = ( (
o2mmi0bsgf != 0.0 ) && ( rtP . DiscreteWindGustModel_Gz != 0.0 ) ) ; }
mirhnoymfv ( rtS , rtB . pxhra0n40a , & rtB . i5wp2ffbry , & rtDW .
i5wp2ffbry , & rtP . i5wp2ffbry , & rtX . i5wp2ffbry , rtP .
Distanceintogustz_d_m , & ( ( XDis * ) ssGetContStateDisabled ( rtS ) ) ->
i5wp2ffbry ) ; avxeaykvgw [ 0 ] = 3.1415926535897931 / rtP .
DiscreteWindGustModel_d_m [ 0 ] * rtB . ohglmemv1w ; avxeaykvgw [ 1 ] =
3.1415926535897931 / rtP . DiscreteWindGustModel_d_m [ 1 ] * rtB .
mirhnoymfvo . cwtrsfu0ve ; avxeaykvgw [ 2 ] = 3.1415926535897931 / rtP .
DiscreteWindGustModel_d_m [ 2 ] * rtB . i5wp2ffbry . cwtrsfu0ve ;
mn3ckvav5d_idx_0 = rtP . DiscreteWindGustModel_v_m [ 0 ] / 2.0 * ( rtP .
u_Value - muDoubleScalarCos ( avxeaykvgw [ 0 ] ) ) ; mn3ckvav5d_idx_1 = rtP .
DiscreteWindGustModel_v_m [ 1 ] / 2.0 * ( rtP . u_Value - muDoubleScalarCos (
avxeaykvgw [ 1 ] ) ) ; mn3ckvav5d_idx_2 = rtP . DiscreteWindGustModel_v_m [ 2
] / 2.0 * ( rtP . u_Value - muDoubleScalarCos ( avxeaykvgw [ 2 ] ) ) ; slat =
( e4kgnowjgg [ 0 ] * e4kgnowjgg [ 0 ] + e4kgnowjgg [ 1 ] * e4kgnowjgg [ 1 ] )
+ e4kgnowjgg [ 2 ] * e4kgnowjgg [ 2 ] ; rtB . bs4j1k4omq = 3.280839895013123
* rtB . hkzk0mcpdz ; if ( ssIsMajorTimeStep ( rtS ) ) { rtDW . d0gx1hbrnk =
rtB . bs4j1k4omq >= rtP . LimitFunction10ftto1000ft_UpperSat ? 1 : rtB .
bs4j1k4omq > rtP . LimitFunction10ftto1000ft_LowerSat ? 0 : - 1 ; } fphi =
rtDW . d0gx1hbrnk == 1 ? rtP . LimitFunction10ftto1000ft_UpperSat : rtDW .
d0gx1hbrnk == - 1 ? rtP . LimitFunction10ftto1000ft_LowerSat : rtB .
bs4j1k4omq ; lat = rtP . Lw_Gain * fphi ; q = rtP . Lw_Gain * rtB .
o1nidekcl5 ; bpIndex [ 0 ] = plook_bincpa ( rtB . bs4j1k4omq , rtP .
PreLookUpIndexSearchaltitude_BreakpointsData , 11U , & frac [ 0 ] , & rtDW .
ffudfy2unb ) ; frac [ 1 ] = rtB . pt0yyggmye ; bpIndex [ 1 ] = rtB .
prdmace1ry ; o2mmi0bsgf = intrp2d_la_pw ( bpIndex , frac , rtP .
MediumHighAltitudeIntensity_Table , 12U , rtP .
MediumHighAltitudeIntensity_maxIndex ) ; if ( ssIsSampleHit ( rtS , 4 , 0 ) )
{ rtB . nsbfkssthw [ 0 ] = rtB . dp0qd0ktnr [ 0 ] * rtDW . joib1qjnje [ 0 ] ;
rtB . nsbfkssthw [ 1 ] = rtB . dp0qd0ktnr [ 1 ] * rtDW . joib1qjnje [ 1 ] ;
rtB . nsbfkssthw [ 2 ] = rtB . dp0qd0ktnr [ 2 ] * rtDW . joib1qjnje [ 2 ] ;
rtB . nsbfkssthw [ 3 ] = rtB . dp0qd0ktnr [ 3 ] * rtDW . joib1qjnje [ 3 ] ; }
if ( ssIsMajorTimeStep ( rtS ) ) { if ( rtDW . oo4gabx5lo != 0 ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; rtDW . oo4gabx5lo = 0 ;
} rtB . gsfqejjcfo = muDoubleScalarSqrt ( slat ) ; } else if ( slat < 0.0 ) {
rtB . gsfqejjcfo = - muDoubleScalarSqrt ( muDoubleScalarAbs ( slat ) ) ; rtDW
. oo4gabx5lo = 1 ; } else { rtB . gsfqejjcfo = muDoubleScalarSqrt ( slat ) ;
} slat = 3.280839895013123 * rtB . gsfqejjcfo ; if ( ssIsSampleHit ( rtS , 1
, 0 ) && ssIsMajorTimeStep ( rtS ) ) { if ( rtP .
DrydenWindTurbulenceModelContinuousqr_T_on > 0.0 ) { if ( ! rtDW . ikp5xo5ucv
) { if ( ssGetTaskTime ( rtS , 1 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } ( void ) memset ( & (
( ( XDis * ) ssGetContStateDisabled ( rtS ) ) -> obrj2mgpdb ) , 0 , 2 *
sizeof ( boolean_T ) ) ; rtX . obrj2mgpdb [ 0 ] = rtP . pgw_p_IC ; rtX .
obrj2mgpdb [ 1 ] = rtP . pgw_p_IC ; rtDW . ikp5xo5ucv = true ; } } else { if
( rtDW . ikp5xo5ucv ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ;
( void ) memset ( & ( ( ( XDis * ) ssGetContStateDisabled ( rtS ) ) ->
obrj2mgpdb ) , 1 , 2 * sizeof ( boolean_T ) ) ; rtDW . ikp5xo5ucv = false ; }
} } if ( rtDW . ikp5xo5ucv ) { cospsi = 0.8 / slat ; if ( cospsi < 0.0 ) {
cospsi = - muDoubleScalarSqrt ( - cospsi ) ; } else { cospsi =
muDoubleScalarSqrt ( cospsi ) ; } temp1 = slat * rtB . plmewtggpi ; if ( (
lat < 0.0 ) && ( rtP . Constant1_Value > muDoubleScalarFloor ( rtP .
Constant1_Value ) ) ) { sinphi = - muDoubleScalarPower ( - lat , rtP .
Constant1_Value ) ; } else { sinphi = muDoubleScalarPower ( lat , rtP .
Constant1_Value ) ; } rtB . f3yq1g0dcx [ 0 ] = ( cospsi / sinphi * rtB .
fffhvqsyn4 * rtB . nsbfkssthw [ 3 ] - rtX . obrj2mgpdb [ 0 ] ) * temp1 ; if (
( q < 0.0 ) && ( rtP . Constant1_Value > muDoubleScalarFloor ( rtP .
Constant1_Value ) ) ) { sinphi = - muDoubleScalarPower ( - q , rtP .
Constant1_Value ) ; } else { sinphi = muDoubleScalarPower ( q , rtP .
Constant1_Value ) ; } rtB . f3yq1g0dcx [ 1 ] = ( cospsi / sinphi * rtB .
fffhvqsyn4 * rtB . nsbfkssthw [ 3 ] - rtX . obrj2mgpdb [ 1 ] ) * temp1 ; if (
ssIsMajorTimeStep ( rtS ) ) { srUpdateBC ( rtDW . byzsjh0px3 ) ; } } if (
ssIsSampleHit ( rtS , 1 , 0 ) && ssIsMajorTimeStep ( rtS ) ) { if ( rtP .
DrydenWindTurbulenceModelContinuousqr_T_on > 0.0 ) { if ( ! rtDW . nztae4vpib
) { if ( ssGetTaskTime ( rtS , 1 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } ( void ) memset ( & (
( ( XDis * ) ssGetContStateDisabled ( rtS ) ) -> jcf02uzlnt ) , 0 , 4 *
sizeof ( boolean_T ) ) ; rtX . jcf02uzlnt [ 0 ] = rtP . wg_p1_IC ; rtX .
go1vncxzci [ 0 ] = rtP . wg_p2_IC ; rtX . jcf02uzlnt [ 1 ] = rtP . wg_p1_IC ;
rtX . go1vncxzci [ 1 ] = rtP . wg_p2_IC ; rtDW . nztae4vpib = true ; } } else
{ if ( rtDW . nztae4vpib ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS
) ; ( void ) memset ( & ( ( ( XDis * ) ssGetContStateDisabled ( rtS ) ) ->
jcf02uzlnt ) , 1 , 4 * sizeof ( boolean_T ) ) ; rtB . ircgwytey3 [ 0 ] = rtP
. wgw_Y0 ; rtB . ircgwytey3 [ 1 ] = rtP . wgw_Y0 ; rtDW . nztae4vpib = false
; } } } if ( rtDW . nztae4vpib ) { cospsi = lat / slat ; frac [ 0 ] = rtP .
upi_Gain_ii5rsz2pui * cospsi ; lat = cospsi ; cospsi = q / slat ; frac [ 1 ]
= rtP . upi_Gain_ii5rsz2pui * cospsi ; if ( ssIsMajorTimeStep ( rtS ) ) { if
( rtDW . gylng33jck != 0 ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS
) ; rtDW . gylng33jck = 0 ; } sinphi = muDoubleScalarSqrt ( frac [ 0 ] ) ;
mc31ssftz2_idx_1 = muDoubleScalarSqrt ( frac [ 1 ] ) ; } else { if ( frac [ 0
] < 0.0 ) { sinphi = - muDoubleScalarSqrt ( muDoubleScalarAbs ( frac [ 0 ] )
) ; rtDW . gylng33jck = 1 ; } else { sinphi = muDoubleScalarSqrt ( frac [ 0 ]
) ; } if ( frac [ 1 ] < 0.0 ) { mc31ssftz2_idx_1 = - muDoubleScalarSqrt (
muDoubleScalarAbs ( frac [ 1 ] ) ) ; rtDW . gylng33jck = 1 ; } else {
mc31ssftz2_idx_1 = muDoubleScalarSqrt ( frac [ 1 ] ) ; } } rtB . ircgwytey3 [
0 ] = rtB . pk4ybsmrx1 * rtX . go1vncxzci [ 0 ] ; rtB . ircgwytey3 [ 1 ] =
o2mmi0bsgf * rtX . go1vncxzci [ 1 ] ; rtB . lxqebjchye [ 0 ] = ( sinphi * rtB
. nsbfkssthw [ 2 ] - rtX . jcf02uzlnt [ 0 ] ) / lat ; rtB . c5gx5xznq1 [ 0 ]
= ( rtB . lxqebjchye [ 0 ] * rtB . kxuhls1vu4 * lat + ( rtX . jcf02uzlnt [ 0
] - rtX . go1vncxzci [ 0 ] ) ) / lat ; rtB . lxqebjchye [ 1 ] = (
mc31ssftz2_idx_1 * rtB . nsbfkssthw [ 2 ] - rtX . jcf02uzlnt [ 1 ] ) / cospsi
; rtB . c5gx5xznq1 [ 1 ] = ( rtB . lxqebjchye [ 1 ] * rtB . kxuhls1vu4 *
cospsi + ( rtX . jcf02uzlnt [ 1 ] - rtX . go1vncxzci [ 1 ] ) ) / cospsi ; if
( ssIsMajorTimeStep ( rtS ) ) { srUpdateBC ( rtDW . n5gzkaa0nd ) ; } } if (
ssIsSampleHit ( rtS , 1 , 0 ) && ssIsMajorTimeStep ( rtS ) ) { if ( rtP .
DrydenWindTurbulenceModelContinuousqr_T_on > 0.0 ) { if ( ! rtDW . auringonfr
) { if ( ssGetTaskTime ( rtS , 1 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } ( void ) memset ( & (
( ( XDis * ) ssGetContStateDisabled ( rtS ) ) -> b5qj413fkr ) , 0 , 2 *
sizeof ( boolean_T ) ) ; rtX . b5qj413fkr [ 0 ] = rtP . qgw_p_IC ; rtX .
b5qj413fkr [ 1 ] = rtP . qgw_p_IC ; rtDW . auringonfr = true ; } } else { if
( rtDW . auringonfr ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ;
( void ) memset ( & ( ( ( XDis * ) ssGetContStateDisabled ( rtS ) ) ->
b5qj413fkr ) , 1 , 2 * sizeof ( boolean_T ) ) ; rtB . agqbvy2aau [ 0 ] = rtP
. qgw_Y0 ; rtB . agqbvy2aau [ 1 ] = rtP . qgw_Y0 ; rtDW . auringonfr = false
; } } } if ( rtDW . auringonfr ) { temp1 = rtP . pi4_Gain * slat ; rtB .
agqbvy2aau [ 0 ] = ( rtB . ircgwytey3 [ 0 ] / slat - rtX . b5qj413fkr [ 0 ] )
* ( temp1 / rtB . bav3d1hobe ) ; rtB . agqbvy2aau [ 1 ] = ( rtB . ircgwytey3
[ 1 ] / slat - rtX . b5qj413fkr [ 1 ] ) * ( temp1 / rtB . bav3d1hobe ) ; if (
ssIsMajorTimeStep ( rtS ) ) { srUpdateBC ( rtDW . npachunt01 ) ; } } if (
ssIsMajorTimeStep ( rtS ) ) { rtDW . m2dc0faool = rtB . bs4j1k4omq >= rtP .
LimitHeighth1000ft_UpperSat ? 1 : rtB . bs4j1k4omq > rtP .
LimitHeighth1000ft_LowerSat ? 0 : - 1 ; } cospsi = ( rtDW . m2dc0faool == 1 ?
rtP . LimitHeighth1000ft_UpperSat : rtDW . m2dc0faool == - 1 ? rtP .
LimitHeighth1000ft_LowerSat : rtB . bs4j1k4omq ) * 0.000823 + 0.177 ; if (
cospsi < 0.0 ) { cospsi = - muDoubleScalarPower ( - cospsi , 0.4 ) ; } else {
cospsi = muDoubleScalarPower ( cospsi , 0.4 ) ; } temp1 = 1.0 / cospsi * rtB
. pk4ybsmrx1 ; cospsi = 0.000823 * fphi + 0.177 ; if ( cospsi < 0.0 ) {
cospsi = - muDoubleScalarPower ( - cospsi , 1.2 ) ; } else { cospsi =
muDoubleScalarPower ( cospsi , 1.2 ) ; } fphi /= cospsi ; lat = rtP . Lv_Gain
* fphi ; q = rtP . Lv_Gain * rtB . o1nidekcl5 ; if ( ssIsSampleHit ( rtS , 1
, 0 ) && ssIsMajorTimeStep ( rtS ) ) { if ( rtP .
DrydenWindTurbulenceModelContinuousqr_T_on > 0.0 ) { if ( ! rtDW . ahb01lfm54
) { if ( ssGetTaskTime ( rtS , 1 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } ( void ) memset ( & (
( ( XDis * ) ssGetContStateDisabled ( rtS ) ) -> ds4avnfqd3 ) , 0 , 4 *
sizeof ( boolean_T ) ) ; rtX . ds4avnfqd3 [ 0 ] = rtP . vg_p1_IC ; rtX .
p11ukldqr3 [ 0 ] = rtP . vgw_p2_IC ; rtX . ds4avnfqd3 [ 1 ] = rtP . vg_p1_IC
; rtX . p11ukldqr3 [ 1 ] = rtP . vgw_p2_IC ; rtDW . ahb01lfm54 = true ; } }
else { if ( rtDW . ahb01lfm54 ) { ssSetBlockStateForSolverChangedAtMajorStep
( rtS ) ; ( void ) memset ( & ( ( ( XDis * ) ssGetContStateDisabled ( rtS ) )
-> ds4avnfqd3 ) , 1 , 4 * sizeof ( boolean_T ) ) ; rtB . k3touvbkw2 [ 0 ] =
rtP . vgw_Y0 ; rtB . k3touvbkw2 [ 1 ] = rtP . vgw_Y0 ; rtDW . ahb01lfm54 =
false ; } } } if ( rtDW . ahb01lfm54 ) { cospsi = lat / slat ; frac [ 0 ] =
rtP . upi_Gain_dmjomzxo4q * cospsi ; lat = cospsi ; cospsi = q / slat ; frac
[ 1 ] = rtP . upi_Gain_dmjomzxo4q * cospsi ; if ( ssIsMajorTimeStep ( rtS ) )
{ if ( rtDW . o2i2c0ygga != 0 ) { ssSetBlockStateForSolverChangedAtMajorStep
( rtS ) ; rtDW . o2i2c0ygga = 0 ; } sinphi = muDoubleScalarSqrt ( frac [ 0 ]
) ; mc31ssftz2_idx_1 = muDoubleScalarSqrt ( frac [ 1 ] ) ; } else { if ( frac
[ 0 ] < 0.0 ) { sinphi = - muDoubleScalarSqrt ( muDoubleScalarAbs ( frac [ 0
] ) ) ; rtDW . o2i2c0ygga = 1 ; } else { sinphi = muDoubleScalarSqrt ( frac [
0 ] ) ; } if ( frac [ 1 ] < 0.0 ) { mc31ssftz2_idx_1 = - muDoubleScalarSqrt (
muDoubleScalarAbs ( frac [ 1 ] ) ) ; rtDW . o2i2c0ygga = 1 ; } else {
mc31ssftz2_idx_1 = muDoubleScalarSqrt ( frac [ 1 ] ) ; } } rtB . ckuncjpeqo [
0 ] = ( sinphi * rtB . nsbfkssthw [ 1 ] - rtX . ds4avnfqd3 [ 0 ] ) / lat ;
rtB . gehuvldt5n [ 0 ] = ( rtB . ckuncjpeqo [ 0 ] * lat * rtP . sqrt3_Gain +
( rtX . ds4avnfqd3 [ 0 ] - rtX . p11ukldqr3 [ 0 ] ) ) / lat ; rtB .
ckuncjpeqo [ 1 ] = ( mc31ssftz2_idx_1 * rtB . nsbfkssthw [ 1 ] - rtX .
ds4avnfqd3 [ 1 ] ) / cospsi ; rtB . gehuvldt5n [ 1 ] = ( rtB . ckuncjpeqo [ 1
] * cospsi * rtP . sqrt3_Gain + ( rtX . ds4avnfqd3 [ 1 ] - rtX . p11ukldqr3 [
1 ] ) ) / cospsi ; rtB . k3touvbkw2 [ 0 ] = temp1 * rtX . p11ukldqr3 [ 0 ] ;
rtB . k3touvbkw2 [ 1 ] = o2mmi0bsgf * rtX . p11ukldqr3 [ 1 ] ; if (
ssIsMajorTimeStep ( rtS ) ) { srUpdateBC ( rtDW . gkuxqeqcdi ) ; } } if (
ssIsSampleHit ( rtS , 1 , 0 ) && ssIsMajorTimeStep ( rtS ) ) { if ( rtP .
DrydenWindTurbulenceModelContinuousqr_T_on > 0.0 ) { if ( ! rtDW . bdjnrykkzo
) { if ( ssGetTaskTime ( rtS , 1 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } ( void ) memset ( & (
( ( XDis * ) ssGetContStateDisabled ( rtS ) ) -> efv1zergvx ) , 0 , 2 *
sizeof ( boolean_T ) ) ; rtX . efv1zergvx [ 0 ] = rtP . rgw_p_IC ; rtX .
efv1zergvx [ 1 ] = rtP . rgw_p_IC ; rtDW . bdjnrykkzo = true ; } } else { if
( rtDW . bdjnrykkzo ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ;
( void ) memset ( & ( ( ( XDis * ) ssGetContStateDisabled ( rtS ) ) ->
efv1zergvx ) , 1 , 2 * sizeof ( boolean_T ) ) ; rtB . mv5ztdhgch [ 0 ] = rtP
. rgw_Y0 ; rtB . mv5ztdhgch [ 1 ] = rtP . rgw_Y0 ; rtDW . bdjnrykkzo = false
; } } } if ( rtDW . bdjnrykkzo ) { cospsi = rtP . pi3_Gain * slat ; rtB .
mv5ztdhgch [ 0 ] = ( rtB . k3touvbkw2 [ 0 ] / slat - rtX . efv1zergvx [ 0 ] )
* ( cospsi / rtB . bav3d1hobe ) ; rtB . mv5ztdhgch [ 1 ] = ( rtB . k3touvbkw2
[ 1 ] / slat - rtX . efv1zergvx [ 1 ] ) * ( cospsi / rtB . bav3d1hobe ) ; if
( ssIsMajorTimeStep ( rtS ) ) { srUpdateBC ( rtDW . adhcumf2n5 ) ; } } if (
ssIsSampleHit ( rtS , 1 , 0 ) && ssIsMajorTimeStep ( rtS ) ) { if ( rtP .
DrydenWindTurbulenceModelContinuousqr_T_on > 0.0 ) { if ( ! rtDW . aovs2hjxyi
) { if ( ssGetTaskTime ( rtS , 1 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } ( void ) memset ( & (
( ( XDis * ) ssGetContStateDisabled ( rtS ) ) -> galtq11byd ) , 0 , 2 *
sizeof ( boolean_T ) ) ; rtX . galtq11byd [ 0 ] = rtP . ug_p_IC ; rtX .
galtq11byd [ 1 ] = rtP . ug_p_IC ; rtDW . aovs2hjxyi = true ; } } else { if (
rtDW . aovs2hjxyi ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; (
void ) memset ( & ( ( ( XDis * ) ssGetContStateDisabled ( rtS ) ) ->
galtq11byd ) , 1 , 2 * sizeof ( boolean_T ) ) ; rtB . leujf5c51t [ 0 ] = rtP
. ugw_Y0 ; rtB . leujf5c51t [ 1 ] = rtP . ugw_Y0 ; rtDW . aovs2hjxyi = false
; } } } if ( rtDW . aovs2hjxyi ) { lat = fphi / slat ; q = rtB . o1nidekcl5 /
slat ; frac [ 0 ] = rtP . upi_Gain * lat ; frac [ 1 ] = rtP . upi_Gain * q ;
if ( ssIsMajorTimeStep ( rtS ) ) { if ( rtDW . jddl4uj35t != 0 ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; rtDW . jddl4uj35t = 0 ;
} sinphi = muDoubleScalarSqrt ( frac [ 0 ] ) ; mc31ssftz2_idx_1 =
muDoubleScalarSqrt ( frac [ 1 ] ) ; } else { if ( frac [ 0 ] < 0.0 ) { sinphi
= - muDoubleScalarSqrt ( muDoubleScalarAbs ( frac [ 0 ] ) ) ; rtDW .
jddl4uj35t = 1 ; } else { sinphi = muDoubleScalarSqrt ( frac [ 0 ] ) ; } if (
frac [ 1 ] < 0.0 ) { mc31ssftz2_idx_1 = - muDoubleScalarSqrt (
muDoubleScalarAbs ( frac [ 1 ] ) ) ; rtDW . jddl4uj35t = 1 ; } else {
mc31ssftz2_idx_1 = muDoubleScalarSqrt ( frac [ 1 ] ) ; } } rtB . fosv245qat [
0 ] = ( sinphi * rtB . nsbfkssthw [ 0 ] - rtX . galtq11byd [ 0 ] ) / lat ;
rtB . fosv245qat [ 1 ] = ( mc31ssftz2_idx_1 * rtB . nsbfkssthw [ 0 ] - rtX .
galtq11byd [ 1 ] ) / q ; rtB . leujf5c51t [ 0 ] = rtX . galtq11byd [ 0 ] *
temp1 ; rtB . leujf5c51t [ 1 ] = rtX . galtq11byd [ 1 ] * o2mmi0bsgf ; if (
ssIsMajorTimeStep ( rtS ) ) { srUpdateBC ( rtDW . krzqaoxmqz ) ; } }
rtPrevAction = rtDW . jbrbgwy42h ; if ( ssIsMajorTimeStep ( rtS ) ) { if (
rtB . bs4j1k4omq <= 1000.0 ) { rtAction = 0 ; } else if ( rtB . bs4j1k4omq >=
2000.0 ) { rtAction = 1 ; } else { rtAction = 2 ; } rtDW . jbrbgwy42h =
rtAction ; } else { rtAction = rtDW . jbrbgwy42h ; } if ( rtPrevAction !=
rtAction ) { switch ( rtPrevAction ) { case 0 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; break ; case 1 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; break ; case 2 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; break ; } } switch (
rtAction ) { case 0 : if ( rtAction != rtPrevAction ) { if ( ssGetTaskTime (
rtS , 0 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } } muDoubleScalarSinCos
( rtB . pjsvxmsmf0 , & o2mmi0bsgf , & slat ) ; if ( ssIsMajorTimeStep ( rtS )
) { srUpdateBC ( rtDW . fvgndvljcd ) ; } break ; case 1 : if ( rtAction !=
rtPrevAction ) { if ( ssGetTaskTime ( rtS , 0 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } } if (
ssIsMajorTimeStep ( rtS ) ) { srUpdateBC ( rtDW . ejm2q33rwd ) ; } break ;
case 2 : if ( rtAction != rtPrevAction ) { if ( ssGetTaskTime ( rtS , 0 ) !=
ssGetTStart ( rtS ) ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ;
} } muDoubleScalarSinCos ( rtB . pjsvxmsmf0 , & o2mmi0bsgf , & slat ) ; if (
ssIsMajorTimeStep ( rtS ) ) { srUpdateBC ( rtDW . pkklmgxn3w ) ; } break ; }
rtPrevAction = rtDW . nb15cebgib ; if ( ssIsMajorTimeStep ( rtS ) ) { if (
rtB . bs4j1k4omq <= 1000.0 ) { rtAction = 0 ; } else if ( rtB . bs4j1k4omq >=
2000.0 ) { rtAction = 1 ; } else { rtAction = 2 ; } rtDW . nb15cebgib =
rtAction ; } else { rtAction = rtDW . nb15cebgib ; } if ( rtPrevAction !=
rtAction ) { switch ( rtPrevAction ) { case 0 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; break ; case 1 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; break ; case 2 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; break ; } } switch (
rtAction ) { case 0 : if ( rtAction != rtPrevAction ) { if ( ssGetTaskTime (
rtS , 0 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } } muDoubleScalarSinCos
( rtB . pjsvxmsmf0 , & o2mmi0bsgf , & slat ) ; e4kgnowjgg [ 0 ] = rtB .
leujf5c51t [ 0 ] * slat - o2mmi0bsgf * rtB . k3touvbkw2 [ 0 ] ; e4kgnowjgg [
1 ] = rtB . k3touvbkw2 [ 0 ] * slat + o2mmi0bsgf * rtB . leujf5c51t [ 0 ] ;
for ( oxjy2hf2cl = 0 ; oxjy2hf2cl < 3 ; oxjy2hf2cl ++ ) { avxeaykvgw [
oxjy2hf2cl ] = jjatrktc0c [ oxjy2hf2cl + 6 ] * rtB . ircgwytey3 [ 0 ] + (
jjatrktc0c [ oxjy2hf2cl + 3 ] * e4kgnowjgg [ 1 ] + jjatrktc0c [ oxjy2hf2cl ]
* e4kgnowjgg [ 0 ] ) ; } if ( ssIsMajorTimeStep ( rtS ) ) { srUpdateBC ( rtDW
. jwt5flmp0h ) ; } break ; case 1 : if ( rtAction != rtPrevAction ) { if (
ssGetTaskTime ( rtS , 0 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } } avxeaykvgw [ 0 ] =
rtP . Gain_Gain_bzpcr2c2vd * rtB . leujf5c51t [ 1 ] ; avxeaykvgw [ 1 ] = rtP
. Gain_Gain_bzpcr2c2vd * rtB . k3touvbkw2 [ 1 ] ; avxeaykvgw [ 2 ] = rtP .
Gain_Gain_bzpcr2c2vd * rtB . ircgwytey3 [ 1 ] ; if ( ssIsMajorTimeStep ( rtS
) ) { srUpdateBC ( rtDW . nv0y4bxbiz ) ; } break ; case 2 : if ( rtAction !=
rtPrevAction ) { if ( ssGetTaskTime ( rtS , 0 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } } muDoubleScalarSinCos
( rtB . pjsvxmsmf0 , & o2mmi0bsgf , & slat ) ; avxeaykvgw [ 0 ] = rtB .
leujf5c51t [ 0 ] * slat - o2mmi0bsgf * rtB . k3touvbkw2 [ 0 ] ; avxeaykvgw [
1 ] = rtB . k3touvbkw2 [ 0 ] * slat + o2mmi0bsgf * rtB . leujf5c51t [ 0 ] ;
for ( oxjy2hf2cl = 0 ; oxjy2hf2cl < 3 ; oxjy2hf2cl ++ ) { e4kgnowjgg [
oxjy2hf2cl ] = jjatrktc0c [ oxjy2hf2cl + 6 ] * rtB . ircgwytey3 [ 0 ] + (
jjatrktc0c [ oxjy2hf2cl + 3 ] * avxeaykvgw [ 1 ] + jjatrktc0c [ oxjy2hf2cl ]
* avxeaykvgw [ 0 ] ) ; } o2mmi0bsgf = rtB . bs4j1k4omq - rtP .
max_height_low_Value_bkbuta4yqd ; avxeaykvgw [ 0 ] = ( rtB . leujf5c51t [ 1 ]
- e4kgnowjgg [ 0 ] ) * o2mmi0bsgf / rtB . ntt1gar3xf + e4kgnowjgg [ 0 ] ;
avxeaykvgw [ 1 ] = ( rtB . k3touvbkw2 [ 1 ] - e4kgnowjgg [ 1 ] ) * o2mmi0bsgf
/ rtB . ntt1gar3xf + e4kgnowjgg [ 1 ] ; avxeaykvgw [ 2 ] = ( rtB . ircgwytey3
[ 1 ] - e4kgnowjgg [ 2 ] ) * o2mmi0bsgf / rtB . ntt1gar3xf + e4kgnowjgg [ 2 ]
; if ( ssIsMajorTimeStep ( rtS ) ) { srUpdateBC ( rtDW . cvvf5wjoub ) ; }
break ; } avxeaykvgw [ 0 ] *= 0.3048 ; avxeaykvgw [ 1 ] *= 0.3048 ; rtB .
ewd50jnfc5 = 3.280839895013123 * rtB . hkzk0mcpdz ; if ( ssIsMajorTimeStep (
rtS ) ) { rtDW . n4rcpokqpc = rtB . ewd50jnfc5 >= rtP . uftinf_UpperSat ? 1 :
rtB . ewd50jnfc5 > rtP . uftinf_LowerSat ? 0 : - 1 ; } o2mmi0bsgf = ( rtDW .
n4rcpokqpc == 1 ? rtP . uftinf_UpperSat : rtDW . n4rcpokqpc == - 1 ? rtP .
uftinf_LowerSat : rtB . ewd50jnfc5 ) * rtP . hz0_Gain ; if (
ssIsMajorTimeStep ( rtS ) ) { if ( rtDW . fnjwuvlmmj != 0 ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; rtDW . fnjwuvlmmj = 0 ;
} } else { if ( o2mmi0bsgf < 0.0 ) { o2mmi0bsgf = 0.0 ; rtDW . fnjwuvlmmj = 1
; } } o2mmi0bsgf = muDoubleScalarLog ( o2mmi0bsgf ) / rtB . or3uazt4fw ; for
( oxjy2hf2cl = 0 ; oxjy2hf2cl < 3 ; oxjy2hf2cl ++ ) { daxlvr32qq [ oxjy2hf2cl
] = jjatrktc0c [ oxjy2hf2cl + 6 ] * ( o2mmi0bsgf * rtB . fzxirwzcjx [ 2 ] ) +
( jjatrktc0c [ oxjy2hf2cl + 3 ] * ( o2mmi0bsgf * rtB . fzxirwzcjx [ 1 ] ) +
o2mmi0bsgf * rtB . fzxirwzcjx [ 0 ] * jjatrktc0c [ oxjy2hf2cl ] ) ; } if (
ssIsSampleHit ( rtS , 1 , 0 ) ) { rtDW . gofb51vunu = ( ssGetTaskTime ( rtS ,
1 ) >= rtP . Step_Time ) ; if ( rtDW . gofb51vunu == 1 ) { fphi = rtP .
Step_YFinal ; } else { fphi = rtP . Step_Y0 ; } rtB . e4lsqrzqjh = rtP . alt0
+ fphi ; } temp1 = rtP . StateSpace_C * rtX . lwmusv4p0k ; if ( rtDW .
e0khmaiz1x == ( rtInf ) ) { rtB . j2oyzznxjz = temp1 ; } else { slat =
ssGetTaskTime ( rtS , 0 ) - rtDW . e0khmaiz1x ; fphi = slat * rtP .
RateLimiter_RisingLim ; o2mmi0bsgf = temp1 - rtDW . foeszin0cg ; if (
o2mmi0bsgf > fphi ) { rtB . j2oyzznxjz = rtDW . foeszin0cg + fphi ; } else {
slat *= rtP . RateLimiter_FallingLim ; if ( o2mmi0bsgf < slat ) { rtB .
j2oyzznxjz = rtDW . foeszin0cg + slat ; } else { rtB . j2oyzznxjz = temp1 ; }
} } { real_T * temp_table = ( real_T * ) & rtDW . gyluhpuobq [ 0 ] ; real_T *
pres_table = ( real_T * ) & rtDW . hfu1jte3yo [ 0 ] ; CalcPAltCOESA ( & rtB .
dv0mxcbo4y , & rtB . jeyqu0v1xd , temp_table , pres_table , 1 ) ; } if (
ssIsSampleHit ( rtS , 2 , 0 ) ) { rtB . c2uwt5jnat = rtB . jeyqu0v1xd ;
jb2eust2qa = rtB . fdu3hdd1wq ; } rtB . erkqk25euy = rtB . j2oyzznxjz - rtB .
c2uwt5jnat ; if ( ssIsSampleHit ( rtS , 2 , 0 ) ) { { ij1arac1oi = ( rtP .
AltController_C [ 0 ] ) * rtDW . dh0tof3kep [ 0 ] + ( rtP . AltController_C [
1 ] ) * rtDW . dh0tof3kep [ 1 ] ; ij1arac1oi += rtP . AltController_D * rtB .
erkqk25euy ; } rtB . chm2baiq0o = ij1arac1oi - jb2eust2qa ; { rtB .
hevlxlfvxw = ( rtP . ThetaController_D ) * rtB . chm2baiq0o ; rtB .
hevlxlfvxw += rtP . ThetaController_C * rtDW . glj000y2ul ; } } o2mmi0bsgf =
( gkpo2ijrbk_idx_0 * gkpo2ijrbk_idx_0 + gkpo2ijrbk_idx_1 * gkpo2ijrbk_idx_1 )
+ gkpo2ijrbk_idx_2 * gkpo2ijrbk_idx_2 ; if ( ssIsMajorTimeStep ( rtS ) ) { if
( rtDW . bh43qqaomf != 0 ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS
) ; rtDW . bh43qqaomf = 0 ; } rtB . j1pkkc3ac5 = muDoubleScalarSqrt (
o2mmi0bsgf ) ; } else if ( o2mmi0bsgf < 0.0 ) { rtB . j1pkkc3ac5 = -
muDoubleScalarSqrt ( muDoubleScalarAbs ( o2mmi0bsgf ) ) ; rtDW . bh43qqaomf =
1 ; } else { rtB . j1pkkc3ac5 = muDoubleScalarSqrt ( o2mmi0bsgf ) ; } if (
ssIsSampleHit ( rtS , 1 , 0 ) ) { rtPrevAction = rtDW . cal4znigzs ; rtAction
= - 1 ; if ( ssIsMajorTimeStep ( rtS ) ) { rtDW . ex1irzrias =
muDoubleScalarIsNaN ( rtB . j1pkkc3ac5 ) ; if ( ! rtDW . ex1irzrias ) {
rtAction = 0 ; } rtDW . cal4znigzs = rtAction ; } else { rtAction = rtDW .
cal4znigzs ; } if ( ( rtPrevAction != rtAction ) && ( rtPrevAction == 0 ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } if ( rtAction == 0 ) {
if ( 0 != rtPrevAction ) { if ( ssGetTaskTime ( rtS , 1 ) != ssGetTStart (
rtS ) ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } } if (
ssIsMajorTimeStep ( rtS ) ) { rtDW . jteqln0zcu = ( rtB . j1pkkc3ac5 >= rtP .
Constant_Value_pmyhtxrqrr ) ; } assessmentPtrVar = ( void * ) &
assessmentVar_m ; if ( rtDW . jteqln0zcu ) { assessmentVar_m = 0 ;
rt_SlioAccessorUpdate ( 1 , 3 , rtDW . h4fprkvunp , ssGetT ( rtS ) ,
assessmentPtrVar ) ; } else { S = rtS ; diag = CreateDiagnosticAsVoidPtr (
"Simulink:blocks:AssertionAssert" , 2 , 5 ,
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Validate Inputs/If Action Subsystem1/Assertion"
, 2 , ssGetT ( rtS ) ) ; rt_ssSet_slErrMsg ( S , diag ) ; ssSetStopRequested
( rtS , ( int ) ssGetT ( rtS ) ) ; assessmentVar_m = 1 ;
rt_SlioAccessorUpdate ( 1 , 3 , rtDW . h4fprkvunp , ssGetT ( rtS ) ,
assessmentPtrVar ) ; } if ( ssIsMajorTimeStep ( rtS ) ) { srUpdateBC ( rtDW .
l3k54bxdj1 ) ; } } rtPrevAction = rtDW . o3p0d0pvrk ; rtAction = - 1 ; if (
ssIsMajorTimeStep ( rtS ) ) { rtDW . pldxv1lqbx = muDoubleScalarIsNaN ( rtB .
ivvuf1kwbw ) ; if ( ! rtDW . pldxv1lqbx ) { rtAction = 0 ; } rtDW .
o3p0d0pvrk = rtAction ; } else { rtAction = rtDW . o3p0d0pvrk ; } if ( (
rtPrevAction != rtAction ) && ( rtPrevAction == 0 ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } if ( rtAction == 0 ) {
if ( 0 != rtPrevAction ) { if ( ssGetTaskTime ( rtS , 1 ) != ssGetTStart (
rtS ) ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } } if (
ssIsMajorTimeStep ( rtS ) ) { rtDW . dvdytaltsj = ( rtB . ivvuf1kwbw >= rtP .
Constant_Value_gun4ws4ai2 ) ; } assessmentPtrVar = ( void * ) &
assessmentVar_i ; if ( rtDW . dvdytaltsj ) { assessmentVar_i = 0 ;
rt_SlioAccessorUpdate ( 1 , 3 , rtDW . gu0lptjayg , ssGetT ( rtS ) ,
assessmentPtrVar ) ; } else { S = rtS ; diag = CreateDiagnosticAsVoidPtr (
"Simulink:blocks:AssertionAssert" , 2 , 5 ,
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Validate Inputs/If Action Subsystem2/Assertion"
, 2 , ssGetT ( rtS ) ) ; rt_ssSet_slErrMsg ( S , diag ) ; ssSetStopRequested
( rtS , ( int ) ssGetT ( rtS ) ) ; assessmentVar_i = 1 ;
rt_SlioAccessorUpdate ( 1 , 3 , rtDW . gu0lptjayg , ssGetT ( rtS ) ,
assessmentPtrVar ) ; } if ( ssIsMajorTimeStep ( rtS ) ) { srUpdateBC ( rtDW .
p4vcgbddhj ) ; } } rtPrevAction = rtDW . mxjlbo5rjs ; rtAction = - 1 ; if (
ssIsMajorTimeStep ( rtS ) ) { rtDW . guox3atybt = muDoubleScalarIsNaN ( rtB .
dv0mxcbo4y ) ; if ( ! rtDW . guox3atybt ) { rtAction = 0 ; } rtDW .
mxjlbo5rjs = rtAction ; } else { rtAction = rtDW . mxjlbo5rjs ; } if ( (
rtPrevAction != rtAction ) && ( rtPrevAction == 0 ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } if ( rtAction == 0 ) {
if ( 0 != rtPrevAction ) { if ( ssGetTaskTime ( rtS , 1 ) != ssGetTStart (
rtS ) ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } } if (
ssIsMajorTimeStep ( rtS ) ) { rtDW . oq5swglxht = ( rtB . dv0mxcbo4y >= rtP .
Constant_Value_mq1pkcil10 ) ; } assessmentPtrVar = ( void * ) &
assessmentVar_e ; if ( rtDW . oq5swglxht ) { assessmentVar_e = 0 ;
rt_SlioAccessorUpdate ( 1 , 3 , rtDW . cygydsjrfb , ssGetT ( rtS ) ,
assessmentPtrVar ) ; } else { S = rtS ; diag = CreateDiagnosticAsVoidPtr (
"Simulink:blocks:AssertionAssert" , 2 , 5 ,
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Validate Inputs/If Action Subsystem3/Assertion"
, 2 , ssGetT ( rtS ) ) ; rt_ssSet_slErrMsg ( S , diag ) ; ssSetStopRequested
( rtS , ( int ) ssGetT ( rtS ) ) ; assessmentVar_e = 1 ;
rt_SlioAccessorUpdate ( 1 , 3 , rtDW . cygydsjrfb , ssGetT ( rtS ) ,
assessmentPtrVar ) ; } if ( ssIsMajorTimeStep ( rtS ) ) { srUpdateBC ( rtDW .
poolnl3e1h ) ; } } } jb2eust2qa = rtP . seaLevelSOS_Value / rtB . ivvuf1kwbw
; o2mmi0bsgf = rtB . dv0mxcbo4y * rtB . kbkk0nabqt [ 1 ] * rtP . Po_Value ;
if ( ssIsMajorTimeStep ( rtS ) ) { if ( rtDW . hmyyl45jap != 0 ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; rtDW . hmyyl45jap = 0 ;
} slat = muDoubleScalarSqrt ( o2mmi0bsgf ) ; } else if ( o2mmi0bsgf < 0.0 ) {
slat = - muDoubleScalarSqrt ( muDoubleScalarAbs ( o2mmi0bsgf ) ) ; rtDW .
hmyyl45jap = 1 ; } else { slat = muDoubleScalarSqrt ( o2mmi0bsgf ) ; } rtB .
iiwmf2tmrv = jb2eust2qa * slat ; if ( ssIsSampleHit ( rtS , 1 , 0 ) ) {
rtPrevAction = rtDW . mdzj2hr4jc ; rtAction = - 1 ; if ( ssIsMajorTimeStep (
rtS ) ) { if ( rtP . Constant_Value_iw00e14laz < 0.0 ) { fphi =
muDoubleScalarCeil ( rtP . Constant_Value_iw00e14laz ) ; } else { fphi =
muDoubleScalarFloor ( rtP . Constant_Value_iw00e14laz ) ; } if (
muDoubleScalarIsNaN ( fphi ) || muDoubleScalarIsInf ( fphi ) ) { fphi = 0.0 ;
} else { fphi = muDoubleScalarRem ( fphi , 4.294967296E+9 ) ; } switch ( fphi
< 0.0 ? - ( int32_T ) ( uint32_T ) - fphi : ( int32_T ) ( uint32_T ) fphi ) {
case 1 : rtAction = 0 ; break ; case 2 : rtAction = 1 ; break ; case 3 : case
4 : case 5 : case 6 : rtAction = 2 ; break ; } rtDW . mdzj2hr4jc = rtAction ;
} else { rtAction = rtDW . mdzj2hr4jc ; } if ( ( rtPrevAction != rtAction )
&& ( rtPrevAction == 2 ) ) { switch ( rtDW . dvxffgw44x ) { case 0 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; break ; case 1 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; break ; } rtDW .
dvxffgw44x = - 1 ; switch ( rtDW . lfjjbagrjo ) { case 0 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; break ; case 1 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; break ; } rtDW .
lfjjbagrjo = - 1 ; switch ( rtDW . bvueotiv4b ) { case 0 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; switch ( rtDW .
g4vr3lujuz ) { case 0 : ljzopg4h4v ( rtS , & rtDW . cjigmwyptgo ) ; break ;
case 1 : ljzopg4h4v ( rtS , & rtDW . aqq0sq0gty ) ; break ; } rtDW .
g4vr3lujuz = - 1 ; break ; case 1 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; switch ( rtDW .
bfxedqpzaa ) { case 0 : ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ;
break ; case 1 : ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; break ;
} rtDW . bfxedqpzaa = - 1 ; break ; } rtDW . bvueotiv4b = - 1 ; switch ( rtDW
. hug50n4wps ) { case 0 : ssSetBlockStateForSolverChangedAtMajorStep ( rtS )
; break ; case 1 : ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; break
; } rtDW . hug50n4wps = - 1 ; } switch ( rtAction ) { case 0 : if ( rtAction
!= rtPrevAction ) { if ( ssGetTaskTime ( rtS , 1 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } } if (
ssIsMajorTimeStep ( rtS ) ) { srUpdateBC ( rtDW . dnxut4pgtq ) ; } break ;
case 1 : if ( rtAction != rtPrevAction ) { if ( ssGetTaskTime ( rtS , 1 ) !=
ssGetTStart ( rtS ) ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ;
} } if ( ssIsMajorTimeStep ( rtS ) ) { srUpdateBC ( rtDW . pdsijyzqws ) ; }
break ; case 2 : if ( rtAction != rtPrevAction ) { if ( ssGetTaskTime ( rtS ,
1 ) != ssGetTStart ( rtS ) ) { ssSetBlockStateForSolverChangedAtMajorStep (
rtS ) ; } } if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { rtPrevAction = rtDW .
dvxffgw44x ; rtAction = ( int8_T ) ! ( rtP . Constant_Value_iw00e14laz == 5.0
) ; rtDW . dvxffgw44x = rtAction ; if ( rtPrevAction != rtAction ) { switch (
rtPrevAction ) { case 0 : ssSetBlockStateForSolverChangedAtMajorStep ( rtS )
; break ; case 1 : ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; break
; } } if ( rtAction == 0 ) { if ( 0 != rtPrevAction ) { if ( ssGetTaskTime (
rtS , 1 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } } rtB . bz2agq32f5 =
rtB . j1pkkc3ac5 / rtB . iiwmf2tmrv / rtP . seaLevelSOS_Value ; srUpdateBC (
rtDW . faa1lc2tvb ) ; } else { if ( 1 != rtPrevAction ) { if ( ssGetTaskTime
( rtS , 1 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } } rtB . bz2agq32f5 =
rtB . j1pkkc3ac5 / rtP . seaLevelSOS_Value ; srUpdateBC ( rtDW . b3phozbfji )
; } rtPrevAction = rtDW . lfjjbagrjo ; rtAction = ( int8_T ) ! ( rtP .
Constant_Value_iw00e14laz == 5.0 ) ; rtDW . lfjjbagrjo = rtAction ; if (
rtPrevAction != rtAction ) { switch ( rtPrevAction ) { case 0 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; break ; case 1 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; break ; } } if (
rtAction == 0 ) { if ( 0 != rtPrevAction ) { if ( ssGetTaskTime ( rtS , 1 )
!= ssGetTStart ( rtS ) ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS )
; } } rtB . gakkrw1mku = rtP . Constant_Value_iw00e14laz ; rtB . iqcro5zqc5 =
rtB . ivvuf1kwbw ; rtB . khr4lzxr01 = rtB . dv0mxcbo4y ; rtB . otbeb0smep =
rtP . gamma_Value ; rtB . mbiqcrghxm = rtP . seaLevelPstatic_Value ; rtB .
gxcomph2oi = rtP . seaLevelSOS_Value ; rtB . lldfszhlwc = rtB . j1pkkc3ac5 /
rtB . iiwmf2tmrv ; srUpdateBC ( rtDW . f1edxk4pdy ) ; } else { if ( 1 !=
rtPrevAction ) { if ( ssGetTaskTime ( rtS , 1 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } } rtB . gakkrw1mku =
rtP . Constant_Value_iw00e14laz ; rtB . lldfszhlwc = rtB . j1pkkc3ac5 ; rtB .
iqcro5zqc5 = rtB . ivvuf1kwbw ; rtB . khr4lzxr01 = rtB . dv0mxcbo4y ; rtB .
otbeb0smep = rtP . gamma_Value ; rtB . mbiqcrghxm = rtP .
seaLevelPstatic_Value ; rtB . gxcomph2oi = rtP . seaLevelSOS_Value ;
srUpdateBC ( rtDW . axi4q1he0b ) ; } rtPrevAction = rtDW . bvueotiv4b ;
rtAction = ( int8_T ) ( ( ! ( rtP . Constant2_Value_nwrewbsrgx == 1.0 ) ) ||
( ! ( rtB . bz2agq32f5 < 5.0 ) ) ) ; rtDW . bvueotiv4b = rtAction ; if (
rtPrevAction != rtAction ) { switch ( rtPrevAction ) { case 0 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; switch ( rtDW .
g4vr3lujuz ) { case 0 : ljzopg4h4v ( rtS , & rtDW . cjigmwyptgo ) ; break ;
case 1 : ljzopg4h4v ( rtS , & rtDW . aqq0sq0gty ) ; break ; } rtDW .
g4vr3lujuz = - 1 ; break ; case 1 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; switch ( rtDW .
bfxedqpzaa ) { case 0 : ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ;
break ; case 1 : ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; break ;
} rtDW . bfxedqpzaa = - 1 ; break ; } } if ( rtAction == 0 ) { if ( 0 !=
rtPrevAction ) { if ( ssGetTaskTime ( rtS , 1 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } } rtPrevAction = rtDW
. g4vr3lujuz ; rtAction = - 1 ; if ( rtB . gakkrw1mku < 0.0 ) { fphi =
muDoubleScalarCeil ( rtB . gakkrw1mku ) ; } else { fphi = muDoubleScalarFloor
( rtB . gakkrw1mku ) ; } if ( muDoubleScalarIsNaN ( fphi ) ||
muDoubleScalarIsInf ( fphi ) ) { fphi = 0.0 ; } else { fphi =
muDoubleScalarRem ( fphi , 4.294967296E+9 ) ; } switch ( fphi < 0.0 ? - (
int32_T ) ( uint32_T ) - fphi : ( int32_T ) ( uint32_T ) fphi ) { case 3 :
case 5 : rtAction = 0 ; break ; case 4 : case 6 : rtAction = 1 ; break ; }
rtDW . g4vr3lujuz = rtAction ; if ( rtPrevAction != rtAction ) { switch (
rtPrevAction ) { case 0 : ljzopg4h4v ( rtS , & rtDW . cjigmwyptgo ) ; break ;
case 1 : ljzopg4h4v ( rtS , & rtDW . aqq0sq0gty ) ; break ; } } switch (
rtAction ) { case 0 : if ( rtAction != rtPrevAction ) { narklmqo2k ( rtS ) ;
} cjigmwyptg ( rtS , rtB . bz2agq32f5 , rtB . khr4lzxr01 , rtB . lldfszhlwc ,
rtB . iqcro5zqc5 , & rtB . pcxo4cuead , & rtDW . cjigmwyptgo ) ; srUpdateBC (
rtDW . cjigmwyptgo . jijajg5nkb ) ; break ; case 1 : if ( rtAction !=
rtPrevAction ) { narklmqo2k ( rtS ) ; } cjigmwyptg ( rtS , rtB . bz2agq32f5 ,
rtB . khr4lzxr01 , rtB . lldfszhlwc , rtB . iqcro5zqc5 , & rtB . pcxo4cuead ,
& rtDW . aqq0sq0gty ) ; srUpdateBC ( rtDW . aqq0sq0gty . jijajg5nkb ) ; break
; } srUpdateBC ( rtDW . ojqxnfreck ) ; } else { if ( 1 != rtPrevAction ) { if
( ssGetTaskTime ( rtS , 1 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } } if ( ssIsSampleHit (
rtS , 1 , 0 ) ) { rtPrevAction = rtDW . bfxedqpzaa ; rtAction = - 1 ; if (
rtB . gakkrw1mku < 0.0 ) { fphi = muDoubleScalarCeil ( rtB . gakkrw1mku ) ; }
else { fphi = muDoubleScalarFloor ( rtB . gakkrw1mku ) ; } if (
muDoubleScalarIsNaN ( fphi ) || muDoubleScalarIsInf ( fphi ) ) { fphi = 0.0 ;
} else { fphi = muDoubleScalarRem ( fphi , 4.294967296E+9 ) ; } switch ( fphi
< 0.0 ? - ( int32_T ) ( uint32_T ) - fphi : ( int32_T ) ( uint32_T ) fphi ) {
case 3 : case 5 : rtAction = 0 ; break ; case 4 : case 6 : rtAction = 1 ;
break ; } rtDW . bfxedqpzaa = rtAction ; switch ( rtAction ) { case 0 : if (
rtAction != rtPrevAction ) { if ( ssGetTaskTime ( rtS , 1 ) != ssGetTStart (
rtS ) ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } } if (
ssIsSampleHit ( rtS , 1 , 0 ) ) { if ( rtB . iqcro5zqc5 >= rtB . lldfszhlwc )
{ o2mmi0bsgf = rtB . otbeb0smep + rtP . Bias1_Bias_cjyekcjkqd ; jb2eust2qa =
1.0 / o2mmi0bsgf * rtB . otbeb0smep ; slat = rtB . lldfszhlwc / rtB .
iqcro5zqc5 ; o2mmi0bsgf = slat * slat * ( o2mmi0bsgf / rtP .
Constant_Value_bfoige4wh4 ) + rtP . Bias_Bias ; if ( ( o2mmi0bsgf < 0.0 ) &&
( jb2eust2qa > muDoubleScalarFloor ( jb2eust2qa ) ) ) { o2mmi0bsgf = -
muDoubleScalarPower ( - o2mmi0bsgf , jb2eust2qa ) ; } else { o2mmi0bsgf =
muDoubleScalarPower ( o2mmi0bsgf , jb2eust2qa ) ; } jb2eust2qa = ( o2mmi0bsgf
+ rtP . Bias2_Bias ) * rtB . khr4lzxr01 ; } else { o2mmi0bsgf = rtB .
otbeb0smep + rtP . Bias1_Bias ; jb2eust2qa = 1.0 / o2mmi0bsgf ; slat = rtB .
lldfszhlwc / rtB . iqcro5zqc5 ; slat *= slat ; fphi = rtB . otbeb0smep + rtP
. Bias3_Bias ; o2mmi0bsgf = fphi * fphi / ( rtB . otbeb0smep * rtP .
Constant2_Value_pdjiwz3ftx - 1.0 / slat * ( o2mmi0bsgf * rtP .
Constant_Value_nv1uo0d3mg ) ) ; if ( ( o2mmi0bsgf < 0.0 ) && ( jb2eust2qa >
muDoubleScalarFloor ( jb2eust2qa ) ) ) { o2mmi0bsgf = - muDoubleScalarPower (
- o2mmi0bsgf , jb2eust2qa ) ; } else { o2mmi0bsgf = muDoubleScalarPower (
o2mmi0bsgf , jb2eust2qa ) ; } jb2eust2qa = fphi / rtP .
Constant_Value_nv1uo0d3mg * slat * rtB . khr4lzxr01 * o2mmi0bsgf - rtB .
khr4lzxr01 ; } o2mmi0bsgf = jb2eust2qa / rtB . mbiqcrghxm + rtP .
Bias5_Bias_czj0v4rf55 ; slat = rtB . otbeb0smep + rtP . Bias4_Bias_cy2cy0rkru
; fphi = 1.0 / rtB . otbeb0smep * slat ; if ( ( o2mmi0bsgf < 0.0 ) && ( fphi
> muDoubleScalarFloor ( fphi ) ) ) { o2mmi0bsgf = - muDoubleScalarPower ( -
o2mmi0bsgf , fphi ) ; } else { o2mmi0bsgf = muDoubleScalarPower ( o2mmi0bsgf
, fphi ) ; } o2mmi0bsgf = ( o2mmi0bsgf + rtP . Bias6_Bias_cgt3io1in0 ) / slat
* rtP . Constant1_Value_jb5j0tvdpj ; if ( ssIsMajorTimeStep ( rtS ) ) { if (
rtDW . glvphpzltx != 0 ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS )
; rtDW . glvphpzltx = 0 ; } slat = muDoubleScalarSqrt ( o2mmi0bsgf ) ; } else
if ( o2mmi0bsgf < 0.0 ) { slat = - muDoubleScalarSqrt ( muDoubleScalarAbs (
o2mmi0bsgf ) ) ; rtDW . glvphpzltx = 1 ; } else { slat = muDoubleScalarSqrt (
o2mmi0bsgf ) ; } o2mmi0bsgf = slat * rtB . gxcomph2oi ; if ( ! ( o2mmi0bsgf <
rtB . gxcomph2oi ) ) { if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { i = 1 ; do {
assessmentPtrVar = ( void * ) & assessmentVar_p ; if ( i <= rtP .
CompareToConstant1_const ) { assessmentVar_p = 0 ; rt_SlioAccessorUpdate ( 1
, 3 , rtDW . fodolp1u3u , ssGetT ( rtS ) , assessmentPtrVar ) ; } else { S =
rtS ; diag = CreateDiagnosticAsVoidPtr ( "Simulink:blocks:AssertionAssert" ,
2 , 5 ,
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/Supersonic/Supersonic/Assertion"
, 2 , ssGetT ( rtS ) ) ; rt_ssSet_slErrMsg ( S , diag ) ; ssSetStopRequested
( rtS , ( int ) ssGetT ( rtS ) ) ; assessmentVar_p = 1 ;
rt_SlioAccessorUpdate ( 1 , 3 , rtDW . fodolp1u3u , ssGetT ( rtS ) ,
assessmentPtrVar ) ; } slat = rtDW . btjxdj1nnm ; fphi = rtB . otbeb0smep +
rtP . Bias5_Bias ; cospsi = rtB . gxcomph2oi / rtDW . btjxdj1nnm ; temp1 =
rtB . otbeb0smep + rtP . Bias4_Bias ; cospsi = fphi * fphi / ( rtB .
otbeb0smep * rtP . Constant1_Value_iv5ptljryc - cospsi * cospsi * rtP .
Constant3_Value_h0acrtutky * temp1 ) ; temp1 = 1.0 / temp1 ; if ( ( cospsi <
0.0 ) && ( temp1 > muDoubleScalarFloor ( temp1 ) ) ) { cospsi = -
muDoubleScalarPower ( - cospsi , temp1 ) ; } else { cospsi =
muDoubleScalarPower ( cospsi , temp1 ) ; } fphi = ( jb2eust2qa / rtB .
mbiqcrghxm + rtP . Bias6_Bias ) * rtP . Constant4_Value / fphi / cospsi ; if
( ssIsMajorTimeStep ( rtS ) ) { if ( rtDW . fk3ir50wko != 0 ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; rtDW . fk3ir50wko = 0 ;
} temp1 = muDoubleScalarSqrt ( fphi ) ; } else if ( fphi < 0.0 ) { temp1 = -
muDoubleScalarSqrt ( muDoubleScalarAbs ( fphi ) ) ; rtDW . fk3ir50wko = 1 ; }
else { temp1 = muDoubleScalarSqrt ( fphi ) ; } rtB . lzfl1px3we = rtB .
gxcomph2oi * temp1 ; rtDW . btjxdj1nnm = rtB . lzfl1px3we ; i ++ ; } while (
muDoubleScalarAbs ( slat - rtB . lzfl1px3we ) > rtP . CompareToConstant_const
) ; } srUpdateBC ( rtDW . giro4eplfi ) ; rtB . pcxo4cuead = rtB . lzfl1px3we
; } else { rtB . pcxo4cuead = o2mmi0bsgf ; } } srUpdateBC ( rtDW . hudho2xp0v
) ; break ; case 1 : if ( rtAction != rtPrevAction ) { if ( ssGetTaskTime (
rtS , 1 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } } if ( ssIsSampleHit (
rtS , 1 , 0 ) ) { if ( rtB . gxcomph2oi >= rtB . lldfszhlwc ) { o2mmi0bsgf =
rtB . otbeb0smep + rtP . Bias1_Bias_ncd3jtrabd ; jb2eust2qa = 1.0 /
o2mmi0bsgf * rtB . otbeb0smep ; slat = rtB . lldfszhlwc / rtB . gxcomph2oi ;
o2mmi0bsgf = slat * slat * ( o2mmi0bsgf / rtP . Constant_Value_kueaisvxhv ) +
rtP . Bias_Bias_kruphzigt4 ; if ( ( o2mmi0bsgf < 0.0 ) && ( jb2eust2qa >
muDoubleScalarFloor ( jb2eust2qa ) ) ) { o2mmi0bsgf = - muDoubleScalarPower (
- o2mmi0bsgf , jb2eust2qa ) ; } else { o2mmi0bsgf = muDoubleScalarPower (
o2mmi0bsgf , jb2eust2qa ) ; } jb2eust2qa = ( o2mmi0bsgf + rtP .
Bias2_Bias_lotf5js1s3 ) * rtB . mbiqcrghxm ; } else { o2mmi0bsgf = rtB .
otbeb0smep + rtP . Bias1_Bias_hfkjqppc2q ; jb2eust2qa = 1.0 / o2mmi0bsgf ;
slat = rtB . lldfszhlwc / rtB . gxcomph2oi ; slat *= slat ; fphi = rtB .
otbeb0smep + rtP . Bias3_Bias_jd2w4riqmn ; o2mmi0bsgf = fphi * fphi / ( rtB .
otbeb0smep * rtP . Constant2_Value_bopb4ki4xd - 1.0 / slat * ( o2mmi0bsgf *
rtP . Constant_Value_lbccuktfh2 ) ) ; if ( ( o2mmi0bsgf < 0.0 ) && (
jb2eust2qa > muDoubleScalarFloor ( jb2eust2qa ) ) ) { o2mmi0bsgf = -
muDoubleScalarPower ( - o2mmi0bsgf , jb2eust2qa ) ; } else { o2mmi0bsgf =
muDoubleScalarPower ( o2mmi0bsgf , jb2eust2qa ) ; } jb2eust2qa = fphi / rtP .
Constant_Value_lbccuktfh2 * slat * rtB . mbiqcrghxm * o2mmi0bsgf - rtB .
mbiqcrghxm ; } o2mmi0bsgf = jb2eust2qa / rtB . khr4lzxr01 + rtP .
Bias5_Bias_meya4vfioi ; slat = rtB . otbeb0smep + rtP . Bias4_Bias_odezqxp5ct
; fphi = 1.0 / rtB . otbeb0smep * slat ; if ( ( o2mmi0bsgf < 0.0 ) && ( fphi
> muDoubleScalarFloor ( fphi ) ) ) { o2mmi0bsgf = - muDoubleScalarPower ( -
o2mmi0bsgf , fphi ) ; } else { o2mmi0bsgf = muDoubleScalarPower ( o2mmi0bsgf
, fphi ) ; } o2mmi0bsgf = ( o2mmi0bsgf + rtP . Bias6_Bias_jskr1b1mi4 ) / slat
* rtP . Constant1_Value_ej01j13uct ; if ( ssIsMajorTimeStep ( rtS ) ) { if (
rtDW . fg4worawde != 0 ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS )
; rtDW . fg4worawde = 0 ; } slat = muDoubleScalarSqrt ( o2mmi0bsgf ) ; } else
if ( o2mmi0bsgf < 0.0 ) { slat = - muDoubleScalarSqrt ( muDoubleScalarAbs (
o2mmi0bsgf ) ) ; rtDW . fg4worawde = 1 ; } else { slat = muDoubleScalarSqrt (
o2mmi0bsgf ) ; } o2mmi0bsgf = slat * rtB . iqcro5zqc5 ; if ( ! ( o2mmi0bsgf <
rtB . iqcro5zqc5 ) ) { if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { i = 1 ; do {
assessmentPtrVar = ( void * ) & assessmentVar ; if ( i <= rtP .
CompareToConstant1_const_gvieufb20s ) { assessmentVar = 0 ;
rt_SlioAccessorUpdate ( 1 , 3 , rtDW . ib0jqzpceq , ssGetT ( rtS ) ,
assessmentPtrVar ) ; } else { S = rtS ; diag = CreateDiagnosticAsVoidPtr (
"Simulink:blocks:AssertionAssert" , 2 , 5 ,
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/Supersonic/Supersonic/Assertion"
, 2 , ssGetT ( rtS ) ) ; rt_ssSet_slErrMsg ( S , diag ) ; ssSetStopRequested
( rtS , ( int ) ssGetT ( rtS ) ) ; assessmentVar = 1 ; rt_SlioAccessorUpdate
( 1 , 3 , rtDW . ib0jqzpceq , ssGetT ( rtS ) , assessmentPtrVar ) ; } slat =
rtDW . lwgbxvspmn ; fphi = rtB . otbeb0smep + rtP . Bias5_Bias_mcem25dje2 ;
cospsi = rtB . iqcro5zqc5 / rtDW . lwgbxvspmn ; temp1 = rtB . otbeb0smep +
rtP . Bias4_Bias_pl5eddn2p1 ; cospsi = fphi * fphi / ( rtB . otbeb0smep * rtP
. Constant1_Value_pb5yn2hc0w - cospsi * cospsi * rtP .
Constant3_Value_kvwuhyt22j * temp1 ) ; temp1 = 1.0 / temp1 ; if ( ( cospsi <
0.0 ) && ( temp1 > muDoubleScalarFloor ( temp1 ) ) ) { cospsi = -
muDoubleScalarPower ( - cospsi , temp1 ) ; } else { cospsi =
muDoubleScalarPower ( cospsi , temp1 ) ; } fphi = ( jb2eust2qa / rtB .
khr4lzxr01 + rtP . Bias6_Bias_hm1r0zlf2u ) * rtP . Constant4_Value_kox0vku1gy
/ fphi / cospsi ; if ( ssIsMajorTimeStep ( rtS ) ) { if ( rtDW . n4ytmke31n
!= 0 ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; rtDW .
n4ytmke31n = 0 ; } temp1 = muDoubleScalarSqrt ( fphi ) ; } else if ( fphi <
0.0 ) { temp1 = - muDoubleScalarSqrt ( muDoubleScalarAbs ( fphi ) ) ; rtDW .
n4ytmke31n = 1 ; } else { temp1 = muDoubleScalarSqrt ( fphi ) ; } rtB .
euq0m2wkrk = rtB . iqcro5zqc5 * temp1 ; rtDW . lwgbxvspmn = rtB . euq0m2wkrk
; i ++ ; } while ( muDoubleScalarAbs ( slat - rtB . euq0m2wkrk ) > rtP .
CompareToConstant_const_j4cxad0q0g ) ; } srUpdateBC ( rtDW . mxamlbyck0 ) ;
rtB . pcxo4cuead = rtB . euq0m2wkrk ; } else { rtB . pcxo4cuead = o2mmi0bsgf
; } } srUpdateBC ( rtDW . fc1bretpi5 ) ; break ; } } srUpdateBC ( rtDW .
i1eyrq4uwi ) ; } rtPrevAction = rtDW . hug50n4wps ; rtAction = ( int8_T ) ! (
rtP . Constant_Value_iw00e14laz == 6.0 ) ; rtDW . hug50n4wps = rtAction ; if
( rtPrevAction != rtAction ) { switch ( rtPrevAction ) { case 0 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; break ; case 1 :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; break ; } } if (
rtAction == 0 ) { if ( 0 != rtPrevAction ) { if ( ssGetTaskTime ( rtS , 1 )
!= ssGetTStart ( rtS ) ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS )
; } } srUpdateBC ( rtDW . ngahvntjaq ) ; } else { if ( 1 != rtPrevAction ) {
if ( ssGetTaskTime ( rtS , 1 ) != ssGetTStart ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } } srUpdateBC ( rtDW .
drdj3xeohi ) ; } } if ( ssIsMajorTimeStep ( rtS ) ) { srUpdateBC ( rtDW .
dd1fatoipk ) ; } break ; } } for ( oxjy2hf2cl = 0 ; oxjy2hf2cl < 3 ;
oxjy2hf2cl ++ ) { e4kgnowjgg [ oxjy2hf2cl ] = 3.280839895013123 * ( rtB .
ddawuzzdzz [ oxjy2hf2cl + 6 ] * gkpo2ijrbk_idx_2 + ( rtB . ddawuzzdzz [
oxjy2hf2cl + 3 ] * gkpo2ijrbk_idx_1 + rtB . ddawuzzdzz [ oxjy2hf2cl ] *
gkpo2ijrbk_idx_0 ) ) ; } muDoubleScalarSinCos ( muDoubleScalarAtan2 (
e4kgnowjgg [ 2 ] , e4kgnowjgg [ 0 ] ) , & jb2eust2qa , & o2mmi0bsgf ) ;
jjatrktc0c [ 0 ] = o2mmi0bsgf ; jjatrktc0c [ 1 ] = rtP .
Constant1_Value_cviw4ezr5s ; jjatrktc0c [ 2 ] = - jb2eust2qa ; jjatrktc0c [ 3
] = rtP . Constant_Value_aagndz5zvc ; jjatrktc0c [ 4 ] = rtP .
Constant2_Value_faxwqrunex ; jjatrktc0c [ 5 ] = rtP .
Constant4_Value_ctnx5cdtqt ; jjatrktc0c [ 6 ] = jb2eust2qa ; jjatrktc0c [ 7 ]
= rtP . Constant3_Value_hkckgaoniw ; jjatrktc0c [ 8 ] = o2mmi0bsgf ;
mn3ckvav5d_idx_0 = gkpo2ijrbk_idx_0 - ( ( daxlvr32qq [ 0 ] + avxeaykvgw [ 0 ]
) + mn3ckvav5d_idx_0 ) ; mn3ckvav5d_idx_1 = gkpo2ijrbk_idx_1 - ( ( daxlvr32qq
[ 1 ] + avxeaykvgw [ 1 ] ) + mn3ckvav5d_idx_1 ) ; o2mmi0bsgf =
gkpo2ijrbk_idx_2 - ( ( 0.3048 * avxeaykvgw [ 2 ] + daxlvr32qq [ 2 ] ) +
mn3ckvav5d_idx_2 ) ; fphi = ( ( mn3ckvav5d_idx_0 * mn3ckvav5d_idx_0 +
mn3ckvav5d_idx_1 * mn3ckvav5d_idx_1 ) + o2mmi0bsgf * o2mmi0bsgf ) * rtB .
nts3iqoscx * rtP . u2rhoV2_Gain ; lat = 0.020885434233150126 * fphi ;
mc31ssftz2_idx_1 = rtP . AerodynamicForcesandMoments_S * lat ; jb2eust2qa =
muDoubleScalarAtan2 ( abmm43etbq_idx_1 , abmm43etbq_idx_0 ) ; i =
plook_s32dd_binxp ( 57.295779513082323 * jb2eust2qa , rtP .
alpha_BreakpointsData , 9U , & temp1 , & rtDW . bu3swunyz2 ) ; o2mmi0bsgf = (
mn3ckvav5d_idx_0 * mn3ckvav5d_idx_0 + mn3ckvav5d_idx_1 * mn3ckvav5d_idx_1 ) +
o2mmi0bsgf * o2mmi0bsgf ; if ( ssIsMajorTimeStep ( rtS ) ) { if ( rtDW .
fk0au12fsx != 0 ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; rtDW
. fk0au12fsx = 0 ; } slat = muDoubleScalarSqrt ( o2mmi0bsgf ) ; } else if (
o2mmi0bsgf < 0.0 ) { slat = - muDoubleScalarSqrt ( muDoubleScalarAbs (
o2mmi0bsgf ) ) ; rtDW . fk0au12fsx = 1 ; } else { slat = muDoubleScalarSqrt (
o2mmi0bsgf ) ; } slat /= rtB . ivvuf1kwbw ; dsrahxycol = plook_s32dd_binxp (
slat , rtP . Mach_BreakpointsData , 3U , & cospsi , & rtDW . eyrf2vyldh ) ;
kkmrxg5n4s = plook_s32dd_binxp ( 3.280839895013123 * rtB . d2gfe2p42l , rtP .
altitude_BreakpointsData , 7U , & q , & rtDW . nzxqnu5jmv ) ; frac_kt [ 0 ] =
temp1 ; frac_kt [ 1 ] = cospsi ; frac_kt [ 2 ] = q ; bpIndex_kt [ 0 ] = i ;
bpIndex_kt [ 1 ] = dsrahxycol ; bpIndex_kt [ 2 ] = kkmrxg5n4s ; lon =
intrp3d_s32dl_pw ( bpIndex_kt , frac_kt , rtP . CD_Table , rtP . CD_dimSize )
; frac_f2 [ 0 ] = temp1 ; frac_f2 [ 1 ] = cospsi ; frac_f2 [ 2 ] = q ;
bpIndex_f2 [ 0 ] = i ; bpIndex_f2 [ 1 ] = dsrahxycol ; bpIndex_f2 [ 2 ] =
kkmrxg5n4s ; slam = intrp3d_s32dl_pw ( bpIndex_f2 , frac_f2 , rtP . CYb_Table
, rtP . CYb_dimSize ) ; o2mmi0bsgf = ( abmm43etbq_idx_0 * abmm43etbq_idx_0 +
rtB . ohifllm5xd ) + abmm43etbq_idx_1 * abmm43etbq_idx_1 ; if (
ssIsMajorTimeStep ( rtS ) ) { if ( rtDW . pa1f5cjxua != 0 ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; rtDW . pa1f5cjxua = 0 ;
} alt = muDoubleScalarSqrt ( o2mmi0bsgf ) ; } else if ( o2mmi0bsgf < 0.0 ) {
alt = - muDoubleScalarSqrt ( muDoubleScalarAbs ( o2mmi0bsgf ) ) ; rtDW .
pa1f5cjxua = 1 ; } else { alt = muDoubleScalarSqrt ( o2mmi0bsgf ) ; }
o2mmi0bsgf = rtP . Constant12_Value / alt ; if ( o2mmi0bsgf > 1.0 ) {
o2mmi0bsgf = 1.0 ; } else { if ( o2mmi0bsgf < - 1.0 ) { o2mmi0bsgf = - 1.0 ;
} } o2mmi0bsgf = muDoubleScalarAsin ( o2mmi0bsgf ) ; alt = 57.295779513082323
* o2mmi0bsgf ; frac_ck [ 0 ] = temp1 ; frac_ck [ 1 ] = cospsi ; frac_ck [ 2 ]
= q ; bpIndex_ck [ 0 ] = i ; bpIndex_ck [ 1 ] = dsrahxycol ; bpIndex_ck [ 2 ]
= kkmrxg5n4s ; sinphi = intrp3d_s32dl_pw ( bpIndex_ck , frac_ck , rtP .
CL_Table , rtP . CL_dimSize ) ; mn3ckvav5d_idx_0 = rtP . coefAdjust_Gain [ 0
] * lon * mc31ssftz2_idx_1 ; mn3ckvav5d_idx_1 = slam * alt * rtP .
coefAdjust_Gain [ 1 ] * mc31ssftz2_idx_1 ; mn3ckvav5d_idx_2 = rtP .
coefAdjust_Gain [ 2 ] * sinphi * mc31ssftz2_idx_1 ; muDoubleScalarSinCos (
muDoubleScalarAtan2 ( e4kgnowjgg [ 2 ] , e4kgnowjgg [ 0 ] ) , & lon , & slam
) ; kh2xrb5jub [ 0 ] = slam ; kh2xrb5jub [ 1 ] = rtP .
Constant1_Value_e0wp0jqtlj ; kh2xrb5jub [ 2 ] = - lon ; kh2xrb5jub [ 3 ] =
rtP . Constant_Value_dk11ift0i1 ; kh2xrb5jub [ 4 ] = rtP .
Constant2_Value_b5ilyky2la ; kh2xrb5jub [ 5 ] = rtP .
Constant4_Value_bqgwz3qflq ; kh2xrb5jub [ 6 ] = lon ; kh2xrb5jub [ 7 ] = rtP
. Constant3_Value_d4mh0cif3h ; kh2xrb5jub [ 8 ] = slam ; for ( oxjy2hf2cl = 0
; oxjy2hf2cl < 3 ; oxjy2hf2cl ++ ) { frac_kt [ oxjy2hf2cl ] = kh2xrb5jub [
oxjy2hf2cl + 6 ] * rtB . mzb2t2xbjn [ 2 ] + ( kh2xrb5jub [ oxjy2hf2cl + 3 ] *
rtB . mzb2t2xbjn [ 1 ] + kh2xrb5jub [ oxjy2hf2cl ] * rtB . mzb2t2xbjn [ 0 ] )
; } frac_fs [ 0 ] = temp1 ; frac_fs [ 1 ] = cospsi ; frac_fs [ 2 ] = q ;
bpIndex_fs [ 0 ] = i ; bpIndex_fs [ 1 ] = dsrahxycol ; bpIndex_fs [ 2 ] =
kkmrxg5n4s ; lon = intrp3d_s32dl_pw ( bpIndex_fs , frac_fs , rtP . Clb_Table
, rtP . Clb_dimSize ) ; lon *= alt ; frac_dz [ 0 ] = temp1 ; frac_dz [ 1 ] =
cospsi ; frac_dz [ 2 ] = q ; bpIndex_dz [ 0 ] = i ; bpIndex_dz [ 1 ] =
dsrahxycol ; bpIndex_dz [ 2 ] = kkmrxg5n4s ; slam = intrp3d_s32dl_pw (
bpIndex_dz , frac_dz , rtP . Cm_Table , rtP . Cm_dimSize ) ; frac_o [ 0 ] =
temp1 ; frac_o [ 1 ] = cospsi ; frac_o [ 2 ] = q ; bpIndex_o [ 0 ] = i ;
bpIndex_o [ 1 ] = dsrahxycol ; bpIndex_o [ 2 ] = kkmrxg5n4s ; sinphi =
intrp3d_s32dl_pw ( bpIndex_o , frac_o , rtP . Cnb_Table , rtP . Cnb_dimSize )
; alt *= sinphi ; sinphi = ( mn3ckvav5d_idx_1 * frac_kt [ 2 ] -
mn3ckvav5d_idx_2 * frac_kt [ 1 ] ) + rtP .
AerodynamicForcesandMoments_b_blpao4glfe * mc31ssftz2_idx_1 * lon ; lon = (
mn3ckvav5d_idx_2 * frac_kt [ 0 ] - mn3ckvav5d_idx_0 * frac_kt [ 2 ] ) + rtP .
AerodynamicForcesandMoments_cbar_ect3p4btxl * mc31ssftz2_idx_1 * slam ;
mc31ssftz2_idx_1 = ( mn3ckvav5d_idx_0 * frac_kt [ 1 ] - mn3ckvav5d_idx_1 *
frac_kt [ 0 ] ) + rtP . AerodynamicForcesandMoments_b_blpao4glfe *
mc31ssftz2_idx_1 * alt ; for ( oxjy2hf2cl = 0 ; oxjy2hf2cl < 3 ; oxjy2hf2cl
++ ) { frac_fs [ oxjy2hf2cl ] = jjatrktc0c [ 3 * oxjy2hf2cl + 2 ] *
mc31ssftz2_idx_1 + ( jjatrktc0c [ 3 * oxjy2hf2cl + 1 ] * lon + jjatrktc0c [ 3
* oxjy2hf2cl ] * sinphi ) ; } muDoubleScalarSinCos ( muDoubleScalarAtan2 (
e4kgnowjgg [ 2 ] , e4kgnowjgg [ 0 ] ) , & mc31ssftz2_idx_1 , & alt ) ;
jjatrktc0c [ 0 ] = alt ; jjatrktc0c [ 1 ] = rtP . Constant1_Value_dttko5tnn2
; jjatrktc0c [ 2 ] = - mc31ssftz2_idx_1 ; jjatrktc0c [ 3 ] = rtP .
Constant_Value_jkivyeh0ov ; jjatrktc0c [ 4 ] = rtP .
Constant2_Value_d2teymf5hj ; jjatrktc0c [ 5 ] = rtP .
Constant4_Value_iqinklay2r ; jjatrktc0c [ 6 ] = mc31ssftz2_idx_1 ; jjatrktc0c
[ 7 ] = rtP . Constant3_Value_goletwl4mi ; jjatrktc0c [ 8 ] = alt ; lat *=
rtP . AerodynamicForcesandMoments_S_miuqcjxiwz ; frac_d [ 0 ] = temp1 ;
frac_d [ 1 ] = cospsi ; frac_d [ 2 ] = q ; bpIndex_d [ 0 ] = i ; bpIndex_d [
1 ] = dsrahxycol ; bpIndex_d [ 2 ] = kkmrxg5n4s ; mc31ssftz2_idx_1 =
intrp3d_s32dl_pw ( bpIndex_d , frac_d , rtP . CYp_Table , rtP . CYp_dimSize )
; lon = muDoubleScalarAtan2 ( e4kgnowjgg [ 2 ] , e4kgnowjgg [ 0 ] ) ;
muDoubleScalarSinCos ( lon , & fzwvbhycmn_idx_0 , & bkk2o2zacc_idx_0 ) ;
muDoubleScalarSinCos ( rtP . u_Value_a3c51jcqno , & fzwvbhycmn_idx_1 , &
bkk2o2zacc_idx_1 ) ; kh2xrb5jub [ 0 ] = bkk2o2zacc_idx_0 * bkk2o2zacc_idx_1 ;
kh2xrb5jub [ 1 ] = - ( fzwvbhycmn_idx_1 * bkk2o2zacc_idx_0 ) ; kh2xrb5jub [ 2
] = - fzwvbhycmn_idx_0 ; kh2xrb5jub [ 3 ] = fzwvbhycmn_idx_1 ; kh2xrb5jub [ 4
] = bkk2o2zacc_idx_1 ; kh2xrb5jub [ 5 ] = rtP . Constant_Value_nm4nmql4xz ;
kh2xrb5jub [ 6 ] = fzwvbhycmn_idx_0 * bkk2o2zacc_idx_1 ; kh2xrb5jub [ 7 ] = -
( fzwvbhycmn_idx_0 * fzwvbhycmn_idx_1 ) ; kh2xrb5jub [ 8 ] = bkk2o2zacc_idx_0
; muDoubleScalarSinCos ( lon , & slam , & alt ) ; if ( ssIsSampleHit ( rtS ,
1 , 0 ) ) { rtB . onh5wf0mbq = 57.295779513082323 * rtDW . hgabqtd4hg ; rtB .
a4jldatvmm = - ( 0.017453292519943295 * rtB . onh5wf0mbq ) ; } alt *= rtP .
u_Value_pyce5cxhso ; rtB . gectl3vjgg = rtX . c3n310npj5 ; frac_kt [ 0 ] =
57.295779513082323 * rtP . Constant2_Value_dp3hb0zdw4 * 0.017453292519943295
+ - ( rtP . u_Value_pyce5cxhso * slam ) ; frac_kt [ 1 ] = 57.295779513082323
* rtB . gectl3vjgg * 0.017453292519943295 + rtB . a4jldatvmm ; frac_kt [ 2 ]
= 57.295779513082323 * rtP . Constant3_Value_dde35bhip2 *
0.017453292519943295 + alt ; for ( oxjy2hf2cl = 0 ; oxjy2hf2cl < 3 ;
oxjy2hf2cl ++ ) { daxlvr32qq [ oxjy2hf2cl ] = 57.295779513082323 * (
kh2xrb5jub [ oxjy2hf2cl + 6 ] * frac_kt [ 2 ] + ( kh2xrb5jub [ oxjy2hf2cl + 3
] * frac_kt [ 1 ] + kh2xrb5jub [ oxjy2hf2cl ] * frac_kt [ 0 ] ) ) ; } frac_l
[ 0 ] = temp1 ; frac_l [ 1 ] = cospsi ; frac_l [ 2 ] = q ; bpIndex_l [ 0 ] =
i ; bpIndex_l [ 1 ] = dsrahxycol ; bpIndex_l [ 2 ] = kkmrxg5n4s ; alt =
intrp3d_s32dl_pw ( bpIndex_l , frac_l , rtP . CLad_Table , rtP . CLad_dimSize
) ; frac_n [ 0 ] = temp1 ; frac_n [ 1 ] = cospsi ; frac_n [ 2 ] = q ;
bpIndex_n [ 0 ] = i ; bpIndex_n [ 1 ] = dsrahxycol ; bpIndex_n [ 2 ] =
kkmrxg5n4s ; lon = intrp3d_s32dl_pw ( bpIndex_n , frac_n , rtP . CLq_Table ,
rtP . CLq_dimSize ) ; frac_b [ 0 ] = temp1 ; frac_b [ 1 ] = cospsi ; frac_b [
2 ] = q ; bpIndex_b [ 0 ] = i ; bpIndex_b [ 1 ] = dsrahxycol ; bpIndex_b [ 2
] = kkmrxg5n4s ; slam = intrp3d_s32dl_pw ( bpIndex_b , frac_b , rtP .
Clp_Table , rtP . Clp_dimSize ) ; frac_k [ 0 ] = temp1 ; frac_k [ 1 ] =
cospsi ; frac_k [ 2 ] = q ; bpIndex_k [ 0 ] = i ; bpIndex_k [ 1 ] =
dsrahxycol ; bpIndex_k [ 2 ] = kkmrxg5n4s ; sinphi = intrp3d_s32dl_pw (
bpIndex_k , frac_k , rtP . Clr_Table , rtP . Clr_dimSize ) ; fzwvbhycmn_idx_0
= slam * daxlvr32qq [ 0 ] + sinphi * daxlvr32qq [ 2 ] ; frac_c [ 0 ] = temp1
; frac_c [ 1 ] = cospsi ; frac_c [ 2 ] = q ; bpIndex_c [ 0 ] = i ; bpIndex_c
[ 1 ] = dsrahxycol ; bpIndex_c [ 2 ] = kkmrxg5n4s ; slam = intrp3d_s32dl_pw (
bpIndex_c , frac_c , rtP . Cmq_Table , rtP . Cmq_dimSize ) ; frac_f [ 0 ] =
temp1 ; frac_f [ 1 ] = cospsi ; frac_f [ 2 ] = q ; bpIndex_f [ 0 ] = i ;
bpIndex_f [ 1 ] = dsrahxycol ; bpIndex_f [ 2 ] = kkmrxg5n4s ; sinphi =
intrp3d_s32dl_pw ( bpIndex_f , frac_f , rtP . Cmad_Table , rtP . Cmad_dimSize
) ; frac_j [ 0 ] = temp1 ; frac_j [ 1 ] = cospsi ; frac_j [ 2 ] = q ;
bpIndex_j [ 0 ] = i ; bpIndex_j [ 1 ] = dsrahxycol ; bpIndex_j [ 2 ] =
kkmrxg5n4s ; bkk2o2zacc_idx_0 = intrp3d_s32dl_pw ( bpIndex_j , frac_j , rtP .
Cnp_Table , rtP . Cnp_dimSize ) ; frac_g [ 0 ] = temp1 ; frac_g [ 1 ] =
cospsi ; frac_g [ 2 ] = q ; bpIndex_g [ 0 ] = i ; bpIndex_g [ 1 ] =
dsrahxycol ; bpIndex_g [ 2 ] = kkmrxg5n4s ; temp1 = intrp3d_s32dl_pw (
bpIndex_g , frac_g , rtP . Cnr_Table , rtP . Cnr_dimSize ) ; gzwrrirgjp [ 0 ]
= rtP . Gain1_Gain [ 0 ] * rtP . zero_Value ; gzwrrirgjp [ 1 ] =
mc31ssftz2_idx_1 * daxlvr32qq [ 0 ] * rtP . Gain1_Gain [ 1 ] ; gzwrrirgjp [ 2
] = ( alt * rtB . onh5wf0mbq + lon * daxlvr32qq [ 1 ] ) * rtP . Gain1_Gain [
2 ] ; gzwrrirgjp [ 3 ] = rtP . Gain1_Gain [ 3 ] * fzwvbhycmn_idx_0 ;
gzwrrirgjp [ 4 ] = ( slam * daxlvr32qq [ 1 ] + sinphi * rtB . onh5wf0mbq ) *
rtP . Gain1_Gain [ 4 ] ; gzwrrirgjp [ 5 ] = ( bkk2o2zacc_idx_0 * daxlvr32qq [
0 ] + temp1 * daxlvr32qq [ 2 ] ) * rtP . Gain1_Gain [ 5 ] ; fzwvbhycmn_idx_0
= ( e4kgnowjgg [ 0 ] * e4kgnowjgg [ 0 ] + e4kgnowjgg [ 1 ] * e4kgnowjgg [ 1 ]
) + e4kgnowjgg [ 2 ] * e4kgnowjgg [ 2 ] ; if ( ssIsMajorTimeStep ( rtS ) ) {
if ( rtDW . mewl43zgov != 0 ) { ssSetBlockStateForSolverChangedAtMajorStep (
rtS ) ; rtDW . mewl43zgov = 0 ; } bkk2o2zacc_idx_0 = muDoubleScalarSqrt (
fzwvbhycmn_idx_0 ) ; } else if ( fzwvbhycmn_idx_0 < 0.0 ) { bkk2o2zacc_idx_0
= - muDoubleScalarSqrt ( muDoubleScalarAbs ( fzwvbhycmn_idx_0 ) ) ; rtDW .
mewl43zgov = 1 ; } else { bkk2o2zacc_idx_0 = muDoubleScalarSqrt (
fzwvbhycmn_idx_0 ) ; } for ( oxjy2hf2cl = 0 ; oxjy2hf2cl < 6 ; oxjy2hf2cl ++
) { gzwrrirgjp [ oxjy2hf2cl ] = gzwrrirgjp [ oxjy2hf2cl ] / rtP .
u_Value_nkmodb5shf / bkk2o2zacc_idx_0 ; } daxlvr32qq [ 0 ] = rtP .
coefAdjust_Gain_m5bwkewsju [ 0 ] * gzwrrirgjp [ 0 ] * lat ; daxlvr32qq [ 1 ]
= rtP . coefAdjust_Gain_m5bwkewsju [ 1 ] * gzwrrirgjp [ 1 ] * lat ;
daxlvr32qq [ 2 ] = rtP . coefAdjust_Gain_m5bwkewsju [ 2 ] * gzwrrirgjp [ 2 ]
* lat ; muDoubleScalarSinCos ( muDoubleScalarAtan2 ( e4kgnowjgg [ 2 ] ,
e4kgnowjgg [ 0 ] ) , & fzwvbhycmn_idx_0 , & bkk2o2zacc_idx_0 ) ; kh2xrb5jub [
0 ] = bkk2o2zacc_idx_0 ; kh2xrb5jub [ 1 ] = rtP . Constant1_Value_idys3ziitq
; kh2xrb5jub [ 2 ] = - fzwvbhycmn_idx_0 ; kh2xrb5jub [ 3 ] = rtP .
Constant_Value_lr2cknagpa ; kh2xrb5jub [ 4 ] = rtP .
Constant2_Value_axatlcv2rs ; kh2xrb5jub [ 5 ] = rtP .
Constant4_Value_aprubsstdi ; kh2xrb5jub [ 6 ] = fzwvbhycmn_idx_0 ; kh2xrb5jub
[ 7 ] = rtP . Constant3_Value_ockepn4d1g ; kh2xrb5jub [ 8 ] =
bkk2o2zacc_idx_0 ; for ( oxjy2hf2cl = 0 ; oxjy2hf2cl < 3 ; oxjy2hf2cl ++ ) {
frac_kt [ oxjy2hf2cl ] = kh2xrb5jub [ oxjy2hf2cl + 6 ] * rtB . ot3vk2b455 [ 2
] + ( kh2xrb5jub [ oxjy2hf2cl + 3 ] * rtB . ot3vk2b455 [ 1 ] + kh2xrb5jub [
oxjy2hf2cl ] * rtB . ot3vk2b455 [ 0 ] ) ; } mc31ssftz2_idx_1 = ( daxlvr32qq [
1 ] * frac_kt [ 2 ] - daxlvr32qq [ 2 ] * frac_kt [ 1 ] ) + rtP .
AerodynamicForcesandMoments_b * lat * gzwrrirgjp [ 3 ] ; slam = ( daxlvr32qq
[ 2 ] * frac_kt [ 0 ] - daxlvr32qq [ 0 ] * frac_kt [ 2 ] ) + rtP .
AerodynamicForcesandMoments_cbar * lat * gzwrrirgjp [ 4 ] ; fzwvbhycmn_idx_0
= ( daxlvr32qq [ 0 ] * frac_kt [ 1 ] - daxlvr32qq [ 1 ] * frac_kt [ 0 ] ) +
rtP . AerodynamicForcesandMoments_b * lat * gzwrrirgjp [ 5 ] ; for (
oxjy2hf2cl = 0 ; oxjy2hf2cl < 3 ; oxjy2hf2cl ++ ) { frac_fs [ oxjy2hf2cl ] =
( frac_fs [ oxjy2hf2cl ] + ( jjatrktc0c [ 3 * oxjy2hf2cl + 2 ] *
fzwvbhycmn_idx_0 + ( jjatrktc0c [ 3 * oxjy2hf2cl + 1 ] * slam + jjatrktc0c [
3 * oxjy2hf2cl ] * mc31ssftz2_idx_1 ) ) ) * 1.3558179483314003 ; }
fzwvbhycmn_idx_0 = rtP . Sref * fphi ; i = plook_s32dd_binxp (
57.295779513082323 * jb2eust2qa , rtP . alpha_BreakpointsData_gcueg2tgd0 , 9U
, & bkk2o2zacc_idx_0 , & rtDW . osz5yvmvo1 ) ; dsrahxycol = plook_s32dd_binxp
( slat , rtP . Mach_BreakpointsData_jg33uk3jci , 3U , & fphi , & rtDW .
otuccisgv4 ) ; kkmrxg5n4s = plook_s32dd_binxp ( 3.280839895013123 * rtB .
d2gfe2p42l , rtP . altitude_BreakpointsData_p0oyeupgfw , 7U , & slat , & rtDW
. kz3kayz3v2 ) ; if ( ssIsMajorTimeStep ( rtS ) ) { switch ( rtDW .
li4wvvnvo0 ) { case 0 : if ( rtX . m2cgemps3b [ 0 ] <= rtP . mindef_elevator
) { rtX . m2cgemps3b [ 0 ] = rtP . mindef_elevator ; if ( rtX . m2cgemps3b [
1 ] > 0.0 ) { rtDW . li4wvvnvo0 = 0 ; } else { rtX . m2cgemps3b [ 1 ] = 0.0 ;
rtDW . li4wvvnvo0 = 1 ; } ssSetBlockStateForSolverChangedAtMajorStep ( rtS )
; } if ( rtX . m2cgemps3b [ 0 ] >= rtP . maxdef_elevator ) { rtX . m2cgemps3b
[ 0 ] = rtP . maxdef_elevator ; if ( rtX . m2cgemps3b [ 1 ] < 0.0 ) { rtDW .
li4wvvnvo0 = 0 ; } else { rtX . m2cgemps3b [ 1 ] = 0.0 ; rtDW . li4wvvnvo0 =
2 ; } ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } if ( rtX .
m2cgemps3b [ 1 ] <= - rtP . NonlinearSecondOrderActuator1_fin_maxrate ) { rtX
. m2cgemps3b [ 1 ] = - rtP . NonlinearSecondOrderActuator1_fin_maxrate ; rtDW
. li4wvvnvo0 = 3 ; ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } if
( rtX . m2cgemps3b [ 1 ] >= rtP . NonlinearSecondOrderActuator1_fin_maxrate )
{ rtX . m2cgemps3b [ 1 ] = rtP . NonlinearSecondOrderActuator1_fin_maxrate ;
rtDW . li4wvvnvo0 = 4 ; ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ;
} break ; case 1 : if ( rtX . m2cgemps3b [ 0 ] > rtP . mindef_elevator ) { if
( rtX . m2cgemps3b [ 0 ] >= rtP . maxdef_elevator ) { rtDW . li4wvvnvo0 = 2 ;
rtX . m2cgemps3b [ 0 ] = rtP . maxdef_elevator ; } else { rtDW . li4wvvnvo0 =
0 ; } } else { rtX . m2cgemps3b [ 0 ] = rtP . mindef_elevator ; rtX .
m2cgemps3b [ 1 ] = 0.0 ; } break ; case 2 : if ( rtX . m2cgemps3b [ 0 ] < rtP
. maxdef_elevator ) { if ( rtX . m2cgemps3b [ 0 ] <= rtP . mindef_elevator )
{ rtDW . li4wvvnvo0 = 1 ; rtX . m2cgemps3b [ 0 ] = rtP . mindef_elevator ; }
else { rtDW . li4wvvnvo0 = 0 ; } } else { rtX . m2cgemps3b [ 0 ] = rtP .
maxdef_elevator ; rtX . m2cgemps3b [ 1 ] = 0.0 ; } break ; case 3 : if ( rtX
. m2cgemps3b [ 1 ] > - rtP . NonlinearSecondOrderActuator1_fin_maxrate ) { if
( rtX . m2cgemps3b [ 1 ] >= rtP . NonlinearSecondOrderActuator1_fin_maxrate )
{ rtDW . li4wvvnvo0 = 4 ; rtX . m2cgemps3b [ 1 ] = rtP .
NonlinearSecondOrderActuator1_fin_maxrate ; } else { rtDW . li4wvvnvo0 = 0 ;
} } else { rtX . m2cgemps3b [ 1 ] = - rtP .
NonlinearSecondOrderActuator1_fin_maxrate ; } if ( rtX . m2cgemps3b [ 0 ] <=
rtP . mindef_elevator ) { rtX . m2cgemps3b [ 0 ] = rtP . mindef_elevator ;
rtX . m2cgemps3b [ 1 ] = 0.0 ; rtDW . li4wvvnvo0 = 1 ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } break ; case 4 : if (
rtX . m2cgemps3b [ 1 ] < rtP . NonlinearSecondOrderActuator1_fin_maxrate ) {
if ( rtX . m2cgemps3b [ 1 ] <= - rtP .
NonlinearSecondOrderActuator1_fin_maxrate ) { rtDW . li4wvvnvo0 = 3 ; rtX .
m2cgemps3b [ 1 ] = - rtP . NonlinearSecondOrderActuator1_fin_maxrate ; } else
{ rtDW . li4wvvnvo0 = 0 ; } } else { rtX . m2cgemps3b [ 1 ] = rtP .
NonlinearSecondOrderActuator1_fin_maxrate ; } if ( rtX . m2cgemps3b [ 0 ] >=
rtP . maxdef_elevator ) { rtX . m2cgemps3b [ 0 ] = rtP . maxdef_elevator ;
rtX . m2cgemps3b [ 1 ] = 0.0 ; rtDW . li4wvvnvo0 = 2 ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } break ; } }
mc31ssftz2_idx_1 = rtX . m2cgemps3b [ 0 ] ; slam = rtX . m2cgemps3b [ 1 ] ;
oxjy2hf2cl = plook_s32dd_binxp ( rtX . m2cgemps3b [ 0 ] , rtP .
delta_BreakpointsData , 4U , & q , & rtDW . kjeb1ibj4p ) ; frac_m [ 0 ] =
bkk2o2zacc_idx_0 ; frac_m [ 1 ] = fphi ; frac_m [ 2 ] = slat ; frac_m [ 3 ] =
q ; bpIndex_m [ 0 ] = i ; bpIndex_m [ 1 ] = dsrahxycol ; bpIndex_m [ 2 ] =
kkmrxg5n4s ; bpIndex_m [ 3 ] = oxjy2hf2cl ; temp1 = intrp4d_s32dl_pw (
bpIndex_m , frac_m , rtP . DCDI_Table , rtP . DCDI_dimSize ) ; frac_i [ 0 ] =
q ; frac_i [ 1 ] = fphi ; frac_i [ 2 ] = slat ; bpIndex_i [ 0 ] = oxjy2hf2cl
; bpIndex_i [ 1 ] = dsrahxycol ; bpIndex_i [ 2 ] = kkmrxg5n4s ; cospsi =
intrp3d_s32dl_pw ( bpIndex_i , frac_i , rtP . DCL_Table , rtP . DCL_dimSize )
; frac_e [ 0 ] = q ; frac_e [ 1 ] = fphi ; frac_e [ 2 ] = slat ; bpIndex_e [
0 ] = oxjy2hf2cl ; bpIndex_e [ 1 ] = dsrahxycol ; bpIndex_e [ 2 ] =
kkmrxg5n4s ; q = intrp3d_s32dl_pw ( bpIndex_e , frac_e , rtP . DCm_Table ,
rtP . DCm_dimSize ) ; frac_d [ 0 ] = ( ( rtP . Constant1_Value_n5qw4ehq0c +
temp1 ) + rtP . Constant1_Value_nyr0ia51bl ) * rtP .
coefAdjust_Gain_oacgeai0xk [ 0 ] * fzwvbhycmn_idx_0 ; frac_d [ 2 ] = ( ( rtP
. Constant1_Value_n5qw4ehq0c + cospsi ) + rtP . Constant1_Value_nyr0ia51bl )
* rtP . coefAdjust_Gain_oacgeai0xk [ 2 ] * fzwvbhycmn_idx_0 ; frac_p [ 0 ] =
bkk2o2zacc_idx_0 ; frac_p [ 1 ] = fphi ; frac_p [ 2 ] = slat ; bpIndex_p [ 0
] = i ; bpIndex_p [ 1 ] = dsrahxycol ; bpIndex_p [ 2 ] = kkmrxg5n4s ;
bkk2o2zacc_idx_0 = intrp3d_s32dl_pw ( bpIndex_p , frac_p , rtP . Xcp_Table ,
rtP . Xcp_dimSize ) ; bkk2o2zacc_idx_0 *= rtP . cbar ; if ( ssIsSampleHit (
rtS , 1 , 0 ) ) { rtB . nrzu1kxyjs [ 0 ] = rtP . Thrust_LMN_Value [ 0 ] ; rtB
. nrzu1kxyjs [ 1 ] = rtP . Thrust_LMN_Value [ 1 ] ; rtB . nrzu1kxyjs [ 2 ] =
rtP . Thrust_LMN_Value [ 2 ] ; } rtB . bf5sj4v0nw = ( ( ( ( ( ( 0.0 -
bkk2o2zacc_idx_0 ) * frac_d [ 2 ] - ( 0.0 - rtP . zero1_Value ) * frac_d [ 0
] ) + ( ( rtP . Constant1_Value_n5qw4ehq0c + q ) + rtP .
Constant1_Value_nyr0ia51bl ) * ( rtP . cbar * fzwvbhycmn_idx_0 ) ) + frac_fs
[ 1 ] ) + rtB . nrzu1kxyjs [ 1 ] ) - rtP . Constant2_Value_gr5f00mfnm * rtB .
gectl3vjgg ) / rtP . uDOFBodyAxes_Iyy ; muDoubleScalarSinCos ( rtB .
fdu3hdd1wq , & fzwvbhycmn_idx_0 , & bkk2o2zacc_idx_0 ) ; muDoubleScalarSinCos
( muDoubleScalarAtan2 ( e4kgnowjgg [ 2 ] , e4kgnowjgg [ 0 ] ) , & slat , &
fphi ) ; jjatrktc0c [ 0 ] = fphi ; jjatrktc0c [ 1 ] = rtP .
Constant1_Value_fvdtk4fzi5 ; jjatrktc0c [ 2 ] = - slat ; jjatrktc0c [ 3 ] =
rtP . Constant_Value_pzjjbfwyj5 ; jjatrktc0c [ 4 ] = rtP .
Constant2_Value_ms43idsvzx ; jjatrktc0c [ 5 ] = rtP .
Constant4_Value_dv4oyrtdrl ; jjatrktc0c [ 6 ] = slat ; jjatrktc0c [ 7 ] = rtP
. Constant3_Value_mgyvggblh0 ; jjatrktc0c [ 8 ] = fphi ; muDoubleScalarSinCos
( muDoubleScalarAtan2 ( e4kgnowjgg [ 2 ] , e4kgnowjgg [ 0 ] ) , & slat , &
fphi ) ; kh2xrb5jub [ 0 ] = fphi ; kh2xrb5jub [ 1 ] = rtP .
Constant1_Value_ishhczx4h3 ; kh2xrb5jub [ 2 ] = - slat ; kh2xrb5jub [ 3 ] =
rtP . Constant_Value_mbry10wmmx ; kh2xrb5jub [ 4 ] = rtP .
Constant2_Value_bed4d3ljnh ; kh2xrb5jub [ 5 ] = rtP .
Constant4_Value_env11uw4lz ; kh2xrb5jub [ 6 ] = slat ; kh2xrb5jub [ 7 ] = rtP
. Constant3_Value_l1ogwvmgck ; kh2xrb5jub [ 8 ] = fphi ; slat =
look1_plinlcpw ( rtP . throttle_Value , rtP . ThrustX_bp01Data , rtP .
ThrustX_tableData , & rtDW . mjfthrgmwy , 6U ) ; if ( ssIsSampleHit ( rtS , 1
, 0 ) ) { rtB . nw4xrcz5qc [ 0 ] = rtP . Thrust_YZ_Value [ 0 ] ; rtB .
nw4xrcz5qc [ 1 ] = rtP . Thrust_YZ_Value [ 1 ] ; } for ( oxjy2hf2cl = 0 ;
oxjy2hf2cl < 3 ; oxjy2hf2cl ++ ) { e4kgnowjgg [ oxjy2hf2cl ] = jjatrktc0c [ 3
* oxjy2hf2cl + 2 ] * mn3ckvav5d_idx_2 + ( jjatrktc0c [ 3 * oxjy2hf2cl + 1 ] *
mn3ckvav5d_idx_1 + jjatrktc0c [ 3 * oxjy2hf2cl ] * mn3ckvav5d_idx_0 ) ;
avxeaykvgw [ oxjy2hf2cl ] = kh2xrb5jub [ 3 * oxjy2hf2cl + 2 ] * daxlvr32qq [
2 ] + ( kh2xrb5jub [ 3 * oxjy2hf2cl + 1 ] * daxlvr32qq [ 1 ] + kh2xrb5jub [ 3
* oxjy2hf2cl ] * daxlvr32qq [ 0 ] ) ; } rtB . gtmxqvxrzk [ 0 ] = ( ( ( ( (
e4kgnowjgg [ 0 ] + avxeaykvgw [ 0 ] ) * 4.4482216152605005 + frac_d [ 0 ] ) +
or3lxhab5k [ 0 ] ) + slat ) / rtP . mass + - fzwvbhycmn_idx_0 * rtP .
uDOFBodyAxes_g ) + ( rtP . MatrixGain_Gain [ 0 ] * abmm43etbq_idx_0 + rtP .
MatrixGain_Gain [ 2 ] * abmm43etbq_idx_1 ) * rtB . gectl3vjgg ; rtB .
gtmxqvxrzk [ 1 ] = ( ( ( ( ( e4kgnowjgg [ 2 ] + avxeaykvgw [ 2 ] ) *
4.4482216152605005 + frac_d [ 2 ] ) + or3lxhab5k [ 2 ] ) + rtB . nw4xrcz5qc [
1 ] ) / rtP . mass + bkk2o2zacc_idx_0 * rtP . uDOFBodyAxes_g ) + ( rtP .
MatrixGain_Gain [ 1 ] * abmm43etbq_idx_0 + rtP . MatrixGain_Gain [ 3 ] *
abmm43etbq_idx_1 ) * rtB . gectl3vjgg ; muDoubleScalarSinCos ( rtP .
Constant_Value_n35gokrdfy , & avxeaykvgw [ 0 ] , & frac_d [ 0 ] ) ;
muDoubleScalarSinCos ( rtB . fdu3hdd1wq , & avxeaykvgw [ 1 ] , & frac_d [ 1 ]
) ; muDoubleScalarSinCos ( rtP . Constant_Value_n35gokrdfy , &
mn3ckvav5d_idx_0 , & frac_d [ 2 ] ) ; jjatrktc0c [ 0 ] = frac_d [ 1 ] *
frac_d [ 0 ] ; jjatrktc0c [ 1 ] = mn3ckvav5d_idx_0 * avxeaykvgw [ 1 ] *
frac_d [ 0 ] - frac_d [ 2 ] * avxeaykvgw [ 0 ] ; jjatrktc0c [ 2 ] = frac_d [
2 ] * avxeaykvgw [ 1 ] * frac_d [ 0 ] + mn3ckvav5d_idx_0 * avxeaykvgw [ 0 ] ;
jjatrktc0c [ 3 ] = frac_d [ 1 ] * avxeaykvgw [ 0 ] ; jjatrktc0c [ 4 ] =
mn3ckvav5d_idx_0 * avxeaykvgw [ 1 ] * avxeaykvgw [ 0 ] + frac_d [ 2 ] *
frac_d [ 0 ] ; jjatrktc0c [ 5 ] = frac_d [ 2 ] * avxeaykvgw [ 1 ] *
avxeaykvgw [ 0 ] - mn3ckvav5d_idx_0 * frac_d [ 0 ] ; jjatrktc0c [ 6 ] = -
avxeaykvgw [ 1 ] ; jjatrktc0c [ 7 ] = mn3ckvav5d_idx_0 * frac_d [ 1 ] ;
jjatrktc0c [ 8 ] = frac_d [ 2 ] * frac_d [ 1 ] ; for ( oxjy2hf2cl = 0 ;
oxjy2hf2cl < 3 ; oxjy2hf2cl ++ ) { frac_d [ oxjy2hf2cl ] = jjatrktc0c [ 3 *
oxjy2hf2cl + 2 ] * abmm43etbq_idx_1 + ( jjatrktc0c [ 3 * oxjy2hf2cl + 1 ] *
rtP . Constant_Value_n35gokrdfy + jjatrktc0c [ 3 * oxjy2hf2cl ] *
abmm43etbq_idx_0 ) ; } rtB . nazotke0rm [ 0 ] = frac_d [ 0 ] ; rtB .
nazotke0rm [ 1 ] = frac_d [ 2 ] ; muDoubleScalarSinCos ( jb2eust2qa , &
hxkhshlt5m_idx_0 , & ksjzavltcv_idx_0 ) ; muDoubleScalarSinCos ( o2mmi0bsgf ,
& hxkhshlt5m_idx_1 , & ksjzavltcv_idx_1 ) ; jjatrktc0c [ 0 ] =
ksjzavltcv_idx_0 * ksjzavltcv_idx_1 ; jjatrktc0c [ 6 ] = hxkhshlt5m_idx_0 *
ksjzavltcv_idx_1 ; jjatrktc0c [ 1 ] = - ( hxkhshlt5m_idx_1 * ksjzavltcv_idx_0
) ; jjatrktc0c [ 7 ] = - ( hxkhshlt5m_idx_0 * hxkhshlt5m_idx_1 ) ; jjatrktc0c
[ 2 ] = - hxkhshlt5m_idx_0 ; jjatrktc0c [ 3 ] = hxkhshlt5m_idx_1 ; jjatrktc0c
[ 4 ] = ksjzavltcv_idx_1 ; jjatrktc0c [ 5 ] = rtP . Constant_Value_jdmivpanfl
; jjatrktc0c [ 8 ] = ksjzavltcv_idx_0 ; muDoubleScalarSinCos ( jb2eust2qa , &
oyontdhyku_idx_0 , & elm2mlhmo3_idx_0 ) ; muDoubleScalarSinCos ( o2mmi0bsgf ,
& oyontdhyku_idx_1 , & elm2mlhmo3_idx_1 ) ; kh2xrb5jub [ 0 ] =
elm2mlhmo3_idx_0 * elm2mlhmo3_idx_1 ; kh2xrb5jub [ 6 ] = oyontdhyku_idx_0 *
elm2mlhmo3_idx_1 ; kh2xrb5jub [ 1 ] = - ( oyontdhyku_idx_1 * elm2mlhmo3_idx_0
) ; kh2xrb5jub [ 7 ] = - ( oyontdhyku_idx_0 * oyontdhyku_idx_1 ) ; kh2xrb5jub
[ 2 ] = - oyontdhyku_idx_0 ; kh2xrb5jub [ 3 ] = oyontdhyku_idx_1 ; kh2xrb5jub
[ 4 ] = elm2mlhmo3_idx_1 ; kh2xrb5jub [ 5 ] = rtP . Constant_Value_fwsa41mimo
; kh2xrb5jub [ 8 ] = elm2mlhmo3_idx_0 ; for ( oxjy2hf2cl = 0 ; oxjy2hf2cl < 3
; oxjy2hf2cl ++ ) { avxeaykvgw [ oxjy2hf2cl ] = kh2xrb5jub [ 3 * oxjy2hf2cl +
2 ] * rtB . gtmxqvxrzk [ 1 ] + ( kh2xrb5jub [ 3 * oxjy2hf2cl + 1 ] * rtP .
Constant1_Value_aypmrruauh + kh2xrb5jub [ 3 * oxjy2hf2cl ] * rtB . gtmxqvxrzk
[ 0 ] ) ; e4kgnowjgg [ oxjy2hf2cl ] = jjatrktc0c [ oxjy2hf2cl + 6 ] *
gkpo2ijrbk_idx_2 + ( jjatrktc0c [ oxjy2hf2cl + 3 ] * gkpo2ijrbk_idx_1 +
jjatrktc0c [ oxjy2hf2cl ] * gkpo2ijrbk_idx_0 ) ; } rtB . pxggbjiorr =
avxeaykvgw [ 2 ] / e4kgnowjgg [ 0 ] + rtB . gectl3vjgg ; if (
ssIsMajorTimeStep ( rtS ) ) { switch ( rtDW . bkbk0mdq0s ) { case 0 : if (
rtX . fi3net4hji [ 0 ] <= rtP . mindef_aileron ) { rtX . fi3net4hji [ 0 ] =
rtP . mindef_aileron ; if ( rtX . fi3net4hji [ 1 ] > 0.0 ) { rtDW .
bkbk0mdq0s = 0 ; } else { rtX . fi3net4hji [ 1 ] = 0.0 ; rtDW . bkbk0mdq0s =
1 ; } ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } if ( rtX .
fi3net4hji [ 0 ] >= rtP . maxdef_aileron ) { rtX . fi3net4hji [ 0 ] = rtP .
maxdef_aileron ; if ( rtX . fi3net4hji [ 1 ] < 0.0 ) { rtDW . bkbk0mdq0s = 0
; } else { rtX . fi3net4hji [ 1 ] = 0.0 ; rtDW . bkbk0mdq0s = 2 ; }
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } if ( rtX . fi3net4hji
[ 1 ] <= - rtP . NonlinearSecondOrderActuator_fin_maxrate ) { rtX .
fi3net4hji [ 1 ] = - rtP . NonlinearSecondOrderActuator_fin_maxrate ; rtDW .
bkbk0mdq0s = 3 ; ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } if (
rtX . fi3net4hji [ 1 ] >= rtP . NonlinearSecondOrderActuator_fin_maxrate ) {
rtX . fi3net4hji [ 1 ] = rtP . NonlinearSecondOrderActuator_fin_maxrate ;
rtDW . bkbk0mdq0s = 4 ; ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ;
} break ; case 1 : if ( rtX . fi3net4hji [ 0 ] > rtP . mindef_aileron ) { if
( rtX . fi3net4hji [ 0 ] >= rtP . maxdef_aileron ) { rtDW . bkbk0mdq0s = 2 ;
rtX . fi3net4hji [ 0 ] = rtP . maxdef_aileron ; } else { rtDW . bkbk0mdq0s =
0 ; } } else { rtX . fi3net4hji [ 0 ] = rtP . mindef_aileron ; rtX .
fi3net4hji [ 1 ] = 0.0 ; } break ; case 2 : if ( rtX . fi3net4hji [ 0 ] < rtP
. maxdef_aileron ) { if ( rtX . fi3net4hji [ 0 ] <= rtP . mindef_aileron ) {
rtDW . bkbk0mdq0s = 1 ; rtX . fi3net4hji [ 0 ] = rtP . mindef_aileron ; }
else { rtDW . bkbk0mdq0s = 0 ; } } else { rtX . fi3net4hji [ 0 ] = rtP .
maxdef_aileron ; rtX . fi3net4hji [ 1 ] = 0.0 ; } break ; case 3 : if ( rtX .
fi3net4hji [ 1 ] > - rtP . NonlinearSecondOrderActuator_fin_maxrate ) { if (
rtX . fi3net4hji [ 1 ] >= rtP . NonlinearSecondOrderActuator_fin_maxrate ) {
rtDW . bkbk0mdq0s = 4 ; rtX . fi3net4hji [ 1 ] = rtP .
NonlinearSecondOrderActuator_fin_maxrate ; } else { rtDW . bkbk0mdq0s = 0 ; }
} else { rtX . fi3net4hji [ 1 ] = - rtP .
NonlinearSecondOrderActuator_fin_maxrate ; } if ( rtX . fi3net4hji [ 0 ] <=
rtP . mindef_aileron ) { rtX . fi3net4hji [ 0 ] = rtP . mindef_aileron ; rtX
. fi3net4hji [ 1 ] = 0.0 ; rtDW . bkbk0mdq0s = 1 ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } break ; case 4 : if (
rtX . fi3net4hji [ 1 ] < rtP . NonlinearSecondOrderActuator_fin_maxrate ) {
if ( rtX . fi3net4hji [ 1 ] <= - rtP .
NonlinearSecondOrderActuator_fin_maxrate ) { rtDW . bkbk0mdq0s = 3 ; rtX .
fi3net4hji [ 1 ] = - rtP . NonlinearSecondOrderActuator_fin_maxrate ; } else
{ rtDW . bkbk0mdq0s = 0 ; } } else { rtX . fi3net4hji [ 1 ] = rtP .
NonlinearSecondOrderActuator_fin_maxrate ; } if ( rtX . fi3net4hji [ 0 ] >=
rtP . maxdef_aileron ) { rtX . fi3net4hji [ 0 ] = rtP . maxdef_aileron ; rtX
. fi3net4hji [ 1 ] = 0.0 ; rtDW . bkbk0mdq0s = 2 ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } break ; } } rtB .
kjlhrll14x = rtB . exldqi4iq0 - rtX . fi3net4hji [ 0 ] ; if (
ssIsMajorTimeStep ( rtS ) ) { rtDW . ogz3an5p1g = rtB . kjlhrll14x >= rtP .
NonlinearSecondOrderActuator_fin_maxrate * 2.0 * rtP . z_act / rtP . wn_act ?
1 : rtB . kjlhrll14x > - rtP . NonlinearSecondOrderActuator_fin_maxrate * 2.0
* rtP . z_act / rtP . wn_act ? 0 : - 1 ; } rtB . jfhgj1iwfd = ( rtDW .
ogz3an5p1g == 1 ? rtP . NonlinearSecondOrderActuator_fin_maxrate * 2.0 * rtP
. z_act / rtP . wn_act : rtDW . ogz3an5p1g == - 1 ? - rtP .
NonlinearSecondOrderActuator_fin_maxrate * 2.0 * rtP . z_act / rtP . wn_act :
rtB . kjlhrll14x ) * ( rtP . wn_act * rtP . wn_act ) - 2.0 * rtP . z_act *
rtP . wn_act * rtX . fi3net4hji [ 1 ] ; if ( ssIsSampleHit ( rtS , 2 , 0 ) )
{ if ( rtB . hevlxlfvxw > rtP . maxdef_elevator ) { rtB . c12vlekqly = rtP .
maxdef_elevator ; } else if ( rtB . hevlxlfvxw < rtP . mindef_elevator ) {
rtB . c12vlekqly = rtP . mindef_elevator ; } else { rtB . c12vlekqly = rtB .
hevlxlfvxw ; } } rtB . nf1iua4byr = rtB . c12vlekqly - mc31ssftz2_idx_1 ; if
( ssIsMajorTimeStep ( rtS ) ) { rtDW . afsnxqrf1q = rtB . nf1iua4byr >= rtP .
NonlinearSecondOrderActuator1_fin_maxrate * 2.0 * rtP . z_act / rtP . wn_act
? 1 : rtB . nf1iua4byr > - rtP . NonlinearSecondOrderActuator1_fin_maxrate *
2.0 * rtP . z_act / rtP . wn_act ? 0 : - 1 ; } rtB . n0vzjwkja5 = ( rtDW .
afsnxqrf1q == 1 ? rtP . NonlinearSecondOrderActuator1_fin_maxrate * 2.0 * rtP
. z_act / rtP . wn_act : rtDW . afsnxqrf1q == - 1 ? - rtP .
NonlinearSecondOrderActuator1_fin_maxrate * 2.0 * rtP . z_act / rtP . wn_act
: rtB . nf1iua4byr ) * ( rtP . wn_act * rtP . wn_act ) - 2.0 * rtP . z_act *
rtP . wn_act * slam ; if ( ssIsMajorTimeStep ( rtS ) ) { switch ( rtDW .
b02jv5hvxo ) { case 0 : if ( rtX . aldehcvxgf [ 0 ] <= rtP . mindef_rudder )
{ rtX . aldehcvxgf [ 0 ] = rtP . mindef_rudder ; if ( rtX . aldehcvxgf [ 1 ]
> 0.0 ) { rtDW . b02jv5hvxo = 0 ; } else { rtX . aldehcvxgf [ 1 ] = 0.0 ;
rtDW . b02jv5hvxo = 1 ; } ssSetBlockStateForSolverChangedAtMajorStep ( rtS )
; } if ( rtX . aldehcvxgf [ 0 ] >= rtP . maxdef_rudder ) { rtX . aldehcvxgf [
0 ] = rtP . maxdef_rudder ; if ( rtX . aldehcvxgf [ 1 ] < 0.0 ) { rtDW .
b02jv5hvxo = 0 ; } else { rtX . aldehcvxgf [ 1 ] = 0.0 ; rtDW . b02jv5hvxo =
2 ; } ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } if ( rtX .
aldehcvxgf [ 1 ] <= - rtP . NonlinearSecondOrderActuator2_fin_maxrate ) { rtX
. aldehcvxgf [ 1 ] = - rtP . NonlinearSecondOrderActuator2_fin_maxrate ; rtDW
. b02jv5hvxo = 3 ; ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } if
( rtX . aldehcvxgf [ 1 ] >= rtP . NonlinearSecondOrderActuator2_fin_maxrate )
{ rtX . aldehcvxgf [ 1 ] = rtP . NonlinearSecondOrderActuator2_fin_maxrate ;
rtDW . b02jv5hvxo = 4 ; ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ;
} break ; case 1 : if ( rtX . aldehcvxgf [ 0 ] > rtP . mindef_rudder ) { if (
rtX . aldehcvxgf [ 0 ] >= rtP . maxdef_rudder ) { rtDW . b02jv5hvxo = 2 ; rtX
. aldehcvxgf [ 0 ] = rtP . maxdef_rudder ; } else { rtDW . b02jv5hvxo = 0 ; }
} else { rtX . aldehcvxgf [ 0 ] = rtP . mindef_rudder ; rtX . aldehcvxgf [ 1
] = 0.0 ; } break ; case 2 : if ( rtX . aldehcvxgf [ 0 ] < rtP .
maxdef_rudder ) { if ( rtX . aldehcvxgf [ 0 ] <= rtP . mindef_rudder ) { rtDW
. b02jv5hvxo = 1 ; rtX . aldehcvxgf [ 0 ] = rtP . mindef_rudder ; } else {
rtDW . b02jv5hvxo = 0 ; } } else { rtX . aldehcvxgf [ 0 ] = rtP .
maxdef_rudder ; rtX . aldehcvxgf [ 1 ] = 0.0 ; } break ; case 3 : if ( rtX .
aldehcvxgf [ 1 ] > - rtP . NonlinearSecondOrderActuator2_fin_maxrate ) { if (
rtX . aldehcvxgf [ 1 ] >= rtP . NonlinearSecondOrderActuator2_fin_maxrate ) {
rtDW . b02jv5hvxo = 4 ; rtX . aldehcvxgf [ 1 ] = rtP .
NonlinearSecondOrderActuator2_fin_maxrate ; } else { rtDW . b02jv5hvxo = 0 ;
} } else { rtX . aldehcvxgf [ 1 ] = - rtP .
NonlinearSecondOrderActuator2_fin_maxrate ; } if ( rtX . aldehcvxgf [ 0 ] <=
rtP . mindef_rudder ) { rtX . aldehcvxgf [ 0 ] = rtP . mindef_rudder ; rtX .
aldehcvxgf [ 1 ] = 0.0 ; rtDW . b02jv5hvxo = 1 ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } break ; case 4 : if (
rtX . aldehcvxgf [ 1 ] < rtP . NonlinearSecondOrderActuator2_fin_maxrate ) {
if ( rtX . aldehcvxgf [ 1 ] <= - rtP .
NonlinearSecondOrderActuator2_fin_maxrate ) { rtDW . b02jv5hvxo = 3 ; rtX .
aldehcvxgf [ 1 ] = - rtP . NonlinearSecondOrderActuator2_fin_maxrate ; } else
{ rtDW . b02jv5hvxo = 0 ; } } else { rtX . aldehcvxgf [ 1 ] = rtP .
NonlinearSecondOrderActuator2_fin_maxrate ; } if ( rtX . aldehcvxgf [ 0 ] >=
rtP . maxdef_rudder ) { rtX . aldehcvxgf [ 0 ] = rtP . maxdef_rudder ; rtX .
aldehcvxgf [ 1 ] = 0.0 ; rtDW . b02jv5hvxo = 2 ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } break ; } } rtB .
cchzsu55nk = rtB . eyzfoa2rgd - rtX . aldehcvxgf [ 0 ] ; if (
ssIsMajorTimeStep ( rtS ) ) { rtDW . le0andlo2k = rtB . cchzsu55nk >= rtP .
NonlinearSecondOrderActuator2_fin_maxrate * 2.0 * rtP . z_act / rtP . wn_act
? 1 : rtB . cchzsu55nk > - rtP . NonlinearSecondOrderActuator2_fin_maxrate *
2.0 * rtP . z_act / rtP . wn_act ? 0 : - 1 ; } rtB . chmbb41gxy = ( rtDW .
le0andlo2k == 1 ? rtP . NonlinearSecondOrderActuator2_fin_maxrate * 2.0 * rtP
. z_act / rtP . wn_act : rtDW . le0andlo2k == - 1 ? - rtP .
NonlinearSecondOrderActuator2_fin_maxrate * 2.0 * rtP . z_act / rtP . wn_act
: rtB . cchzsu55nk ) * ( rtP . wn_act * rtP . wn_act ) - 2.0 * rtP . z_act *
rtP . wn_act * rtX . aldehcvxgf [ 1 ] ; if ( ssIsSampleHit ( rtS , 2 , 0 ) )
{ } UNUSED_PARAMETER ( tid ) ; } void MdlOutputsTID5 ( int_T tid ) { real_T
nrtirxrxp4 ; int32_T i ; real_T iif4lj1wep_idx_0 ; real_T iif4lj1wep_idx_1 ;
real_T iif4lj1wep_idx_2 ; real_T iif4lj1wep_idx_3 ; rtB . dlbqs3jbq5 = rtP .
Constant6_Value ; rtB . a2xdtn2hjy = rtP . heading0 ; rtB . pjsvxmsmf0 =
0.017453292519943295 * rtP . DrydenWindTurbulenceModelContinuousqr_Wdeg ; rtB
. pk4ybsmrx1 = 3.280839895013123 * rtP .
DrydenWindTurbulenceModelContinuousqr_W20 * rtP . sigma_wg_Gain ; rtB .
bav3d1hobe = 3.280839895013123 * rtP .
DrydenWindTurbulenceModelContinuousqr_Wingspan ; rtB . plmewtggpi = rtP .
Constant3_Value / rtB . bav3d1hobe ; if ( ( rtB . plmewtggpi < 0.0 ) && ( rtP
. Constant2_Value > muDoubleScalarFloor ( rtP . Constant2_Value ) ) ) { rtB .
fffhvqsyn4 = - muDoubleScalarPower ( - rtB . plmewtggpi , rtP .
Constant2_Value ) ; } else { rtB . fffhvqsyn4 = muDoubleScalarPower ( rtB .
plmewtggpi , rtP . Constant2_Value ) ; } if ( ssIsMajorTimeStep ( rtS ) ) {
srUpdateBC ( rtDW . byzsjh0px3 ) ; } rtB . o1nidekcl5 = 3.280839895013123 *
rtP . DrydenWindTurbulenceModelContinuousqr_L_high ; if ( ssIsMajorTimeStep (
rtS ) ) { if ( rtDW . olplly2yli != 0 ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; rtDW . olplly2yli = 0 ;
} rtB . kxuhls1vu4 = muDoubleScalarSqrt ( rtP . Constant_Value ) ; srUpdateBC
( rtDW . n5gzkaa0nd ) ; } else if ( rtP . Constant_Value < 0.0 ) { rtB .
kxuhls1vu4 = - muDoubleScalarSqrt ( muDoubleScalarAbs ( rtP . Constant_Value
) ) ; rtDW . olplly2yli = 1 ; } else { rtB . kxuhls1vu4 = muDoubleScalarSqrt
( rtP . Constant_Value ) ; } rtB . prdmace1ry = plook_bincpa ( rtP .
DrydenWindTurbulenceModelContinuousqr_TurbProb , rtP .
PreLookUpIndexSearchprobofexceed_BreakpointsData , 6U , & rtB . pt0yyggmye ,
& rtDW . bxnoyp5430 ) ; if ( ssIsMajorTimeStep ( rtS ) ) { if ( rtDW .
ijpkeyczg1 != 0 ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; rtDW
. ijpkeyczg1 = 0 ; } iif4lj1wep_idx_0 = muDoubleScalarSqrt ( rtP .
WhiteNoise_pwr [ 0 ] ) ; iif4lj1wep_idx_1 = muDoubleScalarSqrt ( rtP .
WhiteNoise_pwr [ 1 ] ) ; iif4lj1wep_idx_2 = muDoubleScalarSqrt ( rtP .
WhiteNoise_pwr [ 2 ] ) ; iif4lj1wep_idx_3 = muDoubleScalarSqrt ( rtP .
WhiteNoise_pwr [ 3 ] ) ; if ( rtDW . e25tjl33um != 0 ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; rtDW . e25tjl33um = 0 ;
} nrtirxrxp4 = muDoubleScalarSqrt ( rtP . WhiteNoise_Ts ) ; } else { if ( rtP
. WhiteNoise_pwr [ 0 ] < 0.0 ) { iif4lj1wep_idx_0 = - muDoubleScalarSqrt (
muDoubleScalarAbs ( rtP . WhiteNoise_pwr [ 0 ] ) ) ; rtDW . ijpkeyczg1 = 1 ;
} else { iif4lj1wep_idx_0 = muDoubleScalarSqrt ( rtP . WhiteNoise_pwr [ 0 ] )
; } if ( rtP . WhiteNoise_pwr [ 1 ] < 0.0 ) { iif4lj1wep_idx_1 = -
muDoubleScalarSqrt ( muDoubleScalarAbs ( rtP . WhiteNoise_pwr [ 1 ] ) ) ;
rtDW . ijpkeyczg1 = 1 ; } else { iif4lj1wep_idx_1 = muDoubleScalarSqrt ( rtP
. WhiteNoise_pwr [ 1 ] ) ; } if ( rtP . WhiteNoise_pwr [ 2 ] < 0.0 ) {
iif4lj1wep_idx_2 = - muDoubleScalarSqrt ( muDoubleScalarAbs ( rtP .
WhiteNoise_pwr [ 2 ] ) ) ; rtDW . ijpkeyczg1 = 1 ; } else { iif4lj1wep_idx_2
= muDoubleScalarSqrt ( rtP . WhiteNoise_pwr [ 2 ] ) ; } if ( rtP .
WhiteNoise_pwr [ 3 ] < 0.0 ) { iif4lj1wep_idx_3 = - muDoubleScalarSqrt (
muDoubleScalarAbs ( rtP . WhiteNoise_pwr [ 3 ] ) ) ; rtDW . ijpkeyczg1 = 1 ;
} else { iif4lj1wep_idx_3 = muDoubleScalarSqrt ( rtP . WhiteNoise_pwr [ 3 ] )
; } if ( rtP . WhiteNoise_Ts < 0.0 ) { nrtirxrxp4 = - muDoubleScalarSqrt (
muDoubleScalarAbs ( rtP . WhiteNoise_Ts ) ) ; rtDW . e25tjl33um = 1 ; } else
{ nrtirxrxp4 = muDoubleScalarSqrt ( rtP . WhiteNoise_Ts ) ; } } rtB .
dp0qd0ktnr [ 0 ] = iif4lj1wep_idx_0 / nrtirxrxp4 ; rtB . dp0qd0ktnr [ 1 ] =
iif4lj1wep_idx_1 / nrtirxrxp4 ; rtB . dp0qd0ktnr [ 2 ] = iif4lj1wep_idx_2 /
nrtirxrxp4 ; rtB . dp0qd0ktnr [ 3 ] = iif4lj1wep_idx_3 / nrtirxrxp4 ;
muDoubleScalarSinCos ( 0.017453292519943295 * rtP . WindShearModel_Wdeg , &
nrtirxrxp4 , & iif4lj1wep_idx_0 ) ; rtB . fzxirwzcjx [ 0 ] = - rtP .
WindShearModel_W_20 * iif4lj1wep_idx_0 ; rtB . fzxirwzcjx [ 1 ] = - rtP .
WindShearModel_W_20 * nrtirxrxp4 ; rtB . fzxirwzcjx [ 2 ] = - rtP .
WindShearModel_W_20 * rtP . Wdeg1_Value ; nrtirxrxp4 = rtP .
ref_heightz0_Value ; if ( ssIsMajorTimeStep ( rtS ) ) { if ( rtDW .
auligz0bp0 != 0 ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; rtDW
. auligz0bp0 = 0 ; } } else { if ( rtP . ref_heightz0_Value < 0.0 ) {
nrtirxrxp4 = 0.0 ; rtDW . auligz0bp0 = 1 ; } } rtB . or3uazt4fw =
muDoubleScalarLog ( nrtirxrxp4 ) ; rtB . mb51iyxxjy = rtP .
min_height_high_Value - rtP . max_height_low_Value ; rtB . ntt1gar3xf = rtP .
min_height_high_Value_okjndayw2f - rtP . max_height_low_Value_bkbuta4yqd ;
iif4lj1wep_idx_0 = rtP . gamma_Value - rtP . one1_Value ; nrtirxrxp4 = 1.0 /
iif4lj1wep_idx_0 ; iif4lj1wep_idx_0 = rtP . const_Value * rtP . const_Value *
iif4lj1wep_idx_0 * rtP . two_Value + rtP . one_Value ; if ( (
iif4lj1wep_idx_0 < 0.0 ) && ( nrtirxrxp4 > muDoubleScalarFloor ( nrtirxrxp4 )
) ) { rtB . kbkk0nabqt [ 0 ] = - muDoubleScalarPower ( - iif4lj1wep_idx_0 ,
nrtirxrxp4 ) ; } else { rtB . kbkk0nabqt [ 0 ] = muDoubleScalarPower (
iif4lj1wep_idx_0 , nrtirxrxp4 ) ; } nrtirxrxp4 *= rtP . gamma_Value ; if ( (
iif4lj1wep_idx_0 < 0.0 ) && ( nrtirxrxp4 > muDoubleScalarFloor ( nrtirxrxp4 )
) ) { rtB . kbkk0nabqt [ 1 ] = - muDoubleScalarPower ( - iif4lj1wep_idx_0 ,
nrtirxrxp4 ) ; } else { rtB . kbkk0nabqt [ 1 ] = muDoubleScalarPower (
iif4lj1wep_idx_0 , nrtirxrxp4 ) ; } rtB . ohifllm5xd = rtP . Constant12_Value
* rtP . Constant12_Value ; for ( i = 0 ; i < 3 ; i ++ ) { rtB . ot3vk2b455 [
i ] = rtP . zero1_Value_jquxqq4qvz [ i ] - rtP . zero3_Value [ i ] ; rtB .
ddawuzzdzz [ 3 * i ] = rtP . Constant_Value_olio5liftr [ i ] ; rtB .
ddawuzzdzz [ 3 * i + 1 ] = rtP . Constant_Value_olio5liftr [ i + 3 ] ; rtB .
ddawuzzdzz [ 3 * i + 2 ] = rtP . Constant_Value_olio5liftr [ i + 6 ] ; rtB .
mzb2t2xbjn [ i ] = rtP . zero3_Value [ i ] - rtP . zero3_Value [ i ] ; } if (
0.0 > rtP . maxdef_aileron ) { rtB . exldqi4iq0 = rtP . maxdef_aileron ; }
else if ( 0.0 < rtP . mindef_aileron ) { rtB . exldqi4iq0 = rtP .
mindef_aileron ; } else { rtB . exldqi4iq0 = 0.0 ; } if ( 0.0 > rtP .
maxdef_rudder ) { rtB . eyzfoa2rgd = rtP . maxdef_rudder ; } else if ( 0.0 <
rtP . mindef_rudder ) { rtB . eyzfoa2rgd = rtP . mindef_rudder ; } else { rtB
. eyzfoa2rgd = 0.0 ; } if ( muDoubleScalarAbs ( rtP . LatLong0 [ 0 ] ) > rtP
. CompareToConstant_const_fkrbb1l4we ) { nrtirxrxp4 = muDoubleScalarMod ( rtP
. LatLong0 [ 0 ] + rtP . Bias_Bias_epi214vgpp , rtP .
Constant2_Value_oe5zwfuhbf ) + rtP . Bias1_Bias_itbskgc1ah ; } else {
nrtirxrxp4 = rtP . LatLong0 [ 0 ] ; } iif4lj1wep_idx_0 = muDoubleScalarAbs (
nrtirxrxp4 ) ; if ( iif4lj1wep_idx_0 > rtP .
CompareToConstant_const_o4jsn2vuck ) { rtB . knraoiqthj = ( (
iif4lj1wep_idx_0 + rtP . Bias_Bias_aujzrf41ca ) * rtP . Gain_Gain_fmc2fxjyzt
+ rtP . Bias1_Bias_jirwvifxga ) * muDoubleScalarSign ( nrtirxrxp4 ) ;
nrtirxrxp4 = rtP . Constant_Value_iatzfmuax5 ; } else { rtB . knraoiqthj =
nrtirxrxp4 ; nrtirxrxp4 = rtP . Constant1_Value_jbkvuyvxyw ; } nrtirxrxp4 +=
rtP . LatLong0 [ 1 ] ; if ( muDoubleScalarAbs ( nrtirxrxp4 ) > rtP .
CompareToConstant_const_n1tmrxoekt ) { rtB . oxryesp4l3 = muDoubleScalarMod (
nrtirxrxp4 + rtP . Bias_Bias_gall5umyqi , rtP . Constant2_Value_pkv2f4fmve )
+ rtP . Bias1_Bias_g0mi2jjfta ; } else { rtB . oxryesp4l3 = nrtirxrxp4 ; }
nrtirxrxp4 = 0.017453292519943295 * rtB . knraoiqthj ; iif4lj1wep_idx_0 = rtP
. f_Value - rtP . Constant_Value_ddl3nhb2x0 ; iif4lj1wep_idx_1 = rtP .
Constant_Value_hdq4ys1wku - iif4lj1wep_idx_0 * iif4lj1wep_idx_0 ; if (
ssIsMajorTimeStep ( rtS ) ) { if ( rtDW . jutuodwsau != 0 ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; rtDW . jutuodwsau = 0 ;
} iif4lj1wep_idx_0 = muDoubleScalarSqrt ( iif4lj1wep_idx_1 ) ; } else if (
iif4lj1wep_idx_1 < 0.0 ) { iif4lj1wep_idx_0 = - muDoubleScalarSqrt (
muDoubleScalarAbs ( iif4lj1wep_idx_1 ) ) ; rtDW . jutuodwsau = 1 ; } else {
iif4lj1wep_idx_0 = muDoubleScalarSqrt ( iif4lj1wep_idx_1 ) ; }
iif4lj1wep_idx_1 = muDoubleScalarSin ( nrtirxrxp4 ) ; iif4lj1wep_idx_1 = rtP
. Constant_Value_ltgnw1q5be - iif4lj1wep_idx_0 * iif4lj1wep_idx_0 *
iif4lj1wep_idx_1 * iif4lj1wep_idx_1 ; if ( ssIsMajorTimeStep ( rtS ) ) { if (
rtDW . n12ashwpk5 != 0 ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS )
; rtDW . n12ashwpk5 = 0 ; } iif4lj1wep_idx_2 = muDoubleScalarSqrt (
iif4lj1wep_idx_1 ) ; } else if ( iif4lj1wep_idx_1 < 0.0 ) { iif4lj1wep_idx_2
= - muDoubleScalarSqrt ( muDoubleScalarAbs ( iif4lj1wep_idx_1 ) ) ; rtDW .
n12ashwpk5 = 1 ; } else { iif4lj1wep_idx_2 = muDoubleScalarSqrt (
iif4lj1wep_idx_1 ) ; } iif4lj1wep_idx_2 = rtP . Constant1_Value_hjtl2lfjek /
iif4lj1wep_idx_2 ; rtB . ktgttcsket = muDoubleScalarAtan2 ( rtP .
Constant2_Value_dnwbrdkxjw , ( rtP . Constant_Value_bf4fbrpvxe -
iif4lj1wep_idx_0 * iif4lj1wep_idx_0 ) * iif4lj1wep_idx_2 / iif4lj1wep_idx_1 )
; rtB . dtrbet0r1l = muDoubleScalarAtan2 ( rtP . Constant3_Value_hlef1jp2hn ,
iif4lj1wep_idx_2 * muDoubleScalarCos ( nrtirxrxp4 ) ) ; muDoubleScalarSinCos
( 0.017453292519943295 * rtP . heading0 , & rtB . dojqnhucc1 , & rtB .
kkwizmpnnu ) ; rtB . p1ud1zl0l5 = rtP . Constant10_Value * rtB . kkwizmpnnu ;
rtB . h1os02q0lx = rtP . Constant10_Value * rtB . dojqnhucc1 ;
UNUSED_PARAMETER ( tid ) ; } void MdlUpdate ( int_T tid ) { int32_T uMode ;
rtDW . afe03te4gm = 0 ; if ( rtDW . kk3mavtp0m ) { if ( rtX . jxx3w5csvx ==
rtP . Distanceintogustx_d_m ) { switch ( rtDW . hueaazkach ) { case 3 : if (
rtB . gsfqejjcfo < 0.0 ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS )
; rtDW . hueaazkach = 1 ; } break ; case 1 : if ( rtB . gsfqejjcfo >= 0.0 ) {
rtDW . hueaazkach = 3 ; ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ;
} break ; default : ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; if (
rtB . gsfqejjcfo < 0.0 ) { rtDW . hueaazkach = 1 ; } else { rtDW . hueaazkach
= 3 ; } break ; } } else if ( rtX . jxx3w5csvx == rtP .
DistanceintoGustxLimitedtogustlengthd_LowerSat ) { switch ( rtDW . hueaazkach
) { case 4 : if ( rtB . gsfqejjcfo > 0.0 ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; rtDW . hueaazkach = 2 ;
} break ; case 2 : if ( rtB . gsfqejjcfo <= 0.0 ) { rtDW . hueaazkach = 4 ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } break ; default :
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; if ( rtB . gsfqejjcfo >
0.0 ) { rtDW . hueaazkach = 2 ; } else { rtDW . hueaazkach = 4 ; } break ; }
} else { rtDW . hueaazkach = 0 ; } } ilmclpxlmx ( rtS , rtB . gsfqejjcfo , &
rtDW . mirhnoymfvo , & rtP . mirhnoymfvo , & rtX . mirhnoymfvo , rtP .
Distanceintogusty_d_m ) ; ilmclpxlmx ( rtS , rtB . gsfqejjcfo , & rtDW .
i5wp2ffbry , & rtP . i5wp2ffbry , & rtX . i5wp2ffbry , rtP .
Distanceintogustz_d_m ) ; if ( ssIsSampleHit ( rtS , 4 , 0 ) ) { rtDW .
joib1qjnje [ 0 ] = rt_nrand_Upu32_Yd_f_pw_snf ( & rtDW . khqvqeq4d4 [ 0 ] ) *
rtP . WhiteNoise_StdDev + rtP . WhiteNoise_Mean ; rtDW . joib1qjnje [ 1 ] =
rt_nrand_Upu32_Yd_f_pw_snf ( & rtDW . khqvqeq4d4 [ 1 ] ) * rtP .
WhiteNoise_StdDev + rtP . WhiteNoise_Mean ; rtDW . joib1qjnje [ 2 ] =
rt_nrand_Upu32_Yd_f_pw_snf ( & rtDW . khqvqeq4d4 [ 2 ] ) * rtP .
WhiteNoise_StdDev + rtP . WhiteNoise_Mean ; rtDW . joib1qjnje [ 3 ] =
rt_nrand_Upu32_Yd_f_pw_snf ( & rtDW . khqvqeq4d4 [ 3 ] ) * rtP .
WhiteNoise_StdDev + rtP . WhiteNoise_Mean ; } rtDW . foeszin0cg = rtB .
j2oyzznxjz ; rtDW . e0khmaiz1x = ssGetTaskTime ( rtS , 0 ) ; if (
ssIsSampleHit ( rtS , 2 , 0 ) ) { { real_T xnew [ 2 ] ; xnew [ 0 ] = ( rtP .
AltController_A [ 0 ] ) * rtDW . dh0tof3kep [ 0 ] + ( rtP . AltController_A [
1 ] ) * rtDW . dh0tof3kep [ 1 ] ; xnew [ 0 ] += rtP . AltController_B * rtB .
erkqk25euy ; xnew [ 1 ] = ( rtP . AltController_A [ 2 ] ) * rtDW . dh0tof3kep
[ 0 ] ; ( void ) memcpy ( & rtDW . dh0tof3kep [ 0 ] , xnew , sizeof ( real_T
) * 2 ) ; } { rtDW . glj000y2ul = rtB . chm2baiq0o + rtP . ThetaController_A
* rtDW . glj000y2ul ; } } if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { rtDW .
hgabqtd4hg = rtB . pxggbjiorr ; } uMode = rtDW . li4wvvnvo0 ; if ( ( ( ( rtDW
. li4wvvnvo0 == 1 ) || ( rtDW . li4wvvnvo0 == 3 ) ) && ( rtB . n0vzjwkja5 >
0.0 ) ) || ( ( ( rtDW . li4wvvnvo0 == 2 ) || ( rtDW . li4wvvnvo0 == 4 ) ) &&
( rtB . n0vzjwkja5 < 0.0 ) ) ) { uMode = 0 ; } if ( rtDW . li4wvvnvo0 !=
uMode ) { rtDW . li4wvvnvo0 = uMode ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } uMode = rtDW .
bkbk0mdq0s ; if ( ( ( ( rtDW . bkbk0mdq0s == 1 ) || ( rtDW . bkbk0mdq0s == 3
) ) && ( rtB . jfhgj1iwfd > 0.0 ) ) || ( ( ( rtDW . bkbk0mdq0s == 2 ) || (
rtDW . bkbk0mdq0s == 4 ) ) && ( rtB . jfhgj1iwfd < 0.0 ) ) ) { uMode = 0 ; }
if ( rtDW . bkbk0mdq0s != uMode ) { rtDW . bkbk0mdq0s = uMode ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } uMode = rtDW .
b02jv5hvxo ; if ( ( ( ( rtDW . b02jv5hvxo == 1 ) || ( rtDW . b02jv5hvxo == 3
) ) && ( rtB . chmbb41gxy > 0.0 ) ) || ( ( ( rtDW . b02jv5hvxo == 2 ) || (
rtDW . b02jv5hvxo == 4 ) ) && ( rtB . chmbb41gxy < 0.0 ) ) ) { uMode = 0 ; }
if ( rtDW . b02jv5hvxo != uMode ) { rtDW . b02jv5hvxo = uMode ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } UNUSED_PARAMETER ( tid
) ; } void MdlUpdateTID5 ( int_T tid ) { UNUSED_PARAMETER ( tid ) ; } void
MdlDerivatives ( void ) { XDot * _rtXdot ; XDis * _rtXdis ; _rtXdis = ( (
XDis * ) ssGetContStateDisabled ( rtS ) ) ; _rtXdot = ( ( XDot * ) ssGetdX (
rtS ) ) ; _rtXdot -> oyqwpsgdfy = rtB . gectl3vjgg ; _rtXdot -> nb321hhpjg [
0 ] = rtB . nazotke0rm [ 0 ] ; _rtXdot -> nmbwexfepb [ 0 ] = rtB . gtmxqvxrzk
[ 0 ] ; _rtXdot -> nb321hhpjg [ 1 ] = rtB . nazotke0rm [ 1 ] ; _rtXdot ->
nmbwexfepb [ 1 ] = rtB . gtmxqvxrzk [ 1 ] ; if ( rtDW . kk3mavtp0m ) { if ( (
rtDW . hueaazkach != 3 ) && ( rtDW . hueaazkach != 4 ) ) { _rtXdot ->
jxx3w5csvx = rtB . gsfqejjcfo ; _rtXdis -> jxx3w5csvx = false ; } else {
_rtXdot -> jxx3w5csvx = 0.0 ; _rtXdis -> jxx3w5csvx = ( ( rtDW . hueaazkach
== 3 ) || ( rtDW . hueaazkach == 4 ) || _rtXdis -> jxx3w5csvx ) ; } } else {
( ( XDot * ) ssGetdX ( rtS ) ) -> jxx3w5csvx = 0.0 ; } n5ulcqjymx ( rtB .
gsfqejjcfo , & rtDW . mirhnoymfvo , & _rtXdis -> mirhnoymfvo , & _rtXdot ->
mirhnoymfvo ) ; n5ulcqjymx ( rtB . gsfqejjcfo , & rtDW . i5wp2ffbry , &
_rtXdis -> i5wp2ffbry , & _rtXdot -> i5wp2ffbry ) ; if ( rtDW . ikp5xo5ucv )
{ _rtXdot -> obrj2mgpdb [ 0 ] = rtB . f3yq1g0dcx [ 0 ] ; _rtXdot ->
obrj2mgpdb [ 1 ] = rtB . f3yq1g0dcx [ 1 ] ; } else { { real_T * dx ; int_T i
; dx = & ( ( ( XDot * ) ssGetdX ( rtS ) ) -> obrj2mgpdb [ 0 ] ) ; for ( i = 0
; i < 2 ; i ++ ) { dx [ i ] = 0.0 ; } } } if ( rtDW . nztae4vpib ) { _rtXdot
-> jcf02uzlnt [ 0 ] = rtB . lxqebjchye [ 0 ] ; _rtXdot -> go1vncxzci [ 0 ] =
rtB . c5gx5xznq1 [ 0 ] ; _rtXdot -> jcf02uzlnt [ 1 ] = rtB . lxqebjchye [ 1 ]
; _rtXdot -> go1vncxzci [ 1 ] = rtB . c5gx5xznq1 [ 1 ] ; } else { { real_T *
dx ; int_T i ; dx = & ( ( ( XDot * ) ssGetdX ( rtS ) ) -> jcf02uzlnt [ 0 ] )
; for ( i = 0 ; i < 4 ; i ++ ) { dx [ i ] = 0.0 ; } } } if ( rtDW .
auringonfr ) { _rtXdot -> b5qj413fkr [ 0 ] = rtB . agqbvy2aau [ 0 ] ; _rtXdot
-> b5qj413fkr [ 1 ] = rtB . agqbvy2aau [ 1 ] ; } else { { real_T * dx ; int_T
i ; dx = & ( ( ( XDot * ) ssGetdX ( rtS ) ) -> b5qj413fkr [ 0 ] ) ; for ( i =
0 ; i < 2 ; i ++ ) { dx [ i ] = 0.0 ; } } } if ( rtDW . ahb01lfm54 ) {
_rtXdot -> ds4avnfqd3 [ 0 ] = rtB . ckuncjpeqo [ 0 ] ; _rtXdot -> p11ukldqr3
[ 0 ] = rtB . gehuvldt5n [ 0 ] ; _rtXdot -> ds4avnfqd3 [ 1 ] = rtB .
ckuncjpeqo [ 1 ] ; _rtXdot -> p11ukldqr3 [ 1 ] = rtB . gehuvldt5n [ 1 ] ; }
else { { real_T * dx ; int_T i ; dx = & ( ( ( XDot * ) ssGetdX ( rtS ) ) ->
ds4avnfqd3 [ 0 ] ) ; for ( i = 0 ; i < 4 ; i ++ ) { dx [ i ] = 0.0 ; } } } if
( rtDW . bdjnrykkzo ) { _rtXdot -> efv1zergvx [ 0 ] = rtB . mv5ztdhgch [ 0 ]
; _rtXdot -> efv1zergvx [ 1 ] = rtB . mv5ztdhgch [ 1 ] ; } else { { real_T *
dx ; int_T i ; dx = & ( ( ( XDot * ) ssGetdX ( rtS ) ) -> efv1zergvx [ 0 ] )
; for ( i = 0 ; i < 2 ; i ++ ) { dx [ i ] = 0.0 ; } } } if ( rtDW .
aovs2hjxyi ) { _rtXdot -> galtq11byd [ 0 ] = rtB . fosv245qat [ 0 ] ; _rtXdot
-> galtq11byd [ 1 ] = rtB . fosv245qat [ 1 ] ; } else { { real_T * dx ; int_T
i ; dx = & ( ( ( XDot * ) ssGetdX ( rtS ) ) -> galtq11byd [ 0 ] ) ; for ( i =
0 ; i < 2 ; i ++ ) { dx [ i ] = 0.0 ; } } } _rtXdot -> lwmusv4p0k = 0.0 ;
_rtXdot -> lwmusv4p0k += rtP . StateSpace_A * rtX . lwmusv4p0k ; _rtXdot ->
lwmusv4p0k += rtP . StateSpace_B * rtB . e4lsqrzqjh ; _rtXdot -> c3n310npj5 =
rtB . bf5sj4v0nw ; switch ( rtDW . li4wvvnvo0 ) { case 0 : _rtXdot ->
m2cgemps3b [ 0 ] = rtX . m2cgemps3b [ 1 ] ; _rtXdot -> m2cgemps3b [ 1 ] = rtB
. n0vzjwkja5 ; break ; case 3 : _rtXdot -> m2cgemps3b [ 0 ] = rtX .
m2cgemps3b [ 1 ] ; _rtXdot -> m2cgemps3b [ 1 ] = 0.0 ; break ; case 4 :
_rtXdot -> m2cgemps3b [ 0 ] = rtX . m2cgemps3b [ 1 ] ; _rtXdot -> m2cgemps3b
[ 1 ] = 0.0 ; break ; case 1 : _rtXdot -> m2cgemps3b [ 0 ] = 0.0 ; _rtXdot ->
m2cgemps3b [ 1 ] = 0.0 ; break ; case 2 : _rtXdot -> m2cgemps3b [ 0 ] = 0.0 ;
_rtXdot -> m2cgemps3b [ 1 ] = 0.0 ; break ; } switch ( rtDW . bkbk0mdq0s ) {
case 0 : _rtXdot -> fi3net4hji [ 0 ] = rtX . fi3net4hji [ 1 ] ; _rtXdot ->
fi3net4hji [ 1 ] = rtB . jfhgj1iwfd ; break ; case 3 : _rtXdot -> fi3net4hji
[ 0 ] = rtX . fi3net4hji [ 1 ] ; _rtXdot -> fi3net4hji [ 1 ] = 0.0 ; break ;
case 4 : _rtXdot -> fi3net4hji [ 0 ] = rtX . fi3net4hji [ 1 ] ; _rtXdot ->
fi3net4hji [ 1 ] = 0.0 ; break ; case 1 : _rtXdot -> fi3net4hji [ 0 ] = 0.0 ;
_rtXdot -> fi3net4hji [ 1 ] = 0.0 ; break ; case 2 : _rtXdot -> fi3net4hji [
0 ] = 0.0 ; _rtXdot -> fi3net4hji [ 1 ] = 0.0 ; break ; } switch ( rtDW .
b02jv5hvxo ) { case 0 : _rtXdot -> aldehcvxgf [ 0 ] = rtX . aldehcvxgf [ 1 ]
; _rtXdot -> aldehcvxgf [ 1 ] = rtB . chmbb41gxy ; break ; case 3 : _rtXdot
-> aldehcvxgf [ 0 ] = rtX . aldehcvxgf [ 1 ] ; _rtXdot -> aldehcvxgf [ 1 ] =
0.0 ; break ; case 4 : _rtXdot -> aldehcvxgf [ 0 ] = rtX . aldehcvxgf [ 1 ] ;
_rtXdot -> aldehcvxgf [ 1 ] = 0.0 ; break ; case 1 : _rtXdot -> aldehcvxgf [
0 ] = 0.0 ; _rtXdot -> aldehcvxgf [ 1 ] = 0.0 ; break ; case 2 : _rtXdot ->
aldehcvxgf [ 0 ] = 0.0 ; _rtXdot -> aldehcvxgf [ 1 ] = 0.0 ; break ; } } void
MdlProjection ( void ) { } void MdlZeroCrossings ( void ) { ZCV * _rtZCSV ;
_rtZCSV = ( ( ZCV * ) ssGetSolverZcSignalVector ( rtS ) ) ; _rtZCSV ->
lzeqoaloix = rtB . cbt1v4nio1 [ 0 ] ; _rtZCSV -> d1hk41evnx = rtB .
chuqvl4sya - rtP . CompareToConstant_const_n1mct5qu1a ; _rtZCSV -> hqcohimthh
= rtB . hpaztoaur1 ; _rtZCSV -> gc52dxkvvu = rtB . pwdhurk11o - rtP .
CompareToConstant_const_omj1fbfglm ; _rtZCSV -> kuqfbhlzzf = rtB . pstvhythqv
; _rtZCSV -> oxin4qa23u = rtB . hjs5mprgdw - rtP .
CompareToConstant_const_d4qgtkvaqb ; _rtZCSV -> gn1kedvloc = rtB . hpaztoaur1
; _rtZCSV -> hz5xr0iwvo = rtB . pkgvmqtajm [ 1 ] ; _rtZCSV -> ojq25he1w4 =
rtB . lvp0utv5zh - rtP . DiscreteWindGustModel_t_0 ; if ( rtDW . kk3mavtp0m )
{ if ( ( rtDW . hueaazkach == 1 ) && ( rtX . jxx3w5csvx >= rtP .
Distanceintogustx_d_m ) ) { _rtZCSV -> egav2gulw1 = 0.0 ; } else { _rtZCSV ->
egav2gulw1 = rtX . jxx3w5csvx - rtP . Distanceintogustx_d_m ; } if ( ( rtDW .
hueaazkach == 2 ) && ( rtX . jxx3w5csvx <= rtP .
DistanceintoGustxLimitedtogustlengthd_LowerSat ) ) { _rtZCSV -> hubhp0n4tj =
0.0 ; } else { _rtZCSV -> hubhp0n4tj = rtX . jxx3w5csvx - rtP .
DistanceintoGustxLimitedtogustlengthd_LowerSat ; } if ( ( rtDW . hueaazkach
== 3 ) || ( rtDW . hueaazkach == 4 ) ) { _rtZCSV -> hnljyyxcdw = rtB .
gsfqejjcfo ; } else { _rtZCSV -> hnljyyxcdw = 0.0 ; } } else { { real_T *
zcsv = & ( ( ( ZCV * ) ssGetSolverZcSignalVector ( rtS ) ) -> egav2gulw1 ) ;
int_T i ; for ( i = 0 ; i < 3 ; i ++ ) { zcsv [ i ] = 0.0 ; } } } aeerk0ugtn
( rtB . gsfqejjcfo , & rtDW . mirhnoymfvo , & rtP . mirhnoymfvo , & rtX .
mirhnoymfvo , & _rtZCSV -> mirhnoymfvo , rtP . Distanceintogusty_d_m ) ;
aeerk0ugtn ( rtB . gsfqejjcfo , & rtDW . i5wp2ffbry , & rtP . i5wp2ffbry , &
rtX . i5wp2ffbry , & _rtZCSV -> i5wp2ffbry , rtP . Distanceintogustz_d_m ) ;
_rtZCSV -> b5xpex1gz2 = rtB . bs4j1k4omq - rtP .
LimitFunction10ftto1000ft_UpperSat ; _rtZCSV -> kt114whjjn = rtB . bs4j1k4omq
- rtP . LimitFunction10ftto1000ft_LowerSat ; _rtZCSV -> ghnvbi3vun = rtB .
bs4j1k4omq - rtP . LimitHeighth1000ft_UpperSat ; _rtZCSV -> ovhuivd3z2 = rtB
. bs4j1k4omq - rtP . LimitHeighth1000ft_LowerSat ; _rtZCSV -> k4agqastic [ 0
] = 0.0 ; _rtZCSV -> k4agqastic [ 1 ] = 0.0 ; if ( rtB . bs4j1k4omq <= 1000.0
) { _rtZCSV -> k4agqastic [ 0 ] = 1.0 ; } else { if ( rtB . bs4j1k4omq >=
2000.0 ) { _rtZCSV -> k4agqastic [ 1 ] = 1.0 ; } } _rtZCSV -> h2nqdokrmz [ 0
] = 0.0 ; _rtZCSV -> h2nqdokrmz [ 1 ] = 0.0 ; if ( rtB . bs4j1k4omq <= 1000.0
) { _rtZCSV -> h2nqdokrmz [ 0 ] = 1.0 ; } else { if ( rtB . bs4j1k4omq >=
2000.0 ) { _rtZCSV -> h2nqdokrmz [ 1 ] = 1.0 ; } } _rtZCSV -> porjn24fbf =
rtB . ewd50jnfc5 - rtP . uftinf_UpperSat ; _rtZCSV -> f34qpd1zsm = rtB .
ewd50jnfc5 - rtP . uftinf_LowerSat ; _rtZCSV -> ngnlqom0qz = ssGetT ( rtS ) -
rtP . Step_Time ; { ( ( ZCV * ) ssGetSolverZcSignalVector ( rtS ) ) ->
ja4ilf4myr = 0.0 ; } if ( rtDW . cal4znigzs == 0 ) { _rtZCSV -> ja4ilf4myr =
rtB . j1pkkc3ac5 - rtP . Constant_Value_pmyhtxrqrr ; } { ( ( ZCV * )
ssGetSolverZcSignalVector ( rtS ) ) -> ixjmldywkk = 0.0 ; } if ( rtDW .
o3p0d0pvrk == 0 ) { _rtZCSV -> ixjmldywkk = rtB . ivvuf1kwbw - rtP .
Constant_Value_gun4ws4ai2 ; } { ( ( ZCV * ) ssGetSolverZcSignalVector ( rtS )
) -> cbt5odztjp = 0.0 ; } if ( rtDW . mxjlbo5rjs == 0 ) { _rtZCSV ->
cbt5odztjp = rtB . dv0mxcbo4y - rtP . Constant_Value_mq1pkcil10 ; } _rtZCSV
-> grqwdvhlin = rtX . m2cgemps3b [ 0 ] - rtP . mindef_elevator ; _rtZCSV ->
dmgrwrmwg5 = rtX . m2cgemps3b [ 0 ] - rtP . maxdef_elevator ; _rtZCSV ->
nk1g0dangh = rtX . m2cgemps3b [ 1 ] - ( - rtP .
NonlinearSecondOrderActuator1_fin_maxrate ) ; _rtZCSV -> elhkxuqxgq = rtX .
m2cgemps3b [ 1 ] - rtP . NonlinearSecondOrderActuator1_fin_maxrate ; _rtZCSV
-> cimcujnpcn = 0.0 ; if ( rtDW . li4wvvnvo0 != 0 ) { _rtZCSV -> cimcujnpcn =
rtB . n0vzjwkja5 ; } _rtZCSV -> lrfqm10zu4 = rtX . fi3net4hji [ 0 ] - rtP .
mindef_aileron ; _rtZCSV -> ofizq1py51 = rtX . fi3net4hji [ 0 ] - rtP .
maxdef_aileron ; _rtZCSV -> ajmmycxne1 = rtX . fi3net4hji [ 1 ] - ( - rtP .
NonlinearSecondOrderActuator_fin_maxrate ) ; _rtZCSV -> aiyxrxey2a = rtX .
fi3net4hji [ 1 ] - rtP . NonlinearSecondOrderActuator_fin_maxrate ; _rtZCSV
-> kkznorvmx3 = 0.0 ; if ( rtDW . bkbk0mdq0s != 0 ) { _rtZCSV -> kkznorvmx3 =
rtB . jfhgj1iwfd ; } _rtZCSV -> okdb0qcnrk = rtB . kjlhrll14x - rtP .
NonlinearSecondOrderActuator_fin_maxrate * 2.0 * rtP . z_act / rtP . wn_act ;
_rtZCSV -> kjqit3l212 = rtB . kjlhrll14x - - rtP .
NonlinearSecondOrderActuator_fin_maxrate * 2.0 * rtP . z_act / rtP . wn_act ;
_rtZCSV -> a1kostazuj = rtB . nf1iua4byr - rtP .
NonlinearSecondOrderActuator1_fin_maxrate * 2.0 * rtP . z_act / rtP . wn_act
; _rtZCSV -> iasqm0qyqg = rtB . nf1iua4byr - - rtP .
NonlinearSecondOrderActuator1_fin_maxrate * 2.0 * rtP . z_act / rtP . wn_act
; _rtZCSV -> gpzhpfpdtr = rtX . aldehcvxgf [ 0 ] - rtP . mindef_rudder ;
_rtZCSV -> cufjvfhmpt = rtX . aldehcvxgf [ 0 ] - rtP . maxdef_rudder ;
_rtZCSV -> kil22di02h = rtX . aldehcvxgf [ 1 ] - ( - rtP .
NonlinearSecondOrderActuator2_fin_maxrate ) ; _rtZCSV -> lebfshs02m = rtX .
aldehcvxgf [ 1 ] - rtP . NonlinearSecondOrderActuator2_fin_maxrate ; _rtZCSV
-> lrgoafpgt4 = 0.0 ; if ( rtDW . b02jv5hvxo != 0 ) { _rtZCSV -> lrgoafpgt4 =
rtB . chmbb41gxy ; } _rtZCSV -> k1gnz04lfw = rtB . cchzsu55nk - rtP .
NonlinearSecondOrderActuator2_fin_maxrate * 2.0 * rtP . z_act / rtP . wn_act
; _rtZCSV -> hvjawqgn5b = rtB . cchzsu55nk - - rtP .
NonlinearSecondOrderActuator2_fin_maxrate * 2.0 * rtP . z_act / rtP . wn_act
; } void MdlTerminate ( void ) { rtDW . ib0jqzpceq = rt_SlioAccessorRelease (
1 , 1 , rtDW . ib0jqzpceq ) ; rtDW . fodolp1u3u = rt_SlioAccessorRelease ( 1
, 1 , rtDW . fodolp1u3u ) ; rtDW . h4fprkvunp = rt_SlioAccessorRelease ( 1 ,
1 , rtDW . h4fprkvunp ) ; rtDW . gu0lptjayg = rt_SlioAccessorRelease ( 1 , 1
, rtDW . gu0lptjayg ) ; rtDW . cygydsjrfb = rt_SlioAccessorRelease ( 1 , 1 ,
rtDW . cygydsjrfb ) ; if ( rt_slioCatalogue ( ) != ( NULL ) ) { void * *
slioCatalogueAddr = rt_slioCatalogueAddr ( ) ; rtwSaveDatasetsToMatFile (
rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) ,
rt_GetMatSigstreamLoggingFileName ( ) ) ; rtwTerminateSlioCatalogue (
slioCatalogueAddr ) ; * slioCatalogueAddr = NULL ; } } void
MdlInitializeSizes ( void ) { ssSetNumContStates ( rtS , 32 ) ;
ssSetNumPeriodicContStates ( rtS , 1 ) ; ssSetNumY ( rtS , 0 ) ; ssSetNumU (
rtS , 0 ) ; ssSetDirectFeedThrough ( rtS , 0 ) ; ssSetNumSampleTimes ( rtS ,
5 ) ; ssSetNumBlocks ( rtS , 1017 ) ; ssSetNumBlockIO ( rtS , 125 ) ;
ssSetNumBlockParams ( rtS , 3229433 ) ; } void MdlInitializeSampleTimes (
void ) { ssSetSampleTime ( rtS , 0 , 0.0 ) ; ssSetSampleTime ( rtS , 1 , 0.0
) ; ssSetSampleTime ( rtS , 2 , 0.016666666666666666 ) ; ssSetSampleTime (
rtS , 3 , 0.033333333333333333 ) ; ssSetSampleTime ( rtS , 4 , 0.1 ) ;
ssSetOffsetTime ( rtS , 0 , 0.0 ) ; ssSetOffsetTime ( rtS , 1 , 1.0 ) ;
ssSetOffsetTime ( rtS , 2 , 0.0 ) ; ssSetOffsetTime ( rtS , 3 , 0.0 ) ;
ssSetOffsetTime ( rtS , 4 , 0.0 ) ; } void raccel_set_checksum ( ) {
ssSetChecksumVal ( rtS , 0 , 3787883694U ) ; ssSetChecksumVal ( rtS , 1 ,
3449449585U ) ; ssSetChecksumVal ( rtS , 2 , 185154508U ) ; ssSetChecksumVal
( rtS , 3 , 4030028274U ) ; }
#if defined(_MSC_VER)
#pragma optimize( "", off )
#endif
SimStruct * raccel_register_model ( void ) { static struct _ssMdlInfo mdlInfo
; ( void ) memset ( ( char * ) rtS , 0 , sizeof ( SimStruct ) ) ; ( void )
memset ( ( char * ) & mdlInfo , 0 , sizeof ( struct _ssMdlInfo ) ) ;
ssSetMdlInfoPtr ( rtS , & mdlInfo ) ; { static time_T mdlPeriod [
NSAMPLE_TIMES ] ; static time_T mdlOffset [ NSAMPLE_TIMES ] ; static time_T
mdlTaskTimes [ NSAMPLE_TIMES ] ; static int_T mdlTsMap [ NSAMPLE_TIMES ] ;
static int_T mdlSampleHits [ NSAMPLE_TIMES ] ; static boolean_T
mdlTNextWasAdjustedPtr [ NSAMPLE_TIMES ] ; static int_T mdlPerTaskSampleHits
[ NSAMPLE_TIMES * NSAMPLE_TIMES ] ; static time_T mdlTimeOfNextSampleHit [
NSAMPLE_TIMES ] ; { int_T i ; for ( i = 0 ; i < NSAMPLE_TIMES ; i ++ ) {
mdlPeriod [ i ] = 0.0 ; mdlOffset [ i ] = 0.0 ; mdlTaskTimes [ i ] = 0.0 ;
mdlTsMap [ i ] = i ; mdlSampleHits [ i ] = 1 ; } } ssSetSampleTimePtr ( rtS ,
& mdlPeriod [ 0 ] ) ; ssSetOffsetTimePtr ( rtS , & mdlOffset [ 0 ] ) ;
ssSetSampleTimeTaskIDPtr ( rtS , & mdlTsMap [ 0 ] ) ; ssSetTPtr ( rtS , &
mdlTaskTimes [ 0 ] ) ; ssSetSampleHitPtr ( rtS , & mdlSampleHits [ 0 ] ) ;
ssSetTNextWasAdjustedPtr ( rtS , & mdlTNextWasAdjustedPtr [ 0 ] ) ;
ssSetPerTaskSampleHitsPtr ( rtS , & mdlPerTaskSampleHits [ 0 ] ) ;
ssSetTimeOfNextSampleHitPtr ( rtS , & mdlTimeOfNextSampleHit [ 0 ] ) ; }
ssSetSolverMode ( rtS , SOLVER_MODE_SINGLETASKING ) ; { ssSetBlockIO ( rtS ,
( ( void * ) & rtB ) ) ; ( void ) memset ( ( ( void * ) & rtB ) , 0 , sizeof
( B ) ) ; { int32_T i ; for ( i = 0 ; i < 9 ; i ++ ) { rtB . ddawuzzdzz [ i ]
= 0.0 ; } for ( i = 0 ; i < 6 ; i ++ ) { rtB . di0bxuhii0 [ i ] = 0.0 ; } rtB
. pkgvmqtajm [ 0 ] = 0.0 ; rtB . pkgvmqtajm [ 1 ] = 0.0 ; rtB . cbt1v4nio1 [
0 ] = 0.0 ; rtB . cbt1v4nio1 [ 1 ] = 0.0 ; rtB . chuqvl4sya = 0.0 ; rtB .
hpaztoaur1 = 0.0 ; rtB . pwdhurk11o = 0.0 ; rtB . akzllltikr = 0.0 ; rtB .
pstvhythqv = 0.0 ; rtB . hjs5mprgdw = 0.0 ; rtB . ckkqbbvet5 = 0.0 ; rtB .
lauuzxpsuv = 0.0 ; rtB . kpfxhsrdzc = 0.0 ; rtB . d2gfe2p42l = 0.0 ; rtB .
fdu3hdd1wq = 0.0 ; rtB . gr2e1rit1v [ 0 ] = 0.0 ; rtB . gr2e1rit1v [ 1 ] =
0.0 ; rtB . jeyqu0v1xd = 0.0 ; rtB . erkqk25euy = 0.0 ; rtB . chm2baiq0o =
0.0 ; rtB . hevlxlfvxw = 0.0 ; rtB . j1pkkc3ac5 = 0.0 ; rtB . iiwmf2tmrv =
0.0 ; rtB . onh5wf0mbq = 0.0 ; rtB . a4jldatvmm = 0.0 ; rtB . gectl3vjgg =
0.0 ; rtB . nrzu1kxyjs [ 0 ] = 0.0 ; rtB . nrzu1kxyjs [ 1 ] = 0.0 ; rtB .
nrzu1kxyjs [ 2 ] = 0.0 ; rtB . bf5sj4v0nw = 0.0 ; rtB . nw4xrcz5qc [ 0 ] =
0.0 ; rtB . nw4xrcz5qc [ 1 ] = 0.0 ; rtB . gtmxqvxrzk [ 0 ] = 0.0 ; rtB .
gtmxqvxrzk [ 1 ] = 0.0 ; rtB . nazotke0rm [ 0 ] = 0.0 ; rtB . nazotke0rm [ 1
] = 0.0 ; rtB . pxggbjiorr = 0.0 ; rtB . kjlhrll14x = 0.0 ; rtB . jfhgj1iwfd
= 0.0 ; rtB . c12vlekqly = 0.0 ; rtB . nf1iua4byr = 0.0 ; rtB . n0vzjwkja5 =
0.0 ; rtB . cchzsu55nk = 0.0 ; rtB . chmbb41gxy = 0.0 ; rtB . dlbqs3jbq5 =
0.0 ; rtB . a2xdtn2hjy = 0.0 ; rtB . kbkk0nabqt [ 0 ] = 0.0 ; rtB .
kbkk0nabqt [ 1 ] = 0.0 ; rtB . ohifllm5xd = 0.0 ; rtB . ot3vk2b455 [ 0 ] =
0.0 ; rtB . ot3vk2b455 [ 1 ] = 0.0 ; rtB . ot3vk2b455 [ 2 ] = 0.0 ; rtB .
mzb2t2xbjn [ 0 ] = 0.0 ; rtB . mzb2t2xbjn [ 1 ] = 0.0 ; rtB . mzb2t2xbjn [ 2
] = 0.0 ; rtB . exldqi4iq0 = 0.0 ; rtB . eyzfoa2rgd = 0.0 ; rtB . knraoiqthj
= 0.0 ; rtB . oxryesp4l3 = 0.0 ; rtB . ktgttcsket = 0.0 ; rtB . dtrbet0r1l =
0.0 ; rtB . dojqnhucc1 = 0.0 ; rtB . kkwizmpnnu = 0.0 ; rtB . p1ud1zl0l5 =
0.0 ; rtB . h1os02q0lx = 0.0 ; rtB . fh1jzoqwae = 0.0 ; rtB . netnloma2z =
0.0 ; rtB . bz2agq32f5 = 0.0 ; rtB . gakkrw1mku = 0.0 ; rtB . lldfszhlwc =
0.0 ; rtB . iqcro5zqc5 = 0.0 ; rtB . khr4lzxr01 = 0.0 ; rtB . otbeb0smep =
0.0 ; rtB . mbiqcrghxm = 0.0 ; rtB . gxcomph2oi = 0.0 ; rtB . pcxo4cuead =
0.0 ; rtB . euq0m2wkrk = 0.0 ; rtB . lzfl1px3we = 0.0 ; rtB . c2uwt5jnat =
0.0 ; rtB . e4lsqrzqjh = 0.0 ; rtB . j2oyzznxjz = 0.0 ; rtB . hkzk0mcpdz =
0.0 ; rtB . cjcp2dooxg = 0.0 ; rtB . ivvuf1kwbw = 0.0 ; rtB . dv0mxcbo4y =
0.0 ; rtB . nts3iqoscx = 0.0 ; rtB . lvp0utv5zh = 0.0 ; rtB . bs4j1k4omq =
0.0 ; rtB . nsbfkssthw [ 0 ] = 0.0 ; rtB . nsbfkssthw [ 1 ] = 0.0 ; rtB .
nsbfkssthw [ 2 ] = 0.0 ; rtB . nsbfkssthw [ 3 ] = 0.0 ; rtB . gsfqejjcfo =
0.0 ; rtB . ewd50jnfc5 = 0.0 ; rtB . pjsvxmsmf0 = 0.0 ; rtB . pk4ybsmrx1 =
0.0 ; rtB . bav3d1hobe = 0.0 ; rtB . o1nidekcl5 = 0.0 ; rtB . pt0yyggmye =
0.0 ; rtB . dp0qd0ktnr [ 0 ] = 0.0 ; rtB . dp0qd0ktnr [ 1 ] = 0.0 ; rtB .
dp0qd0ktnr [ 2 ] = 0.0 ; rtB . dp0qd0ktnr [ 3 ] = 0.0 ; rtB . fzxirwzcjx [ 0
] = 0.0 ; rtB . fzxirwzcjx [ 1 ] = 0.0 ; rtB . fzxirwzcjx [ 2 ] = 0.0 ; rtB .
or3uazt4fw = 0.0 ; rtB . ntt1gar3xf = 0.0 ; rtB . mb51iyxxjy = 0.0 ; rtB .
lxqebjchye [ 0 ] = 0.0 ; rtB . lxqebjchye [ 1 ] = 0.0 ; rtB . ircgwytey3 [ 0
] = 0.0 ; rtB . ircgwytey3 [ 1 ] = 0.0 ; rtB . c5gx5xznq1 [ 0 ] = 0.0 ; rtB .
c5gx5xznq1 [ 1 ] = 0.0 ; rtB . kxuhls1vu4 = 0.0 ; rtB . ckuncjpeqo [ 0 ] =
0.0 ; rtB . ckuncjpeqo [ 1 ] = 0.0 ; rtB . gehuvldt5n [ 0 ] = 0.0 ; rtB .
gehuvldt5n [ 1 ] = 0.0 ; rtB . k3touvbkw2 [ 0 ] = 0.0 ; rtB . k3touvbkw2 [ 1
] = 0.0 ; rtB . fosv245qat [ 0 ] = 0.0 ; rtB . fosv245qat [ 1 ] = 0.0 ; rtB .
leujf5c51t [ 0 ] = 0.0 ; rtB . leujf5c51t [ 1 ] = 0.0 ; rtB . mv5ztdhgch [ 0
] = 0.0 ; rtB . mv5ztdhgch [ 1 ] = 0.0 ; rtB . agqbvy2aau [ 0 ] = 0.0 ; rtB .
agqbvy2aau [ 1 ] = 0.0 ; rtB . f3yq1g0dcx [ 0 ] = 0.0 ; rtB . f3yq1g0dcx [ 1
] = 0.0 ; rtB . plmewtggpi = 0.0 ; rtB . fffhvqsyn4 = 0.0 ; rtB . ohglmemv1w
= 0.0 ; rtB . h1fm141rey = 0.0F ; rtB . hllvbobsss = 0.0F ; rtB . augghaqbk3
= 0.0F ; rtB . i5wp2ffbry . cwtrsfu0ve = 0.0 ; rtB . mirhnoymfvo . cwtrsfu0ve
= 0.0 ; } } { real_T * x = ( real_T * ) & rtX ; ssSetContStates ( rtS , x ) ;
( void ) memset ( ( void * ) x , 0 , sizeof ( X ) ) ; } { void * dwork = (
void * ) & rtDW ; ssSetRootDWork ( rtS , dwork ) ; ( void ) memset ( dwork ,
0 , sizeof ( DW ) ) ; rtDW . dh0tof3kep [ 0 ] = 0.0 ; rtDW . dh0tof3kep [ 1 ]
= 0.0 ; rtDW . glj000y2ul = 0.0 ; { int32_T i ; for ( i = 0 ; i < 8 ; i ++ )
{ rtDW . gyluhpuobq [ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i < 8 ; i
++ ) { rtDW . hfu1jte3yo [ i ] = 0.0 ; } } rtDW . hgabqtd4hg = 0.0 ; rtDW .
lwgbxvspmn = 0.0 ; rtDW . btjxdj1nnm = 0.0 ; rtDW . foeszin0cg = 0.0 ; rtDW .
e0khmaiz1x = 0.0 ; { int32_T i ; for ( i = 0 ; i < 8 ; i ++ ) { rtDW .
csnixtyelx [ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i < 8 ; i ++ ) {
rtDW . aqckczktpc [ i ] = 0.0 ; } } rtDW . joib1qjnje [ 0 ] = 0.0 ; rtDW .
joib1qjnje [ 1 ] = 0.0 ; rtDW . joib1qjnje [ 2 ] = 0.0 ; rtDW . joib1qjnje [
3 ] = 0.0 ; rtDW . aqq0sq0gty . czibmh2e1r [ 0 ] = 0.0 ; rtDW . aqq0sq0gty .
czibmh2e1r [ 1 ] = 0.0 ; rtDW . aqq0sq0gty . czibmh2e1r [ 2 ] = 0.0 ; {
int32_T i ; for ( i = 0 ; i < 3660 ; i ++ ) { rtDW . aqq0sq0gty . bsnqpkum4j
[ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i < 3660 ; i ++ ) { rtDW .
aqq0sq0gty . ng2ho4wdw5 [ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i <
3660 ; i ++ ) { rtDW . aqq0sq0gty . emyp25k2ra [ i ] = 0.0 ; } } { int32_T i
; for ( i = 0 ; i < 88 ; i ++ ) { rtDW . aqq0sq0gty . ke3jfhfxg1 [ i ] = 0.0
; } } { int32_T i ; for ( i = 0 ; i < 322080 ; i ++ ) { rtDW . aqq0sq0gty .
c1uyo1uy5e [ i ] = 0.0 ; } } rtDW . aqq0sq0gty . l0xygmktrt [ 0 ] = 0.0 ;
rtDW . aqq0sq0gty . l0xygmktrt [ 1 ] = 0.0 ; rtDW . aqq0sq0gty . l0xygmktrt [
2 ] = 0.0 ; { int32_T i ; for ( i = 0 ; i < 3660 ; i ++ ) { rtDW . aqq0sq0gty
. ec4ih5lgh0 [ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i < 3660 ; i ++ )
{ rtDW . aqq0sq0gty . hf0jovm0ds [ i ] = 0.0 ; } } { int32_T i ; for ( i = 0
; i < 3660 ; i ++ ) { rtDW . aqq0sq0gty . d2jzmsgxqy [ i ] = 0.0 ; } } {
int32_T i ; for ( i = 0 ; i < 88 ; i ++ ) { rtDW . aqq0sq0gty . iel1urtae2 [
i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i < 322080 ; i ++ ) { rtDW .
aqq0sq0gty . mv4eqkyxzg [ i ] = 0.0 ; } } rtDW . aqq0sq0gty . o3sl5fyijx [ 0
] = 0.0 ; rtDW . aqq0sq0gty . o3sl5fyijx [ 1 ] = 0.0 ; rtDW . aqq0sq0gty .
o3sl5fyijx [ 2 ] = 0.0 ; { int32_T i ; for ( i = 0 ; i < 3660 ; i ++ ) { rtDW
. aqq0sq0gty . iownbpqjgp [ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i <
3660 ; i ++ ) { rtDW . aqq0sq0gty . htq3wzf05k [ i ] = 0.0 ; } } { int32_T i
; for ( i = 0 ; i < 3660 ; i ++ ) { rtDW . aqq0sq0gty . mhvuogwz5g [ i ] =
0.0 ; } } { int32_T i ; for ( i = 0 ; i < 88 ; i ++ ) { rtDW . aqq0sq0gty .
e4boznuo11 [ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i < 322080 ; i ++ )
{ rtDW . aqq0sq0gty . oltjiqpr2r [ i ] = 0.0 ; } } rtDW . aqq0sq0gty .
kbkjlm3ow2 [ 0 ] = 0.0 ; rtDW . aqq0sq0gty . kbkjlm3ow2 [ 1 ] = 0.0 ; rtDW .
aqq0sq0gty . kbkjlm3ow2 [ 2 ] = 0.0 ; { int32_T i ; for ( i = 0 ; i < 3660 ;
i ++ ) { rtDW . aqq0sq0gty . gpo4oh4yoc [ i ] = 0.0 ; } } { int32_T i ; for (
i = 0 ; i < 3660 ; i ++ ) { rtDW . aqq0sq0gty . lbrztkveec [ i ] = 0.0 ; } }
{ int32_T i ; for ( i = 0 ; i < 3660 ; i ++ ) { rtDW . aqq0sq0gty .
hbisxcpsrg [ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i < 88 ; i ++ ) {
rtDW . aqq0sq0gty . maz32p5ch2 [ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ;
i < 322080 ; i ++ ) { rtDW . aqq0sq0gty . otcphh42tj [ i ] = 0.0 ; } } rtDW .
aqq0sq0gty . mucstbyucg [ 0 ] = 0.0 ; rtDW . aqq0sq0gty . mucstbyucg [ 1 ] =
0.0 ; rtDW . aqq0sq0gty . mucstbyucg [ 2 ] = 0.0 ; { int32_T i ; for ( i = 0
; i < 3660 ; i ++ ) { rtDW . aqq0sq0gty . lvndzs1soq [ i ] = 0.0 ; } } {
int32_T i ; for ( i = 0 ; i < 3660 ; i ++ ) { rtDW . aqq0sq0gty . pcfohnqayd
[ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i < 3660 ; i ++ ) { rtDW .
aqq0sq0gty . gvwl2w40zt [ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i < 88
; i ++ ) { rtDW . aqq0sq0gty . llvsxfyht1 [ i ] = 0.0 ; } } { int32_T i ; for
( i = 0 ; i < 322080 ; i ++ ) { rtDW . aqq0sq0gty . c1emqgsxgf [ i ] = 0.0 ;
} } rtDW . cjigmwyptgo . czibmh2e1r [ 0 ] = 0.0 ; rtDW . cjigmwyptgo .
czibmh2e1r [ 1 ] = 0.0 ; rtDW . cjigmwyptgo . czibmh2e1r [ 2 ] = 0.0 ; {
int32_T i ; for ( i = 0 ; i < 3660 ; i ++ ) { rtDW . cjigmwyptgo . bsnqpkum4j
[ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i < 3660 ; i ++ ) { rtDW .
cjigmwyptgo . ng2ho4wdw5 [ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i <
3660 ; i ++ ) { rtDW . cjigmwyptgo . emyp25k2ra [ i ] = 0.0 ; } } { int32_T i
; for ( i = 0 ; i < 88 ; i ++ ) { rtDW . cjigmwyptgo . ke3jfhfxg1 [ i ] = 0.0
; } } { int32_T i ; for ( i = 0 ; i < 322080 ; i ++ ) { rtDW . cjigmwyptgo .
c1uyo1uy5e [ i ] = 0.0 ; } } rtDW . cjigmwyptgo . l0xygmktrt [ 0 ] = 0.0 ;
rtDW . cjigmwyptgo . l0xygmktrt [ 1 ] = 0.0 ; rtDW . cjigmwyptgo . l0xygmktrt
[ 2 ] = 0.0 ; { int32_T i ; for ( i = 0 ; i < 3660 ; i ++ ) { rtDW .
cjigmwyptgo . ec4ih5lgh0 [ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i <
3660 ; i ++ ) { rtDW . cjigmwyptgo . hf0jovm0ds [ i ] = 0.0 ; } } { int32_T i
; for ( i = 0 ; i < 3660 ; i ++ ) { rtDW . cjigmwyptgo . d2jzmsgxqy [ i ] =
0.0 ; } } { int32_T i ; for ( i = 0 ; i < 88 ; i ++ ) { rtDW . cjigmwyptgo .
iel1urtae2 [ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i < 322080 ; i ++ )
{ rtDW . cjigmwyptgo . mv4eqkyxzg [ i ] = 0.0 ; } } rtDW . cjigmwyptgo .
o3sl5fyijx [ 0 ] = 0.0 ; rtDW . cjigmwyptgo . o3sl5fyijx [ 1 ] = 0.0 ; rtDW .
cjigmwyptgo . o3sl5fyijx [ 2 ] = 0.0 ; { int32_T i ; for ( i = 0 ; i < 3660 ;
i ++ ) { rtDW . cjigmwyptgo . iownbpqjgp [ i ] = 0.0 ; } } { int32_T i ; for
( i = 0 ; i < 3660 ; i ++ ) { rtDW . cjigmwyptgo . htq3wzf05k [ i ] = 0.0 ; }
} { int32_T i ; for ( i = 0 ; i < 3660 ; i ++ ) { rtDW . cjigmwyptgo .
mhvuogwz5g [ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i < 88 ; i ++ ) {
rtDW . cjigmwyptgo . e4boznuo11 [ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ;
i < 322080 ; i ++ ) { rtDW . cjigmwyptgo . oltjiqpr2r [ i ] = 0.0 ; } } rtDW
. cjigmwyptgo . kbkjlm3ow2 [ 0 ] = 0.0 ; rtDW . cjigmwyptgo . kbkjlm3ow2 [ 1
] = 0.0 ; rtDW . cjigmwyptgo . kbkjlm3ow2 [ 2 ] = 0.0 ; { int32_T i ; for ( i
= 0 ; i < 3660 ; i ++ ) { rtDW . cjigmwyptgo . gpo4oh4yoc [ i ] = 0.0 ; } } {
int32_T i ; for ( i = 0 ; i < 3660 ; i ++ ) { rtDW . cjigmwyptgo . lbrztkveec
[ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i < 3660 ; i ++ ) { rtDW .
cjigmwyptgo . hbisxcpsrg [ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i < 88
; i ++ ) { rtDW . cjigmwyptgo . maz32p5ch2 [ i ] = 0.0 ; } } { int32_T i ;
for ( i = 0 ; i < 322080 ; i ++ ) { rtDW . cjigmwyptgo . otcphh42tj [ i ] =
0.0 ; } } rtDW . cjigmwyptgo . mucstbyucg [ 0 ] = 0.0 ; rtDW . cjigmwyptgo .
mucstbyucg [ 1 ] = 0.0 ; rtDW . cjigmwyptgo . mucstbyucg [ 2 ] = 0.0 ; {
int32_T i ; for ( i = 0 ; i < 3660 ; i ++ ) { rtDW . cjigmwyptgo . lvndzs1soq
[ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i < 3660 ; i ++ ) { rtDW .
cjigmwyptgo . pcfohnqayd [ i ] = 0.0 ; } } { int32_T i ; for ( i = 0 ; i <
3660 ; i ++ ) { rtDW . cjigmwyptgo . gvwl2w40zt [ i ] = 0.0 ; } } { int32_T i
; for ( i = 0 ; i < 88 ; i ++ ) { rtDW . cjigmwyptgo . llvsxfyht1 [ i ] = 0.0
; } } { int32_T i ; for ( i = 0 ; i < 322080 ; i ++ ) { rtDW . cjigmwyptgo .
c1emqgsxgf [ i ] = 0.0 ; } } } { static DataTypeTransInfo dtInfo ; ( void )
memset ( ( char_T * ) & dtInfo , 0 , sizeof ( dtInfo ) ) ;
ssSetModelMappingInfo ( rtS , & dtInfo ) ; dtInfo . numDataTypes = 15 ;
dtInfo . dataTypeSizes = & rtDataTypeSizes [ 0 ] ; dtInfo . dataTypeNames = &
rtDataTypeNames [ 0 ] ; dtInfo . BTransTable = & rtBTransTable ; dtInfo .
PTransTable = & rtPTransTable ; dtInfo . dataTypeInfoTable =
rtDataTypeInfoTable ; } Simulink_Simulation_InitializeDataMapInfo ( ) ;
ssSetIsRapidAcceleratorActive ( rtS , true ) ; ssSetRootSS ( rtS , rtS ) ;
ssSetVersion ( rtS , SIMSTRUCT_VERSION_LEVEL2 ) ; ssSetModelName ( rtS ,
"Simulink_Simulation" ) ; ssSetPath ( rtS , "Simulink_Simulation" ) ;
ssSetTStart ( rtS , 0.0 ) ; ssSetTFinal ( rtS , 75.0 ) ; { static RTWLogInfo
rt_DataLoggingInfo ; rt_DataLoggingInfo . loggingInterval = NULL ;
ssSetRTWLogInfo ( rtS , & rt_DataLoggingInfo ) ; } { { static int_T
rt_LoggedStateWidths [ ] = { 2 , 1 , 2 , 1 , 2 , 2 , 2 , 1 , 2 , 2 , 2 , 2 ,
2 , 2 , 2 , 2 , 1 , 1 , 1 , 2 , 1 } ; static int_T
rt_LoggedStateNumDimensions [ ] = { 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1
, 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 } ; static int_T
rt_LoggedStateDimensions [ ] = { 2 , 1 , 2 , 1 , 2 , 2 , 2 , 1 , 2 , 2 , 2 ,
2 , 2 , 2 , 2 , 2 , 1 , 1 , 1 , 2 , 1 } ; static boolean_T
rt_LoggedStateIsVarDims [ ] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0
, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 } ; static BuiltInDTypeId
rt_LoggedStateDataTypeIds [ ] = { SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE } ;
static int_T rt_LoggedStateComplexSignals [ ] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 } ; static
RTWPreprocessingFcnPtr rt_LoggingStatePreprocessingFcnPtrs [ ] = { ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) } ; static
const char_T * rt_LoggedStateLabels [ ] = { "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "DSTATE" , "DSTATE" } ; static const char_T *
rt_LoggedStateBlockNames [ ] = {
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF (Body Axes)/Position"
,
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF (Body Axes)/Theta"
,
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF (Body Axes)/U,w"
,
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF (Body Axes)/q"
,
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator1/Integrator,\nSecond-Order\nLimited"
,
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator/Integrator,\nSecond-Order\nLimited"
,
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator2/Integrator,\nSecond-Order\nLimited"
,
"Simulink_Simulation/Pilot/Transfer Fcn\n(with initial outputs)/State Space"
,
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model \n(Continuous (+q +r))/Filters on velocities/Hwgw(s)/wg_p1"
,
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model \n(Continuous (+q +r))/Filters on velocities/Hwgw(s)/wg_p2"
,
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model \n(Continuous (+q +r))/Filters on velocities/Hvgw(s)/vg_p1"
,
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model \n(Continuous (+q +r))/Filters on velocities/Hvgw(s)/vgw_p2"
,
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model \n(Continuous (+q +r))/Filters on velocities/Hugw(s)/ug_p"
,
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model \n(Continuous (+q +r))/Filters on angular rates/Hrgw/rgw_p"
,
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model \n(Continuous (+q +r))/Filters on angular rates/Hqgw/qgw_p"
,
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model \n(Continuous (+q +r))/Filters on angular rates/Hpgw/pgw_p"
,
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into\ngust (z)/Distance into\nGust (x)\n(Limited to gust length d)\n"
,
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into\ngust (y)/Distance into\nGust (x)\n(Limited to gust length d)\n"
,
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into\ngust (x)/Distance into\nGust (x)\n(Limited to gust length d)"
,
"Simulink_Simulation/Vehicle System Model/Avionics/Autopilot/Alt Controller"
,
"Simulink_Simulation/Vehicle System Model/Avionics/Autopilot/Theta Controller"
} ; static const char_T * rt_LoggedStateNames [ ] = { "" , "" , "" , "" , ""
, "" , "" , "" , "" , "" , "" , "" , "" , "" , "" , "" , "" , "" , "" , "" ,
"" } ; static boolean_T rt_LoggedStateCrossMdlRef [ ] = { 0 , 0 , 0 , 0 , 0 ,
0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 } ; static
RTWLogDataTypeConvert rt_RTWLogDataTypeConvert [ ] = { { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } } ; static RTWLogSignalInfo
rt_LoggedStateSignalInfo = { 21 , rt_LoggedStateWidths ,
rt_LoggedStateNumDimensions , rt_LoggedStateDimensions ,
rt_LoggedStateIsVarDims , ( NULL ) , ( NULL ) , rt_LoggedStateDataTypeIds ,
rt_LoggedStateComplexSignals , ( NULL ) , rt_LoggingStatePreprocessingFcnPtrs
, { rt_LoggedStateLabels } , ( NULL ) , ( NULL ) , ( NULL ) , {
rt_LoggedStateBlockNames } , { rt_LoggedStateNames } ,
rt_LoggedStateCrossMdlRef , rt_RTWLogDataTypeConvert } ; static void *
rt_LoggedStateSignalPtrs [ 21 ] ; rtliSetLogXSignalPtrs ( ssGetRTWLogInfo (
rtS ) , ( LogSignalPtrsType ) rt_LoggedStateSignalPtrs ) ;
rtliSetLogXSignalInfo ( ssGetRTWLogInfo ( rtS ) , & rt_LoggedStateSignalInfo
) ; rt_LoggedStateSignalPtrs [ 0 ] = ( void * ) & rtX . nb321hhpjg [ 0 ] ;
rt_LoggedStateSignalPtrs [ 1 ] = ( void * ) & rtX . oyqwpsgdfy ;
rt_LoggedStateSignalPtrs [ 2 ] = ( void * ) & rtX . nmbwexfepb [ 0 ] ;
rt_LoggedStateSignalPtrs [ 3 ] = ( void * ) & rtX . c3n310npj5 ;
rt_LoggedStateSignalPtrs [ 4 ] = ( void * ) & rtX . m2cgemps3b [ 0 ] ;
rt_LoggedStateSignalPtrs [ 5 ] = ( void * ) & rtX . fi3net4hji [ 0 ] ;
rt_LoggedStateSignalPtrs [ 6 ] = ( void * ) & rtX . aldehcvxgf [ 0 ] ;
rt_LoggedStateSignalPtrs [ 7 ] = ( void * ) & rtX . lwmusv4p0k ;
rt_LoggedStateSignalPtrs [ 8 ] = ( void * ) & rtX . jcf02uzlnt [ 0 ] ;
rt_LoggedStateSignalPtrs [ 9 ] = ( void * ) & rtX . go1vncxzci [ 0 ] ;
rt_LoggedStateSignalPtrs [ 10 ] = ( void * ) & rtX . ds4avnfqd3 [ 0 ] ;
rt_LoggedStateSignalPtrs [ 11 ] = ( void * ) & rtX . p11ukldqr3 [ 0 ] ;
rt_LoggedStateSignalPtrs [ 12 ] = ( void * ) & rtX . galtq11byd [ 0 ] ;
rt_LoggedStateSignalPtrs [ 13 ] = ( void * ) & rtX . efv1zergvx [ 0 ] ;
rt_LoggedStateSignalPtrs [ 14 ] = ( void * ) & rtX . b5qj413fkr [ 0 ] ;
rt_LoggedStateSignalPtrs [ 15 ] = ( void * ) & rtX . obrj2mgpdb [ 0 ] ;
rt_LoggedStateSignalPtrs [ 16 ] = ( void * ) & rtX . i5wp2ffbry . peioropukt
; rt_LoggedStateSignalPtrs [ 17 ] = ( void * ) & rtX . mirhnoymfvo .
peioropukt ; rt_LoggedStateSignalPtrs [ 18 ] = ( void * ) & rtX . jxx3w5csvx
; rt_LoggedStateSignalPtrs [ 19 ] = ( void * ) rtDW . dh0tof3kep ;
rt_LoggedStateSignalPtrs [ 20 ] = ( void * ) & rtDW . glj000y2ul ; }
rtliSetLogT ( ssGetRTWLogInfo ( rtS ) , "tout" ) ; rtliSetLogX (
ssGetRTWLogInfo ( rtS ) , "tmp_raccel_xout" ) ; rtliSetLogXFinal (
ssGetRTWLogInfo ( rtS ) , "xFinal" ) ; rtliSetLogVarNameModifier (
ssGetRTWLogInfo ( rtS ) , "none" ) ; rtliSetLogFormat ( ssGetRTWLogInfo ( rtS
) , 2 ) ; rtliSetLogMaxRows ( ssGetRTWLogInfo ( rtS ) , 1000 ) ;
rtliSetLogDecimation ( ssGetRTWLogInfo ( rtS ) , 1 ) ; rtliSetLogY (
ssGetRTWLogInfo ( rtS ) , "" ) ; rtliSetLogYSignalInfo ( ssGetRTWLogInfo (
rtS ) , ( NULL ) ) ; rtliSetLogYSignalPtrs ( ssGetRTWLogInfo ( rtS ) , ( NULL
) ) ; } { static struct _ssStatesInfo2 statesInfo2 ; ssSetStatesInfo2 ( rtS ,
& statesInfo2 ) ; } { static ssPeriodicStatesInfo periodicStatesInfo ;
ssSetPeriodicStatesInfo ( rtS , & periodicStatesInfo ) ;
ssSetPeriodicContStateIndices ( rtS , rtPeriodicIndX ) ; ( void ) memset ( (
void * ) rtPeriodicIndX , 0 , 1 * sizeof ( int_T ) ) ;
ssSetPeriodicContStateRanges ( rtS , rtPeriodicRngX ) ; ( void ) memset ( (
void * ) rtPeriodicRngX , 0 , 2 * sizeof ( real_T ) ) ; } { static
ssJacobianPerturbationBounds jacobianPerturbationBounds ;
ssSetJacobianPerturbationBounds ( rtS , & jacobianPerturbationBounds ) ; } {
static ssSolverInfo slvrInfo ; static boolean_T contStatesDisabled [ 32 ] ;
static real_T absTol [ 32 ] = { 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 ,
1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 ,
1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 ,
1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 ,
1.0E-6 , 1.0E-6 , 1.0E-6 } ; static uint8_T absTolControl [ 32 ] = { 0U , 0U
, 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U ,
0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U } ;
static real_T contStateJacPerturbBoundMinVec [ 32 ] ; static real_T
contStateJacPerturbBoundMaxVec [ 32 ] ; static uint8_T zcAttributes [ 56 ] =
{ ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) ,
( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , (
ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL_DN ) ,
( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_DN ) , ( ZC_EVENT_ALL_UP ) , (
ZC_EVENT_ALL ) , ( ZC_EVENT_ALL_DN ) , ( ZC_EVENT_ALL_UP ) , (
ZC_EVENT_ALL_DN ) , ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL )
, ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL_DN
) , ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_DN ) , ( ZC_EVENT_ALL_UP ) , (
ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , (
ZC_EVENT_ALL ) , ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) ,
( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , (
ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , (
ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_DN )
, ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_DN ) , (
ZC_EVENT_ALL ) , ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_DN ) , ( ZC_EVENT_ALL )
} ; static ssNonContDerivSigInfo nonContDerivSigInfo [ 8 ] = { { 1 * sizeof (
real_T ) , ( char * ) ( & rtB . c12vlekqly ) , ( NULL ) } , { 1 * sizeof (
real_T ) , ( char * ) ( & rtB . a4jldatvmm ) , ( NULL ) } , { 1 * sizeof (
real_T ) , ( char * ) ( & rtB . onh5wf0mbq ) , ( NULL ) } , { 1 * sizeof (
real_T ) , ( char * ) ( & rtB . lauuzxpsuv ) , ( NULL ) } , { 1 * sizeof (
boolean_T ) , ( char * ) ( & rtB . beyrovclvb ) , ( NULL ) } , { 1 * sizeof (
boolean_T ) , ( char * ) ( & rtB . oov0isvnf1 ) , ( NULL ) } , { 1 * sizeof (
real_T ) , ( char * ) ( & rtB . e4lsqrzqjh ) , ( NULL ) } , { 4 * sizeof (
real_T ) , ( char * ) ( & rtB . nsbfkssthw [ 0 ] ) , ( NULL ) } } ; { int i ;
for ( i = 0 ; i < 32 ; ++ i ) { contStateJacPerturbBoundMinVec [ i ] = 0 ;
contStateJacPerturbBoundMaxVec [ i ] = rtGetInf ( ) ; } } ssSetSolverRelTol (
rtS , 0.001 ) ; ssSetStepSize ( rtS , 0.0 ) ; ssSetMinStepSize ( rtS , 0.0 )
; ssSetMaxNumMinSteps ( rtS , - 1 ) ; ssSetMinStepViolatedError ( rtS , 0 ) ;
ssSetMaxStepSize ( rtS , 0.016666666666666666 ) ; ssSetSolverMaxOrder ( rtS ,
- 1 ) ; ssSetSolverRefineFactor ( rtS , 1 ) ; ssSetOutputTimes ( rtS , ( NULL
) ) ; ssSetNumOutputTimes ( rtS , 0 ) ; ssSetOutputTimesOnly ( rtS , 0 ) ;
ssSetOutputTimesIndex ( rtS , 0 ) ; ssSetZCCacheNeedsReset ( rtS , 1 ) ;
ssSetDerivCacheNeedsReset ( rtS , 0 ) ; ssSetNumNonContDerivSigInfos ( rtS ,
8 ) ; ssSetNonContDerivSigInfos ( rtS , nonContDerivSigInfo ) ;
ssSetSolverInfo ( rtS , & slvrInfo ) ; ssSetSolverName ( rtS , "ode45" ) ;
ssSetVariableStepSolver ( rtS , 1 ) ; ssSetSolverConsistencyChecking ( rtS ,
0 ) ; ssSetSolverAdaptiveZcDetection ( rtS , 0 ) ;
ssSetSolverRobustResetMethod ( rtS , 0 ) ; ssSetAbsTolVector ( rtS , absTol )
; ssSetAbsTolControlVector ( rtS , absTolControl ) ;
ssSetSolverAbsTol_Obsolete ( rtS , absTol ) ;
ssSetSolverAbsTolControl_Obsolete ( rtS , absTolControl ) ;
ssSetJacobianPerturbationBoundsMinVec ( rtS , contStateJacPerturbBoundMinVec
) ; ssSetJacobianPerturbationBoundsMaxVec ( rtS ,
contStateJacPerturbBoundMaxVec ) ; ssSetSolverStateProjection ( rtS , 0 ) ;
ssSetSolverMassMatrixType ( rtS , ( ssMatrixType ) 0 ) ;
ssSetSolverMassMatrixNzMax ( rtS , 0 ) ; ssSetModelOutputs ( rtS , MdlOutputs
) ; ssSetModelLogData ( rtS , rt_UpdateTXYLogVars ) ;
ssSetModelLogDataIfInInterval ( rtS , rt_UpdateTXXFYLogVars ) ;
ssSetModelUpdate ( rtS , MdlUpdate ) ; ssSetModelDerivatives ( rtS ,
MdlDerivatives ) ; ssSetSolverZcSignalAttrib ( rtS , zcAttributes ) ;
ssSetSolverNumZcSignals ( rtS , 56 ) ; ssSetModelZeroCrossings ( rtS ,
MdlZeroCrossings ) ; ssSetSolverConsecutiveZCsStepRelTol ( rtS ,
2.8421709430404007E-13 ) ; ssSetSolverMaxConsecutiveZCs ( rtS , 1000 ) ;
ssSetSolverConsecutiveZCsError ( rtS , 2 ) ; ssSetSolverMaskedZcDiagnostic (
rtS , 1 ) ; ssSetSolverIgnoredZcDiagnostic ( rtS , 1 ) ;
ssSetSolverMaxConsecutiveMinStep ( rtS , 1 ) ;
ssSetSolverShapePreserveControl ( rtS , 2 ) ; ssSetTNextTid ( rtS , INT_MIN )
; ssSetTNext ( rtS , rtMinusInf ) ; ssSetSolverNeedsReset ( rtS ) ;
ssSetNumNonsampledZCs ( rtS , 56 ) ; ssSetContStateDisabled ( rtS ,
contStatesDisabled ) ; ssSetSolverMaxConsecutiveMinStep ( rtS , 1 ) ; }
ssSetChecksumVal ( rtS , 0 , 3787883694U ) ; ssSetChecksumVal ( rtS , 1 ,
3449449585U ) ; ssSetChecksumVal ( rtS , 2 , 185154508U ) ; ssSetChecksumVal
( rtS , 3 , 4030028274U ) ; { static const sysRanDType rtAlwaysEnabled =
SUBSYS_RAN_BC_ENABLE ; static RTWExtModeInfo rt_ExtModeInfo ; static const
sysRanDType * systemRan [ 59 ] ; gblRTWExtModeInfo = & rt_ExtModeInfo ;
ssSetRTWExtModeInfo ( rtS , & rt_ExtModeInfo ) ;
rteiSetSubSystemActiveVectorAddresses ( & rt_ExtModeInfo , systemRan ) ;
systemRan [ 0 ] = & rtAlwaysEnabled ; systemRan [ 1 ] = ( sysRanDType * ) &
rtDW . au2zmey5pc ; systemRan [ 2 ] = ( sysRanDType * ) & rtDW . mirhnoymfvo
. jyd1ppenco ; systemRan [ 3 ] = ( sysRanDType * ) & rtDW . i5wp2ffbry .
jyd1ppenco ; systemRan [ 4 ] = ( sysRanDType * ) & rtDW . byzsjh0px3 ;
systemRan [ 5 ] = ( sysRanDType * ) & rtDW . npachunt01 ; systemRan [ 6 ] = (
sysRanDType * ) & rtDW . adhcumf2n5 ; systemRan [ 7 ] = ( sysRanDType * ) &
rtDW . krzqaoxmqz ; systemRan [ 8 ] = ( sysRanDType * ) & rtDW . gkuxqeqcdi ;
systemRan [ 9 ] = ( sysRanDType * ) & rtDW . n5gzkaa0nd ; systemRan [ 10 ] =
( sysRanDType * ) & rtDW . pkklmgxn3w ; systemRan [ 11 ] = ( sysRanDType * )
& rtDW . fvgndvljcd ; systemRan [ 12 ] = ( sysRanDType * ) & rtDW .
ejm2q33rwd ; systemRan [ 13 ] = ( sysRanDType * ) & rtDW . cvvf5wjoub ;
systemRan [ 14 ] = ( sysRanDType * ) & rtDW . jwt5flmp0h ; systemRan [ 15 ] =
( sysRanDType * ) & rtDW . nv0y4bxbiz ; systemRan [ 16 ] = & rtAlwaysEnabled
; systemRan [ 17 ] = & rtAlwaysEnabled ; systemRan [ 18 ] = & rtAlwaysEnabled
; systemRan [ 19 ] = ( sysRanDType * ) & rtDW . pdsijyzqws ; systemRan [ 20 ]
= ( sysRanDType * ) & rtDW . b3phozbfji ; systemRan [ 21 ] = ( sysRanDType *
) & rtDW . faa1lc2tvb ; systemRan [ 22 ] = ( sysRanDType * ) & rtDW .
f1edxk4pdy ; systemRan [ 23 ] = ( sysRanDType * ) & rtDW . axi4q1he0b ;
systemRan [ 24 ] = ( sysRanDType * ) & rtDW . ngahvntjaq ; systemRan [ 25 ] =
( sysRanDType * ) & rtDW . drdj3xeohi ; systemRan [ 26 ] = ( sysRanDType * )
& rtDW . mxamlbyck0 ; systemRan [ 27 ] = ( sysRanDType * ) & rtDW .
mxamlbyck0 ; systemRan [ 28 ] = ( sysRanDType * ) & rtDW . fc1bretpi5 ;
systemRan [ 29 ] = ( sysRanDType * ) & rtDW . fc1bretpi5 ; systemRan [ 30 ] =
( sysRanDType * ) & rtDW . fc1bretpi5 ; systemRan [ 31 ] = ( sysRanDType * )
& rtDW . giro4eplfi ; systemRan [ 32 ] = ( sysRanDType * ) & rtDW .
giro4eplfi ; systemRan [ 33 ] = ( sysRanDType * ) & rtDW . hudho2xp0v ;
systemRan [ 34 ] = ( sysRanDType * ) & rtDW . hudho2xp0v ; systemRan [ 35 ] =
( sysRanDType * ) & rtDW . hudho2xp0v ; systemRan [ 36 ] = ( sysRanDType * )
& rtDW . i1eyrq4uwi ; systemRan [ 37 ] = ( sysRanDType * ) & rtDW .
aqq0sq0gty . johe0vk44j ; systemRan [ 38 ] = ( sysRanDType * ) & rtDW .
aqq0sq0gty . kglzzcbeka ; systemRan [ 39 ] = ( sysRanDType * ) & rtDW .
aqq0sq0gty . ckxwrcsgi3 ; systemRan [ 40 ] = ( sysRanDType * ) & rtDW .
aqq0sq0gty . meuzevkn2y ; systemRan [ 41 ] = ( sysRanDType * ) & rtDW .
aqq0sq0gty . biatop040u ; systemRan [ 42 ] = ( sysRanDType * ) & rtDW .
aqq0sq0gty . jijajg5nkb ; systemRan [ 43 ] = ( sysRanDType * ) & rtDW .
cjigmwyptgo . johe0vk44j ; systemRan [ 44 ] = ( sysRanDType * ) & rtDW .
cjigmwyptgo . kglzzcbeka ; systemRan [ 45 ] = ( sysRanDType * ) & rtDW .
cjigmwyptgo . ckxwrcsgi3 ; systemRan [ 46 ] = ( sysRanDType * ) & rtDW .
cjigmwyptgo . meuzevkn2y ; systemRan [ 47 ] = ( sysRanDType * ) & rtDW .
cjigmwyptgo . biatop040u ; systemRan [ 48 ] = ( sysRanDType * ) & rtDW .
cjigmwyptgo . jijajg5nkb ; systemRan [ 49 ] = ( sysRanDType * ) & rtDW .
ojqxnfreck ; systemRan [ 50 ] = ( sysRanDType * ) & rtDW . dd1fatoipk ;
systemRan [ 51 ] = ( sysRanDType * ) & rtDW . dnxut4pgtq ; systemRan [ 52 ] =
( sysRanDType * ) & rtDW . l3k54bxdj1 ; systemRan [ 53 ] = ( sysRanDType * )
& rtDW . p4vcgbddhj ; systemRan [ 54 ] = ( sysRanDType * ) & rtDW .
poolnl3e1h ; systemRan [ 55 ] = & rtAlwaysEnabled ; systemRan [ 56 ] = &
rtAlwaysEnabled ; systemRan [ 57 ] = & rtAlwaysEnabled ; systemRan [ 58 ] = &
rtAlwaysEnabled ; rteiSetModelMappingInfoPtr ( ssGetRTWExtModeInfo ( rtS ) ,
& ssGetModelMappingInfo ( rtS ) ) ; rteiSetChecksumsPtr ( ssGetRTWExtModeInfo
( rtS ) , ssGetChecksums ( rtS ) ) ; rteiSetTPtr ( ssGetRTWExtModeInfo ( rtS
) , ssGetTPtr ( rtS ) ) ; } rtP . NonlinearSecondOrderActuator1_fin_maxrate =
rtInf ; rtP . NonlinearSecondOrderActuator_fin_maxrate = rtInf ; rtP .
NonlinearSecondOrderActuator2_fin_maxrate = rtInf ; rtP . uftinf_UpperSat =
rtInf ; return rtS ; }
#if defined(_MSC_VER)
#pragma optimize( "", on )
#endif
const int_T gblParameterTuningTid = 5 ; void MdlOutputsParameterSampleTime (
int_T tid ) { MdlOutputsTID5 ( tid ) ; }
