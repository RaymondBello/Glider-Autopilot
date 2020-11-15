#include "ext_types.h"
static DataTypeInfo rtDataTypeInfoTable [ ] = { { "real_T" , 0 , 8 } , {
"real32_T" , 1 , 4 } , { "int8_T" , 2 , 1 } , { "uint8_T" , 3 , 1 } , {
"int16_T" , 4 , 2 } , { "uint16_T" , 5 , 2 } , { "int32_T" , 6 , 4 } , {
"uint32_T" , 7 , 4 } , { "boolean_T" , 8 , 1 } , { "fcn_call_T" , 9 , 0 } , {
"int_T" , 10 , 4 } , { "pointer_T" , 11 , 8 } , { "action_T" , 12 , 8 } , {
"timer_uint32_pair_T" , 13 , 8 } , { "struct_zA9FapfXhrKibafsqrtxI" , 14 ,
25769992 } } ; static uint_T rtDataTypeSizes [ ] = { sizeof ( real_T ) ,
sizeof ( real32_T ) , sizeof ( int8_T ) , sizeof ( uint8_T ) , sizeof (
int16_T ) , sizeof ( uint16_T ) , sizeof ( int32_T ) , sizeof ( uint32_T ) ,
sizeof ( boolean_T ) , sizeof ( fcn_call_T ) , sizeof ( int_T ) , sizeof (
pointer_T ) , sizeof ( action_T ) , 2 * sizeof ( uint32_T ) , sizeof (
struct_zA9FapfXhrKibafsqrtxI ) } ; static const char_T * rtDataTypeNames [ ]
= { "real_T" , "real32_T" , "int8_T" , "uint8_T" , "int16_T" , "uint16_T" ,
"int32_T" , "uint32_T" , "boolean_T" , "fcn_call_T" , "int_T" , "pointer_T" ,
"action_T" , "timer_uint32_pair_T" , "struct_zA9FapfXhrKibafsqrtxI" } ;
static DataTypeTransition rtBTransitions [ ] = { { ( char_T * ) ( & rtB .
pkgvmqtajm [ 0 ] ) , 0 , 0 , 150 } , { ( char_T * ) ( & rtB . prdmace1ry ) ,
7 , 0 , 1 } , { ( char_T * ) ( & rtB . h1fm141rey ) , 1 , 0 , 3 } , { (
char_T * ) ( & rtB . fcdyl4g5vg [ 0 ] ) , 3 , 0 , 408 } , { ( char_T * ) ( &
rtB . oov0isvnf1 ) , 8 , 0 , 6 } , { ( char_T * ) ( & rtB . i5wp2ffbry .
cwtrsfu0ve ) , 0 , 0 , 1 } , { ( char_T * ) ( & rtB . mirhnoymfvo .
cwtrsfu0ve ) , 0 , 0 , 1 } , { ( char_T * ) ( & rtDW . dh0tof3kep [ 0 ] ) , 0
, 0 , 44 } , { ( char_T * ) ( & rtDW . jqnbqtb4sh . LoggedData ) , 11 , 0 , 6
} , { ( char_T * ) ( & rtDW . bu3swunyz2 ) , 6 , 0 , 7 } , { ( char_T * ) ( &
rtDW . mjfthrgmwy ) , 7 , 0 , 7 } , { ( char_T * ) ( & rtDW . afe03te4gm ) ,
10 , 0 , 17 } , { ( char_T * ) ( & rtDW . bh43qqaomf ) , 2 , 0 , 63 } , { (
char_T * ) ( & rtDW . b0fymed3im [ 0 ] ) , 3 , 0 , 20 } , { ( char_T * ) ( &
rtDW . es24y1hork ) , 8 , 0 , 20 } , { ( char_T * ) ( & rtDW . aqq0sq0gty .
czibmh2e1r [ 0 ] ) , 0 , 0 , 1665755 } , { ( char_T * ) ( & rtDW . aqq0sq0gty
. js2t0wh1ap [ 0 ] ) , 11 , 0 , 90 } , { ( char_T * ) ( & rtDW . aqq0sq0gty .
pkty1tadnq [ 0 ] ) , 7 , 0 , 15 } , { ( char_T * ) ( & rtDW . aqq0sq0gty .
jijajg5nkb ) , 2 , 0 , 7 } , { ( char_T * ) ( & rtDW . aqq0sq0gty .
dgmohvowou ) , 3 , 0 , 5 } , { ( char_T * ) ( & rtDW . cjigmwyptgo .
czibmh2e1r [ 0 ] ) , 0 , 0 , 1665755 } , { ( char_T * ) ( & rtDW .
cjigmwyptgo . js2t0wh1ap [ 0 ] ) , 11 , 0 , 90 } , { ( char_T * ) ( & rtDW .
cjigmwyptgo . pkty1tadnq [ 0 ] ) , 7 , 0 , 15 } , { ( char_T * ) ( & rtDW .
cjigmwyptgo . jijajg5nkb ) , 2 , 0 , 7 } , { ( char_T * ) ( & rtDW .
cjigmwyptgo . dgmohvowou ) , 3 , 0 , 5 } , { ( char_T * ) ( & rtDW .
i5wp2ffbry . kaoqin3vcx ) , 10 , 0 , 1 } , { ( char_T * ) ( & rtDW .
i5wp2ffbry . jyd1ppenco ) , 2 , 0 , 1 } , { ( char_T * ) ( & rtDW .
i5wp2ffbry . ltfrc3b41y ) , 8 , 0 , 1 } , { ( char_T * ) ( & rtDW .
mirhnoymfvo . kaoqin3vcx ) , 10 , 0 , 1 } , { ( char_T * ) ( & rtDW .
mirhnoymfvo . jyd1ppenco ) , 2 , 0 , 1 } , { ( char_T * ) ( & rtDW .
mirhnoymfvo . ltfrc3b41y ) , 8 , 0 , 1 } } ; static DataTypeTransitionTable
rtBTransTable = { 31U , rtBTransitions } ; static DataTypeTransition
rtPTransitions [ ] = { { ( char_T * ) ( & rtP . LatLong0 [ 0 ] ) , 0 , 0 ,
3221776 } , { ( char_T * ) ( & rtP .
FlightGearPreconfigured6DoFAnimation_DestinationPort ) , 6 , 0 , 3 } , { (
char_T * ) ( & rtP . x_Y0 ) , 0 , 0 , 7469 } , { ( char_T * ) ( & rtP .
MediumHighAltitudeIntensity_maxIndex [ 0 ] ) , 7 , 0 , 60 } , { ( char_T * )
( & rtP . Packnet_fdmPacketforFlightGear_P1 [ 0 ] ) , 5 , 0 , 19 } , { (
char_T * ) ( & rtP . aqq0sq0gty . M0_maxIndex [ 0 ] ) , 7 , 0 , 50 } , { (
char_T * ) ( & rtP . cjigmwyptgo . M0_maxIndex [ 0 ] ) , 7 , 0 , 50 } , { (
char_T * ) ( & rtP . i5wp2ffbry . x_Y0 ) , 0 , 0 , 3 } , { ( char_T * ) ( &
rtP . mirhnoymfvo . x_Y0 ) , 0 , 0 , 3 } } ; static DataTypeTransitionTable
rtPTransTable = { 9U , rtPTransitions } ;
