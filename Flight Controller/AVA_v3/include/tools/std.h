#ifndef STD_H
#define STD_H

#include <inttypes.h>
#include <stdbool.h>
#include <math.h>


/* stringify a define, e.g. one that was not quoted */
#define _STRINGIFY(s) #s
#define STRINGIFY(s) _STRINGIFY(s)

#define PTR(_f) &_f

#ifndef FALSE
#define FALSE false
#endif
#ifndef TRUE
#define TRUE true
#endif

#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *)0)
#endif
#endif

/* Unit (void) values */
typedef uint8_t unit_t;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_4
#define M_PI_4 (M_PI/4)
#endif

#ifndef M_PI_2
#define M_PI_2 (M_PI/2)
#endif


#ifndef bit_is_set
#define bit_is_set(x, b) ((x >> b) & 0x1)
#endif

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

#define SetBit(a, n) a |= (1 << n)
#define ClearBit(a, n) a &= ~(1 << n)

#define NormRadAngle(x) { \
    while (x > M_PI) x -= 2 * M_PI; \
    while (x < -M_PI) x += 2 * M_PI; \
  }
#define DegOfRad(x) ((x) * (180. / M_PI))
#define DeciDegOfRad(x) ((x) * (1800./ M_PI))
#define RadOfDeg(x) ((x) * (M_PI/180.))
#define RadOfDeciDeg(x) ((x) * (M_PI/1800.))

#define MOfCm(_x) (((float)(_x))/100.)
#define MOfMm(_x) (((float)(_x))/1000.)

#define Min(x,y) (x < y ? x : y)
#define Max(x,y) (x > y ? x : y)

#ifndef ABS
#define ABS(val) ((val) < 0 ? -(val) : (val))
#endif

#define BoundUpper(_x, _max) { if (_x > (_max)) _x = (_max);}


#define Bound(_x, _min, _max) { if (_x > (_max)) _x = (_max); else if (_x < (_min)) _x = (_min); }
#define BoundInverted(_x, _min, _max) {           \
    if ((_x < (_min)) && (_x > (_max))) {         \
      if (abs(_x - (_min)) < abs(_x - (_max)))    \
        _x = (_min);                              \
      else                                        \
        _x = (_max);                              \
    }                                             \
  }
#define BoundWrapped(_x, _min, _max) {            \
    if ((_max) > (_min))                          \
      Bound(_x, _min, _max)                       \
      else                                        \
        BoundInverted(_x, _min, _max)             \
      }
#define BoundAbs(_x, _max) Bound(_x, -(_max), (_max))
#define Clip(_x, _min, _max) ( (_x) < (_min) ? (_min) : (_x) > (_max) ? (_max) : (_x) )
#define ClipAbs(x, max) Clip(x, -(max), (max))
// Align makes the value of x a multiple of a1
#define Align(_x, _a1) (_x%_a1 ? _x + (_a1 - (_x%_a1)) : _x )

#define DeadBand(_x, _v) {            \
    if (_x > (_v))                    \
      _x = _x -(_v);                  \
    else if  (_x < -(_v))             \
      _x = _x +(_v);                  \
    else                              \
      _x = 0;                         \
  }

#define Blend(a, b, rho) (((rho)*(a))+(1-(rho))*(b))


static inline bool str_equal(const char *a, const char *b)
{
  int i = 0;
  while (!(a[i] == 0 && b[i] == 0)) {
    if (a[i] != b[i]) { return FALSE; }
    i++;
  }
  return TRUE;
}

#ifdef __GNUC__
#  define UNUSED __attribute__((__unused__))
#  define WEAK __attribute__((weak))
#else
#  define UNUSED
#  define WEAK
#endif

#if __GNUC__ >= 7
#  define INTENTIONAL_FALLTHRU __attribute__ ((fallthrough));
#else
#  define INTENTIONAL_FALLTHRU
#endif

#endif /* STD_H */
