#ifndef _FIXEDPTC1220_H_
#define _FIXEDPTC1220_H_

#ifndef FIXEDPT_WBITS1220
#define FIXEDPT_WBITS1220	12
#endif

#define FIXEDPT_FBITS1220	(FIXEDPT_BITS - FIXEDPT_WBITS1220)
#define FIXEDPT_FMASK1220	(((fixedpt)1 << FIXEDPT_FBITS1220) - 1)

#define fixedpt_fromint1220(I) ((fixedptd)(I) << FIXEDPT_FBITS1220)
#define fixedptu_fromint1220(I) ((fixedptud)(I) << FIXEDPT_FBITS1220)
#define fixedpt_toint1220(F) ((F) >> FIXEDPT_FBITS1220)
#define fixedpt_xmul1220(A,B)	((fixedpt)(((fixedptd)(A) * (fixedptd)(B)) >> FIXEDPT_FBITS1220))
#define fixedpt_xdiv1220(A,B)	((fixedpt)(((fixedptd)(A) << FIXEDPT_FBITS1220) / (fixedptd)(B)))
#define fixedpt_fracpart1220(A) ((fixedpt)(A) & FIXEDPT_FMASK1220)
#define fixedpt_tofloat1220(T) ((float) ((T)*((float)(1)/(float)(1 << FIXEDPT_FBITS1220))))

#endif
