#ifndef _FIXEDPTC2210_H_
#define _FIXEDPTC2210_H_

#ifndef FIXEDPT_WBITS2210
#define FIXEDPT_WBITS2210	22
#endif

#define FIXEDPT_FBITS2210	(FIXEDPT_BITS - FIXEDPT_WBITS2210)
#define FIXEDPT_FMASK2210	(((fixedpt)1 << FIXEDPT_FBITS2210) - 1)

#define fixedpt_fromint2210(I) ((fixedptd)(I) << FIXEDPT_FBITS2210)
#define fixedptu_fromint2210(I) ((fixedptud)(I) << FIXEDPT_FBITS2210)
#define fixedpt_toint2210(F) ((F) >> FIXEDPT_FBITS2210)
#define fixedpt_xmul2210(A,B)	((fixedpt)(((fixedptd)(A) * (fixedptd)(B)) >> FIXEDPT_FBITS2210))
#define fixedpt_xdiv2210(A,B)	((fixedpt)(((fixedptd)(A) << FIXEDPT_FBITS2210) / (fixedptd)(B)))
#define fixedpt_fracpart2210(A) ((fixedpt)(A) & FIXEDPT_FMASK2210)
#define fixedpt_tofloat2210(T) ((float) ((T)*((float)(1)/(float)(1 << FIXEDPT_FBITS2210))))

#endif
