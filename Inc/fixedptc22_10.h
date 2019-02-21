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


#define FIXEDPT_ONE2210	((fixedpt)((fixedpt)1 << FIXEDPT_FBITS2210))
static inline fixedpt
fixedpt_div2210(fixedpt A, fixedpt B)
{
	return (((fixedptd)A << FIXEDPT_FBITS2210) / (fixedptd)B);
}


static inline fixedpt
fixedpt_sqrt2210(fixedpt A)
{
	int invert = 0;
	int iter = FIXEDPT_FBITS2210;
	int l, i;

	if (A < 0)
		return (-1);
	if (A == 0 || A == FIXEDPT_ONE2210)
		return (A);
	if (A < FIXEDPT_ONE2210 && A > 6) {
		invert = 1;
		A = fixedpt_div2210(FIXEDPT_ONE2210, A);
	}
	if (A > FIXEDPT_ONE2210) {
		int s = A;

		iter = 0;
		while (s > 0) {
			s >>= 2;
			iter++;
		}
	}

	/* Newton's iterations */
	l = (A >> 1) + 1;
	for (i = 0; i < iter; i++)
		l = (l + fixedpt_div2210(A, l)) >> 1;
	if (invert)
		return (fixedpt_div2210(FIXEDPT_ONE2210, l));
	return (l);
}


#endif
