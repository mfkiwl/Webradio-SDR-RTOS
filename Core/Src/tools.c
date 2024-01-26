#include <tools.h>


#ifdef TOOLS_LTOA
char* ltoa(long val, char *buf, int radix)
{
	char *p;
	unsigned long u;

	p = buf;
	if(radix == 0)
	{
		radix = 10;
	}
	if(buf == NULL)
	{
		return NULL;
	}
	if(val < 0)
	{
		*p++ = '-';
		u = -val;
	}
	else
	{
		u = val;
	}
	ultoa(u, p, radix);

	return buf;
}
#endif


#ifdef TOOLS_ULTOA
char* ultoa(unsigned long val, char *buf, int radix)
{
	char *s, *p;
	s = "0123456789abcdefghijklmnopqrstuvwxyz";

	if(radix == 0)
	{
		radix = 10;
	}
	if(buf == NULL)
	{
		return NULL;
	}
	if(val < (unsigned)radix)
	{
		buf[0] = s[val];
		buf[1] = '\0';
	}
	else
	{
		for(p = ultoa(val / ((unsigned)radix), buf, radix); *p; p++);
		ultoa(val % ((unsigned)radix), p, radix);
	}
	return buf;
}
#endif




