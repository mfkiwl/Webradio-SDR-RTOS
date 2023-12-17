#ifndef INC_TOOLS_H_
#define INC_TOOLS_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>




#ifndef ltoa
#define TOOLS_LTOA
char* ltoa(long val, char *buf, int radix);
#endif

#ifndef ultoa
# define TOOLS_ULTOA
char* ultoa(unsigned long val, char *buf, int radix);
#endif

#endif /* INC_TOOLS_H_ */
