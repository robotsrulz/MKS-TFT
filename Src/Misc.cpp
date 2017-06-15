/*
 * Misc.cpp
 *
 * Created: 14/11/2014 19:58:50
 *  Author: David
 */

#include "ecv.h"
#include "Misc.h"

#include <cstring>
#include <cstdlib>

// Safe version of strncpy that ensures that the destination is always null-terminated on return
void safeStrncpy(char* array dst, const char* array src, size_t n)
{
	while (*src != 0 && n > 1)
	{
		*dst++ = *src++;
		--n;
	}
	*dst = 0;
}

// Return true if string a is the same as or starts with string b
bool stringStartsWith(const char* array a, const char* array b)
{
	while (*b != 0)
	{
		if (*a++ != *b++)
		{
			return false;
		}
	}
	return true;
}

char toupper(char c)
{
    if (('a' <= c) && (c <= 'z'))
        return 'A' + (c - 'a');

    return c;
}

char tolower(char c)
{
    if (('A' <= c) && (c <= 'Z'))
        return 'a' + (c - 'A');

    return c;
}

int strcasecmp (const char *s1, const char *s2)
{
    const unsigned char *p1 = (const unsigned char *) s1;
    const unsigned char *p2 = (const unsigned char *) s2;
    int result;

    if (p1 == p2)
        return 0;

    while ((result = tolower (*p1) - tolower (*p2++)) == 0)
        if (*p1++ == '\0')
            break;

    return result;
}

// End
