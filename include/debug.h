#ifndef _DEBUG_H_
#define _DEBUG_H_

#ifdef DEBUG
#define debug1(str) printf("DEBUG %s\n", str);
#define debug2(s1, s2) printf("DEBUG %s %s\n", s1, s2);
#define debug3(s1, s2, s3) printf("DEBUG %s %s %s\n", s1, s2, s3);
#define debug4(s1, s2, s3, s4) printf("DEBUG %s %s %s %s\n", s1, s2, s3, s4);
#else
#define debug(s)
#define debug2(s1, s2)
#define debug3(s1, s2, s3)
#define debug4(s1, s2, s3, s4)
#endif

static inline char * debugInt2String(int i) {
#ifdef DEBUG
    int length = snprintf(NULL, 0, "%d", i);
    char* str = (char*)malloc( length + 1 );
    snprintf( str, length + 1, "%d", i);
    return str;
#else
    return NULL;
#endif
}

#endif