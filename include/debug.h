#ifndef _DEBUG_H_
#define _DEBUG_H_

#ifdef DEBUG
#define _D_ 1
#else
#define _D_ 0
#endif

#define debugs(s) if (_D_ == 1) printf("DEBUG %s\n", s);
#define debugss(s1, s2) if (_D_ == 1) printf("DEBUG %s %s\n", s1, s2);
#define debugsss(s1, s2, s3) if (_D_ == 1) printf("DEBUG %s %s %s\n", s1, s2, s3);
#define debugssi(s1, s2, n) if (_D_ == 1) printf("DEBUG %s %s %d\n", s1, s2, n);
#define debugssss(s1, s2, s3, s4) if (_D_ == 1) printf("DEBUG %s %s %s %s\n", s1, s2, s3, s4);

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