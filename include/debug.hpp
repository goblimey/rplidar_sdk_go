#ifndef _DEBUG_HPP_
#define _DEBUG_HPP_

static inline void debug(char* s) {
#ifdef DEBUG
    printf("DEBUG %s\n", s);
#endif
}

static inline void debug(char* s1, char* s2) {
#ifdef DEBUG
    printf("DEBUG %s %s\n", s1, s2);
#endif
}

static inline void debug(char* s1, char* s2, char* s3) {
#ifdef DEBUG
    printf("DEBUG %s %s %s\n", s1, s2, s3);
#endif
}

static inline void debug(char* s1, char* s2, char* s3, char* s4) {
#ifdef DEBUG
    printf("DEBUG %s %s %s %s\n", s1, s2, s3, s4);
#endif
}

static inline void debug(char* s1, char* s2, int n) {
#ifdef DEBUG
    printf("DEBUG %s %s %d\n", s1, s2, n);
#endif
}

static inline void debug(char* s1, char* s2, int m, int n) {
#ifdef DEBUG
    printf("DEBUG %s %s %d %d\n", s1, s2, m, n);
#endif
}

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