#ifndef _DEBUG_HPP_
#define _DEBUG_HPP_

static inline void debug(bool verbose, const char* s) {
    if ( verbose) {
        printf("DEBUG %s\n", s);
    }
}

static inline void debug(bool verbose, const char* s1, const char* s2) {
    if ( verbose) {
        printf("DEBUG %s %s\n", s1, s2);
    }
}

static inline void debug(bool verbose, const char* s1, const char* s2, const char* s3) {
    if ( verbose) {
        printf("DEBUG %s %s %s\n", s1, s2, s3);
    }
}

static inline void debug(bool verbose, const char* s1, const char* s2, const char* s3, const char* s4) {
    if ( verbose) {
        printf("DEBUG %s %s %s %s\n", s1, s2, s3, s4);
    }
}

static inline void debug(bool verbose, const char* s1, const char* s2, const char* s3, const char* s4, const char* s5) {
    if ( verbose) {
        printf("DEBUG %s %s %s %s %s\n", s1, s2, s3, s4, s5);
    }
}

static inline void debug(bool verbose, const char* s1, const char* s2, const char* s3, const char* s4, const char* s5, const char* s6) {
    if ( verbose) {
        printf("DEBUG %s %s %s %s %s %s\n", s1, s2, s3, s4, s5, s6);
    }
}

static inline void debug(bool verbose, const char* s1, const char* s2, const char* s3, const char* s4, const char* s5, const char* s6, const char* s7) {
    if ( verbose) {
        printf("DEBUG %s %s %s %s %s %s %s\n", s1, s2, s3, s4, s5, s6, s7);
    }
}

static inline void debug(bool verbose, const char* s1, const char* s2, const char* s3, const char* s4, const char* s5, const char* s6, const char* s7, const char * s8) {
    if ( verbose) {
        printf("DEBUG %s %s %s %s %s %s %s %s\n", s1, s2, s3, s4, s5, s6, s7, s8);
    }
}
static inline void debug(bool verbose, const char* s1, const char* s2, const char* s3, const char* s4, const char* s5, const char* s6, const char* s7, const char* s8, const char* s9) {
    if ( verbose) {
        printf("DEBUG %s %s %s %s %s %s %s %s %s\n", s1, s2, s3, s4, s5, s6, s7, s8, s9);
    }
}

static inline void debug(bool verbose, const char* s1, const char* s2, int n) {
    if ( verbose) {
        printf("DEBUG %s %s %d\n", s1, s2, n);
    }
}

static inline void debug(bool verbose, const char* s1, const char* s2, int m, int n) {
    if ( verbose) {
        printf("DEBUG %s %s %d %d\n", s1, s2, m, n);
    }
}

static inline const char* debugInt2String(bool verbose, int i) {
    if ( verbose) {
        // In debug mode this will malloc space which the caller probably will not free, thus leaking a small
        // amount of memory.  Out of debug mode, it's OK.
        int length = snprintf(NULL, 0, "%d", i);
        char* str = (char*)malloc( length + 1 );
        snprintf( str, length + 1, "%d", i);
        return (const char*)str;
    } else {
        return NULL;
    }
}

static inline const char* debugInt2HexString(bool verbose, int i) {
    if ( verbose) {
        int length = snprintf(NULL, 0, "0x%x", i);
        char* str = (char*)malloc( length + 1 );
        snprintf( str, length + 1, "0x%x", i);
        return str;
    } else {
        return NULL;
    }
}

#endif