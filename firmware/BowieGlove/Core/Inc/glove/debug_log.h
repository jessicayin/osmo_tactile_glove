#ifndef INC_GLOVE_DEBUG_LOG_H_
#define INC_GLOVE_DEBUG_LOG_H_

#include <stdio.h>

/*
 * Production firmware must not use the same CDC endpoint for text and binary
 * COBS frames.  Set BOWIE_DEBUG_PRINT to 1 only in a dedicated diagnostic build.
 */
#ifndef BOWIE_DEBUG_PRINT
#define BOWIE_DEBUG_PRINT 0
#endif

#if BOWIE_DEBUG_PRINT
#define BOWIE_LOG(...) printf(__VA_ARGS__)
#else
#define BOWIE_LOG(...) do { } while (0)
#endif

#endif /* INC_GLOVE_DEBUG_LOG_H_ */
