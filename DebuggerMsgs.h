/*!
 *  @file DebuggerMsgs.h
 *
 * 	Arduino library for the RoboHeart.
 *
 */

#ifndef DebuggerMsgs_h
#define DebuggerMsgs_h

#ifndef FILE_IDENTIFIER
#define FILE_IDENTIFIER ""
#endif  // FILE_IDENTIFIER

#define PRINT_IDENTIFIER(dbg_identifier) \
    {                                    \
        _debug->print("[");              \
        _debug->print(dbg_identifier);   \
        _debug->print("_DBG] ");         \
    }
#define DEBUG_IDENTIFIER(x)                   \
    {                                         \
        if (_debug != NULL) {                 \
            PRINT_IDENTIFIER(FILE_IDENTIFIER) \
            _debug->print(x);                 \
        }                                     \
    }
#define DEBUG_LN_IDENTIFIER(x)                \
    {                                         \
        if (_debug != NULL) {                 \
            PRINT_IDENTIFIER(FILE_IDENTIFIER) \
            _debug->println(x);               \
        }                                     \
    }

#define DEBUG(x)              \
    {                         \
        if (_debug != NULL) { \
            _debug->print(x); \
        }                     \
    }
#define DEBUG_LN(x)             \
    {                           \
        if (_debug != NULL) {   \
            _debug->println(x); \
        }                       \
    }

#endif  // DebuggerMsgs_h