#ifndef LOG_h
#define LOG_h

#define DEBUG

#ifdef DEBUG
  #define DEBUG_WRITE(x) Serial.write(x)
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_FLUSH() Serial.flush()
  #define DEBUG_PROGRESS(x) showProgress(x)
#else
  #define DEBUG_WRITE(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_FLUSH()
  #define DEBUG_PROGRESS(x)
#endif

#endif
