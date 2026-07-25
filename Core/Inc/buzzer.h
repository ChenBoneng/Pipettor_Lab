#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

#define NOTE_REST 0U

typedef struct
{
  uint32_t freqHz;
  uint32_t durationMs;
} Note_t;

#endif /* BUZZER_H */
