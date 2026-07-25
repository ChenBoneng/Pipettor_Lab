//
// Created by XunWei on 26-5-5.
//

#ifndef MUSIC_H
#define MUSIC_H

#include "buzzer.h"

#define MUSIC_NOTE_END 0xFFFFU

#define MUSIC_G4  392U
#define MUSIC_AB4 415U
#define MUSIC_A4  440U
#define MUSIC_BB4 466U
#define MUSIC_B4  494U
#define MUSIC_C5  523U
#define MUSIC_DB5 554U
#define MUSIC_D5  587U
#define MUSIC_EB5 622U
#define MUSIC_E5  659U
#define MUSIC_F5  698U
#define MUSIC_FS5 740U
#define MUSIC_G5  784U
#define MUSIC_AB5 831U
#define MUSIC_A5  880U
#define MUSIC_BB5 932U
#define MUSIC_B5  988U
#define MUSIC_C6  1047U
#define MUSIC_DB6 1109U
#define MUSIC_D6  1175U
#define MUSIC_EB6 1245U
#define MUSIC_E6  1319U
#define MUSIC_F6  1397U

#define _1  MUSIC_G4
#define _B2 MUSIC_AB4
#define _2  MUSIC_A4
#define _B3 MUSIC_BB4
#define _3  MUSIC_B4
#define _4  MUSIC_C5
#define _B5 MUSIC_DB5
#define _5  MUSIC_D5
#define _B6 MUSIC_EB5
#define _6  MUSIC_E5
#define _7  MUSIC_FS5
#define _H1 MUSIC_G5
#define _HB2 MUSIC_AB5
#define _H2 MUSIC_A5
#define _HB3 MUSIC_BB5
#define _H3 MUSIC_B5
#define _H4 MUSIC_C6
#define _HB5 MUSIC_DB6
#define _H5 MUSIC_D6
#define _HB6 MUSIC_EB6
#define _H6 MUSIC_E6
#define _H7 MUSIC_F6

#define N4(freq) {freq, 320}, {NOTE_REST, 35}
#define N8(freq) {freq, 160}, {NOTE_REST, 25}
#define N16(freq) {freq, 80}, {NOTE_REST, 15}
#define D4(freq) {freq, 640}, {NOTE_REST, 45}
#define R4      {NOTE_REST, 320}
#define R8      {NOTE_REST, 160}
#define R16     {NOTE_REST, 80}

static const Note_t SeeYouAgain_Intro_x2[] = {
    N4(_5), N4(_H2), N4(_H1), D4(_5),
    N16(_H1), N16(_H2), N16(_H3), N16(_H2), N16(_H1), N16(_H2),

    N4(_5), N4(_H2), N4(_H1), D4(_5),
    N16(_H1), N16(_H2), N16(_H3), N16(_H2), N16(_H1), N16(_H2),

    N4(_5), N4(_H2), N4(_H1), D4(_5),
    N16(_H1), N16(_H2), N16(_H3), N16(_H2), N16(_H1), N16(_H2),

    N4(_5), N4(_H2), N4(_H1), N4(_5), N4(_5), R4,

    {MUSIC_NOTE_END, 0}
};

static const Note_t SeeYouAgain[] = {
    N4(_5), N4(_H2), N4(_H1), N4(_5), N8(_5),
    N16(_H1), N16(_H2), N16(_H3), N16(_H2), N16(_H1), N16(_H2),
    N4(_5), N4(_H2), N4(_H1), N4(_5), N8(_5),
    N16(_H1), N16(_H2), N16(_H3), N16(_H2), N16(_H1), N16(_H2),
    N4(_5), N4(_H2), N4(_H1), N4(_5), N8(_5),
    N16(_H1), N16(_H2), N16(_H3), N16(_H2), N16(_H1), N16(_H2),
    N4(_5), N4(_H2), N4(_H1), N4(_5), N4(_5), N4(_1), N4(_3), N4(_5),

    D4(_6), N4(_5), N4(_5), R8, N4(_H1),
    N4(_2), N4(_2), N8(_1), N8(_2), N4(_3), R8, N8(_3), N4(_5),
    N4(_6), N4(_7), N4(_6), N4(_5), N4(_3), N4(_2), N4(_1),

    N4(_2), N4(_2), N8(_2), N8(_1), R4, R8, N4(_1),
    N4(_3), N4(_5), D4(_6), N4(_5), N4(_5), R8, N4(_H1),
    N4(_2), N4(_2), N8(_1), N8(_3), R8, N8(_2), N8(_3), N4(_5),

    N4(_6), N4(_H1), N4(_H2), N4(_H3), N4(_H2), N4(_H1),
    N4(_5), N4(_6), N4(_H1), N4(_H3), N4(_H3), N4(_H3), N4(_H2), R4,
    N4(_5), N4(_6), N4(_H1), N4(_H3), N4(_H3), N4(_H3), N4(_H2), R4, R4,

    R4, R4, R4, R4, R4, R4, R4, R4,
    N8(_H1), N8(_H1), N8(_H1), N8(_H3),
    N8(_H3), N8(_H1), N8(_H1),
    N8(_H1), N8(_H1), N8(_H1),
    N8(_H1), N8(_H1), N8(_H1), N8(_6), N8(_H3),

    N8(_H1), N8(_H1), N8(_H1),
    N8(_H1), N8(_H1), N8(_H1),
    N8(_H1), N8(_H1), N8(_H1),
    N8(_H1), N8(_H1), N8(_H1), D4(_6), N4(_6),

    N8(_H3), N8(_H1), N8(_H1), N8(_H1),
    N8(_H1), N8(_H1), N8(_H1), N8(_H1),
    N8(_H1), N8(_H1), N4(_3), N4(_5),
    D4(_6), N4(_5), N4(_5), R8, N4(_H1),

    N4(_2), N4(_2), N8(_1), N8(_2), N4(_3), R8, N4(_3), N4(_5),
    N4(_6), N4(_7), N4(_6), N4(_5), N4(_3), N4(_2), N4(_1),

    N4(_2), N4(_2), N8(_2), N8(_1), R4, R8, N4(_1),
    N4(_3), N4(_5), D4(_6), N4(_5), N4(_5), R8, N4(_H1),
    N4(_2), N4(_2), N8(_1), N8(_3), R8, N8(_2), N8(_3), N4(_5),

    N4(_6), N4(_H1), N4(_H2), N4(_H3), N4(_H2), N4(_H1),
    N4(_5), N4(_6), N4(_H1), N4(_H3), N4(_H3), N4(_H3), N4(_H2), R4,
    N4(_5), N4(_6), N4(_H1), N4(_H3), N4(_H3), N4(_H3), N4(_H2), R4,
    N4(_H1), N4(_7), D4(_6), D4(_5), N4(_H1), N4(_7),
    D4(_6), N4(_7), N4(_6), N4(_5), N4(_3), R4,
    R4, R4, R4, R4, R4, R4, R4, R4,

    {MUSIC_NOTE_END, 0}
};
static const Note_t Ni[] = {
    N8(_H1), N8(_H1), N8(_H1), N8(_H2), N8(_H3), N8(_H2), N8(_H1), N8(_H2),
    N8(_H1), N8(_6), N8(_5), N8(_6), N8(_H1), N8(_H2), N8(_H3), N8(_H2),
    N8(_H1), N8(_H1), N8(_H1), N8(_H2), N8(_H3), N8(_H2), N8(_H1), N8(_H2),
    N8(_H1), N8(_6), N8(_5), N8(_6), N4(_H1), R4,

    N8(_H3), N8(_H3), N8(_H2), N8(_H1), N8(_H2), N8(_H1), N8(_6), N8(_5),
    N8(_6), N8(_H1), N8(_H2), N8(_H3), N4(_H2), N4(_H1),
    N8(_H3), N8(_H3), N8(_H2), N8(_H1), N8(_H2), N8(_H1), N8(_6), N8(_5),
    N8(_6), N8(_H1), N8(_H2), N8(_H3), N4(_H2), R4,

    N8(_H1), N8(_H2), N8(_H3), N8(_H5), N8(_H3), N8(_H2), N8(_H1), N8(_6),
    N8(_H1), N8(_H2), N8(_H3), N8(_H5), N4(_H3), N4(_H2),
    N8(_H1), N8(_H2), N8(_H3), N8(_H5), N8(_H3), N8(_H2), N8(_H1), N8(_6),
    N8(_H1), N8(_H2), N8(_H3), N8(_H5), N4(_H3), R4,

    N8(_H5), N8(_H5), N8(_H3), N8(_H2), N8(_H1), N8(_H2), N8(_H3), N8(_H2),
    N8(_H1), N8(_6), N8(_5), N8(_6), N4(_H1), N4(_H2),
    N8(_H3), N8(_H3), N8(_H2), N8(_H1), N8(_H2), N8(_H1), N8(_6), N8(_5),
    N8(_6), N8(_H1), N8(_H2), N8(_H3), D4(_H2), N4(_H1),

    N8(_H1), N8(_H1), N8(_H2), N8(_H3), N8(_H5), N8(_H3), N8(_H2), N8(_H1),
    N8(_6), N8(_H1), N8(_H2), N8(_H3), N4(_H2), N4(_H1),
    N8(_H1), N8(_H1), N8(_H2), N8(_H3), N8(_H5), N8(_H3), N8(_H2), N8(_H1),
    N8(_6), N8(_H1), N8(_H2), N8(_H3), D4(_H2), R4,

    N8(_H3), N8(_H5), N8(_H6), N8(_H5), N8(_H3), N8(_H2), N8(_H1), N8(_H2),
    N8(_H3), N8(_H5), N8(_H6), N8(_H5), N4(_H3), N4(_H2),
    N8(_H1), N8(_H2), N8(_H3), N8(_H5), N8(_H3), N8(_H2), N8(_H1), N8(_6),
    N4(_H1), N4(_H2), D4(_H1), R4,

    {MUSIC_NOTE_END, 0}
};



#undef N4
#undef N8
#undef N16
#undef D4
#undef R4
#undef R8
#undef R16

#endif //MUSIC_H
