/*
 * Copyright (C) 2026 Artem Bambalov
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * RDS texts -- the station name (PS), the long one (Long PS), the radio
 * text (RT) and the enhanced one (eRT) -- put together from their
 * segments, and turned into UTF-8.
 */

#ifndef RDS_TEXT_H
#define RDS_TEXT_H

#include <stddef.h>
#include <stdint.h>

#define RDS_TEXT_MAX_BYTES  128     /* eRT: 32 segments of 4 */
#define RDS_TEXT_MAX_SEGS   32

/* How a text's bytes are coded */
enum rds_charset {
    RDS_CHARSET_BASIC,      /* the RDS character table (EBU G0) */
    RDS_CHARSET_UTF8,
    RDS_CHARSET_UCS2,       /* big endian */
};

typedef struct rds_text_t {
    int size;                       /* bytes the text may have */
    int per;                        /* bytes per segment */
    int terminated;                 /* may end early, at a 0x0D */
    unsigned char raw[RDS_TEXT_MAX_BYTES];
    uint32_t have;                  /* segments taken, a bit each */
    int len;                        /* up to the 0x0D, -1 if none seen */
    unsigned char cand[RDS_TEXT_MAX_SEGS][4];
    uint32_t cand_have;             /* segments with a candidate */
} rds_text;

/* A text of size bytes in segments of per, empty */
void rds_text_init(rds_text *t, int size, int per, int terminated);

/* Start again: a new text */
void rds_text_clear(rds_text *t);

/*
 * One segment's bytes, from a group the chip received clean, or with a
 * corrected block (suspect). Returns 1 once every segment of the text is
 * in.
 */
int rds_text_put(rds_text *t, int seg, const unsigned char *bytes,
                 int suspect);

/* The text's bytes as they stand: its length, up to the end */
int rds_text_length(const rds_text *t);

/*
 * n bytes of text in a charset to UTF-8 in out (out_size with the NUL),
 * trailing blanks off: the length written. Text in the basic table that
 * is valid UTF-8 with characters beyond ASCII is taken as UTF-8: some
 * stations send it so.
 */
size_t rds_to_utf8(const unsigned char *in, int n, enum rds_charset charset,
                   char *out, size_t out_size);

#endif
