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
 * RDS texts put together and turned into UTF-8. See rds_text.h.
 *
 * A segment is taken at once from a group the chip received clean. From
 * one with a block the chip corrected, only once the same bytes came
 * twice: the chip's correction (of up to 3 bits) is wrong often enough to
 * show garbage -- on weak stations here, wrong letters and bytes such as
 * 0xa2 and 0xea where clean groups only ever carried ASCII -- and the
 * same wrong correction twice is not to be had. Throwing such groups away
 * would leave a weak station without text: most of them are right.
 */

#include <string.h>

#include "rds_text.h"

/*
 * The RDS character table (EN 50067 Annex E, the one table IEC 62106 has
 * kept), 0x80..0xff, as UTF-8; checked against the RDS Forum's table and
 * redsea's (src/text/rdsstring.cc), which has 0x8d and 0x9d wrong (beta
 * and g with caron, for sharp s and g with breve). 0x20..0x7e are taken
 * as ASCII: the table has a few signs of its own there (currency sign for
 * $, a bar for ^...) that encoders, ASCII ones, do not mean.
 */
static const char *const ebu_high[128] = {
    "á", "à", "é", "è", "í", "ì", "ó", "ò", "ú", "ù", "Ñ", "Ç", "Ş", "ß", "¡", "Ĳ",
    "â", "ä", "ê", "ë", "î", "ï", "ô", "ö", "û", "ü", "ñ", "ç", "ş", "ğ", "ı", "ĳ",
    "ª", "α", "©", "‰", "Ğ", "ě", "ň", "ő", "π", "€", "£", "$", "←", "↑", "→", "↓",
    "º", "¹", "²", "³", "±", "İ", "ń", "ű", "µ", "¿", "÷", "°", "¼", "½", "¾", "§",
    "Á", "À", "É", "È", "Í", "Ì", "Ó", "Ò", "Ú", "Ù", "Ř", "Č", "Š", "Ž", "Ð", "Ŀ",
    "Â", "Ä", "Ê", "Ë", "Î", "Ï", "Ô", "Ö", "Û", "Ü", "ř", "č", "š", "ž", "đ", "ŀ",
    "Ã", "Å", "Æ", "Œ", "ŷ", "Ý", "Õ", "Ø", "Þ", "Ŋ", "Ŕ", "Ć", "Ś", "Ź", "Ŧ", "ð",
    "ã", "å", "æ", "œ", "ŵ", "ý", "õ", "ø", "þ", "ŋ", "ŕ", "ć", "ś", "ź", "ŧ", " ",
};

void rds_text_init(rds_text *t, int size, int per, int terminated)
{
    memset(t, 0, sizeof(*t));
    t->size = size;
    t->per = per;
    t->terminated = terminated;
    rds_text_clear(t);
}

void rds_text_clear(rds_text *t)
{
    memset(t->raw, ' ', sizeof(t->raw));
    t->have = 0;
    t->cand_have = 0;
    t->len = -1;
}

int rds_text_length(const rds_text *t)
{
    return t->len >= 0 ? t->len : t->size;
}

int rds_text_put(rds_text *t, int seg, const unsigned char *bytes,
                 int suspect)
{
    uint32_t bit, need;
    int i, n, segs;

    if (seg < 0 || seg >= RDS_TEXT_MAX_SEGS || (seg + 1) * t->per > t->size)
        return 0;
    bit = 1u << seg;

    if (suspect) {
        if (!(t->cand_have & bit) || memcmp(t->cand[seg], bytes, t->per)) {
            memcpy(t->cand[seg], bytes, t->per);
            t->cand_have |= bit;
            return 0;
        }
    }
    t->cand_have &= ~bit;

    for (i = 0; i < t->per; i++) {
        n = seg * t->per + i;
        if (t->terminated && bytes[i] == 0x0d) {
            t->len = n;
            break;
        }
        if (n == t->len)            /* the end moved on */
            t->len = -1;
        t->raw[n] = bytes[i];
    }
    t->have |= bit;

    /* complete: every segment up to the end, or all of them */
    segs = t->len >= 0 ? t->len / t->per + 1 : t->size / t->per;
    need = segs >= 32 ? 0xffffffffu : (1u << segs) - 1;
    return (t->have & need) == need;
}

/* Length of the UTF-8 sequence at in[0], or 0 if it is not one */
static int utf8_seq(const unsigned char *in, int n)
{
    int len, i;

    if (in[0] < 0x80)
        return 1;
    if ((in[0] & 0xe0) == 0xc0 && in[0] >= 0xc2)
        len = 2;
    else if ((in[0] & 0xf0) == 0xe0)
        len = 3;
    else if ((in[0] & 0xf8) == 0xf0 && in[0] <= 0xf4)
        len = 4;
    else
        return 0;
    if (len > n)
        return 0;
    for (i = 1; i < len; i++)
        if ((in[i] & 0xc0) != 0x80)
            return 0;
    return len;
}

/* Valid UTF-8 with something beyond ASCII in it */
static int looks_utf8(const unsigned char *in, int n)
{
    int i = 0, wide = 0, len;

    while (i < n) {
        len = utf8_seq(in + i, n - i);
        if (len == 0)
            return 0;
        wide |= len > 1;
        i += len;
    }
    return wide;
}

/* Puts len bytes at *pos, if they fit with the NUL */
static void put_bytes(char *out, size_t out_size, size_t *pos,
                      const char *bytes, size_t len)
{
    if (*pos + len < out_size) {
        memcpy(out + *pos, bytes, len);
        *pos += len;
    }
}

static void put_code_point(char *out, size_t out_size, size_t *pos,
                           unsigned int cp)
{
    char b[3];

    if (cp < 0x80) {
        b[0] = cp;
        put_bytes(out, out_size, pos, b, 1);
    } else if (cp < 0x800) {
        b[0] = 0xc0 | (cp >> 6);
        b[1] = 0x80 | (cp & 0x3f);
        put_bytes(out, out_size, pos, b, 2);
    } else {
        b[0] = 0xe0 | (cp >> 12);
        b[1] = 0x80 | ((cp >> 6) & 0x3f);
        b[2] = 0x80 | (cp & 0x3f);
        put_bytes(out, out_size, pos, b, 3);
    }
}

size_t rds_to_utf8(const unsigned char *in, int n, enum rds_charset charset,
                   char *out, size_t out_size)
{
    size_t pos = 0;
    int i, len;

    if (out_size == 0)
        return 0;
    if (charset == RDS_CHARSET_BASIC && looks_utf8(in, n))
        charset = RDS_CHARSET_UTF8;

    switch (charset) {
    case RDS_CHARSET_BASIC:
        for (i = 0; i < n; i++) {
            unsigned char c = in[i];

            if (c >= 0x80)
                put_bytes(out, out_size, &pos, ebu_high[c - 0x80],
                          strlen(ebu_high[c - 0x80]));
            else
                put_bytes(out, out_size, &pos,
                          (c >= 0x20 && c != 0x7f) ? (const char *) &c : " ",
                          1);
        }
        break;

    case RDS_CHARSET_UTF8:
        for (i = 0; i < n; i += len) {
            len = utf8_seq(in + i, n - i);
            if (len == 0) {
                put_bytes(out, out_size, &pos, "?", 1);
                len = 1;
            } else if (len == 1 && in[i] < 0x20) {
                put_bytes(out, out_size, &pos, " ", 1);
            } else {
                put_bytes(out, out_size, &pos, (const char *) in + i, len);
            }
        }
        break;

    case RDS_CHARSET_UCS2:
        for (i = 0; i + 1 < n; i += 2) {
            unsigned int cp = (in[i] << 8) | in[i + 1];

            if (cp == 0x0d)
                break;
            if (cp >= 0xd800 && cp <= 0xdfff)
                cp = '?';               /* half a pair: not UCS-2 */
            put_code_point(out, out_size, &pos, cp < 0x20 ? ' ' : cp);
        }
        break;
    }

    while (pos > 0 && out[pos - 1] == ' ')
        pos--;
    out[pos] = '\0';
    return pos;
}
