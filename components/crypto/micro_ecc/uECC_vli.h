/* Copyright 2015, Kenneth MacKay. Licensed under the BSD 2-clause license. */

#ifndef _UECC_VLI_H_
#define _UECC_VLI_H_

#include "uECC.h"
#include "types.h"

/* Functions for raw large-integer manipulation. These are only available
   if uECC.c is compiled with CS_UECC_ENABLE_VLI_API defined to 1. */
#ifndef CS_UECC_ENABLE_VLI_API
    #define CS_UECC_ENABLE_VLI_API 0
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#if CS_UECC_ENABLE_VLI_API

void cs_uecc_vli_clear(cs_uecc_word_t *vli, wordcount_t num_words);

/* Constant-time comparison to zero - secure way to compare long integers */
/* Returns 1 if vli == 0, 0 otherwise. */
cs_uecc_word_t cs_uecc_vli_iszero(const cs_uecc_word_t *vli, wordcount_t num_words);

/* Returns nonzero if bit 'bit' of vli is set. */
cs_uecc_word_t cs_uecc_vli_testbit(const cs_uecc_word_t *vli, bitcount_t bit);

/* Counts the number of bits required to represent vli. */
bitcount_t cs_uecc_vli_numbits(const cs_uecc_word_t *vli, const wordcount_t max_words);

/* Sets dest = src. */
void cs_uecc_vli_set(cs_uecc_word_t *dest, const cs_uecc_word_t *src, wordcount_t num_words);

/* Constant-time comparison function - secure way to compare long integers */
/* Returns one if left == right, zero otherwise */
cs_uecc_word_t cs_uecc_vli_equal(const cs_uecc_word_t *left,
                              const cs_uecc_word_t *right,
                              wordcount_t num_words);

/* Constant-time comparison function - secure way to compare long integers */
/* Returns sign of left - right, in constant time. */
cmpresult_t cs_uecc_vli_cmp(const cs_uecc_word_t *left, const cs_uecc_word_t *right, wordcount_t num_words);

/* Computes vli = vli >> 1. */
void cs_uecc_vli_rshift1(cs_uecc_word_t *vli, wordcount_t num_words);

/* Computes result = left + right, returning carry. Can modify in place. */
cs_uecc_word_t cs_uecc_vli_add(cs_uecc_word_t *result,
                         const cs_uecc_word_t *left,
                         const cs_uecc_word_t *right,
                         wordcount_t num_words);

/* Computes result = left - right, returning borrow. Can modify in place. */
cs_uecc_word_t cs_uecc_vli_sub(cs_uecc_word_t *result,
                         const cs_uecc_word_t *left,
                         const cs_uecc_word_t *right,
                         wordcount_t num_words);

/* Computes result = left * right. Result must be 2 * num_words long. */
void cs_uecc_vli_mult(cs_uecc_word_t *result,
                   const cs_uecc_word_t *left,
                   const cs_uecc_word_t *right,
                   wordcount_t num_words);

/* Computes result = left^2. Result must be 2 * num_words long. */
void cs_uecc_vli_square(cs_uecc_word_t *result, const cs_uecc_word_t *left, wordcount_t num_words);

/* Computes result = (left + right) % mod.
   Assumes that left < mod and right < mod, and that result does not overlap mod. */
void cs_uecc_vli_modadd(cs_uecc_word_t *result,
                     const cs_uecc_word_t *left,
                     const cs_uecc_word_t *right,
                     const cs_uecc_word_t *mod,
                     wordcount_t num_words);

/* Computes result = (left - right) % mod.
   Assumes that left < mod and right < mod, and that result does not overlap mod. */
void cs_uecc_vli_modsub(cs_uecc_word_t *result,
                     const cs_uecc_word_t *left,
                     const cs_uecc_word_t *right,
                     const cs_uecc_word_t *mod,
                     wordcount_t num_words);

/* Computes result = product % mod, where product is 2N words long.
   Currently only designed to work for mod == curve->p or curve_n. */
void cs_uecc_vli_mmod(cs_uecc_word_t *result,
                   cs_uecc_word_t *product,
                   const cs_uecc_word_t *mod,
                   wordcount_t num_words);

/* Calculates result = product (mod curve->p), where product is up to
   2 * curve->num_words long. */
void cs_uecc_vli_mmod_fast(cs_uecc_word_t *result, cs_uecc_word_t *product, cs_uecc_curve curve);

/* Computes result = (left * right) % mod.
   Currently only designed to work for mod == curve->p or curve_n. */
void cs_uecc_vli_modmult(cs_uecc_word_t *result,
                      const cs_uecc_word_t *left,
                      const cs_uecc_word_t *right,
                      const cs_uecc_word_t *mod,
                      wordcount_t num_words);

/* Computes result = (left * right) % curve->p. */
void cs_uecc_vli_modmult_fast(cs_uecc_word_t *result,
                           const cs_uecc_word_t *left,
                           const cs_uecc_word_t *right,
                           cs_uecc_curve curve);

/* Computes result = left^2 % mod.
   Currently only designed to work for mod == curve->p or curve_n. */
void cs_uecc_vli_modsquare(cs_uecc_word_t *result,
                        const cs_uecc_word_t *left,
                        const cs_uecc_word_t *mod,
                        wordcount_t num_words);

/* Computes result = left^2 % curve->p. */
void cs_uecc_vli_modsquare_fast(cs_uecc_word_t *result, const cs_uecc_word_t *left, cs_uecc_curve curve);

/* Computes result = (1 / input) % mod.*/
void cs_uecc_vli_modInv(cs_uecc_word_t *result,
                     const cs_uecc_word_t *input,
                     const cs_uecc_word_t *mod,
                     wordcount_t num_words);

#if CS_UECC_SUPPORT_COMPRESSED_POINT
/* Calculates a = sqrt(a) (mod curve->p) */
void cs_uecc_vli_mod_sqrt(cs_uecc_word_t *a, cs_uecc_curve curve);
#endif

/* Converts an integer in uECC native format to big-endian bytes. */
void cs_uecc_vli_nativetobytes(uint8_t *bytes, int num_bytes, const cs_uecc_word_t *native);
/* Converts big-endian bytes to an integer in uECC native format. */
void cs_uecc_vli_bytestonative(cs_uecc_word_t *native, const uint8_t *bytes, int num_bytes);

unsigned cs_uecc_curve_num_words(cs_uecc_curve curve);
unsigned cs_uecc_curve_num_bytes(cs_uecc_curve curve);
unsigned cs_uecc_curve_num_bits(cs_uecc_curve curve);
unsigned cs_uecc_curve_num_n_words(cs_uecc_curve curve);
unsigned cs_uecc_curve_num_n_bytes(cs_uecc_curve curve);
unsigned cs_uecc_curve_num_n_bits(cs_uecc_curve curve);

const cs_uecc_word_t *cs_uecc_curve_p(cs_uecc_curve curve);
const cs_uecc_word_t *cs_uecc_curve_n(cs_uecc_curve curve);
const cs_uecc_word_t *cs_uecc_curve_g(cs_uecc_curve curve);
const cs_uecc_word_t *cs_uecc_curve_b(cs_uecc_curve curve);

int cs_uecc_valid_point(const cs_uecc_word_t *point, cs_uecc_curve curve);

/* Multiplies a point by a scalar. Points are represented by the X coordinate followed by
   the Y coordinate in the same array, both coordinates are curve->num_words long. Note
   that scalar must be curve->num_n_words long (NOT curve->num_words). */
void cs_uecc_point_mult(cs_uecc_word_t *result,
                     const cs_uecc_word_t *point,
                     const cs_uecc_word_t *scalar,
                     cs_uecc_curve curve);

/* Generates a random integer in the range 0 < random < top.
   Both random and top have num_words words. */
int cs_uecc_generate_random_int(cs_uecc_word_t *random,
                             const cs_uecc_word_t *top,
                             wordcount_t num_words);

#endif /* CS_UECC_ENABLE_VLI_API */

#ifdef __cplusplus
} /* end of extern "C" */
#endif

#endif /* _UECC_VLI_H_ */
