/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments
 * Copyright (C) 2026 Kerem Kilic (Ellic Studio): checked start, stop/restart, the sample and rate contract */
/* FM-1 audio out: ALNK0 I2S -> external codec, double-buffered DMA
 * Fixed format: stereo L,R interleaved, signed Q23 in int32_t (sign-extended
 * low 24 bits, NOT shifted left by 8). Silence = 0, range -8388608..8388607.
 * Fixed 44.1 kHz family: stock clock is 24000000/544 ~= 44117.647 Hz.
 * This is not exact 44100 Hz, nor a 48 kHz/resampling API.
 * Application owns two halves of half_words int32_t; init clears both.
 *
 *   fm1_audio_init_checked(buf, buffer_words, half_words, rate, isr, prio)
 *                         validates capacity, RAM range, alignment and clock;
 *                         returns 0 or FM1_AUDIO_ERR_* (warn_unused_result:
 *                         a refused start is silence). IRQs must be off.
 *   fm1_audio_init(buf, half_words, isr, prio) compatibility wrapper, assumes
 *                         capacity >= 2 * half_words; now returns status too.
 *                         isr = asm wrapper (fm1_isr.S) for FM1_IRQ_ALNK0
 *   fm1_audio_start(buf, half_words, isr, prio) the start alone, after the checks; without the clock's only for an
 *                         app that knows why it refused. fm1_audio_pll0(): PLL0 as that check reads it (0: unset)
 *   in the ISR:   p = fm1_audio_pending();  fm1_audio_ack_aux(p);
 *                 if (p & FM1_AUDIO_HALF) { fill fm1_audio_free_half();
 *                                           fm1_audio_ack_half(); }
 *   fm1_audio_stop()      IRQ masked, DMA off, pending cleared. Repeatable.
 *   fm1_audio_line(on)    PC6, the codec's line fm1_audio_init raises once the audio runs: low, the output quiet
 * Restart = stop then init (both halves cleared, DMA position reset).
 * Underrun = DMA repeats stale samples; no automatic silence or detection.
 * See docs/BOARD_IO.md for deadlines, races and application recovery.
 */
#pragma once
#include <stdint.h>
#include "fm1_cc.h"
#include "fm1_time.h"
#include "fm1_irq.h"
#include "fm1_gpio.h"

#ifndef FM1_AUDIO_REG
#define FM1_AUDIO_REG(type, addr) (*(volatile type *)(addr))
#endif
#define FM1_ALNK 0x12E00u
#define FM1_ALNK_CON0 FM1_AUDIO_REG(uint16_t, FM1_ALNK + 0x00u)
#define FM1_ALNK_CON1 FM1_AUDIO_REG(uint16_t, FM1_ALNK + 0x04u)
#define FM1_ALNK_CON2 FM1_AUDIO_REG(uint8_t, FM1_ALNK + 0x08u)
#define FM1_ALNK_CON3 FM1_AUDIO_REG(uint8_t, FM1_ALNK + 0x0Cu)
#define FM1_ALNK_ADR3 FM1_AUDIO_REG(uint32_t, FM1_ALNK + 0x1Cu)
#define FM1_ALNK_LEN FM1_AUDIO_REG(uint16_t, FM1_ALNK + 0x20u)
#define FM1_CLK_CON2 FM1_AUDIO_REG(uint32_t, 0x10014u)
#define FM1_IOMAP_CON5 FM1_AUDIO_REG(uint32_t, 0x51030u)
#define FM1_AUDIO_PLL_CON0 FM1_AUDIO_REG(uint32_t, 0x119A0u)
#define FM1_AUDIO_PLL_CON1 FM1_AUDIO_REG(uint32_t, 0x119A4u)
/* the stock audio PLL (PLL0), as M-VAVE's firmware and m-bootloader leave it running; CON1 compared under
 * 0x07FFFFFF, as fm1_clock.h's fm1_clock_is_stock compares PLL1's CON1 (its top bits need not read back the same) */
#define FM1_AUDIO_PLL_STOCK0 0x45400203u
#define FM1_AUDIO_PLL_STOCK1 0x3F503026u
#define FM1_AUDIO_PLL_CON1_MASK 0x07FFFFFFu
#define FM1_AUDIO_HALF 0x80u      /* CON2 pending: a half buffer is free */
#define FM1_AUDIO_RATE 44100u     /* nominal API rate; physical ratio below */
#define FM1_AUDIO_RATE_NUM 24000000u
#define FM1_AUDIO_RATE_DEN 544u
#define FM1_AUDIO_CHANNELS 2u
#define FM1_AUDIO_SAMPLE_BITS 24u
#define FM1_AUDIO_SAMPLE_MIN (-8388607 - 1)
#define FM1_AUDIO_SAMPLE_MAX 8388607
#define FM1_AUDIO_MAX_HALF_WORDS 65534u /* 16-bit LEN, complete stereo frames */
enum { FM1_AUDIO_OK = 0, FM1_AUDIO_ERR_BUFFER = -1, FM1_AUDIO_ERR_RATE = -2,
       FM1_AUDIO_ERR_IRQ = -3, FM1_AUDIO_ERR_BUSY = -4, FM1_AUDIO_ERR_CLOCK = -5 };

/* Pure validation: only the DMA span must fit in ordinary SRAM; exclude the
 * reserved boot info/mailbox/vector area at the top. Capacity is in int32 words. */
FM1_INLINE int fm1_audio_buffer_valid(const int32_t *buf, uint32_t buffer_words, uint32_t half_words)
{
    uintptr_t addr = (uintptr_t)buf;
    return half_words && half_words <= FM1_AUDIO_MAX_HALF_WORDS && !(half_words & 1u) &&
           buffer_words >= 2u * half_words && !(addr & 3u) &&
           addr >= 0x01C00000u && addr < 0x01C7FD50u &&
           2u * half_words * sizeof(int32_t) <= 0x01C7FD50u - addr;
}

static void fm1__pc_out(uint32_t bit, int v)   /* PC0 MCLK, PC1 BCLK, PC2 LRCK, PC6 data */
{
    uint32_t m = 1u << bit;
    FM1_PR(FM1_PC, FM1_DIE) |= m;
    if (v)
        FM1_PR(FM1_PC, FM1_OUT) |= m;
    else
        FM1_PR(FM1_PC, FM1_OUT) &= ~m;
    FM1_PR(FM1_PC, FM1_DIR) &= ~m;
}

/* the start alone, after its checks (fm1_audio_init_checked's: the buffer, the rate, the IRQ, not busy, the clock);
 * without the clock's only where the app knows why it refused (FRASIL card #163: PLL0 unset, an emulator's).
 * IRQs off */
FM1_INLINE void fm1_audio_start(int32_t *buf, uint32_t half_words, void (*isr)(void), uint32_t prio)
{
    uint32_t i;
    fm1_irq_mask(FM1_IRQ_ALNK0);
    for (i = 0; i < 2u * half_words; i++)
        buf[i] = 0;
    fm1__pc_out(6, 0);
    fm1_delay_ms(5);
    FM1_ALNK_CON0 = 0;
    FM1_ALNK_CON1 = 0;
    FM1_ALNK_CON2 = 0;
    FM1_ALNK_CON3 = 0;
    FM1_IOMAP_CON5 &= ~0xC0u;
    FM1_ALNK_CON3 |= 3u;
    fm1__pc_out(0, 1);
    FM1_ALNK_CON0 |= 0x100u;
    fm1__pc_out(2, 1);
    fm1__pc_out(1, 1);
    FM1_ALNK_CON0 |= 0x80u;
    FM1_ALNK_CON0 &= ~0x40u;
    FM1_ALNK_LEN = (uint16_t)half_words;
    FM1_ALNK_CON0 &= ~0x400u;
    FM1_ALNK_CON0 &= ~0x200u;
    FM1_CLK_CON2 &= 0xFFFFF0FFu;
    FM1_ALNK_CON3 &= ~0x1Cu;
    FM1_ALNK_CON3 = (uint8_t)((FM1_ALNK_CON3 & 0x1Fu) | 0x80u);
    FM1_ALNK_CON1 |= 1u << 12;
    FM1_ALNK_CON1 |= 1u << 14;
    FM1_ALNK_ADR3 = (uint32_t)(uintptr_t)buf;
    fm1__pc_out(6, 1);
    FM1_ALNK_CON1 &= ~(1u << 15);
    FM1_ALNK_CON2 = 0x0Fu;
    fm1_irq_attach(FM1_IRQ_ALNK0, isr, prio);
    __asm__ volatile("csync" ::: "memory"); /* zeroed samples visible before DMA starts */
    FM1_ALNK_CON0 |= 0x800u;
    fm1_delay_ms(5);
    fm1__pc_out(6, 1);
}

/* PLL0's CON0 as the clock check reads it: 0 where nothing set it (an emulator that does not imitate the
 * bootloader's PLL); an FM-1's bootloader always sets it (FRASIL card #163) */
FM1_INLINE uint32_t fm1_audio_pll0(void) { return FM1_AUDIO_PLL_CON0; }

__attribute__((warn_unused_result))
FM1_INLINE int fm1_audio_init_checked(int32_t *buf, uint32_t buffer_words, uint32_t half_words,
                                     uint32_t rate, void (*isr)(void), uint32_t prio)
{
    if (!fm1_audio_buffer_valid(buf, buffer_words, half_words)) return FM1_AUDIO_ERR_BUFFER;
    if (rate != FM1_AUDIO_RATE) return FM1_AUDIO_ERR_RATE;
    if (!isr || prio > 7u) return FM1_AUDIO_ERR_IRQ;
    if (FM1_ALNK_CON0 & 0x800u) return FM1_AUDIO_ERR_BUSY;
    /* Recognized stock audio PLL under M-VAVE and m-bootloader; never retune
     * a shared PLL here. Unknown configurations cannot promise this rate. */
    if (FM1_AUDIO_PLL_CON0 != FM1_AUDIO_PLL_STOCK0 ||
        (FM1_AUDIO_PLL_CON1 & FM1_AUDIO_PLL_CON1_MASK) != (FM1_AUDIO_PLL_STOCK1 & FM1_AUDIO_PLL_CON1_MASK))
        return FM1_AUDIO_ERR_CLOCK;
    fm1_audio_start(buf, half_words, isr, prio);
    return FM1_AUDIO_OK;
}

FM1_INLINE int fm1_audio_init(int32_t *buf, uint32_t half_words, void (*isr)(void), uint32_t prio)
{
    /* Validate before multiplying the legacy size, so it cannot wrap. */
    if (half_words > FM1_AUDIO_MAX_HALF_WORDS) return FM1_AUDIO_ERR_BUFFER;
    return fm1_audio_init_checked(buf, 2u * half_words, half_words, FM1_AUDIO_RATE, isr, prio);
}

FM1_INLINE uint8_t fm1_audio_pending(void) { return FM1_ALNK_CON2; }
FM1_INLINE void fm1_audio_ack_aux(uint8_t p)   /* the other three pendings */
{
    if (p & 0x10u)
        FM1_ALNK_CON2 |= 1u;
    if (p & 0x20u)
        FM1_ALNK_CON2 |= 2u;
    if (p & 0x40u)
        FM1_ALNK_CON2 |= 4u;
}
FM1_INLINE uint32_t fm1_audio_free_half(void) { return ((FM1_ALNK_CON0 >> 15) & 1u) ^ 1u; }   /* 0 / 1 */
FM1_INLINE void fm1_audio_ack_half(void)
{
    __asm__ volatile("csync" ::: "memory"); /* publish completed sample stores before acknowledgement */
    FM1_ALNK_CON2 |= 0x08u;
}
FM1_INLINE void fm1_audio_stop(void)   /* IRQs off, like init; does not change global interrupt state */
{
    fm1_irq_mask(FM1_IRQ_ALNK0);
    FM1_ALNK_CON0 &= ~0x800u;
    __asm__ volatile("csync" ::: "memory"); /* DMA disabled before the caller may reuse its buffer */
    FM1_ALNK_CON2 = 0x0Fu;
    fm1__pc_out(6, 0);
}
FM1_INLINE void fm1_audio_line(int on) { fm1__pc_out(6, on); }
