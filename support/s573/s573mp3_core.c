/* -----------------------------------------------------------------------------
 * s573mp3_core.c - System 573 MP3 service, pure logic half. See the header.
 * Released under the GNU GPL v2.
 * ---------------------------------------------------------------------------- */
#include "s573mp3_core.h"

static void zero(void *p, size_t n)
{
    uint8_t *b = (uint8_t *)p;
    while (n--) *b++ = 0;
}

void s573_core_init(s573_core_t *c)
{
    zero(c, sizeof *c);
    /* rst_acked stays 0: the core-LOAD reset itself bumps rst_epoch, so our very
     * first poll is guaranteed to see a change and go through the ack path. That
     * costs one poll (~5 ms) at startup and gives every reset ONE code path. */
}

void s573_core_on_reset(s573_core_t *c, uint16_t new_rst_epoch)
{
    uint32_t resets = c->resets;
    zero(c, sizeof *c);
    c->rst_epoch = new_rst_epoch;
    c->resets    = resets + 1;
    /* rst_acked / have_cfg deliberately left 0: we ack THIS poll, and the config
     * must be re-read before we may descramble a single byte. Reading the window
     * with a stale key schedule is silent noise, not an error. */
}

void s573_core_set_ctrl(s573_core_t *c, uint16_t flags)
{
    /* bit0 is RESERVED: the descramble scheme is owned by the fabric's OSD bit
     * (emu.sv O[101]) and arrives in the MP3CFG word, not from us. */
    c->ctrl_flags = (uint16_t)(flags & (uint16_t)~S573_CTRL_DDRSBM);
}

void s573_core_apply_cfg(s573_core_t *c, const s573_cfg_t *cfg)
{
    uint32_t start = ((uint32_t)cfg->start_hi << 16) | cfg->start_lo;
    uint32_t end   = ((uint32_t)cfg->end_hi   << 16) | cfg->end_lo;
    /* The scheme is the FABRIC's to state, not ours to guess: it comes from the
     * OSD bit O[101], which k573dio also folds into cfg_epoch -- so a mid-game
     * toggle re-triggers this adoption instead of silently leaving us on the old
     * key schedule. (Earlier this read back a bit WE had set, which made the
     * config word an echo of ourselves and defaulted ddrsbm titles to the wrong
     * scheme; the OSD bit removed that circularity.) */
    c->ddrsbm_want     = (cfg->flags & S573_CTRL_DDRSBM) ? 1 : 0;
    c->ddrsbm_echo_bad = 0;

    s573_desc_init(&c->desc, start, end,
                   cfg->key1, cfg->key2, cfg->key3,
                   c->ddrsbm_want);

    c->in_len   = 0;
    c->in_pos   = 0;
    /* The fabric re-armed on the same pulse that moved cfg_epoch, and its credit
     * baseline went with it -- so ours must too, or our first post-song report
     * would look like a huge delta and dump credit into the new window. */
    c->cons_bytes  = 0;
    c->cfg_epoch   = cfg->epoch;
    c->have_cfg    = 1;
    c->cfg_reloads++;
}

uint16_t s573_core_pcm_free(const s573_core_t *c)
{
    /* one beat held back so full and empty are distinguishable, matching the
     * fabric ring's own convention */
    uint16_t used = (uint16_t)((c->pcm_wr - c->pcm_rd) & (S573_PCM_BEATS - 1));
    return (uint16_t)(S573_PCM_BEATS - 1 - used);
}

int s573_core_should_decode(const s573_core_t *c, uint16_t need_beats)
{
    if (!c->rst_acked || !c->have_cfg) return 0;
    if (!(c->ctrl_flags & S573_CTRL_DRAIN_EN)) return 0;
    return s573_core_pcm_free(c) >= need_beats;
}

uint32_t s573_core_fill(s573_core_t *c, const uint8_t *dram)
{
    uint32_t room, got;

    /* compact first so a partial frame never pins the buffer */
    if (c->in_pos) {
        uint32_t left = c->in_len - c->in_pos;
        uint32_t i;
        for (i = 0; i < left; i++) c->in[i] = c->in[c->in_pos + i];
        c->in_len = left;
        c->in_pos = 0;
    }

    room = S573_IN_WINDOW - c->in_len;
    if (room > S573_IN_CHUNK) room = S573_IN_CHUNK;
    if (!room) return 0;

    got = (uint32_t)s573_desc_pull(&c->desc, dram, c->in + c->in_len, room);
    c->in_len += got;
    return got;
}

void s573_core_consume(s573_core_t *c, uint32_t n)
{
    uint32_t avail = c->in_len - c->in_pos;
    if (n > avail) n = avail;
    c->in_pos     += n;
    c->cons_bytes += n;      /* CUMULATIVE -- never reset except on re-arm */
}

void s573_core_wrote_pcm(s573_core_t *c, uint16_t beats)
{
    c->pcm_wr = (uint16_t)((c->pcm_wr + beats) & (S573_PCM_BEATS - 1));
    c->sync_cnt++;           /* cumulative 8-bit; the fabric diffs mod 256 */
    c->frames++;
}

void s573_core_note_idle(s573_core_t *c)
{
    c->idle_cnt++;
}

uint16_t s573_core_ctrl_events(const s573_core_t *c)
{
    return (uint16_t)(((uint16_t)c->sync_cnt << 8) | c->idle_cnt);
}

uint16_t s573_core_credit_word(const s573_core_t *c)
{
    return (uint16_t)(c->cons_bytes & 0xFFFFu);
}
