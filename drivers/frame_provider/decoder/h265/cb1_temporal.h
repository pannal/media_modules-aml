/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Integer port of CB1's cb1_hdr10_temporal.c (scene-aware L1 baseline, 1 s lookahead),
 * usable in the kernel (no float, no big stack arrays) and in userspace for the
 * equivalence test against the original (cb1-lab src/kport/cb1t_offline.c).
 *
 * Units: picture statistics and motion values are PQ code scaled to 0..65535
 * (CB1's float 0..1 times 65535); pts in 90 kHz ticks; L1 output in 12-bit PQ codes.
 * Only the L1 baseline (min/avg/max) of the original is ported: its calibration
 * prefix (features for CB1's L1/L3 model) is not, but the state it resets is.
 */
#ifndef CB1_TEMPORAL_H
#define CB1_TEMPORAL_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int64_t s64;
#endif

#define CB1T_HZ          90000   /* pts ticks per second */
#define CB1T_MAX_SAMPLES 256
#define CB1T_PLANE       576     /* 32 x 18 grid of the 128x72 preview, every 4th point */
#define CB1T_MOTION      (3 * CB1T_PLANE)  /* planar R, G, B */

enum cb1t_status { CB1T_READY, CB1T_PENDING, CB1T_INVALID };

struct cb1t_frame {
	s64 pts;
	u16 gmin;   /* min over the active area of min(R,G,B) */
	u16 gmax;   /* max over the active area of max(R,G,B) */
	u16 gmean;  /* mean of max(R,G,B) on a 256x144 grid of the active area */
};

struct cb1t_result {
	s64 pts, observed_until;
	u16 l1[3];  /* min, avg, max, 12-bit PQ code */
	u16 peak;   /* scene max, 0..65535 */
};

struct cb1t_sample {
	struct cb1t_frame d;
	u64 picture;
	u32 difference;  /* sum over 1728 of |after - before| */
	u64 shape;       /* sum over 1728 of |576 delta - plane sum of delta| */
	bool exposure, candidate, settled, gap;
};

struct cb1t_ctx {
	struct cb1t_sample samples[CB1T_MAX_SAMPLES];
	u32 count;
	u16 prev_motion[CB1T_MOTION];
	bool active, ended, calibrated;
	s64 calibration_start, frontier, active_cut;
	s64 lookahead;  /* ticks of lookahead, CONFIRM..1 s, kept across resets */
	s64 rise_lead;  /* ticks of lookahead allowed to raise L1 max, 0..1 s */
	u32 rise_rate;  /* max L1 max rise inside a shot, 12-bit codes per second, 0 = unlimited */
	s64 open;       /* ticks after a cut whose pictures raise L1 max from the cut on, 0 = off */
	u32 max_hyst;   /* in-shot L1 max rises smaller than this are not followed, 12-bit codes */
	u32 avg_hyst;   /* in-shot L1 avg follows the baseline at this distance, 12-bit codes, 0 = off */
	u32 cut_hyst;   /* L1 max is held across a cut when the new shot is within this, 12-bit codes, 0 = off */
	bool ramp_valid;
	s64 ramp_pts, ramp_scene;
	u64 ramp_q;     /* last output L1 max, 12-bit codes x CB1T_HZ */
	u64 avg_q;      /* last output L1 avg, 12-bit codes x CB1T_HZ */
	u16 history_low, history_high;
	u64 history_energy, history_mass;
};

void cb1t_reset(struct cb1t_ctx *ctx);
void cb1t_end(struct cb1t_ctx *ctx);
/* Lookahead in ticks, clamped to 0.2..1 s (default 1 s = the original). */
void cb1t_set_lookahead(struct cb1t_ctx *ctx, s64 ticks);
/*
 * How far ahead a brighter picture may raise L1 max, in ticks, 0..1 s
 * (1 s = the original). Cut detection keeps the full lookahead.
 */
void cb1t_set_rise_lead(struct cb1t_ctx *ctx, s64 ticks);
/*
 * Rate limit of L1 max rises inside a shot, in 12-bit PQ codes per second
 * (0 = unlimited = the original). A cut, a fall or a pts going back resets it.
 */
void cb1t_set_rise_rate(struct cb1t_ctx *ctx, u32 codes_per_s);
/*
 * Shot opening window in ticks, 0..1 s (0 = off): pictures of the shot's first
 * `open` ticks that the lookahead already shows count for L1 max from the cut
 * on, so the rise lands on the cut instead of inside the shot.
 */
void cb1t_set_open(struct cb1t_ctx *ctx, s64 ticks);
/*
 * In-shot dead bands, 12-bit PQ codes (0 = off = the original): L1 max stays
 * up to max_hyst under its baseline, L1 avg within avg_hyst of its baseline.
 * A cut resets both to the baseline.
 */
void cb1t_set_max_hyst(struct cb1t_ctx *ctx, u32 codes);
void cb1t_set_avg_hyst(struct cb1t_ctx *ctx, u32 codes);
/*
 * Shot-to-shot dead band of L1 max, 12-bit PQ codes (0 = off = the original):
 * a cut whose new baseline is within cut_hyst of the output keeps the output,
 * so consecutive shots of one scene share a curve. The output may then sit up
 * to cut_hyst above or under the baseline for the whole shot.
 */
void cb1t_set_cut_hyst(struct cb1t_ctx *ctx, u32 codes);
/* Push pictures in display order; picture ids strictly increase. */
enum cb1t_status cb1t_push(struct cb1t_ctx *ctx, const struct cb1t_frame *d,
			   const u16 motion[CB1T_MOTION], u64 picture);
/* READY once the lookahead past this picture was pushed (or after cb1t_end). */
enum cb1t_status cb1t_poll(struct cb1t_ctx *ctx, u64 picture, struct cb1t_result *out);

#endif
