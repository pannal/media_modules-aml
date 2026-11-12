// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Integer port of CB1's cb1_hdr10_temporal.c, see cb1_temporal.h.
 * Float thresholds of the original are kept as exact integer comparisons; the
 * exposure-gain test works on plane-centred values rounded to 1/65535, which can
 * only flip a decision that the original takes within ~1e-5 of its threshold.
 */
#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/math64.h>
#include "cb1_temporal.h"
#else
#include <string.h>
#include "cb1_temporal.h"
static inline u64 div64_u64(u64 a, u64 b) { return a / b; }
static inline s64 div64_s64(s64 a, s64 b) { return a / b; }
#endif

#define ONE     65535u
#define SEC     ((s64)CB1T_HZ)
#define DT_24P  ((u64)3754)          /* 1/24 s, the original's default duration */
#define GAP     SEC                  /* pictures more than 1 s apart = gap */
#define CONFIRM (SEC / 5)            /* a cut is confirmed 0.2 s after it */

static inline u64 u64_min(u64 a, u64 b) { return a < b ? a : b; }
static inline s64 s64_min(s64 a, s64 b) { return a < b ? a : b; }
static inline s64 s64_max(s64 a, s64 b) { return a > b ? a : b; }
static inline u64 u64_absd(s64 v) { return v < 0 ? (u64)-v : (u64)v; }

/* Exposure / shape / difference between two consecutive motion previews. */
static void motion_edge(const u16 *before, const u16 *after, struct cb1t_sample *out)
{
	u32 difference = 0;
	u64 shape = 0, residual = 0;
	bool gain_valid = true;
	unsigned int c, i;

	for (i = 0; i < CB1T_MOTION; ++i)
		difference += (u32)u64_absd((s64)after[i] - before[i]);
	for (c = 0; c < 3; ++c) {
		const u16 *a0 = before + c * CB1T_PLANE, *b0 = after + c * CB1T_PLANE;
		s64 sa = 0, sb = 0, ma, mb, energy = 0, cross = 0, gain_q20;

		for (i = 0; i < CB1T_PLANE; ++i) {
			sa += a0[i];
			sb += b0[i];
		}
		/* |delta - mean(delta)|, exact in units of 1/(576 * 65535) */
		for (i = 0; i < CB1T_PLANE; ++i)
			shape += u64_absd(CB1T_PLANE * ((s64)b0[i] - a0[i]) - (sb - sa));
		/* plane means rounded to 1/65535 for the gain fit */
		ma = div64_s64(2 * sa + CB1T_PLANE, 2 * CB1T_PLANE);
		mb = div64_s64(2 * sb + CB1T_PLANE, 2 * CB1T_PLANE);
		for (i = 0; i < CB1T_PLANE; ++i) {
			s64 a = a0[i] - ma, b = b0[i] - mb;

			energy += a * a;
			cross += a * b;
		}
		/* original: gain = cross/energy if energy > 1e-8 (units 1/65535^2: 42.9) */
		if (energy > 42) {
			gain_valid &= 4 * cross > energy && cross < 4 * energy;
			gain_q20 = div64_s64(cross * (1 << 20), energy);
		} else {
			gain_q20 = 1 << 20;
		}
		if (!gain_valid)
			continue;
		for (i = 0; i < CB1T_PLANE; ++i) {
			s64 a = a0[i] - ma, b = b0[i] - mb;

			residual += u64_absd(b - ((gain_q20 * a) >> 20));
		}
	}
	out->difference = difference;
	out->shape = shape;
	/* mean residual < .008 */
	out->exposure = gain_valid && residual * 1000 < 8ull * CB1T_MOTION * ONE;
}

void cb1t_reset(struct cb1t_ctx *ctx)
{
	ctx->count = 0;
	ctx->active = true;
	ctx->ended = false;
	ctx->calibrated = false;
	ctx->calibration_start = -1;
	ctx->frontier = -1;
	if (ctx->lookahead < CONFIRM || ctx->lookahead > SEC)
		ctx->lookahead = SEC;
	if (ctx->rise_lead <= 0 || ctx->rise_lead > SEC)
		ctx->rise_lead = SEC;
	ctx->ramp_valid = false;
	ctx->active_cut = 0;
	ctx->history_low = ONE;
	ctx->history_high = 0;
	ctx->history_energy = 0;
	ctx->history_mass = 0;
}

void cb1t_end(struct cb1t_ctx *ctx)
{
	ctx->ended = true;
}

void cb1t_set_lookahead(struct cb1t_ctx *ctx, s64 ticks)
{
	ctx->lookahead = ticks < CONFIRM ? CONFIRM : ticks > SEC ? SEC : ticks;
}

void cb1t_set_rise_lead(struct cb1t_ctx *ctx, s64 ticks)
{
	/* 0 is stored as 1 tick: reset treats 0 as unset (1 s) */
	ctx->rise_lead = ticks < 1 ? 1 : ticks > SEC ? SEC : ticks;
}

void cb1t_set_rise_rate(struct cb1t_ctx *ctx, u32 codes_per_s)
{
	ctx->rise_rate = codes_per_s > 4095 ? 4095 : codes_per_s;
}

void cb1t_set_open(struct cb1t_ctx *ctx, s64 ticks)
{
	ctx->open = ticks < 0 ? 0 : ticks > SEC ? SEC : ticks;
}

void cb1t_set_max_hyst(struct cb1t_ctx *ctx, u32 codes)
{
	ctx->max_hyst = codes > 4095 ? 4095 : codes;
}

void cb1t_set_avg_hyst(struct cb1t_ctx *ctx, u32 codes)
{
	ctx->avg_hyst = codes > 4095 ? 4095 : codes;
}

void cb1t_set_cut_hyst(struct cb1t_ctx *ctx, u32 codes)
{
	ctx->cut_hyst = codes > 4095 ? 4095 : codes;
}

/*
 * Output stage: a jump is free at a cut, any move inside a shot can be seen.
 * - max: the baseline only grows inside a shot; a large step dims the mids
 *   visibly even when it lands with the highlight, so it is spread at
 *   rise_rate, and a rise smaller than max_hyst is not followed at all (the
 *   output stays up to max_hyst under the baseline).
 *   A cut between two shots of one scene shows too when the curve jumps on
 *   it: a new baseline within cut_hyst of the output keeps the output, and
 *   the shot then stays inside that band (above or under its baseline).
 * - avg: the baseline is a running mean that drifts on every picture; the
 *   output follows it at a distance of avg_hyst (backlash), so a steady trend
 *   passes through smoothly and jitter does not.
 */
static void ramp(struct cb1t_ctx *ctx, struct cb1t_result *r)
{
	u64 target = (u64)r->l1[2] * CB1T_HZ, avg = (u64)r->l1[1] * CB1T_HZ;
	u64 max_h = (u64)ctx->max_hyst * CB1T_HZ, avg_h = (u64)ctx->avg_hyst * CB1T_HZ;
	u64 cut_h = (u64)ctx->cut_hyst * CB1T_HZ;
	s64 scene = ctx->active_cut;
	bool fresh = !ctx->ramp_valid || scene != ctx->ramp_scene || r->pts <= ctx->ramp_pts;

	bool restart = !ctx->ramp_valid || r->pts <= ctx->ramp_pts;

	if (cut_h && !restart && target <= ctx->ramp_q + cut_h && ctx->ramp_q <= target + cut_h) {
		/* inside the band, at a cut or in the shot: hold */
		r->l1[2] = div64_u64(ctx->ramp_q, CB1T_HZ);
	} else if (fresh || target <= ctx->ramp_q) {
		ctx->ramp_q = target;
	} else {
		u64 follow = target > max_h ? target - max_h : 0;

		if (follow > ctx->ramp_q) {
			u64 allowed = ctx->ramp_q +
				(u64)ctx->rise_rate * (u64)(r->pts - ctx->ramp_pts);

			ctx->ramp_q = ctx->rise_rate && allowed < follow ? allowed : follow;
		}
		r->l1[2] = div64_u64(ctx->ramp_q, CB1T_HZ);
	}
	if (fresh || !avg_h) {
		ctx->avg_q = avg;
	} else {
		if (avg > ctx->avg_q + avg_h)
			ctx->avg_q = avg - avg_h;
		else if (avg + avg_h < ctx->avg_q)
			ctx->avg_q = avg + avg_h;
		r->l1[1] = div64_u64(ctx->avg_q, CB1T_HZ);
	}
	if (r->l1[1] > r->l1[2] - 1)
		r->l1[1] = r->l1[2] - 1;
	ctx->ramp_valid = true;
	ctx->ramp_scene = scene;
	ctx->ramp_pts = r->pts;
}

static bool cut_candidate(const struct cb1t_ctx *ctx, u32 i, u32 observed)
{
	const struct cb1t_sample *s = &ctx->samples[i];
	u32 first = 1, last = observed, j;
	u64 neighbourhood = 0;
	unsigned int count = 0;
	int offset;

	for (j = i; j > 0; --j)
		if (ctx->samples[j].gap) {
			first = j + 1;
			break;
		}
	for (j = i + 1; j < observed; ++j)
		if (ctx->samples[j].gap) {
			last = j;
			break;
		}
	for (offset = -2; offset <= 2; ++offset) {
		s64 next = (s64)i + offset;

		if (offset && next >= first && next < last) {
			neighbourhood += ctx->samples[next].difference;
			++count;
		}
	}
	/* difference > max(.03, 2 * max(.003, mean of neighbours)) */
	if ((u64)s->difference * 1000 <= 30ull * CB1T_MOTION * ONE)
		return false;
	if (count && (u64)s->difference * count <= 2 * neighbourhood)
		return false;
	/* shape > .012 */
	return s->shape * 1000 > 12ull * CB1T_PLANE * CB1T_MOTION * ONE && !s->exposure;
}

enum cb1t_status cb1t_push(struct cb1t_ctx *ctx, const struct cb1t_frame *d,
			   const u16 motion[CB1T_MOTION], u64 picture)
{
	struct cb1t_sample *s;

	if (!ctx->active || d->pts < 0 || d->gmin > d->gmean || d->gmean > d->gmax)
		return CB1T_INVALID;
	if (ctx->count) {
		const struct cb1t_sample *last = &ctx->samples[ctx->count - 1];

		if (last->picture == picture)
			return d->pts == last->d.pts ? CB1T_READY : CB1T_INVALID;
		if (d->pts <= last->d.pts || d->pts < ctx->frontier || picture <= last->picture)
			return CB1T_INVALID;
	}
	if (ctx->ended)
		return CB1T_INVALID;
	if (ctx->count == CB1T_MAX_SAMPLES)
		return CB1T_PENDING;
	s = &ctx->samples[ctx->count];
	s->d = *d;
	s->picture = picture;
	s->difference = 0;
	s->shape = 0;
	s->exposure = s->candidate = s->settled = false;
	s->gap = ctx->count && d->pts - ctx->samples[ctx->count - 1].d.pts > GAP;
	if (ctx->count && !s->gap)
		motion_edge(ctx->prev_motion, motion, s);
	else if (!ctx->count)
		ctx->active_cut = d->pts;
	memcpy(ctx->prev_motion, motion, sizeof(ctx->prev_motion));
	ctx->frontier = d->pts;
	++ctx->count;
	if (ctx->count > 3) {
		u32 i = ctx->count - 3;

		ctx->samples[i].candidate = cut_candidate(ctx, i, ctx->count);
		ctx->samples[i].settled = true;
	}
	return CB1T_READY;
}

static bool scene_boundary(const struct cb1t_ctx *ctx, u32 i, u32 observed)
{
	const struct cb1t_sample *s;

	if (!i || i >= observed)
		return false;
	s = &ctx->samples[i];
	if (s->gap)
		return true;
	return (s->settled && i + 2 < observed) ? s->candidate : cut_candidate(ctx, i, observed);
}

static bool confirmed_cut(const struct cb1t_ctx *ctx, u32 i, u32 observed, s64 available)
{
	const struct cb1t_sample *s;
	u32 consumed;

	if (!scene_boundary(ctx, i, observed))
		return false;
	s = &ctx->samples[i];
	if (s->gap)
		return s->d.pts <= available;
	consumed = i + 2 < observed ? i + 2 : observed - 1;
	return s64_max(s->d.pts + CONFIRM, ctx->samples[consumed].d.pts) <= available;
}

/* round(v * 4095 / 65535) */
static inline u64 to12(u64 v)
{
	return div64_u64(v * 4095 + ONE / 2, ONE);
}

static bool baseline(const struct cb1t_ctx *ctx, u32 start, u32 end, bool cut,
		     s64 lead_end, struct cb1t_result *r)
{
	bool hist = start == 0;
	u64 low = hist && ctx->history_mass ? ctx->history_low : ONE;
	u64 high = hist && ctx->history_mass ? ctx->history_high : 0;
	u64 energy = hist ? ctx->history_energy : 0, mass = hist ? ctx->history_mass : 0;
	u64 l1min, l1avg, l1max;
	u32 i;

	for (i = start; i < end; ++i) {
		const struct cb1t_frame *d = &ctx->samples[i].d;
		u64 dt;

		low = u64_min(low, d->gmin);
		/* a picture past the rise lead does not raise max (yet) */
		if (d->pts <= lead_end)
			high = high > d->gmax ? high : d->gmax;
		if (i + 1 < end || (cut && ctx->samples[i + 1].d.pts - d->pts <= GAP))
			dt = ctx->samples[i + 1].d.pts - d->pts;
		else if (i > start)
			dt = d->pts - ctx->samples[i - 1].d.pts;
		else
			dt = DT_24P;
		energy += dt * d->gmean;
		mass += dt;
	}
	if (!mass)
		return false;
	/* average = clamp(energy / mass, low, high), then to 12 bit */
	if (energy < low * mass)
		l1avg = to12(low);
	else if (energy > high * mass)
		l1avg = to12(high);
	else
		l1avg = div64_u64(energy * 4095 + mass * ONE / 2, mass * ONE);
	l1min = to12(low);
	if (l1min > 12)
		l1min = 12;
	l1max = to12(high);
	if (l1max < 2081)
		l1max = 2081;
	if (l1avg < 819)
		l1avg = 819;
	if (l1avg > l1max - 1)
		l1avg = l1max - 1;
	r->l1[0] = l1min;
	r->l1[1] = l1avg;
	r->l1[2] = l1max;
	r->peak = high;
	return true;
}

static void history_reset(struct cb1t_ctx *ctx, s64 cut)
{
	ctx->active_cut = cut;
	ctx->history_low = ONE;
	ctx->history_high = 0;
	ctx->history_energy = 0;
	ctx->history_mass = 0;
}

enum cb1t_status cb1t_poll(struct cb1t_ctx *ctx, u64 picture, struct cb1t_result *out)
{
	struct cb1t_result result;
	u32 requested = 0, observed, span = 0, start = 0, end, retire, i;
	s64 pts, available;

	if (!ctx->active)
		return CB1T_INVALID;
	while (requested < ctx->count && ctx->samples[requested].picture != picture)
		++requested;
	if (requested == ctx->count)
		return CB1T_INVALID;
	pts = ctx->samples[requested].d.pts;
	if (!ctx->ended && ctx->frontier < pts + ctx->lookahead)
		return CB1T_PENDING;
	available = s64_min(pts + ctx->lookahead, ctx->ended ? ctx->samples[ctx->count - 1].d.pts : ctx->frontier);
	observed = requested + 1;
	while (observed < ctx->count && ctx->samples[observed].d.pts <= pts + ctx->lookahead)
		++observed;

	/* calibration state of the original: a gap restarts the scene history */
	for (i = 1; i <= requested; ++i)
		if (ctx->samples[i].gap)
			span = i;
	if (span && ctx->samples[span].d.pts != ctx->calibration_start) {
		ctx->calibrated = false;
		history_reset(ctx, ctx->samples[span].d.pts);
	}
	if (!ctx->calibrated) {
		ctx->calibrated = true;
		ctx->calibration_start = ctx->samples[span].d.pts;
	}

	end = observed;
	for (i = 1; i < observed; ++i) {
		if (i <= requested) {
			if (confirmed_cut(ctx, i, observed, available))
				start = i;
		} else if (scene_boundary(ctx, i, observed)) {
			/* exclude a future scene before its confirmation can change this frame */
			end = i;
			break;
		}
	}
	if (start && ctx->samples[start].d.pts > ctx->active_cut)
		history_reset(ctx, ctx->samples[start].d.pts);
	while (start < end && ctx->samples[start].d.pts < ctx->active_cut)
		++start;
	memset(&result, 0, sizeof(result));
	result.pts = pts;
	result.observed_until = ctx->samples[observed - 1].d.pts;
	/*
	 * Whatever the lookahead shows of the shot's first `open` ticks raises
	 * max from the cut on, where the jump is hidden, instead of inside the shot.
	 */
	if (start >= end || !baseline(ctx, start, end, end < observed,
				      s64_max(pts + ctx->rise_lead, ctx->active_cut +
					      s64_min(ctx->open, ctx->lookahead)), &result))
		return CB1T_INVALID;
	ramp(ctx, &result);

	/* keep two preceding edge observations; older pictures become scalar history */
	retire = requested > 2 ? requested - 2 : 0;
	for (i = 0; i < retire; ++i)
		if (ctx->samples[i].d.pts >= ctx->active_cut) {
			const struct cb1t_frame *d = &ctx->samples[i].d;
			u64 dt = ctx->samples[i + 1].d.pts - d->pts;

			if (d->gmin < ctx->history_low)
				ctx->history_low = d->gmin;
			if (d->gmax > ctx->history_high)
				ctx->history_high = d->gmax;
			ctx->history_energy += dt * d->gmean;
			ctx->history_mass += dt;
		}
	if (retire) {
		memmove(ctx->samples, ctx->samples + retire,
			(ctx->count - retire) * sizeof(*ctx->samples));
		ctx->count -= retire;
	}
	*out = result;
	return CB1T_READY;
}
