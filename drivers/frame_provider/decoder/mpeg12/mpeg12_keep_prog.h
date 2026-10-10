/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef MPEG12_KEEP_PROG_H
#define MPEG12_KEEP_PROG_H

/* Decoder frame ticks (96000Hz). State belongs to this decoder instance. */
struct mpeg12_keep_prog {
	unsigned int last_rff;
	unsigned int alternations;
	unsigned int remainder;
	unsigned int base_duration;
	unsigned int output_duration;
	unsigned int active;
};

static inline void mpeg12_keep_prog_reset(struct mpeg12_keep_prog *state)
{
	state->last_rff = 2;
	state->alternations = 0;
	state->remainder = 0;
	state->base_duration = 0;
	state->output_duration = 0;
	state->active = 0;
}

/* Return zero unless this current picture has earned film duration. */
static inline unsigned int mpeg12_keep_prog_duration(
	struct mpeg12_keep_prog *state, unsigned int enabled,
	unsigned int progressive, unsigned int progressive_sequence,
	unsigned int rff, unsigned int base_duration)
{
	unsigned int numerator;

	/* NTSC 29.97/30 frames only; PAL and progressive-sequence RFF stay native. */
	if (!enabled || !progressive || progressive_sequence ||
		base_duration < 3200 || base_duration > 3206) {
		mpeg12_keep_prog_reset(state);
		return 0;
	}
	if (state->base_duration != base_duration)
		mpeg12_keep_prog_reset(state);
	state->base_duration = base_duration;
	if (state->last_rff < 2 && state->last_rff != rff) {
		if (state->alternations < 3)
			state->alternations++;
	} else {
		state->alternations = 0;
		state->remainder = 0;
	}
	state->last_rff = rff;
	state->active = state->alternations >= 3;
	state->output_duration = 0;
	if (!state->active)
		return 0;

	/* Five fields per pair of frames; retain every quarter tick for odd bases. */
	numerator = 5 * base_duration + state->remainder;
	state->output_duration = numerator / 4;
	state->remainder = numerator % 4;
	return state->output_duration;
}
#endif
