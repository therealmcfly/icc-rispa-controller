#include "icc.h"

static float clampf(float x, float lo, float hi)
{
	if (x < lo)
	{
		return lo;
	}
	if (x > hi)
	{
		return hi;
	}
	return x;
}

void icc_default_config(IccConfig *cfg)
{
	if (cfg == 0)
	{
		return;
	}

	cfg->slope_q0 = -0.069979275f;
	cfg->slope_q1 = 43.5248f;
	cfg->slope_q2 = -0.909759259f;
	cfg->slope_q3 = -8.636136364f;
	cfg->threshold_q0_to_q1 = -67.6339f;
	cfg->threshold_q1_to_q2 = -24.1091f;
	cfg->threshold_q2_to_q3 = -28.9894f;
	cfg->threshold_q3_to_q0 = -66.9884f;
	cfg->v_reset_default = -67.6336f;
	cfg->v_floor = -67.0f;
	cfg->wait_ms = 4999U;
}

void icc_init(Icc *icc, const IccConfig *cfg)
{
	if (icc == 0 || cfg == 0)
	{
		return;
	}

	icc->state = WAIT;
	icc->v = 0.0f;
	icc->vreset = cfg->v_reset_default;
	icc->vmax = cfg->threshold_q1_to_q2;
	icc->d2 = cfg->slope_q2;
	icc->wait_ms_accum = 0U;
	icc->reset = false;
	icc->initialized = true;
	icc->transition_count = 0U;
}

float icc_hioa(Icc *icc, const IccConfig *cfg, uint32_t dt_ms)
{
	bool did_reset = false;
	float dt_seconds = dt_ms * 1.0e-3f;

	if (icc == 0 || cfg == 0 || dt_ms == 0U || !icc->initialized)
	{
		return 0.0f;
	}

	switch (icc->state)
	{
	case Q0_RESTING:
		if (icc->v < cfg->threshold_q0_to_q1)
		{
			icc->vreset = icc->v;
			icc->vmax = cfg->threshold_q1_to_q2;
			icc->d2 = cfg->slope_q2;
			icc->state = Q1_UPSTROKE;
			did_reset = true;
		}
		break;

	case Q1_UPSTROKE:
		if (icc->v >= icc->vmax)
		{
			icc->vreset = icc->v;
			icc->state = Q2_PLATEAU;
			did_reset = true;
		}
		break;

	case Q2_PLATEAU:
		if (icc->v < cfg->threshold_q2_to_q3)
		{
			icc->vreset = icc->v;
			icc->state = Q3_REPOLARIZATION;
			did_reset = true;
		}
		break;

	case Q3_REPOLARIZATION:
		if (icc->v < cfg->threshold_q3_to_q0)
		{
			icc->vreset = icc->v;
			icc->state = Q0_RESTING;
			did_reset = true;
		}
		break;

	case WAIT:
	default:
		icc->wait_ms_accum += dt_ms;
		if (icc->wait_ms_accum >= cfg->wait_ms)
		{
			icc->vreset = cfg->v_reset_default;
			icc->state = Q0_RESTING;
			icc->wait_ms_accum = 0U;
			did_reset = true;
		}
		break;
	}

	if (did_reset)
	{
		icc->reset = !icc->reset;
		icc->v = icc->vreset;
		icc->transition_count++;
	}

	switch (icc->state)
	{
	case Q0_RESTING:
		icc->v += cfg->slope_q0 * dt_seconds;
		break;

	case Q1_UPSTROKE:
		icc->v += cfg->slope_q1 * dt_seconds;
		icc->v = clampf(icc->v, -1.0e30f, cfg->threshold_q1_to_q2);
		break;

	case Q2_PLATEAU:
		icc->v += icc->d2 * dt_seconds;
		break;

	case Q3_REPOLARIZATION:
		icc->v += cfg->slope_q3 * dt_seconds;
		icc->v = clampf(icc->v, cfg->v_floor, 1.0e30f);
		break;

	case WAIT:
	default:
		break;
	}

	return icc->v;
}

uint8_t icc_state_index(const Icc *icc)
{
	if (icc == 0)
	{
		return (uint8_t)WAIT;
	}
	return (uint8_t)icc->state;
}
