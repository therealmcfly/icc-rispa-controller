#ifndef ICC_H
#define ICC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

	typedef enum IccState
	{
		Q0_RESTING = 0,
		Q1_UPSTROKE = 1,
		Q2_PLATEAU = 2,
		Q3_REPOLARIZATION = 3,
		WAIT = 4
	} IccState;

	typedef struct IccConfig
	{
		float slope_q0;
		float slope_q1;
		float slope_q2;
		float slope_q3;
		float threshold_q0_to_q1;
		float threshold_q1_to_q2;
		float threshold_q2_to_q3;
		float threshold_q3_to_q0;
		float v_reset_default;
		float v_floor;
		uint32_t wait_ms;
	} IccConfig;

	typedef struct Icc
	{
		IccState state;
		float v;
		float vreset;
		float vmax;
		float d2;
		uint32_t wait_ms_accum;
		bool reset;
		bool initialized;
		uint32_t transition_count;
	} Icc;

	void icc_default_config(IccConfig *cfg);
	void icc_init(Icc *state, const IccConfig *cfg);
	float icc_hioa(Icc *state, const IccConfig *cfg, uint32_t dt_ms);
	uint8_t icc_state_index(const Icc *state);

#ifdef __cplusplus
}
#endif

#endif
