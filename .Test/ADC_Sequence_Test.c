/*
    Host test for the per ADC Hw sequence path: immediate activation, deferred selection, and the
    completion that applies it. 1 ADC, no batch.

    Build (host):
        gcc -std=c23 -Wall -Wextra -I<library> -I<library>/.Test -DHAL_PERIPHERAL_PATH_DIRECTORY=HAL \
            <library>/.Test/ADC_Sequence_Test.c -o adc_sequence_test
*/
#define ADC_HW_SEQUENCER_ENABLE true

#include "Peripheral/ADC/ADC_Thread.h"
#include <stdio.h>

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); fails++; } } while (0)

enum { SLOT_IA, SLOT_IB, SLOT_VA, SLOT_VB, SLOT_HEAT, SLOT_VSOURCE, SLOT_COUNT };
enum { SEQUENCE_I, SEQUENCE_V, SEQUENCE_MONITOR, SEQUENCE_COUNT };

static HAL_ADC_T AdcRegisters;
static volatile adc_result_t AdcResults[SLOT_COUNT];

static const ADC_Channel_T CHANNELS[SLOT_COUNT] =
{
    [SLOT_IA]      = ADC_CHANNEL_INIT(SLOT_IA, 10U, NULL, NULL),
    [SLOT_IB]      = ADC_CHANNEL_INIT(SLOT_IB, 11U, NULL, NULL),
    [SLOT_VA]      = ADC_CHANNEL_INIT(SLOT_VA, 12U, NULL, NULL),
    [SLOT_VB]      = ADC_CHANNEL_INIT(SLOT_VB, 13U, NULL, NULL),
    [SLOT_HEAT]    = ADC_CHANNEL_INIT(SLOT_HEAT, 14U, NULL, NULL),
    [SLOT_VSOURCE] = ADC_CHANNEL_INIT(SLOT_VSOURCE, 15U, NULL, NULL),
};

/* Set by the test when a sequence's own handler should request another set */
static ADC_T * p_RequestFrom;
static int RequestSequenceId = -1;
static uint32_t CompleteCount;

static void OnSequenceComplete(void * p_context)
{
    (void)p_context;
    CompleteCount++;
    if (RequestSequenceId >= 0) { ADC_SetNextSequenceDma(p_RequestFrom, (uint8_t)RequestSequenceId); RequestSequenceId = -1; }
}

static const ADC_Sequence_T SEQUENCES[SEQUENCE_COUNT] =
{
    [SEQUENCE_I]       = ADC_SEQUENCE(ADC_MASK_RANGE(SLOT_IA, 2U), OnSequenceComplete, NULL),
    [SEQUENCE_V]       = ADC_SEQUENCE(ADC_MASK_RANGE(SLOT_VA, 2U), OnSequenceComplete, NULL),
    [SEQUENCE_MONITOR] = ADC_SEQUENCE(ADC_MASK_RANGE(SLOT_HEAT, 2U), OnSequenceComplete, NULL),
};

static ADC_State_T AdcState;

static const ADC_T ADC_MODULE =
{
    .P_HAL_ADC = &AdcRegisters,
    .P_STATE = &AdcState,
    .P_CHANNELS = CHANNELS,
    .CHANNEL_COUNT = SLOT_COUNT,
    .P_CHANNEL_RESULTS = AdcResults,
    .P_SEQUENCES = SEQUENCES,
    .SEQUENCE_COUNT = SEQUENCE_COUNT,
};

static ADC_T * const p_Adc = (ADC_T *)&ADC_MODULE;

static void Test_Reset(uint8_t sequenceId)
{
    AdcRegisters = (HAL_ADC_T){ 0 };
    AdcState = (ADC_State_T){ 0 };
    CompleteCount = 0U;
    RequestSequenceId = -1;
    p_RequestFrom = p_Adc;

    ADC_ActivateSequenceDma(p_Adc, sequenceId);
}

/* The Hw converted the programmed range, the transfer landed */
static void Test_TriggerAndIsr(void) { ADC_OnCompleteSequenceDma_ISR(p_Adc); }

/* Activation writes the Hw and leaves nothing pending, so the first completion switches nothing */
static void test_activate_leaves_nothing_pending(void)
{
    Test_Reset(SEQUENCE_I);

    CHECK(AdcRegisters.RangeStart == SLOT_IA);
    CHECK(AdcRegisters.RangeCount == 2U);
    CHECK(ADC_IsSequenceActive(p_Adc, SEQUENCE_I));

    uint32_t writes = AdcRegisters.RangeWrites;
    Test_TriggerAndIsr();
    CHECK(AdcRegisters.RangeWrites == writes);      /* nothing to apply */
    CHECK(ADC_IsSequenceActive(p_Adc, SEQUENCE_I));
}

/* A selection is deferred: the Hw keeps converting the active set until the completion applies it */
static void test_selection_applies_at_the_completion(void)
{
    Test_Reset(SEQUENCE_I);

    ADC_SetNextSequenceDma(p_Adc, SEQUENCE_V);
    CHECK(AdcRegisters.RangeStart == SLOT_IA);      /* not yet */
    CHECK(ADC_IsSequenceActive(p_Adc, SEQUENCE_I));

    Test_TriggerAndIsr();
    CHECK(AdcRegisters.RangeStart == SLOT_VA);      /* applied, where the ADC is idle */
    CHECK(ADC_IsSequenceActive(p_Adc, SEQUENCE_V));

    uint32_t writes = AdcRegisters.RangeWrites;     /* and it repeats, without rewriting */
    Test_TriggerAndIsr();
    CHECK(AdcRegisters.RangeWrites == writes);
}

/* Last request wins */
static void test_last_request_wins(void)
{
    Test_Reset(SEQUENCE_I);

    ADC_SetNextSequenceDma(p_Adc, SEQUENCE_V);
    ADC_SetNextSequenceDma(p_Adc, SEQUENCE_MONITOR);

    Test_TriggerAndIsr();
    CHECK(ADC_IsSequenceActive(p_Adc, SEQUENCE_MONITOR));
}

/*
    A selection made while the completion is running must not be lost. Whether it lands on this
    completion or the one after is not pinned here: it depends on where Next is read relative to the
    handler, and the selection normally comes from the outer handler rather than from COMPLETE.
    What must hold is that it is still selected afterwards, and that it converts.
*/
static void test_selection_during_the_completion_is_not_lost(void)
{
    Test_Reset(SEQUENCE_I);

    ADC_SetNextSequenceDma(p_Adc, SEQUENCE_V);  /* pending: this completion applies V */
    RequestSequenceId = SEQUENCE_MONITOR;       /* and the handler asks for MONITOR while it runs */

    Test_TriggerAndIsr();
    CHECK(CompleteCount == 1U);
    CHECK(ADC_IsSequenceSelected(p_Adc, SEQUENCE_MONITOR));  /* not overwritten by a stale snapshot */

    Test_TriggerAndIsr();                                    /* applied by the completion that follows, at the latest */
    CHECK(ADC_IsSequenceActive(p_Adc, SEQUENCE_MONITOR));
    CHECK(AdcRegisters.RangeStart == SLOT_HEAT);

    uint32_t writes = AdcRegisters.RangeWrites;              /* and then it repeats, without rewriting */
    Test_TriggerAndIsr();
    CHECK(AdcRegisters.RangeWrites == writes);
}

int main(void)
{
    test_activate_leaves_nothing_pending();
    test_selection_applies_at_the_completion();
    test_last_request_wins();
    test_selection_during_the_completion_is_not_lost();

    printf(fails == 0 ? "ADC_Sequence_Test: PASS\n" : "ADC_Sequence_Test: %d FAILED\n", fails);
    return fails;
}
