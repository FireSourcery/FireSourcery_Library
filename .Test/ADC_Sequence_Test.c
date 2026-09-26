/*
    Host test for the per ADC set: completion flags, immediate activation, deferred selection,
    and the completion that applies it. 1 ADC, no batch.

    Build (host):
        gcc -std=c23 -Wall -Wextra -I<library> -I<library>/.Test -DHAL_PERIPHERAL_PATH_DIRECTORY=HAL \
            <library>/.Test/ADC_Sequence_Test.c -o adc_sequence_test
*/
#define ADC_HW_SEQUENCER_ENABLE true

#include "Peripheral/ADC/ADC_Sequence.h"
#include <stdio.h>

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); fails++; } } while (0)

enum { SLOT_IA, SLOT_IB, SLOT_VA, SLOT_VB, SLOT_HEAT, SLOT_VSOURCE, SLOT_COUNT };

static HAL_ADC_T AdcRegisters;
static volatile adc_result_t AdcResults[SLOT_COUNT];
static ADC_State_T AdcState;

static const adc_pin_t PINS[SLOT_COUNT] = { 10U, 11U, 12U, 13U, 14U, 15U };

static const ADC_T ADC_MODULE =
{
    .P_HAL_ADC = &AdcRegisters,
    .P_STATE = &AdcState,
    .P_CHANNEL_PINS = PINS,
    .CHANNEL_COUNT = SLOT_COUNT,
    .P_CHANNEL_RESULTS = AdcResults,
};

static ADC_T * const p_Adc = (ADC_T *)&ADC_MODULE;

/* Set by a test when the set's own handler should select another, as a consumer does */
static const ADC_Sequence_T * p_RequestSequence;
static uint32_t CompleteCount;

static ADC_SequenceTrigger_T Trigger;

static void OnSequenceComplete(void * p_context);

static const ADC_Sequence_T SEQUENCE_I       = ADC_SEQUENCE(&ADC_MODULE, ADC_MASK_RANGE(SLOT_IA, 2U), OnSequenceComplete, NULL);
static const ADC_Sequence_T SEQUENCE_V       = ADC_SEQUENCE(&ADC_MODULE, ADC_MASK_RANGE(SLOT_VA, 2U), OnSequenceComplete, NULL);
static const ADC_Sequence_T SEQUENCE_MONITOR = ADC_SEQUENCE(&ADC_MODULE, ADC_MASK_RANGE(SLOT_HEAT, 2U), OnSequenceComplete, NULL);

static void OnSequenceComplete(void * p_context)
{
    (void)p_context;
    CompleteCount++;
    if (p_RequestSequence != NULL) { ADC_SequenceTrigger_Select(&Trigger, p_RequestSequence); p_RequestSequence = NULL; }
}

static void Test_Reset(const ADC_Sequence_T * p_sequence)
{
    AdcRegisters = (HAL_ADC_T){ 0 };
    AdcState = (ADC_State_T){ 0 };
    Trigger = (ADC_SequenceTrigger_T){ 0 };
    CompleteCount = 0U;
    p_RequestSequence = NULL;

    ADC_SequenceTrigger_ActivateDma(&Trigger, p_sequence);
}

/* The Hw converted the programmed range, the transfer landed */
static void Test_TriggerAndIsr(void) { ADC_SequenceTrigger_OnCompleteDma_ISR(&Trigger); }

/******************************************************************************/
/* Completion flags                                                           */
/******************************************************************************/
/* Take is the edge: the set closes once, and the level does not repeat on the next completion */
static void test_take_is_an_edge(void)
{
    Test_Reset(&SEQUENCE_I);

    CHECK(ADC_Sequence_IsComplete(&SEQUENCE_I) == false);

    ADC_OnCompleteTransfer_ISR(p_Adc, SEQUENCE_I.CHANNELS);
    CHECK(ADC_Sequence_IsComplete(&SEQUENCE_I) == true);        /* peek is a level, and stays */
    CHECK(ADC_Sequence_IsComplete(&SEQUENCE_I) == true);

    CHECK(ADC_Sequence_TakeComplete(&SEQUENCE_I) == true);      /* take consumes it */
    CHECK(ADC_Sequence_TakeComplete(&SEQUENCE_I) == false);
    CHECK(ADC_Sequence_IsComplete(&SEQUENCE_I) == false);
}

/* A partial set keeps its flags, so a set that straddles 2 completions still closes */
static void test_partial_set_keeps_its_flags(void)
{
    Test_Reset(&SEQUENCE_I);

    ADC_OnCompleteTransfer_ISR(p_Adc, ADC_MASK(SLOT_IA));
    CHECK(ADC_Sequence_TakeComplete(&SEQUENCE_I) == false);
    CHECK(ADC_CompleteFlags(p_Adc) == ADC_MASK(SLOT_IA));       /* not consumed */

    ADC_OnCompleteTransfer_ISR(p_Adc, ADC_MASK(SLOT_IB));
    CHECK(ADC_Sequence_TakeComplete(&SEQUENCE_I) == true);
    CHECK(ADC_CompleteFlags(p_Adc) == 0UL);
}

/* A consumer takes only what it names, and leaves the rest for its own consumer */
static void test_take_leaves_what_it_does_not_name(void)
{
    Test_Reset(&SEQUENCE_I);

    ADC_OnCompleteTransfer_ISR(p_Adc, SEQUENCE_I.CHANNELS | SEQUENCE_V.CHANNELS);

    CHECK(ADC_Sequence_TakeComplete(&SEQUENCE_I) == true);
    CHECK(ADC_CompleteFlags(p_Adc) == SEQUENCE_V.CHANNELS);
    CHECK(ADC_Sequence_TakeComplete(&SEQUENCE_V) == true);
}

/******************************************************************************/
/* Selection                                                                  */
/******************************************************************************/
/* Activation writes the Hw and leaves nothing pending, so the first completion switches nothing */
static void test_activate_leaves_nothing_pending(void)
{
    Test_Reset(&SEQUENCE_I);

    CHECK(AdcRegisters.RangeStart == SLOT_IA);
    CHECK(AdcRegisters.RangeCount == 2U);
    CHECK(ADC_SequenceTrigger_IsActive(&Trigger, &SEQUENCE_I));

    uint32_t writes = AdcRegisters.RangeWrites;
    Test_TriggerAndIsr();
    CHECK(AdcRegisters.RangeWrites == writes);      /* nothing to apply */
    CHECK(ADC_SequenceTrigger_IsActive(&Trigger, &SEQUENCE_I));
    CHECK(CompleteCount == 1U);                     /* and the set closed */
}

/* A selection is deferred: the Hw keeps converting the active set until the completion applies it */
static void test_selection_applies_at_the_completion(void)
{
    Test_Reset(&SEQUENCE_I);

    ADC_SequenceTrigger_Select(&Trigger, &SEQUENCE_V);
    CHECK(AdcRegisters.RangeStart == SLOT_IA);      /* not yet */
    CHECK(ADC_SequenceTrigger_IsActive(&Trigger, &SEQUENCE_I));

    Test_TriggerAndIsr();
    CHECK(AdcRegisters.RangeStart == SLOT_VA);      /* applied, where the ADC is idle */
    CHECK(ADC_SequenceTrigger_IsActive(&Trigger, &SEQUENCE_V));

    uint32_t writes = AdcRegisters.RangeWrites;     /* and it repeats, without rewriting */
    Test_TriggerAndIsr();
    CHECK(AdcRegisters.RangeWrites == writes);
}

/* Last request wins */
static void test_last_request_wins(void)
{
    Test_Reset(&SEQUENCE_I);

    ADC_SequenceTrigger_Select(&Trigger, &SEQUENCE_V);
    ADC_SequenceTrigger_Select(&Trigger, &SEQUENCE_MONITOR);

    Test_TriggerAndIsr();
    CHECK(ADC_SequenceTrigger_IsActive(&Trigger, &SEQUENCE_MONITOR));
}

/*
    A selection made while the completion is running must not be lost. Next is read after the
    handler, so a set selected from there applies on this completion rather than the one after.
*/
static void test_selection_during_the_completion_applies_now(void)
{
    Test_Reset(&SEQUENCE_I);

    ADC_SequenceTrigger_Select(&Trigger, &SEQUENCE_V);   /* pending: this completion would apply V */
    p_RequestSequence = &SEQUENCE_MONITOR;               /* and the handler asks for MONITOR while it runs */

    Test_TriggerAndIsr();
    CHECK(CompleteCount == 1U);
    CHECK(ADC_SequenceTrigger_IsSelected(&Trigger, &SEQUENCE_MONITOR));  /* not overwritten by a stale snapshot */
    CHECK(ADC_SequenceTrigger_IsActive(&Trigger, &SEQUENCE_MONITOR));
    CHECK(AdcRegisters.RangeStart == SLOT_HEAT);

    uint32_t writes = AdcRegisters.RangeWrites;          /* and then it repeats, without rewriting */
    Test_TriggerAndIsr();
    CHECK(AdcRegisters.RangeWrites == writes);
}

/******************************************************************************/
/* Software activation, same set                                              */
/******************************************************************************/
/* The walker converts the marked channels 1 fifo at a time, and the set closes on the last */
static void test_software_activation_closes_the_set(void)
{
    AdcRegisters = (HAL_ADC_T){ 0 };
    AdcState = (ADC_State_T){ 0 };
    Trigger = (ADC_SequenceTrigger_T){ 0 };
    CompleteCount = 0U;
    p_RequestSequence = NULL;

    ADC_Sequence_Activate(&SEQUENCE_I);
    CHECK(ADC_IsMarkedAll(p_Adc, SEQUENCE_I.CHANNELS));

    AdcRegisters.R[0U] = 111U;
    ADC_OnComplete_ISR(p_Adc);                      /* SLOT_IA captured, SLOT_IB continues */
    CHECK(ADC_Sequence_Poll(&SEQUENCE_I) == false);

    AdcRegisters.R[0U] = 222U;
    ADC_OnComplete_ISR(p_Adc);                      /* SLOT_IB captured, the set is whole */
    CHECK(ADC_Sequence_Poll(&SEQUENCE_I) == true);
    CHECK(CompleteCount == 1U);

    CHECK(AdcResults[SLOT_IA] == 111U);
    CHECK(AdcResults[SLOT_IB] == 222U);
    CHECK(ADC_IsMarkedAll(p_Adc, SEQUENCE_I.CHANNELS) == false);
}

int main(void)
{
    test_take_is_an_edge();
    test_partial_set_keeps_its_flags();
    test_take_leaves_what_it_does_not_name();
    test_activate_leaves_nothing_pending();
    test_selection_applies_at_the_completion();
    test_last_request_wins();
    test_selection_during_the_completion_applies_now();
    test_software_activation_closes_the_set();

    printf(fails == 0 ? "ADC_Sequence_Test: PASS\n" : "ADC_Sequence_Test: %d FAILED\n", fails);
    return fails;
}
