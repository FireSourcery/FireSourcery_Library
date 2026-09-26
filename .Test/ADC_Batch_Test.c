/*
    Host test for a set spanning 2 ADCs on 1 trigger: the flag based join, the selection it applies,
    and the degenerate 1 part case.

    Build (host):
        gcc -std=c23 -Wall -Wextra -I<library> -I<library>/.Test -DHAL_PERIPHERAL_PATH_DIRECTORY=HAL \
            <library>/.Test/ADC_Batch_Test.c -o adc_batch_test
*/
#define ADC_HW_SEQUENCER_ENABLE true

#include "Peripheral/ADC/ADC_Batch.h"
#include <stdio.h>

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); fails++; } } while (0)

/* ADC0 holds IA IB VA VB, ADC1 holds IC VSOURCE VC HEAT. A set is 1 part on each */
enum { ADC0_IA, ADC0_IB, ADC0_VA, ADC0_VB, ADC0_COUNT };
enum { ADC1_IC, ADC1_VSOURCE, ADC1_VC, ADC1_HEAT, ADC1_COUNT };

static HAL_ADC_T Adc0Registers;
static HAL_ADC_T Adc1Registers;
static volatile adc_result_t Adc0Results[ADC0_COUNT];
static volatile adc_result_t Adc1Results[ADC1_COUNT];
static ADC_State_T Adc0State;
static ADC_State_T Adc1State;

static const adc_pin_t ADC0_PINS[ADC0_COUNT] = { 10U, 11U, 12U, 13U };
static const adc_pin_t ADC1_PINS[ADC1_COUNT] = { 20U, 21U, 22U, 23U };

static const ADC_T ADCS[2U] =
{
    [0U] = { .P_HAL_ADC = &Adc0Registers, .P_STATE = &Adc0State, .P_CHANNEL_PINS = ADC0_PINS, .CHANNEL_COUNT = ADC0_COUNT, .P_CHANNEL_RESULTS = Adc0Results },
    [1U] = { .P_HAL_ADC = &Adc1Registers, .P_STATE = &Adc1State, .P_CHANNEL_PINS = ADC1_PINS, .CHANNEL_COUNT = ADC1_COUNT, .P_CHANNEL_RESULTS = Adc1Results },
};

/******************************************************************************/
/* Sets                                                                       */
/******************************************************************************/
static ADC_BatchTrigger_T Trigger;

/* Set by a test: the consumer selects the next set from inside the join, as a motor does */
static const ADC_Batch_T * p_RequestBatch;
static uint32_t JoinCount;
static adc_result_t JoinedIa, JoinedIc;

static void OnBatchComplete(void * p_context);

static const ADC_Sequence_T I_PARTS[] =
{
    [0U] = ADC_SEQUENCE_FIELDS(&ADCS[0U], ADC_MASK_RANGE(ADC0_IA, 2U), NULL, NULL),
    [1U] = ADC_SEQUENCE_FIELDS(&ADCS[1U], ADC_MASK_RANGE(ADC1_IC, 2U), NULL, NULL),
};

static const ADC_Sequence_T V_PARTS[] =
{
    [0U] = ADC_SEQUENCE_FIELDS(&ADCS[0U], ADC_MASK_RANGE(ADC0_VA, 2U), NULL, NULL),
    [1U] = ADC_SEQUENCE_FIELDS(&ADCS[1U], ADC_MASK_RANGE(ADC1_VC, 2U), NULL, NULL),
};

/* 1 ADC, to check the degenerate batch */
static const ADC_Sequence_T SOLO_PARTS[] =
{
    [0U] = ADC_SEQUENCE_FIELDS(&ADCS[0U], ADC_MASK_RANGE(ADC0_IA, 2U), NULL, NULL),
};

static const ADC_Batch_T I_BATCH    = ADC_BATCH_INIT(I_PARTS, OnBatchComplete, NULL);
static const ADC_Batch_T V_BATCH    = ADC_BATCH_INIT(V_PARTS, OnBatchComplete, NULL);
static const ADC_Batch_T SOLO_BATCH = ADC_BATCH_INIT(SOLO_PARTS, OnBatchComplete, NULL);

static void OnBatchComplete(void * p_context)
{
    (void)p_context;
    JoinCount++;
    JoinedIa = Adc0Results[ADC0_IA];
    JoinedIc = Adc1Results[ADC1_IC];
    if (p_RequestBatch != NULL) { ADC_BatchTrigger_Select(&Trigger, p_RequestBatch); p_RequestBatch = NULL; }
}

/******************************************************************************/
/* Harness                                                                    */
/******************************************************************************/
/* The trigger this value was converted on. Any channel of either ADC decodes the same way */
static adc_result_t ValueOf(uint32_t trigger, adc_channel_t channel) { return (adc_result_t)((trigger * 100U) + channel); }
static uint32_t TriggerOf(adc_result_t value) { return value / 100U; }

static void Test_Reset(const ADC_Batch_T * p_batch)
{
    Adc0Registers = (HAL_ADC_T){ 0 };
    Adc1Registers = (HAL_ADC_T){ 0 };
    Adc0State = (ADC_State_T){ 0 };
    Adc1State = (ADC_State_T){ 0 };
    Trigger = (ADC_BatchTrigger_T){ 0 };
    JoinCount = 0U;
    p_RequestBatch = NULL;

    ADC_BatchTrigger_ActivateDma(&Trigger, p_batch);
}

/* The Hw converted this ADC's programmed range and the transfer landed. The Board's ISR follows */
static void Test_TransferAndIsr(uint8_t adcIndex, uint32_t trigger)
{
    const HAL_ADC_T * p_hal = ADCS[adcIndex].P_HAL_ADC;
    volatile adc_result_t * p_results = ADCS[adcIndex].P_CHANNEL_RESULTS;

    for (uint8_t slot = p_hal->RangeStart; slot < (p_hal->RangeStart + p_hal->RangeCount); slot++) { p_results[slot] = ValueOf(trigger, slot); }

    ADC_BatchTrigger_OnComplete_ISR(&Trigger, (ADC_T *)&ADCS[adcIndex]);
}

/******************************************************************************/
/* Tests                                                                      */
/******************************************************************************/
/* Activation points every part's sequencer at its own slot range */
static void test_activate(void)
{
    Test_Reset(&I_BATCH);

    CHECK(Adc0Registers.RangeStart == ADC0_IA);
    CHECK(Adc0Registers.RangeCount == 2U);
    CHECK(Adc1Registers.RangeStart == ADC1_IC);
    CHECK(Adc1Registers.RangeCount == 2U);
    CHECK(ADC_BatchTrigger_IsActive(&Trigger, &I_BATCH));
}

/* The join runs on the last part to land, once per trigger, in either completion order */
static void test_join_once_per_trigger(void)
{
    Test_Reset(&I_BATCH);

    Test_TransferAndIsr(0U, 1U);
    CHECK(JoinCount == 0U);         /* ADC1 has not landed */
    Test_TransferAndIsr(1U, 1U);
    CHECK(JoinCount == 1U);

    Test_TransferAndIsr(1U, 2U);    /* the other order */
    CHECK(JoinCount == 1U);
    Test_TransferAndIsr(0U, 2U);
    CHECK(JoinCount == 2U);
}

/* Every value the join reports is from the trigger that just completed */
static void test_join_values_share_a_trigger(void)
{
    Test_Reset(&I_BATCH);

    for (uint32_t trigger = 1U; trigger <= 4U; trigger++)
    {
        Test_TransferAndIsr(0U, trigger);
        Test_TransferAndIsr(1U, trigger);
        CHECK(TriggerOf(JoinedIa) == trigger);
        CHECK(TriggerOf(JoinedIc) == trigger);
    }
    CHECK(JoinCount == 4U);
}

/* A selection is 1 store, and reaches every ADC on the same trigger */
static void test_selection_applies_at_the_join(void)
{
    Test_Reset(&I_BATCH);

    ADC_BatchTrigger_Select(&Trigger, &V_BATCH);
    CHECK(Adc0Registers.RangeStart == ADC0_IA);     /* not yet */

    Test_TransferAndIsr(0U, 1U);
    CHECK(Adc0Registers.RangeStart == ADC0_IA);     /* not until every part has landed */

    Test_TransferAndIsr(1U, 1U);
    CHECK(Adc0Registers.RangeStart == ADC0_VA);     /* both reprogrammed in 1 place */
    CHECK(Adc1Registers.RangeStart == ADC1_VC);
    CHECK(ADC_BatchTrigger_IsActive(&Trigger, &V_BATCH));
}

/* A part that misses its transfer costs 1 join, not the phase */
static void test_missed_completion_recovers(void)
{
    Test_Reset(&I_BATCH);

    Test_TransferAndIsr(0U, 1U);    /* ADC1 drops this trigger */
    CHECK(JoinCount == 0U);

    Test_TransferAndIsr(0U, 2U);    /* the next trigger: ADC0 again, still no ADC1 */
    CHECK(JoinCount == 0U);

    Test_TransferAndIsr(1U, 2U);
    CHECK(JoinCount == 1U);
    CHECK(TriggerOf(JoinedIa) == 2U);   /* the newest of each, not the stale one */
    CHECK(TriggerOf(JoinedIc) == 2U);

    Test_TransferAndIsr(0U, 3U);        /* and the phase is intact */
    Test_TransferAndIsr(1U, 3U);
    CHECK(JoinCount == 2U);
}

/* Next is read after the handler, so a set selected from the join applies on this join */
static void test_selection_from_the_join_applies_now(void)
{
    Test_Reset(&I_BATCH);

    p_RequestBatch = &V_BATCH;

    Test_TransferAndIsr(0U, 1U);
    Test_TransferAndIsr(1U, 1U);
    CHECK(JoinCount == 1U);
    CHECK(ADC_BatchTrigger_IsActive(&Trigger, &V_BATCH));
    CHECK(Adc0Registers.RangeStart == ADC0_VA);
}

/* 1 ADC is a batch of 1 part: the same path, joining on its own completion */
static void test_single_part_batch(void)
{
    Test_Reset(&SOLO_BATCH);

    CHECK(Adc0Registers.RangeStart == ADC0_IA);

    Test_TransferAndIsr(0U, 1U);
    CHECK(JoinCount == 1U);
    Test_TransferAndIsr(0U, 2U);
    CHECK(JoinCount == 2U);
    CHECK(TriggerOf(JoinedIa) == 2U);
}

/* An ADC that is not a part of the active batch is ignored, so no join state survives a switch */
static void test_foreign_adc_is_ignored(void)
{
    Test_Reset(&SOLO_BATCH);

    ADC_OnCompleteTransfer_ISR((ADC_T *)&ADCS[1U], ADC_MASK(ADC1_IC));  /* ADC1 converting for someone else */
    ADC_BatchTrigger_OnComplete_ISR(&Trigger, (ADC_T *)&ADCS[1U]);
    CHECK(JoinCount == 0U);

    Test_TransferAndIsr(0U, 1U);
    CHECK(JoinCount == 1U);
    CHECK(ADC_CompleteFlags((ADC_T *)&ADCS[1U]) == ADC_MASK(ADC1_IC));  /* and its flag was not taken */
}

int main(void)
{
    test_activate();
    test_join_once_per_trigger();
    test_join_values_share_a_trigger();
    test_selection_applies_at_the_join();
    test_missed_completion_recovers();
    test_selection_from_the_join_applies_now();
    test_single_part_batch();
    test_foreign_adc_is_ignored();

    printf(fails == 0 ? "ADC_Batch_Test: PASS\n" : "ADC_Batch_Test: %d FAILED\n", fails);
    return fails;
}
