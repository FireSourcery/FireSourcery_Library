/*
    Host test for the ADC_Batch join, on the proposed S32K1 split:
    1 PWM trigger, IA IB on ADC0 and IC VSOURCE on ADC1, each with its own sequencer and transfer.

    The join must fire once per trigger, only after every part has landed, with every value from
    that same trigger, and a selection must reach both ADCs on the same trigger.

    Build (host):
        gcc -std=c23 -Wall -Wextra -I<library> -I<library>/.Test -DHAL_PERIPHERAL_PATH_DIRECTORY=HAL \
            <library>/.Test/ADC_Batch_Test.c -o adc_batch_test
*/
#define ADC_HW_SEQUENCER_ENABLE true

#include "Peripheral/ADC/ADC_Batch.h"
#include <stdio.h>

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); fails++; } } while (0)

/******************************************************************************/
/*
    The board: 2 ADCs on 1 trigger
*/
/******************************************************************************/
enum { ADC0_IA, ADC0_IB, ADC0_VA, ADC0_VB, ADC0_SLOT_COUNT };
enum { ADC1_IC, ADC1_VSOURCE, ADC1_VC, ADC1_HEAT, ADC1_SLOT_COUNT };

enum { ADC_0, ADC_1, ADC_COUNT };

static HAL_ADC_T Adc0Registers;
static HAL_ADC_T Adc1Registers;

static volatile adc_result_t Adc0Results[ADC0_SLOT_COUNT];
static volatile adc_result_t Adc1Results[ADC1_SLOT_COUNT];

static const ADC_Channel_T ADC0_CHANNELS[ADC0_SLOT_COUNT] =
{
    [ADC0_IA] = ADC_CHANNEL_INIT(ADC0_IA, 10U, NULL, NULL),
    [ADC0_IB] = ADC_CHANNEL_INIT(ADC0_IB, 11U, NULL, NULL),
    [ADC0_VA] = ADC_CHANNEL_INIT(ADC0_VA, 12U, NULL, NULL),
    [ADC0_VB] = ADC_CHANNEL_INIT(ADC0_VB, 13U, NULL, NULL),
};

static const ADC_Channel_T ADC1_CHANNELS[ADC1_SLOT_COUNT] =
{
    [ADC1_IC]      = ADC_CHANNEL_INIT(ADC1_IC, 20U, NULL, NULL),
    [ADC1_VSOURCE] = ADC_CHANNEL_INIT(ADC1_VSOURCE, 21U, NULL, NULL),
    [ADC1_VC]      = ADC_CHANNEL_INIT(ADC1_VC, 22U, NULL, NULL),
    [ADC1_HEAT]    = ADC_CHANNEL_INIT(ADC1_HEAT, 23U, NULL, NULL),
};

static ADC_State_T Adc0State;
static ADC_State_T Adc1State;

static const ADC_T ADCS[ADC_COUNT] =
{
    [ADC_0] = { .P_HAL_ADC = &Adc0Registers, .P_STATE = &Adc0State, .P_CHANNELS = ADC0_CHANNELS, .CHANNEL_COUNT = ADC0_SLOT_COUNT,
                .P_CHANNEL_RESULTS = Adc0Results, .P_SEQUENCES = NULL, .SEQUENCE_COUNT = 0U },
    [ADC_1] = { .P_HAL_ADC = &Adc1Registers, .P_STATE = &Adc1State, .P_CHANNELS = ADC1_CHANNELS, .CHANNEL_COUNT = ADC1_SLOT_COUNT,
                .P_CHANNEL_RESULTS = Adc1Results, .P_SEQUENCES = NULL, .SEQUENCE_COUNT = 0U },
};

static ADC_TriggerState_T MOTOR0_TRIGGER;

static const ADC_BatchPart_T MOTOR0_I_PARTS[] =
{
    [0U] = ADC_BATCH_PART(&ADCS[ADC_0], ADC_MASK_RANGE(ADC0_IA, 2U)),   /* IA IB */
    [1U] = ADC_BATCH_PART(&ADCS[ADC_1], ADC_MASK_RANGE(ADC1_IC, 2U)),   /* IC VSOURCE */
};

static const ADC_BatchPart_T MOTOR0_V_PARTS[] =
{
    [0U] = ADC_BATCH_PART(&ADCS[ADC_0], ADC_MASK_RANGE(ADC0_VA, 2U)),   /* VA VB */
    [1U] = ADC_BATCH_PART(&ADCS[ADC_1], ADC_MASK_RANGE(ADC1_VC, 2U)),   /* VC HEAT */
};

/* 1 ADC, to check the degenerate batch */
static const ADC_BatchPart_T MOTOR0_HEAT_PARTS[] =
{
    [0U] = ADC_BATCH_PART(&ADCS[ADC_1], ADC_MASK_RANGE(ADC1_VC, 2U)),
};

/******************************************************************************/
/*
    What the join reports
*/
/******************************************************************************/
static struct
{
    uint32_t Count;
    uint16_t Ia, Ib, Ic, VSource;
    void * p_Context;
}
Joined;

/* Set by a test: the consumer selects the next set from inside the join, as a motor does */
static const ADC_Batch_T * SelectFromJoin;

static void OnBatchComplete(void * p_context)
{
    Joined.Count++;
    Joined.p_Context = p_context;
    Joined.Ia = ADC_ResultOf((ADC_T *)&ADCS[ADC_0], ADC0_IA);
    Joined.Ib = ADC_ResultOf((ADC_T *)&ADCS[ADC_0], ADC0_IB);
    Joined.Ic = ADC_ResultOf((ADC_T *)&ADCS[ADC_1], ADC1_IC);
    Joined.VSource = ADC_ResultOf((ADC_T *)&ADCS[ADC_1], ADC1_VSOURCE);

    if (SelectFromJoin != NULL) { ADC_Batch_Select(&MOTOR0_TRIGGER, (ADC_Batch_T *)SelectFromJoin); SelectFromJoin = NULL; }
}

static const ADC_Batch_T MOTOR0_I_BATCH    = ADC_BATCH_INIT(MOTOR0_I_PARTS, OnBatchComplete, 1U);
static const ADC_Batch_T MOTOR0_V_BATCH    = ADC_BATCH_INIT(MOTOR0_V_PARTS, OnBatchComplete, 2U);
static const ADC_Batch_T MOTOR0_HEAT_BATCH = ADC_BATCH_INIT(MOTOR0_HEAT_PARTS, OnBatchComplete, 3U);

/******************************************************************************/
/*
    The Hw. A trigger converts the slot range each sequencer points at, and the transfer copies
    every slot, as the board's TCD does - the group that did not run re-copies its last results.
*/
/******************************************************************************/
static uint16_t TriggerCount;

/* The trigger this value was converted on. Any channel of any ADC decodes the same way */
#define SAMPLE_OF(Trigger, Adc, Slot)   ((uint16_t)(((Trigger) << 8U) | ((Adc) << 4U) | (Slot)))
#define TRIGGER_OF(Sample)              ((uint16_t)((Sample) >> 8U))

static void Test_Trigger(void)
{
    TriggerCount++;
    for (uint8_t iAdc = 0U; iAdc < ADC_COUNT; iAdc++)
    {
        HAL_ADC_T * p_hal = ADCS[iAdc].P_HAL_ADC;
        for (uint8_t slot = p_hal->RangeStart; slot < (p_hal->RangeStart + p_hal->RangeCount); slot++)
        {
            p_hal->R[slot] = SAMPLE_OF(TriggerCount, iAdc, slot);
        }
        p_hal->IsComplete = true;
    }
}

/* The Board's ISR: the transfer landed, then the join */
static void Test_Isr(uint8_t iAdc)
{
    HAL_ADC_T * p_hal = ADCS[iAdc].P_HAL_ADC;

    for (uint8_t slot = 0U; slot < ADCS[iAdc].CHANNEL_COUNT; slot++) { ADCS[iAdc].P_CHANNEL_RESULTS[slot] = (adc_result_t)p_hal->R[slot]; }
    p_hal->IsComplete = false;

    ADC_Trigger_OnComplete_ISR(&MOTOR0_TRIGGER, (ADC_T *)&ADCS[iAdc]);
}

static void Test_Reset(const ADC_Batch_T * p_batch)
{
    Adc0Registers = (HAL_ADC_T){ 0 };
    Adc1Registers = (HAL_ADC_T){ 0 };
    Adc0State = (ADC_State_T){ 0 };
    Adc1State = (ADC_State_T){ 0 };
    MOTOR0_TRIGGER = (ADC_TriggerState_T){ 0 };
    Joined = (typeof(Joined)){ 0 };
    TriggerCount = 0U;
    SelectFromJoin = NULL;

    ADC_Batch_Activate(&MOTOR0_TRIGGER, (ADC_Batch_T *)p_batch);
}

/******************************************************************************/
/*
    Tests
*/
/******************************************************************************/
/* The join runs on the last part to land, once per trigger, in either completion order */
static void test_join_once_per_trigger(void)
{
    Test_Reset(&MOTOR0_I_BATCH);

    Test_Trigger();
    Test_Isr(ADC_0);
    CHECK(Joined.Count == 0U);      /* ADC1 has not landed */
    Test_Isr(ADC_1);
    CHECK(Joined.Count == 1U);

    Test_Trigger();                 /* the reverse order joins the same way */
    Test_Isr(ADC_1);
    CHECK(Joined.Count == 1U);
    Test_Isr(ADC_0);
    CHECK(Joined.Count == 2U);

    for (uint8_t i = 0U; i < 10U; i++) { Test_Trigger(); Test_Isr(ADC_0); Test_Isr(ADC_1); }
    CHECK(Joined.Count == 12U);     /* exactly 1 per trigger, never 0 and never 2 */
}

/* Every value the join reports is from the trigger that just completed */
static void test_join_values_share_a_trigger(void)
{
    Test_Reset(&MOTOR0_I_BATCH);

    for (uint8_t i = 0U; i < 4U; i++)
    {
        Test_Trigger();
        Test_Isr(ADC_0);
        Test_Isr(ADC_1);

        CHECK(TRIGGER_OF(Joined.Ia) == TriggerCount);
        CHECK(TRIGGER_OF(Joined.Ib) == TriggerCount);
        CHECK(TRIGGER_OF(Joined.Ic) == TriggerCount);
        CHECK(TRIGGER_OF(Joined.VSource) == TriggerCount);
    }
}

/* A selection is 1 store, and reaches every ADC on the same trigger */
static void test_selection_applies_at_the_join(void)
{
    Test_Reset(&MOTOR0_I_BATCH);

    Test_Trigger();
    Test_Isr(ADC_0);

    ADC_Batch_Select(&MOTOR0_TRIGGER, (ADC_Batch_T *)&MOTOR0_V_BATCH);   /* mid flight, after 1 part landed */
    CHECK(ADC_Batch_IsSelected(&MOTOR0_TRIGGER, &MOTOR0_V_BATCH));
    CHECK(ADC_Batch_IsActive(&MOTOR0_TRIGGER, &MOTOR0_I_BATCH));         /* not applied yet */
    CHECK(Adc0Registers.RangeStart == ADC0_IA);                          /* the Hw is still on I */

    Test_Isr(ADC_1);                                                     /* the join applies it */
    CHECK(Joined.p_Context == (void *)1U);                               /* the completing batch is the one reported */
    CHECK(ADC_Batch_IsActive(&MOTOR0_TRIGGER, &MOTOR0_V_BATCH));
    CHECK(Adc0Registers.RangeStart == ADC0_VA);                          /* both ADCs switched */
    CHECK(Adc1Registers.RangeStart == ADC1_VC);

    Test_Trigger();                                                      /* the next trigger converts V on both */
    Test_Isr(ADC_0);
    CHECK(Joined.Count == 1U);                                           /* the switch armed V, so the join still waits for ADC1 */
    Test_Isr(ADC_1);
    CHECK(Joined.Count == 2U);
    CHECK(Joined.p_Context == (void *)2U);

    /* Re-selecting the active batch reprograms nothing */
    uint32_t writes = Adc0Registers.RangeWrites;
    ADC_Batch_Select(&MOTOR0_TRIGGER, (ADC_Batch_T *)&MOTOR0_V_BATCH);
    Test_Trigger();
    Test_Isr(ADC_0);
    Test_Isr(ADC_1);
    CHECK(Adc0Registers.RangeWrites == writes);
}

/* A part that misses its transfer costs 1 join, not the phase */
static void test_missed_completion_recovers(void)
{
    Test_Reset(&MOTOR0_I_BATCH);

    Test_Trigger();
    Test_Isr(ADC_0);
    Test_Isr(ADC_1);
    CHECK(Joined.Count == 1U);

    Test_Trigger();             /* ADC1's transfer is lost */
    Test_Isr(ADC_0);
    CHECK(Joined.Count == 1U);  /* no join: ADC1 is still marked */

    Test_Trigger();             /* both land again */
    Test_Isr(ADC_0);
    Test_Isr(ADC_1);
    CHECK(Joined.Count == 2U);
    CHECK(TRIGGER_OF(Joined.Ia) == TriggerCount);   /* and the values are fresh, not one trigger apart */
    CHECK(TRIGGER_OF(Joined.Ic) == TriggerCount);

    Test_Trigger();             /* back in step */
    Test_Isr(ADC_0);
    Test_Isr(ADC_1);
    CHECK(Joined.Count == 3U);
}

/*
    The consumer selects from inside the join, as a motor does when the capture it just took decides
    the next set. Next is read after ON_COMPLETE, so it applies at that same join - every ADC of the
    trigger is idle there, and the set sent to the Hw is the one recorded and armed.
*/
static void test_selection_from_the_join_applies_now(void)
{
    Test_Reset(&MOTOR0_I_BATCH);
    SelectFromJoin = &MOTOR0_V_BATCH;

    Test_Trigger();
    Test_Isr(ADC_0);
    Test_Isr(ADC_1);                                                /* the join runs ON_COMPLETE, which selects V */
    CHECK(Joined.Count == 1U);
    CHECK(Joined.p_Context == (void *)1U);                          /* reported as the I batch, which is what converted */
    CHECK(ADC_Batch_IsActive(&MOTOR0_TRIGGER, &MOTOR0_V_BATCH));    /* and V is active, both ADCs reprogrammed */
    CHECK(Adc0Registers.RangeStart == ADC0_VA);
    CHECK(Adc1Registers.RangeStart == ADC1_VC);

    Test_Trigger();
    Test_Isr(ADC_0);
    Test_Isr(ADC_1);
    CHECK(Joined.Count == 2U);
    CHECK(Joined.p_Context == (void *)2U);
}

/* 1 ADC is a batch of 1 part: the same path, joining on its own completion */
static void test_single_part_batch(void)
{
    Test_Reset(&MOTOR0_HEAT_BATCH);

    for (uint8_t i = 0U; i < 4U; i++)
    {
        Test_Trigger();
        Test_Isr(ADC_1);
        CHECK(Joined.Count == (uint32_t)(i + 1U));
    }
    CHECK(Joined.p_Context == (void *)3U);
}

/* Activation points the sequencer at the set's slot range, and arms every part */
static void test_activate(void)
{
    Test_Reset(&MOTOR0_I_BATCH);

    CHECK(Adc0Registers.RangeStart == ADC0_IA);
    CHECK(Adc0Registers.RangeCount == 2U);
    CHECK(Adc1Registers.RangeStart == ADC1_IC);
    CHECK(Adc1Registers.RangeCount == 2U);
    CHECK(ADC_Batch_IsActive(&MOTOR0_TRIGGER, &MOTOR0_I_BATCH));
    CHECK(ADC_Batch_IsSelected(&MOTOR0_TRIGGER, &MOTOR0_I_BATCH));  /* Next must not be left unset */
    CHECK(Adc0State.ChannelMarkers == ADC_MASK_RANGE(ADC0_IA, 2U));
    CHECK(Adc1State.ChannelMarkers == ADC_MASK_RANGE(ADC1_IC, 2U));
}

/*
    An individual channel mark shares [ChannelMarkers] with the join, so it is visible to it.
    Documents which way that interferes.
*/
static void test_individual_mark(void)
{
    Test_Reset(&MOTOR0_I_BATCH);

    /* Outside the active part's channels: invisible to the join, which only tests its own set */
    Test_Trigger();
    ADC_MarkChannel((ADC_T *)&ADCS[ADC_0], ADC0_VA);        /* VA is not in SEQUENCE_IAB */
    Test_Isr(ADC_0);
    Test_Isr(ADC_1);
    CHECK(Joined.Count == 1U);
    CHECK(ADC_IsMarked((ADC_T *)&ADCS[ADC_0], ADC0_VA));    /* nothing walks markers when Hw sequenced, so the bit stays */

    /* Inside the part's channels, after that part landed: the part reads as pending again */
    Test_Trigger();
    Test_Isr(ADC_0);
    ADC_MarkChannel((ADC_T *)&ADCS[ADC_0], ADC0_IA);        /* IA is in SEQUENCE_IAB */
    Test_Isr(ADC_1);
    CHECK(Joined.Count == 1U);                              /* the join is held off for this cycle */

    Test_Trigger();                                         /* the wholesale clear recovers it */
    Test_Isr(ADC_0);
    Test_Isr(ADC_1);
    CHECK(Joined.Count == 2U);
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
    test_individual_mark();

    printf(fails == 0 ? "ADC_Batch_Test: PASS\n" : "ADC_Batch_Test: %d FAILED\n", fails);
    return fails;
}
