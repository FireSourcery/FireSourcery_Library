/* Host round-trip test for RingT.h. Guard values around every destination catch stride bugs. */
#include "Framework/Ring/RingT.h"
#include <stdio.h>

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); fails++; } } while (0)

#define LEN 8U
static const Ring_Type_T U32 = RING_TYPE_INIT(sizeof(uint32_t), LEN);

static struct { Ring_State_T State; uint32_t Buffer[LEN]; } ring;
static Ring_State_T * const p_ring = &ring.State;

/* destination sandwiched between guards: any over-copy trips them */
typedef struct { uint32_t lo; uint32_t v; uint32_t hi; } guarded_t;
#define GUARDED { 0xA5A5A5A5U, 0U, 0x5A5A5A5AU }
#define CHECK_GUARDS(g) CHECK((g).lo == 0xA5A5A5A5U && (g).hi == 0x5A5A5A5AU)

static void test_basic_fifo(void)
{
    RingT_Clear(U32, p_ring);
    CHECK(RingT_IsEmpty(U32, p_ring));
    CHECK(RingT_GetCapacity(U32, p_ring) == LEN);
    CHECK(RingT_GetFullCount(U32, p_ring) == 0U);

    for (uint32_t i = 0U; i < LEN; i++) { CHECK(RingT_PushBack(U32, p_ring, &i)); }
    CHECK(RingT_IsFull(U32, p_ring));
    CHECK(RingT_GetFullCount(U32, p_ring) == LEN);
    CHECK(RingT_GetEmptyCount(U32, p_ring) == 0U);

    uint32_t overflow = 99U;
    CHECK(RingT_PushBack(U32, p_ring, &overflow) == false);   /* must not overwrite */

    for (uint32_t i = 0U; i < LEN; i++)
    {
        guarded_t g = GUARDED;
        CHECK(RingT_PopFront(U32, p_ring, &g.v));
        CHECK(g.v == i);
        CHECK_GUARDS(g);
    }
    CHECK(RingT_IsEmpty(U32, p_ring));
    guarded_t g = GUARDED;
    CHECK(RingT_PopFront(U32, p_ring, &g.v) == false);
}

static void test_front_back_at(void)
{
    RingT_Clear(U32, p_ring);
    for (uint32_t i = 10U; i < 14U; i++) { CHECK(RingT_PushBack(U32, p_ring, &i)); }

    CHECK(*(uint32_t *)RingT_Front(U32, p_ring) == 10U);
    CHECK(*(uint32_t *)RingT_Back(U32, p_ring) == 13U);
    /* Back must point inside Buffer, never at the Head/Tail cursors */
    CHECK((uint8_t *)RingT_Back(U32, p_ring) >= (uint8_t *)ring.Buffer);
    CHECK((uint8_t *)RingT_Back(U32, p_ring) < (uint8_t *)(ring.Buffer + LEN));

    for (size_t i = 0U; i < 4U; i++)
    {
        CHECK(*(uint32_t *)RingT_At(U32, p_ring, i) == 10U + i);
        guarded_t g = GUARDED;
        CHECK(RingT_PeekAt(U32, p_ring, i, &g.v));
        CHECK(g.v == 10U + i);
        CHECK_GUARDS(g);            /* PeekAt copying LENGTH bytes instead of TYPE_SIZE trips here */
    }
    CHECK(RingT_At(U32, p_ring, 4U) == NULL);
    guarded_t g = GUARDED;
    CHECK(RingT_PeekAt(U32, p_ring, 4U, &g.v) == false);
}

static void test_lifo_and_removes(void)
{
    RingT_Clear(U32, p_ring);
    for (uint32_t i = 0U; i < 4U; i++) { CHECK(RingT_PushBack(U32, p_ring, &i)); }

    guarded_t g = GUARDED;
    CHECK(RingT_PopBack(U32, p_ring, &g.v));
    CHECK(g.v == 3U);
    CHECK_GUARDS(g);

    uint32_t front = 77U;
    CHECK(RingT_PushFront(U32, p_ring, &front));
    CHECK(*(uint32_t *)RingT_Front(U32, p_ring) == 77U);
    CHECK(RingT_GetFullCount(U32, p_ring) == 4U);

    CHECK(RingT_RemoveFront(U32, p_ring, 2U));
    CHECK(RingT_GetFullCount(U32, p_ring) == 2U);
    CHECK(*(uint32_t *)RingT_Front(U32, p_ring) == 1U);
    CHECK(RingT_RemoveBack(U32, p_ring, 5U) == false);
    CHECK(RingT_RemoveBack(U32, p_ring, 2U));
    CHECK(RingT_IsEmpty(U32, p_ring));
}

/* drive the cursors past the wrap point, then batch across it */
static void test_batch_wrap(void)
{
    RingT_Clear(U32, p_ring);
    uint32_t scratch;
    for (uint32_t i = 0U; i < 6U; i++) { CHECK(RingT_PushBack(U32, p_ring, &i)); }
    for (uint32_t i = 0U; i < 6U; i++) { CHECK(RingT_PopFront(U32, p_ring, &scratch)); }
    CHECK(_RingT_ArrayIndexOf(U32, p_ring->Tail) == 6U);     /* 2 slots to the end of the array */

    const uint32_t src[LEN] = { 100U, 101U, 102U, 103U, 104U, 105U, 106U, 107U };
    CHECK(RingT_PushBackArray(U32, p_ring, src, LEN));       /* 2 units, then 6 wrapped */
    CHECK(RingT_GetFullCount(U32, p_ring) == LEN);
    CHECK(RingT_IsFull(U32, p_ring));

    for (size_t i = 0U; i < LEN; i++) { CHECK(*(uint32_t *)RingT_At(U32, p_ring, i) == 100U + i); }

    uint32_t dest[LEN + 2U];
    dest[LEN] = 0xDEADBEEFU;
    dest[LEN + 1U] = 0xDEADBEEFU;
    CHECK(RingT_PopFrontArray(U32, p_ring, dest, LEN));
    for (size_t i = 0U; i < LEN; i++) { CHECK(dest[i] == 100U + i); }
    CHECK(dest[LEN] == 0xDEADBEEFU && dest[LEN + 1U] == 0xDEADBEEFU);
    CHECK(RingT_IsEmpty(U32, p_ring));

    CHECK(RingT_PopFrontArray(U32, p_ring, dest, 1U) == false);
}

/* non-wrapping batch must leave the second segment empty */
static void test_batch_no_wrap(void)
{
    RingT_Clear(U32, p_ring);
    for (size_t i = 0U; i < LEN; i++) { ring.Buffer[i] = 0U; }  /* Clear resets cursors only, not storage */

    const uint32_t src[6] = { 7U, 8U, 9U, 10U, 11U, 12U };
    CHECK(RingT_PushBackArray(U32, p_ring, src, 3U));
    CHECK(RingT_GetFullCount(U32, p_ring) == 3U);
    /* slots past the run must be untouched: a byte/unit count mix-up would have spilled into them */
    CHECK(ring.Buffer[3] == 0U && ring.Buffer[4] == 0U);

    uint32_t dest[3] = { 0U, 0U, 0U };
    CHECK(RingT_PeekFrontArray(U32, p_ring, dest, 3U));
    CHECK(dest[0] == 7U && dest[1] == 8U && dest[2] == 9U);
    CHECK(RingT_GetFullCount(U32, p_ring) == 3U);            /* peek does not consume */

    CHECK(RingT_PushBackArray(U32, p_ring, src, 6U) == false);
    CHECK(RingT_PushBackMax(U32, p_ring, src, 6U) == 5U);    /* clamped to EmptyCount */
    CHECK(RingT_IsFull(U32, p_ring));
    CHECK(RingT_PopFrontMax(U32, p_ring, dest, 2U) == 2U);
    CHECK(dest[0] == 7U && dest[1] == 8U);
}

/* cursor counters must survive wrapping past SIZE_MAX-ish values */
static void test_each_helpers(void)
{
    RingT_Clear(U32, p_ring);
    const uint32_t src[4] = { 1U, 2U, 3U, 4U };
    _RingT_PushBackEach(U32, p_ring, src, 4U);
    CHECK(RingT_GetFullCount(U32, p_ring) == 4U);

    uint32_t dest[6] = { 0U, 0U, 0U, 0U, 0xC0FFEEU, 0xC0FFEEU };
    _RingT_PopFrontEach(U32, p_ring, dest, 4U);
    CHECK(dest[0] == 1U && dest[1] == 2U && dest[2] == 3U && dest[3] == 4U);
    CHECK(dest[4] == 0xC0FFEEU && dest[5] == 0xC0FFEEU);     /* stride LENGTH instead of TYPE_SIZE trips here */
}

static void test_byte_ring(void)
{
    static struct { Ring_State_T State; uint8_t Buffer[16]; } b;
    static const Ring_Type_T U8 = RING_TYPE_INIT(sizeof(uint8_t), 16U);

    RingT_Clear(U8, &b.State);
    for (uint8_t i = 0U; i < 16U; i++) { CHECK(RingT_PushBack(U8, &b.State, &i)); }
    CHECK(RingT_IsFull(U8, &b.State));
    CHECK(*(uint8_t *)RingT_Back(U8, &b.State) == 15U);

    uint8_t out[20];
    out[16] = 0xEEU;
    CHECK(RingT_PopFrontArray(U8, &b.State, out, 16U));
    for (uint8_t i = 0U; i < 16U; i++) { CHECK(out[i] == i); }
    CHECK(out[16] == 0xEEU);
}

/* ---- appended: contiguous segments and overwrite policy ---- */

/* the two spans must exactly reconstruct the logical contents, wrapped or not */
static void check_front_spans_match(size_t expect_count, uint32_t first_value)
{
    ArraySpanT_T a = RingT_FrontSpan(U32, p_ring);
    ArraySpanT_T b = RingT_FrontSpanWrap(U32, p_ring);

    CHECK(a.TYPE_SIZE == sizeof(uint32_t) && b.TYPE_SIZE == sizeof(uint32_t));
    CHECK(a.LENGTH + b.LENGTH == expect_count);
    CHECK(ArraySpan_Size(a) == a.LENGTH * sizeof(uint32_t));
    /* both segments must land inside the payload buffer */
    CHECK((uint8_t *)a.P_BUFFER >= (uint8_t *)ring.Buffer && (uint8_t *)a.P_BUFFER <= (uint8_t *)(ring.Buffer + LEN));
    CHECK((uint8_t *)b.P_BUFFER == (uint8_t *)ring.Buffer);       /* wrapped part always starts at Buffer[0] */

    for (size_t i = 0U; i < a.LENGTH; i++) { CHECK(*(uint32_t *)ArraySpan_At(a, i) == first_value + i); }
    for (size_t i = 0U; i < b.LENGTH; i++) { CHECK(*(uint32_t *)ArraySpan_At(b, i) == first_value + a.LENGTH + i); }
}

static void test_spans_no_wrap(void)
{
    RingT_Clear(U32, p_ring);
    for (uint32_t i = 50U; i < 55U; i++) { CHECK(RingT_PushBack(U32, p_ring, &i)); }

    ArraySpanT_T a = RingT_FrontSpan(U32, p_ring);
    ArraySpanT_T b = RingT_FrontSpanWrap(U32, p_ring);
    CHECK(a.LENGTH == 5U);
    CHECK(b.LENGTH == 0U);                                        /* no wrap -> empty second segment */
    check_front_spans_match(5U, 50U);

    /* free space: Tail at 5, so 3 contiguous to the end then 0 wrapped (ring holds 5 of 8) */
    ArraySpanT_T f = RingT_BackSpan(U32, p_ring);
    ArraySpanT_T g = RingT_BackSpanWrap(U32, p_ring);
    CHECK(f.LENGTH + g.LENGTH == RingT_GetEmptyCount(U32, p_ring));
    CHECK(f.LENGTH == 3U && g.LENGTH == 0U);
}

static void test_spans_wrapped(void)
{
    RingT_Clear(U32, p_ring);
    uint32_t scratch;
    for (uint32_t i = 0U; i < 6U; i++) { CHECK(RingT_PushBack(U32, p_ring, &i)); }
    for (uint32_t i = 0U; i < 6U; i++) { CHECK(RingT_PopFront(U32, p_ring, &scratch)); }
    /* Head = Tail = 6; push 5 so the run wraps: 2 at [6..7], 3 at [0..2] */
    for (uint32_t i = 200U; i < 205U; i++) { CHECK(RingT_PushBack(U32, p_ring, &i)); }

    ArraySpanT_T a = RingT_FrontSpan(U32, p_ring);
    ArraySpanT_T b = RingT_FrontSpanWrap(U32, p_ring);
    CHECK(a.LENGTH == 2U);
    CHECK(b.LENGTH == 3U);
    check_front_spans_match(5U, 200U);

    ArraySpanT_T f = RingT_BackSpan(U32, p_ring);
    ArraySpanT_T g = RingT_BackSpanWrap(U32, p_ring);
    CHECK(f.LENGTH + g.LENGTH == 3U);                             /* 8 capacity - 5 held */

    /* zero-copy consume via claim/finish, using the span pair then RemoveFront */
    uint32_t out[8];
    size_t n = 0U;
    ArraySpan_CopyTo(a, &out[0]);  n += a.LENGTH;
    ArraySpan_CopyTo(b, &out[n]);  n += b.LENGTH;
    CHECK(n == 5U);
    for (size_t i = 0U; i < 5U; i++) { CHECK(out[i] == 200U + i); }
    CHECK(RingT_RemoveFront(U32, p_ring, n));
    CHECK(RingT_IsEmpty(U32, p_ring));
}

static void test_spans_empty_and_full(void)
{
    RingT_Clear(U32, p_ring);
    CHECK(RingT_FrontSpan(U32, p_ring).LENGTH == 0U);
    CHECK(RingT_FrontSpanWrap(U32, p_ring).LENGTH == 0U);
    CHECK(RingT_BackSpan(U32, p_ring).LENGTH + RingT_BackSpanWrap(U32, p_ring).LENGTH == LEN);

    for (uint32_t i = 0U; i < LEN; i++) { CHECK(RingT_PushBack(U32, p_ring, &i)); }
    CHECK(RingT_BackSpan(U32, p_ring).LENGTH == 0U);
    CHECK(RingT_BackSpanWrap(U32, p_ring).LENGTH == 0U);
    CHECK(RingT_FrontSpan(U32, p_ring).LENGTH + RingT_FrontSpanWrap(U32, p_ring).LENGTH == LEN);
}

static void test_overwrite_unit(void)
{
    RingT_Clear(U32, p_ring);
    for (uint32_t i = 0U; i < LEN; i++) { CHECK(RingT_PushBackOverwrite(U32, p_ring, &i) == false); }
    CHECK(RingT_IsFull(U32, p_ring));

    uint32_t v = 100U;
    CHECK(RingT_PushBackOverwrite(U32, p_ring, &v) == true);      /* evicts oldest */
    CHECK(RingT_GetFullCount(U32, p_ring) == LEN);                /* count must not grow past capacity */
    CHECK(*(uint32_t *)RingT_Front(U32, p_ring) == 1U);           /* 0 was dropped */
    CHECK(*(uint32_t *)RingT_Back(U32, p_ring) == 100U);

    for (uint32_t i = 101U; i < 105U; i++) { CHECK(RingT_PushBackOverwrite(U32, p_ring, &i)); }
    CHECK(RingT_GetFullCount(U32, p_ring) == LEN);
    CHECK(*(uint32_t *)RingT_Front(U32, p_ring) == 5U);
    for (size_t i = 0U; i < LEN; i++)
    {
        uint32_t expect = (i < 3U) ? (uint32_t)(5U + i) : (uint32_t)(100U + (i - 3U));
        CHECK(*(uint32_t *)RingT_At(U32, p_ring, i) == expect);
    }
}

static void test_overwrite_front(void)
{
    RingT_Clear(U32, p_ring);
    for (uint32_t i = 0U; i < LEN; i++) { CHECK(RingT_PushBack(U32, p_ring, &i)); }
    uint32_t v = 77U;
    CHECK(RingT_PushFrontOverwrite(U32, p_ring, &v) == true);     /* evicts newest from the back */
    CHECK(RingT_GetFullCount(U32, p_ring) == LEN);
    CHECK(*(uint32_t *)RingT_Front(U32, p_ring) == 77U);
    CHECK(*(uint32_t *)RingT_Back(U32, p_ring) == LEN - 2U);      /* 7 dropped */
}

static void test_overwrite_array(void)
{
    /* fits with room to spare: nothing lost */
    RingT_Clear(U32, p_ring);
    const uint32_t a3[3] = { 1U, 2U, 3U };
    CHECK(RingT_PushBackOverwriteArray(U32, p_ring, a3, 3U) == 0U);
    CHECK(RingT_GetFullCount(U32, p_ring) == 3U);

    /* needs to evict: 8 held max, 3 present + 6 incoming -> evict 1 */
    const uint32_t a6[6] = { 10U, 11U, 12U, 13U, 14U, 15U };
    CHECK(RingT_PushBackOverwriteArray(U32, p_ring, a6, 6U) == 1U);
    CHECK(RingT_GetFullCount(U32, p_ring) == LEN);
    CHECK(*(uint32_t *)RingT_Front(U32, p_ring) == 2U);           /* 1 was evicted */
    CHECK(*(uint32_t *)RingT_Back(U32, p_ring) == 15U);

    /* count larger than capacity: leading source units are skipped, ring keeps the last LEN */
    RingT_Clear(U32, p_ring);
    const uint32_t a12[12] = { 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U };
    CHECK(RingT_PushBackOverwriteArray(U32, p_ring, a12, 12U) == 4U);   /* 4 source units skipped */
    CHECK(RingT_GetFullCount(U32, p_ring) == LEN);
    for (size_t i = 0U; i < LEN; i++) { CHECK(*(uint32_t *)RingT_At(U32, p_ring, i) == 4U + i); }

    /* exactly capacity into a full ring: every held unit is evicted */
    const uint32_t a8[8] = { 20U, 21U, 22U, 23U, 24U, 25U, 26U, 27U };
    CHECK(RingT_PushBackOverwriteArray(U32, p_ring, a8, LEN) == LEN);
    for (size_t i = 0U; i < LEN; i++) { CHECK(*(uint32_t *)RingT_At(U32, p_ring, i) == 20U + i); }

    /* wrapped destination */
    RingT_Clear(U32, p_ring);
    uint32_t scratch;
    for (uint32_t i = 0U; i < 6U; i++) { CHECK(RingT_PushBack(U32, p_ring, &i)); }
    for (uint32_t i = 0U; i < 6U; i++) { CHECK(RingT_PopFront(U32, p_ring, &scratch)); }
    CHECK(RingT_PushBackOverwriteArray(U32, p_ring, a6, 6U) == 0U);
    CHECK(RingT_GetFullCount(U32, p_ring) == 6U);
    for (size_t i = 0U; i < 6U; i++) { CHECK(*(uint32_t *)RingT_At(U32, p_ring, i) == 10U + i); }
}

int main(void)
{
    test_basic_fifo();
    test_front_back_at();
    test_lifo_and_removes();
    test_batch_wrap();
    test_batch_no_wrap();
    test_each_helpers();
    test_byte_ring();
    test_spans_no_wrap();
    test_spans_wrapped();
    test_spans_empty_and_full();
    test_overwrite_unit();
    test_overwrite_front();
    test_overwrite_array();
    if (fails == 0) { printf("ALL PASS\n"); } else { printf("%d FAILURES\n", fails); }
    return fails != 0;
}
