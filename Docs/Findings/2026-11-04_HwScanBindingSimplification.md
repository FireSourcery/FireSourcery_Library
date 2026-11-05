# Hw scan binding: what simplified it

**Question** — A board's binding for its Hw sequenced ADC scan was rewritten between 2026-09-25 and
2026-11-04. It came out with one completion ISR instead of two, and no branches in it. Which changes
did that, and which rules do they generalize to?

**Method** — a source read of the binding at both dates, from the application repo's history: a
two-ADC S32K1 board (PDB, eDMA), with a current set and a phase voltage set of 4 slots each on the
PWM triggered ADC. Live lines are counted with comments and blank lines removed.

The hardware design is the same at both dates and is not part of the comparison. It is one slot
table holding both sets, one PDB channel, and one transfer of all 8 slots, with the PDB enable mask
selecting the set. Attempts before 2026-09-25 (2026-04: a PDB, a transfer and a buffer per set) were
notes, not a working form, and are left out.

**Result**

| | 2026-09-25 | 2026-11-04 |
|---|---|---|
| Completion ISRs | 2, one per scan mode | 1: clear the flag, then `ADC_SequenceTrigger_OnCompleteDma_ISR` |
| Under the mode `#if` | both ISRs, both selection hooks (one an empty stub), the next-set variable | one statement per branch: the first set, chosen at init |
| The set that completed | read back from the PDB enable mask | `p_ActiveSequence` |
| The next set | the board's: a `volatile` set id, a snapshot, a compare, an activate per set | `ADC_SequenceTrigger_T`; the board calls `ADC_SequenceTrigger_SetNext` with a conditional |
| Board functions per set | 7: 3 activates, a set test, a flag clear, a result accessor per ADC | none; 3 one-line `ADC_Sequence_T` records and their maps |
| Capture calls | 1 per channel: 4 for the current set, 3 for the voltage set | 1 per consumer: 2 and 1 |
| Names per channel | a slot enum, an alias define, the pin in 2 tables, a `1 << slot` term in each set mask, the accessor | 1 enum constant and 1 pin entry; set masks as ranges |
| Result buffers | defined by the application with external linkage, `extern` in the board header | `static` in the binding header, which only the application's translation unit includes |
| Live lines | binding header 107 | binding header 75, now with the transfer setup; the library's `ADC_Sequence.h` 68 |

**Conclusion** — The reduction is in the software structure, by seven of the ten rules in
[HardwareBinding.md](../Conventions/HardwareBinding.md). Rules 1, 5 and 6 already held on 2026-09-25.

| Rule | Removed |
|---|---|
| [2](../Conventions/HardwareBinding.md#2-make-the-variation-a-record-and-run-one-path-over-it) | the second ISR, and the per-set branches in the first |
| [3](../Conventions/HardwareBinding.md#3-answer-a-generic-protocol-once-in-the-library-the-board-keeps-only-its-map) | the next-set id, its snapshot, the compare, the activate branch |
| [4](../Conventions/HardwareBinding.md#4-keep-the-record-of-what-software-decided-dont-read-it-back-from-the-hardware) | the PDB enable mask read |
| [7](../Conventions/HardwareBinding.md#7-capture-in-the-unit-the-data-lands-in) | 4 of the 7 capture calls |
| [8](../Conventions/HardwareBinding.md#8-one-name-and-one-table-per-fact) | the alias defines, the second pin table, the per-slot mask terms, the accessors |
| [9](../Conventions/HardwareBinding.md#9-place-code-by-what-it-names) | the `extern` buffers, and the board's init writing the application's destinations |
| [10](../Conventions/HardwareBinding.md#10-compose-in-one-place-select-in-one-place) | the functions and the state under the mode `#if` |

Rules 2 and 3 removed most of it: the second ISR and every branch in the first. Rule 2 made rule 10's
narrow mode test possible: once the modes differ only in which record is first, the `#if` covers one
statement.

What it cost:
- The protocol moved more than it shrank. The board header lost 32 live lines, and the library's
  `ADC_Sequence.h` (68 live lines, including the software started form) has one user so far.
- The map runs through a function pointer from the ISR. The 2026-09-25 ISR called the captures
  directly.
- Not measured: the ISR's cycles in either form. Neither form has run on hardware.
