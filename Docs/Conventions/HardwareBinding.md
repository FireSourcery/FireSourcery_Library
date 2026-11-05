# Hardware Binding

Rules for the code that binds a peripheral to the application: the HAL functions, the board's setup,
its binding header, and the library modules they compose. Drawn from the ADC scan on an S32K1 board
(PDB, eDMA). Each rule names what it replaces. The evidence is in
[2026-11-04_HwScanBindingSimplification.md](../Findings/2026-11-04_HwScanBindingSimplification.md),
and the shape the rules produce for a sequenced scan is [HwSequencedScan.md](../Patterns/HwSequencedScan.md).

| # | Rule | Replaces | Backed by |
|---|---|---|---|
| 1 | Vary one hardware resource; keep the rest fixed | a resource set per variant, and a multi-write switch | |
| 2 | Make the variation a record, and run one path over it | a code path per variant, and branches on which ran | [Descriptor.md](../Patterns/Descriptor.md) |
| 3 | Answer a generic protocol once, in the library; the board keeps only its map | protocol state rewritten in each board | [InformationExpert.md](../Principles/InformationExpert.md) |
| 4 | Keep the record of what software decided; don't read it back from the hardware | decoding a register to recover a choice | |
| 5 | Build one path; keep the alternatives as notes | complete alternative pipelines with no caller | [ConditionalCompilation.md](ConditionalCompilation.md) |
| 6 | Give a peripheral with its own rate its own trigger | a shared trigger, and an init order rule | |
| 7 | Capture in the unit the data lands in | a call per channel | |
| 8 | One name and one table per fact | alias defines, repeated tables, rename-only accessors | [WrapperSurface.md](../Patterns/WrapperSurface.md) |
| 9 | Place code by what it names | `extern` instances, and setup reaching into the application | |
| 10 | Compose in one place, select in one place | config structs, long init signatures, wide mode tests | [InterfaceSegregation.md](../Principles/InterfaceSegregation.md), [GuardedCall.md](../Patterns/GuardedCall.md) |

---

## 1. Vary one hardware resource; keep the rest fixed

Find the one register that can express the variation, and make everything else constant for the
image.
- On S32K1, the result registers keep their last value, one SC1 table holds every set, and one
  transfer copies every slot on each completion. That copy is valid whichever set ran, so one
  transfer and one buffer serve every mode, and the PDB channel's enable mask alone selects the set:
  one write.
- Instead of: a trigger, a transfer and a buffer per set. The switch is then several writes that must
  agree (requests off and on, triggers off and on), and each reader must know which buffer holds its
  channel.

## 2. Make the variation a record, and run one path over it

Where code paths differ only in *which set* and *which handler*, the set and the handler are a
`const` record, and one path runs the active record.
- A scan set is an `ADC_Sequence_T`: its ADC, channel range, map and context. The completion ISR runs
  the active set's map, so it holds no branch on which set ran. A mode with nothing to select is one
  more record, over every slot, and the ISR is the same.
- Instead of: one completion ISR per mode under `#if`, each branching on the set.

## 3. Answer a generic protocol once, in the library; the board keeps only its map

A protocol that every board meets the same way is the library's. The board keeps what it alone
knows.
- "Request the next set from any thread, apply it at completion" is the same for every Hw triggered
  scan: `ADC_SequenceTrigger_T` holds it (`p_ActiveSequence`, `p_NextSequence`). The board keeps the
  map: which slot feeds which consumer.
- Instead of: a board-owned `volatile` set id, a snapshot of it in the ISR, a compare, and an activate
  function per set.

## 4. Keep the record of what software decided; don't read it back from the hardware

What software chose, software records. Hardware state is read for what only the hardware knows.
- The set that completed is `p_ActiveSequence`.
- Instead of: decoding the PDB channel's enable mask in the ISR to find which set ran. The read back
  couples the ISR to the register's layout, and restates a choice the code already made.

## 5. Build one path; keep the alternatives as notes

An alternative implementation is kept as a note, or as code that is commented out, until it is
chosen. Only the chosen path is compiled and maintained.
- Instead of: two complete pipelines (a PDB sequenced scan and a DMA linked chain), each with its own
  config struct and composite init, and neither with a caller.

## 6. Give a peripheral with its own rate its own trigger

A conversion with its own rate and no consumer waiting on its completion runs on its own trigger,
with no ISR. Its readers pull from the buffer.
- The slow monitors free run on their own PDB. A 1 ms watchdog restarts the scan if it stops.
- Instead of: riding on another peripheral's trigger, which brings an init order rule and couples
  their rates.

## 7. Capture in the unit the data lands in

When a set lands at once, its consumer takes it in one call.
- `Motor_Analog_CaptureIabc` takes the three phase currents.
- Instead of: one capture call per channel. Each call is a separate entry into the consumer, and its
  set is whole only after the last.

## 8. One name and one table per fact

Each channel has one name and one entry, in the table that defines it. Everything else derives from
that.
- One enum per ADC, whose members are the channel IDs and whose last member is the count, and one pin
  array in the same order, sized by the count. Results are read as `Results[NAME]`. Set masks
  are ranges, `ADC_MASK_RANGE(first, count)`.
- Instead of: a slot enum plus an alias define per channel, the pins listed in two tables, a
  `1 << slot` term per channel in each set mask, and a `ResultOf(slot)` accessor. A rename adds no
  concept, so it fails the test in [WrapperSurface.md](../Patterns/WrapperSurface.md).

## 9. Place code by what it names

Code that names an application instance goes in the board's binding header, which is included into
the application's translation unit, after the instances. Setup that names none stays in the board's
own translation unit.
- The binding header holds the result buffers (`static`), the set records, the completion ISR and the
  transfer setup that writes the buffers' addresses. The board's `.c` holds the SC1 tables, the PDB,
  TRGMUX and the watchdog.
- Instead of: application-defined buffers with external linkage, `extern` in a board header, and the
  board's init writing the application's addresses. A header that names the instances and is
  included into a second translation unit fails to compile, or, with `static` buffers, gives that
  unit a silent second copy.

## 10. Compose in one place, select in one place

- **HAL functions set one register group each**, with few parameters. The board composes them with its
  compile-time constants, so no config struct carries the constants to a composite init.
- **One entry composes the per-instance setup.** The application calls the board's DMA init once,
  and does not name each ADC.
- **Prebuilt variants are functions**, named `<Owner>_<Peripheral[n]>_<Variant>_<Verb>`
  (`_HwTrigger_ISR`, `_DMA_Init`). The vector, or the entry point, selects one under one `#if`.
- **A mode test covers only the statement that differs.** Once the modes are records (rule 2), the
  `#if` chooses which record is first, and guards the one call that only one mode makes.
- Instead of: config structs of 8 fields, inits with 7 parameters, and a mode `#if` around whole
  ISRs and hooks, one of them an empty stub.
