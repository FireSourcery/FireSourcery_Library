# Findings

What we measured or learned, dated: harness runs, build and size comparisons, codegen probes,
source reads. Findings are the evidence behind [../Decisions/](../Decisions/README.md). A later
finding may supersede an earlier one; say so in both.

---

## Entry shape

`YYYY-MM-DD_Title.md`, so the folder reads in order.

```markdown
# Title

**Question** — what we wanted to know.
**Method** — the harness, build, probe or source; enough to run it again.
**Result** — the numbers.
**Conclusion** — what it means, and which decisions it bears on.
```

---

## Records

| Record | About |
|---|---|
| [2026-10-14_CorruptLooseBlobs.md](2026-10-14_CorruptLooseBlobs.md) | Three corrupt loose blobs break history searches; refetch fix |
| [2026-11-04_HwScanBindingSimplification.md](2026-11-04_HwScanBindingSimplification.md) | A Hw scan binding, 2026-09-25 vs 2026-11-04: one ISR from two, and which binding rules did it |
