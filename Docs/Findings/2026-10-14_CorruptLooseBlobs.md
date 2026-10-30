# Corrupt loose blobs in the library repo

**Question** — why `git log -S` errors on the library repo.

**Method** — `git fsck`, then `git cat-file` on each object it reports.

**Result** — three corrupt loose blobs in `.git/objects`, file mtimes 2026-02 and 2026-04. Two are
zero-length, one fails to unpack:

| Object | Path |
|---|---|
| `44dc7fe8` | `Type/accessor.h` |
| `4d1d9b18` | `Motor/Motor/Sensor/SinCos/SinCos_Sensor.c` |
| `93292715` | `Motor/Motor/Motor_User.c` |

They are old versions, in commits such as 152eaa8, 390e4e3, e552875, e7b140c and 41f4bd3. The
current files are intact.

**Conclusion** — `git log -S` / `-G` / `--find-object` and `fsck` fail when they reach these blobs;
`gc` or `repack` may too. Every affected commit is on origin/main, so the remote holds intact copies.
Fix: move the three object files aside, `git fetch --refetch origin`, then `git fsck`. Not applied as
of 2026-10-29; check before assuming it still holds.
