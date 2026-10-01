# ADR 0005: Retain raw trial records as compressed campaign archives in git

- Status: **accepted** as A, 2026-09-30 (author decision)
- Date: 2026-09-21

## Context

`results/` is gitignored and terminal records run 1.3--5.8 MB each, so raw
evidence is lost by default: nine cited Gate 0 record sets were destroyed that
way. Committed audit summaries (`studies/gate0/*_audit.json`) carry per-record
SHA-256 digests and a campaign digest, so a set stays identifiable after its
bulk is gone, but identifiable is not auditable. The confirmatory set is 432
trials, about 2 GB raw, and plain git tracking of raw JSON is not viable.

The options on record were git-lfs, an external archive addressed by content
hashes, or a reduced record schema. Measurement adds a fourth.

## Measurement

Records are verbose JSON and compress 21--31x with `tar | xz -9`:

| Campaign | Raw | `.tar.xz` | Ratio |
|---|---|---|---|
| `fault_matrix_campaign_raw` (42) | 223.3 MB | 9.1 MB | 24x |
| `fault_matrix_campaign_r2_raw` (42) | 66.8 MB | 2.1 MB | 31x |
| `fault_matrix_campaign_r3_raw` (42) | 69.1 MB | 2.5 MB | 28x |
| `roundtrip_20_postfix_raw` (20) | 107.5 MB | 5.1 MB | 21x |

At the least favourable ratio the confirmatory set is about 100 MB compressed.
Split per paired block it stays far below GitHub's 50 MB per-file limit.

## Options

- **A. Compressed campaign archives committed to git (recommended).** Each
  campaign's raw directory is committed as `<campaign>.tar.xz` beside its
  audit, split so no file exceeds 45 MB. Needs no service, quota, or client
  extension; clones carry the evidence; history makes deletion visible. Cost:
  repository growth of roughly 100 MB over the study, permanent in history.
- **B. git-lfs.** Keeps raw JSON browsable, but not installed here, GitHub's
  free LFS quota is 1 GB storage and 1 GB/month bandwidth, and evidence then
  depends on an external service and every collaborator having LFS.
- **C. External archive by content hash.** Smallest repository, but it is one
  more place to lose data and was the de facto state that already failed.
- **D. Reduced record schema.** Decides now which fields matter and discards
  the rest, which removes the ability to re-analyse after a surprise.

## Decision

**A is adopted, 2026-09-30.** Every campaign that backs a claim commits `studies/<gate>/records/
<campaign>.tar.xz` (split at 45 MB) and its audit, whose `record_digests` and
`campaign_digest` let anyone verify an extracted archive with
`TrialStore.campaign_digest`. Debug and smoke runs are not retained. The
existing Gate 0 campaigns are committed retroactively; compressed copies with
SHA-256 sums already exist outside the repository at
`C:\Users\risha\morphology_record_archives\` as an interim safeguard.

## Adoption

The restore-and-verify check this ADR required before adoption is
`modular_robot_benchmarks/record_archive.py`, exposed as
`retain_campaign_records`. Archiving and verification are one operation: the
archive is extracted to a scratch directory, each audited record is rehashed,
the campaign digest is recomputed with the same formula `TrialStore` uses, and
both must match the committed audit. A size guard refuses any part over the
limit, default 45 MB.

Verification is scoped to **the audited set**, not the directory. Two Gate 0
record directories hold qualification sub-records their campaign audit never
scored -- `gate0_item1` audits 6 records in a directory of 19, and
`gate0_item1_fdir1` 10 in a directory of 32 -- so a directory-wide comparison
would have reported those extras as corruption and made the archives
unverifiable. Every audited record must be present and unchanged; retained
extras are reported as a count. The two item-1 audits also predate
`campaign_digest`, so it is checked when present and its absence is reported
rather than passed over silently; the per-record digests are what a campaign
digest is computed from, so a set with complete record digests is covered
either way.

The interim external safeguard this ADR described was **nine days stale** when
A was adopted: the `morphology_record_archives` directory beside the repository
was last written 2026-09-21 and covered no campaign collected after ADR 0007 --
not the two failed round-trip attempts, not the quarantined partial set, and
not `gate0_item1_fdir1`, the only Gate 0 evidence still valid. That is the
failure mode this ADR exists to end, recurring while the ADR sat proposed, and
it is why the mechanism belongs in the repository rather than beside it.
