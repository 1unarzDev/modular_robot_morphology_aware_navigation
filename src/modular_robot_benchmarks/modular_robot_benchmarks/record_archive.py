"""Retain a campaign's raw records as a compressed archive (ADR 0005).

``results/`` is gitignored and terminal records run 1.3--5.8 MB each, so raw
evidence is lost by default: nine cited Gate 0 record sets were destroyed that
way, and every number computed from them became unauditable prose. Records are
verbose JSON and compress 21--31x, so a campaign that backs a claim is small
enough to commit whole beside its audit.

An archive is only evidence if it can be shown to be the audited set, so
archiving and verification are one operation here. The archive is extracted to a
scratch directory, ``TrialStore.campaign_digest`` is recomputed over the
extracted records, and it must equal the digest the committed audit recorded.
A size guard splits parts below the 45 MB this repository keeps clear of
GitHub's per-file limit.

Nothing here decides what a record contains or which campaigns matter: it
retains what a campaign already wrote.
"""

from __future__ import annotations

from pathlib import Path
import argparse
import hashlib
import json
import shutil
import tarfile
import tempfile

from .records import TrialStore


PART_LIMIT_BYTES = 45 * 1024 * 1024


class ArchiveMismatch(RuntimeError):
    """The archive does not restore to the record set the audit describes."""


def _campaign_digest_over(digests: dict) -> str:
    """TrialStore.campaign_digest's formula, over a chosen subset of records."""
    joined = "".join(f"{trial}:{digest}" + chr(10)
                     for trial, digest in sorted(digests.items()))
    return hashlib.sha256(joined.encode("utf-8")).hexdigest()


def _parts_for(destination: Path) -> list[Path]:
    """Existing pieces of ``destination``, split or whole."""
    if destination.is_file():
        return [destination]
    parts = sorted(destination.parent.glob(destination.name + ".part*"))
    return parts


def archive_campaign(raw_dir: Path, destination: Path,
                     part_limit_bytes: int = PART_LIMIT_BYTES) -> list[Path]:
    """Compress every record in ``raw_dir`` to ``destination``, split if large.

    Returns the parts written. A set that exceeds the part limit is written as
    ``<destination>.part00``, ``.part01``, ... and restores by concatenation.
    """
    raw_dir, destination = Path(raw_dir), Path(destination)
    records = sorted(raw_dir.glob("*.json"))
    if not records:
        raise ValueError(f"no records to archive in {raw_dir}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    for stale in _parts_for(destination):
        stale.unlink()

    with tempfile.TemporaryDirectory() as scratch:
        staged = Path(scratch) / destination.name
        with tarfile.open(staged, "w:xz", preset=9) as archive:
            for record in records:
                archive.add(record, arcname=record.name)
        if staged.stat().st_size <= part_limit_bytes:
            shutil.copy2(staged, destination)
            return [destination]
        parts = []
        with staged.open("rb") as stream:
            index = 0
            while True:
                chunk = stream.read(part_limit_bytes)
                if not chunk:
                    break
                part = destination.parent / f"{destination.name}.part{index:02d}"
                part.write_bytes(chunk)
                parts.append(part)
                index += 1
    return parts


def restore_campaign(destination: Path, into_dir: Path) -> Path:
    """Extract the archive at ``destination`` (whole or split) into ``into_dir``."""
    destination, into_dir = Path(destination), Path(into_dir)
    parts = _parts_for(destination)
    if not parts:
        raise FileNotFoundError(f"no archive at {destination}")
    into_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as scratch:
        joined = Path(scratch) / destination.name
        with joined.open("wb") as stream:
            for part in parts:
                stream.write(part.read_bytes())
        with tarfile.open(joined, "r:xz") as archive:
            archive.extractall(into_dir, filter="data")
    return into_dir


def verify_archive(destination: Path, audit_path: Path) -> dict:
    """Prove the archive restores to the record set ``audit_path`` was computed on.

    Raises ArchiveMismatch rather than reporting a discrepancy quietly: an
    archive that cannot be tied to its audit is not evidence.
    """
    audit = json.loads(Path(audit_path).read_text(encoding="utf-8"))
    expected_digests = audit.get("record_digests") or {}
    expected_campaign = audit.get("campaign_digest")
    if not expected_digests:
        raise ArchiveMismatch(
            f"{audit_path} carries no record_digests to verify against")

    with tempfile.TemporaryDirectory() as scratch:
        restored = restore_campaign(destination, Path(scratch) / "records")
        by_stem = TrialStore(restored).digests()
    # Two digest conventions exist here: campaign audits key by trial id, which
    # is the file stem TrialStore uses, while the item-1 qualification audits
    # key by file name. Resolve against whichever the audit used, so a naming
    # convention is not reported as missing evidence.
    digests = dict(by_stem)
    digests.update({f"{stem}.json": digest for stem, digest in by_stem.items()})

    # The audited set is what has to be proven, not the directory: a raw
    # directory can hold qualification sub-records its campaign audit never
    # scored, and retaining those must not make the archive unverifiable.
    missing = sorted(set(expected_digests) - set(digests))
    changed = sorted(trial for trial, digest in expected_digests.items()
                     if trial in digests and digests[trial] != digest)
    audited = {trial: digests[trial] for trial in expected_digests
               if trial in digests}
    campaign = _campaign_digest_over(audited)
    retained = len(by_stem) - len(audited)
    mismatched_campaign = (expected_campaign is not None
                           and campaign != expected_campaign)
    if missing or changed or mismatched_campaign:
        raise ArchiveMismatch(
            f"archive does not match {audit_path}: missing={missing} "
            f"changed={changed} campaign_digest={campaign} "
            f"expected={expected_campaign}")
    return {
        "verified": True,
        "records": len(expected_digests),
        "unaudited_files_retained": retained,
        "campaign_digest": campaign,
        "campaign_digest_checked": expected_campaign is not None,
        "parts": [part.name for part in _parts_for(Path(destination))],
        "bytes": sum(part.stat().st_size for part in _parts_for(Path(destination))),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Retain a campaign's raw records beside its audit (ADR 0005)")
    parser.add_argument("--raw", type=Path, required=True,
                        help="the campaign's raw record directory")
    parser.add_argument("--audit", type=Path, required=True,
                        help="the committed audit whose digests the archive must match")
    parser.add_argument("--output", type=Path, required=True,
                        help="destination .tar.xz, split at the part limit if needed")
    parser.add_argument("--part-limit-mb", type=float, default=45.0)
    parser.add_argument("--verify-only", action="store_true",
                        help="verify an existing archive without rewriting it")
    arguments = parser.parse_args()

    limit = int(arguments.part_limit_mb * 1024 * 1024)
    if not arguments.verify_only:
        parts = archive_campaign(arguments.raw, arguments.output, limit)
        for part in parts:
            size = part.stat().st_size
            if size > limit:
                raise SystemExit(
                    f"size guard: {part} is {size} bytes, over the {limit} limit")
    report = verify_archive(arguments.output, arguments.audit)
    print(json.dumps(report, indent=2, sort_keys=True))
