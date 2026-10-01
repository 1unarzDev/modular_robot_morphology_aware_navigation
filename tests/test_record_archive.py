"""An archive is only evidence if it restores to the set its audit describes.

Cover for ADR 0005: results/ is gitignored and nine cited Gate 0 record sets
were destroyed that way, so a campaign that backs a claim is committed as a
compressed archive beside its audit. These tests are the restore-and-verify
check the ADR required before adoption.
"""

import json

import pytest

from modular_robot_benchmarks.record_archive import (
    ArchiveMismatch, _campaign_digest_over, archive_campaign, restore_campaign,
    verify_archive,
)
from modular_robot_benchmarks.records import TrialStore


def _campaign(directory, count=4, filler="x"):
    directory.mkdir(parents=True, exist_ok=True)
    for index in range(count):
        payload = {"trial": f"r{index:02d}", "terminal_status": "completed",
                   "filler": filler * 2048}
        (directory / f"campaign-r{index:02d}.json").write_text(
            json.dumps(payload, sort_keys=True, indent=2), encoding="utf-8")
    return directory


def _audit_for(directory, path):
    store = TrialStore(directory)
    path.write_text(json.dumps({
        "record_digests": store.digests(),
        "campaign_digest": store.campaign_digest(),
    }, sort_keys=True, indent=2), encoding="utf-8")
    return path


def test_an_archive_restores_byte_for_byte_and_verifies(tmp_path):
    raw = _campaign(tmp_path / "raw")
    audit = _audit_for(raw, tmp_path / "audit.json")
    parts = archive_campaign(raw, tmp_path / "records" / "campaign.tar.xz")
    assert len(parts) == 1

    report = verify_archive(tmp_path / "records" / "campaign.tar.xz", audit)
    assert report["verified"] is True
    assert report["records"] == 4

    restored = restore_campaign(tmp_path / "records" / "campaign.tar.xz",
                                tmp_path / "restored")
    for original in sorted(raw.glob("*.json")):
        assert (restored / original.name).read_bytes() == original.read_bytes()


def test_a_record_altered_after_archiving_is_caught(tmp_path):
    raw = _campaign(tmp_path / "raw")
    audit = _audit_for(raw, tmp_path / "audit.json")
    archive_campaign(raw, tmp_path / "campaign.tar.xz")
    # The audit now describes a set the archive no longer contains.
    (raw / "campaign-r01.json").write_text(
        '{"trial": "tampered"}', encoding="utf-8")
    _audit_for(raw, audit)
    with pytest.raises(ArchiveMismatch, match="campaign_digest"):
        verify_archive(tmp_path / "campaign.tar.xz", audit)


def test_a_short_archive_is_caught(tmp_path):
    raw = _campaign(tmp_path / "raw", count=5)
    audit = _audit_for(raw, tmp_path / "audit.json")
    (raw / "campaign-r04.json").unlink()
    archive_campaign(raw, tmp_path / "campaign.tar.xz")
    with pytest.raises(ArchiveMismatch, match="missing"):
        verify_archive(tmp_path / "campaign.tar.xz", audit)


def test_an_audit_without_digests_cannot_stand_in_for_verification(tmp_path):
    raw = _campaign(tmp_path / "raw")
    audit = tmp_path / "audit.json"
    audit.write_text(json.dumps({"passed": True}), encoding="utf-8")
    archive_campaign(raw, tmp_path / "campaign.tar.xz")
    with pytest.raises(ArchiveMismatch, match="no record_digests"):
        verify_archive(tmp_path / "campaign.tar.xz", audit)


def test_a_set_over_the_part_limit_splits_and_still_verifies(tmp_path):
    raw = _campaign(tmp_path / "raw", count=6, filler="abcdefgh")
    audit = _audit_for(raw, tmp_path / "audit.json")
    # A deliberately tiny limit: the split path must be exercised, not assumed.
    parts = archive_campaign(raw, tmp_path / "campaign.tar.xz", part_limit_bytes=256)
    assert len(parts) > 1
    assert all(part.stat().st_size <= 256 for part in parts)
    assert [part.name for part in parts] == sorted(part.name for part in parts)
    assert verify_archive(tmp_path / "campaign.tar.xz", audit)["verified"] is True


def test_rearchiving_replaces_stale_parts_rather_than_mixing_them(tmp_path):
    raw = _campaign(tmp_path / "raw", count=6, filler="abcdefgh")
    archive_campaign(raw, tmp_path / "campaign.tar.xz", part_limit_bytes=256)
    audit = _audit_for(raw, tmp_path / "audit.json")
    parts = archive_campaign(raw, tmp_path / "campaign.tar.xz")
    assert len(parts) == 1
    assert not list(tmp_path.glob("campaign.tar.xz.part*"))
    assert verify_archive(tmp_path / "campaign.tar.xz", audit)["verified"] is True


def test_an_empty_directory_is_refused_rather_than_archived(tmp_path):
    empty = tmp_path / "raw"
    empty.mkdir()
    with pytest.raises(ValueError, match="no records"):
        archive_campaign(empty, tmp_path / "campaign.tar.xz")


def test_the_subset_digest_matches_TrialStore_for_a_whole_set(tmp_path):
    # The archive's campaign digest must be the same function the audits used,
    # or verification would reject sets it should accept.
    raw = _campaign(tmp_path / "raw")
    store = TrialStore(raw)
    assert _campaign_digest_over(store.digests()) == store.campaign_digest()


def test_unaudited_records_may_be_retained_without_failing_verification(tmp_path):
    raw = _campaign(tmp_path / "raw")
    audit = _audit_for(raw, tmp_path / "audit.json")
    # A qualification sub-record the campaign audit never scored.
    (raw / "campaign-extra-qualification.json").write_text(
        '{"kind": "sub_record"}', encoding="utf-8")
    archive_campaign(raw, tmp_path / "campaign.tar.xz")
    report = verify_archive(tmp_path / "campaign.tar.xz", audit)
    assert report["verified"] is True
    assert report["records"] == 4
    assert report["unaudited_files_retained"] == 1


def test_an_audit_without_a_campaign_digest_still_verifies_on_record_digests(tmp_path):
    raw = _campaign(tmp_path / "raw")
    store = TrialStore(raw)
    audit = tmp_path / "audit.json"
    audit.write_text(json.dumps({"record_digests": store.digests()}),
                     encoding="utf-8")
    archive_campaign(raw, tmp_path / "campaign.tar.xz")
    report = verify_archive(tmp_path / "campaign.tar.xz", audit)
    assert report["verified"] is True
    assert report["campaign_digest_checked"] is False
    # A changed record is still caught without one.
    (raw / "campaign-r00.json").write_text('{"trial": "tampered"}', encoding="utf-8")
    archive_campaign(raw, tmp_path / "campaign.tar.xz")
    with pytest.raises(ArchiveMismatch, match="changed"):
        verify_archive(tmp_path / "campaign.tar.xz", audit)


def test_an_audit_keyed_by_file_name_verifies_too(tmp_path):
    # The item-1 qualification audits key record_digests by file name, where
    # campaign audits key by trial id. Both must verify against one archive.
    raw = _campaign(tmp_path / "raw")
    store = TrialStore(raw)
    by_name = {f"{stem}.json": digest for stem, digest in store.digests().items()}
    audit = tmp_path / "audit.json"
    audit.write_text(json.dumps({"record_digests": by_name}), encoding="utf-8")
    archive_campaign(raw, tmp_path / "campaign.tar.xz")
    report = verify_archive(tmp_path / "campaign.tar.xz", audit)
    assert report["verified"] is True
    assert report["records"] == 4
    assert report["unaudited_files_retained"] == 0
