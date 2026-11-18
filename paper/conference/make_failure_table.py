"""Table: terminal status by method and family, from the raw terminal records.

The frozen analysis reports the failure taxonomy pooled over methods and
families, while the interpretation rules (statistical analysis plan, declared
2026-11-04 before any confirmatory outcome was read) only allow a mechanism
statement as a count by method and family. This tabulates exactly that:
each record's single terminal status, which already names the earliest
evaluator cause, grouped and counted. It decides nothing and excludes nothing.
Neutral controls (every fourth layout) are one row; the family rows hold the
manipulated layouts.

    python paper/conference/make_failure_table.py <raw-record-dir> \
        paper/conference/failure_table.tex [failure_table.json]

infrastructure_macros.tex is written beside the table.

<raw-record-dir> may instead be the first part of the retained archive
(`studies/confirmatory_v3/records/confirmatory_v3_raw.tar.xz.part00`); the
parts are then streamed in order and nothing is extracted.

It also splits the runner's infrastructure attempts by when they failed: a
stack readiness timeout is before the mission; anything else (a DDS transport
that failed to initialize, a stack process death, a clock discontinuity, an
observer killed by the wall watchdog) relabelled a mission already under way.

Stdlib only, reading one record at a time, so it runs on the WSL host Python.
"""
from __future__ import annotations

import io
import json
import sys
import tarfile
from pathlib import Path

METHODS = ("route_first_adaptation", "geometry_coupled",
           "feasibility_coupled", "sensing_feasibility_coupled")
HEADERS = ("route first", "geometry", "feasibility", r"sens.\,+\,feas.")
MACRO_METHODS = ("RouteFirst", "Geometry", "Feasibility", "Sensing")
FAMILIES = (("reconfiguration_workspace", "Reconfiguration workspace", "Workspace"),
            ("docking_observability", "Docking observability", "Docking"),
            ("combined_constraints", "Combined constraints", "Combined"))
# Each status is one evaluator cause (analysis plan, "Outcome definitions").
GROUPS = (("Completed", "Completed", ("completed",)),
          ("Planning failure", "Planning", ("planning_failure",)),
          ("Contact", "Contact", ("collision",)),
          ("Docking refused", "Docking", ("docking_failure", "unsafe_topology")),
          ("Localization lost", "Localization", ("localization_lost",)),
          ("Other", "Other", None))


def _neutral(layout_id: str) -> bool:
    return int(layout_id.rsplit("-", 1)[1]) % 4 == 0


def _group(status: str) -> str:
    for _, key, statuses in GROUPS:
        if statuses is not None and status in statuses:
            return key
    return "Other"


class _Concatenated(io.RawIOBase):
    """The archive's split parts read as one stream."""

    def __init__(self, paths: list[Path]):
        self._files = [path.open("rb") for path in paths]
        self._index = 0

    def readable(self) -> bool:
        return True

    def readinto(self, buffer) -> int:
        while self._index < len(self._files):
            count = self._files[self._index].readinto(buffer)
            if count:
                return count
            self._index += 1
        return 0


def _records(raw: Path):
    """Terminal records, from a directory of them or from the split archive."""
    raw = Path(raw)
    if raw.is_dir():
        for path in sorted(raw.glob("*.json")):
            with path.open(encoding="utf-8") as stream:
                yield json.load(stream)
        return
    stem = raw.name.rsplit(".part", 1)[0]
    parts = sorted(raw.parent.glob(stem + ".part*"))
    if not parts:
        raise SystemExit(f"no records at {raw}")
    archive = tarfile.open(fileobj=io.BufferedReader(_Concatenated(parts)), mode="r|xz")
    for member in archive:
        if not (member.isfile() and member.name.endswith(".json")):
            continue
        try:
            record = json.load(archive.extractfile(member))
        except ValueError:
            continue
        if isinstance(record, dict) and "spec" in record and "terminal_status" in record:
            yield record


def tabulate(raw: Path) -> dict:
    """{stratum: {method: {group: count, "trials": n}}}, stratum a family or "neutral"."""
    counts: dict = {}
    other_statuses: dict = {}
    infrastructure = {"before_mission": 0, "during_mission": 0, "trials_retried": 0,
                      "during_mission_then_completed": {}}
    for record in _records(raw):
        spec, status = record["spec"], record["terminal_status"]
        attempts = record.get("infrastructure_attempts") or []
        infrastructure["trials_retried"] += len(attempts) > 1
        during = False
        for attempt in attempts:
            if attempt.get("terminal_status") != "infrastructure_failure":
                continue
            if str(attempt.get("message", "")).startswith("stack readiness timeout"):
                infrastructure["before_mission"] += 1
            else:
                infrastructure["during_mission"] += 1
                during = True
        if during and status == "completed":
            methods = infrastructure["during_mission_then_completed"]
            methods[spec["method"]] = methods.get(spec["method"], 0) + 1
        stratum = "neutral" if _neutral(spec["layout_id"]) else spec["family"]
        cell = counts.setdefault(stratum, {}).setdefault(spec["method"], {"trials": 0})
        group = _group(status)
        cell[group] = cell.get(group, 0) + 1
        cell["trials"] += 1
        if group == "Other":
            other_statuses[status] = other_statuses.get(status, 0) + 1
    return {"counts": counts, "other_statuses": other_statuses,
            "infrastructure_attempts": infrastructure}


def _cell(counts: dict, stratum: str, method: str, group: str) -> int:
    return counts.get(stratum, {}).get(method, {}).get(group, 0)


def _trials(counts: dict, stratum: str, method: str) -> int:
    return counts.get(stratum, {}).get(method, {}).get("trials", 0)


def render(table: dict) -> str:
    counts = table["counts"]
    per_cell = {_trials(counts, family, method) for family, _, _ in FAMILIES
                for method in METHODS}
    lines = [
        f"% Generated by make_failure_table.py from the raw terminal records; do not edit by hand.",
        r"\begin{table}[t]", r"\centering",
        r"\caption{Terminal status by method in manipulated layouts"
        + (f" ({per_cell.pop()} trials per cell)" if len(per_cell) == 1 else "")
        + r", each status naming the earliest evaluator cause, and completion in the"
        r" neutral controls. Descriptive counts; no test.}",
        r"\label{tab:failures}",
        r"\setlength{\tabcolsep}{4pt}",
        r"\begin{tabular}{@{}lrrrr@{}}", r"\toprule",
        " & " + " & ".join(HEADERS) + r" \\",
    ]
    for family, title, _ in FAMILIES:
        lines += [r"\midrule", rf"\multicolumn{{5}}{{@{{}}l}}{{\emph{{{title}}}}} \\"]
        for label, key, _ in GROUPS:
            values = [_cell(counts, family, method, key) for method in METHODS]
            if key in ("Localization", "Other") and not any(values):
                continue
            lines.append(rf"\quad {label} & " + " & ".join(map(str, values)) + r" \\")
    lines += [r"\midrule", r"Neutral controls completed & " + " & ".join(
        f"{_cell(counts, 'neutral', method, 'Completed')}/{_trials(counts, 'neutral', method)}"
        for method in METHODS) + r" \\"]
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    return "\n".join(lines + [macros(table)]) + "\n"


def macros(table: dict) -> str:
    """`\\resTax<Method><Family><Group>` counts and `\\resTrials<Method><Family>`."""
    counts = table["counts"]
    defined = []
    strata = [(family, key) for family, _, key in FAMILIES] + [("neutral", "Neutral")]
    for method, mkey in zip(METHODS, MACRO_METHODS):
        for stratum, fkey in strata:
            defined.append(rf"\newcommand{{\resTrials{mkey}{fkey}}}"
                           f"{{{_trials(counts, stratum, method)}}}")
            for _, group, _ in GROUPS:
                defined.append(rf"\newcommand{{\resTax{mkey}{fkey}{group}}}"
                               f"{{{_cell(counts, stratum, method, group)}}}")
    return "\n".join(defined)


def infrastructure_macros(table: dict) -> str:
    """The retry split, in its own file: Section IV cites it before the table is input."""
    infrastructure = table["infrastructure_attempts"]
    then_completed = infrastructure["during_mission_then_completed"]
    defined = ["% Generated by make_failure_table.py from the raw terminal records; do not edit by hand."]
    defined += [
            rf"\newcommand{{\resInfraBeforeMission}}{{{infrastructure['before_mission']}}}",
            rf"\newcommand{{\resInfraDuringMission}}{{{infrastructure['during_mission']}}}",
            rf"\newcommand{{\resInfraDuringThenCompleted}}{{{sum(then_completed.values())}}}",
            rf"\newcommand{{\resInfraDuringThenCompletedSensing}}"
            f"{{{then_completed.get('sensing_feasibility_coupled', 0)}}}",
    ]
    return "\n".join(defined) + "\n"


def main() -> None:
    table = tabulate(Path(sys.argv[1]))
    Path(sys.argv[2]).write_text(render(table), encoding="utf-8")
    Path(sys.argv[2]).with_name("infrastructure_macros.tex").write_text(
        infrastructure_macros(table), encoding="utf-8")
    if len(sys.argv) > 3:
        Path(sys.argv[3]).write_text(json.dumps(table, indent=2, sort_keys=True) + "\n",
                                     encoding="utf-8")


if __name__ == "__main__":
    main()
