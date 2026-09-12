from __future__ import annotations

from html import escape
from pathlib import Path


COLORS = ("#64748b", "#f59e0b", "#0ea5e9", "#16a34a")


def _document(width: int, height: int, body: list[str], title: str) -> str:
    return "\n".join([
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        f"<title>{escape(title)}</title>",
        '<rect width="100%" height="100%" fill="white"/>',
        '<style>text{font-family:system-ui,sans-serif;fill:#172033}.title{font-size:20px;font-weight:700}.label{font-size:12px}.tick{font-size:11px;fill:#526071}.grid{stroke:#dce2e8;stroke-width:1}.axis{stroke:#7b8794;stroke-width:1.2}</style>',
        *body, "</svg>", "",
    ])


def _short_method(value: str) -> str:
    return value.replace("_coupled", "").replace("route_first_adaptation", "route first").replace("_", " ")


def _completion_figure(result: dict) -> str:
    width, height, left, right = 780, 340, 205, 735
    body = ['<text class="title" x="24" y="30">Mission completion by method</text>']
    for tick in range(6):
        x = left + (right - left) * tick / 5
        body.extend([
            f'<line class="grid" x1="{x:.1f}" y1="48" x2="{x:.1f}" y2="285"/>',
            f'<text class="tick" x="{x:.1f}" y="305" text-anchor="middle">{tick / 5:.1f}</text>',
        ])
    for index, row in enumerate(result["methods"]):
        y = 80 + index * 52
        value = row["completion_rate"]
        low, high = row["completion_rate_ci95"]
        x, xlow, xhigh = (left + (right - left) * item for item in (value, low, high))
        body.extend([
            f'<text class="label" x="195" y="{y + 4}" text-anchor="end">{escape(_short_method(row["method"]))}</text>',
            f'<rect x="{left}" y="{y - 10}" width="{max(0, x-left):.1f}" height="20" fill="{COLORS[index]}" opacity="0.75"/>',
            f'<line x1="{xlow:.1f}" y1="{y}" x2="{xhigh:.1f}" y2="{y}" stroke="#172033" stroke-width="2"/>',
            f'<line x1="{xlow:.1f}" y1="{y-6}" x2="{xlow:.1f}" y2="{y+6}" stroke="#172033"/>',
            f'<line x1="{xhigh:.1f}" y1="{y-6}" x2="{xhigh:.1f}" y2="{y+6}" stroke="#172033"/>',
            f'<circle cx="{x:.1f}" cy="{y}" r="4" fill="#172033"/>',
        ])
    body.append('<text class="label" x="470" y="330" text-anchor="middle">Completion proportion (95% layout-cluster bootstrap CI)</text>')
    return _document(width, height, body, "Mission completion by method")


def _effect_figure(result: dict) -> str:
    width, height, left, right = 820, 270, 250, 770
    rows = result["primary_contrasts"]
    values = [number for row in rows for number in (
        row["estimate_treatment_minus_control"], *row["ci95_layout_bootstrap"])]
    limit = max(0.1, min(1.0, max(abs(value) for value in values) * 1.25))
    scale = lambda value: left + (value + limit) * (right - left) / (2 * limit)
    body = ['<text class="title" x="24" y="30">Primary paired completion effects</text>']
    for tick in range(5):
        value = -limit + 2 * limit * tick / 4
        x = scale(value)
        body.extend([
            f'<line class="grid" x1="{x:.1f}" y1="50" x2="{x:.1f}" y2="205"/>',
            f'<text class="tick" x="{x:.1f}" y="225" text-anchor="middle">{value:.2f}</text>',
        ])
    body.append(f'<line x1="{scale(0):.1f}" y1="45" x2="{scale(0):.1f}" y2="205" stroke="#172033" stroke-width="1.5"/>')
    for index, row in enumerate(rows):
        y = 90 + index * 72
        low, high = row["ci95_layout_bootstrap"]
        label = f'{_short_method(row["treatment"])} − {_short_method(row["control"])}'
        body.extend([
            f'<text class="label" x="240" y="{y + 4}" text-anchor="end">{escape(label)}</text>',
            f'<line x1="{scale(low):.1f}" y1="{y}" x2="{scale(high):.1f}" y2="{y}" stroke="#0f6c8d" stroke-width="3"/>',
            f'<circle cx="{scale(row["estimate_treatment_minus_control"]):.1f}" cy="{y}" r="6" fill="#0f6c8d"/>',
        ])
    body.append('<text class="label" x="510" y="255" text-anchor="middle">Absolute completion-rate difference (treatment − control)</text>')
    return _document(width, height, body, "Primary paired completion effects")


def _calibration_figure(result: dict) -> str:
    width, height, left, right, top, bottom = 560, 430, 70, 520, 55, 360
    sx = lambda value: left + value * (right - left)
    sy = lambda value: bottom - value * (bottom - top)
    body = ['<text class="title" x="24" y="30">Transition-risk calibration</text>']
    for tick in range(6):
        value = tick / 5
        x, y = sx(value), sy(value)
        body.extend([
            f'<line class="grid" x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{bottom}"/>',
            f'<line class="grid" x1="{left}" y1="{y:.1f}" x2="{right}" y2="{y:.1f}"/>',
            f'<text class="tick" x="{x:.1f}" y="{bottom+20}" text-anchor="middle">{value:.1f}</text>',
            f'<text class="tick" x="{left-10}" y="{y+4:.1f}" text-anchor="end">{value:.1f}</text>',
        ])
    body.append(f'<line x1="{left}" y1="{bottom}" x2="{right}" y2="{top}" stroke="#7b8794" stroke-dasharray="5 4"/>')
    for index, (method, rows) in enumerate(result["transition_calibration_by_method"].items()):
        points = " ".join(f'{sx(row["mean_prediction"]):.1f},{sy(row["observed_frequency"]):.1f}' for row in rows)
        color = COLORS[index % len(COLORS)]
        body.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2"/>')
        for row in rows:
            radius = min(8.0, 2.5 + row["count"] ** 0.5 / 2)
            body.append(f'<circle cx="{sx(row["mean_prediction"]):.1f}" cy="{sy(row["observed_frequency"]):.1f}" r="{radius:.1f}" fill="{color}" opacity="0.8"/>')
        body.append(f'<text class="tick" x="{left + 5}" y="{bottom + 42 + index*15}"><tspan fill="{color}">●</tspan> {escape(_short_method(method))}</text>')
    body.extend([
        f'<text class="label" x="{(left+right)/2}" y="{height-10}" text-anchor="middle">Predicted success probability</text>',
        f'<text class="label" transform="translate(18 {(top+bottom)/2}) rotate(-90)" text-anchor="middle">Observed success frequency</text>',
    ])
    return _document(width, height, body, "Transition-risk calibration")


def write_svg_figures(result: dict, output: str | Path) -> None:
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "completion.svg").write_text(_completion_figure(result), encoding="utf-8")
    (output / "primary_effects.svg").write_text(_effect_figure(result), encoding="utf-8")
    (output / "transition_calibration.svg").write_text(_calibration_figure(result), encoding="utf-8")
