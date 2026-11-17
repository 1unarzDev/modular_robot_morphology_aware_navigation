"""Every committed paper figure is well-formed SVG.

`figure_results.svg` once carried a bare "p < 0.001", which the browser that
prints figures for pdflatex refuses, so Fig. 3 built as an empty box.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

FIGURES = sorted((Path(__file__).resolve().parents[1] / "paper" / "conference").glob("figure_*.svg"))


@pytest.mark.parametrize("figure", FIGURES, ids=[f.name for f in FIGURES])
def test_figure_is_well_formed_svg(figure):
    assert ET.parse(figure).getroot().tag.endswith("svg")


def test_there_are_figures():
    assert len(FIGURES) >= 3
