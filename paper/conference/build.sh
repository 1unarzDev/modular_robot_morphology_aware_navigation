#!/bin/bash
# Build the conference paper on the Windows host (Git Bash, MiKTeX, Edge).
# pdflatex cannot read SVG, so each figure_*.svg is printed to a PDF page of
# exactly its own size by headless Edge first. Output goes to build/ (ignored).
#
#     bash paper/conference/build.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
out="$here/build"
edge="${EDGE:-/c/Program Files (x86)/Microsoft/Edge/Application/msedge.exe}"
mkdir -p "$out"
cp "$here/main.tex" "$out/"
for svg in "$here"/figure_*.svg; do
  name="$(basename "$svg" .svg)"
  read -r width height < <(head -c 400 "$svg" |
    sed -n 's/.*width="\([0-9.]*\)" height="\([0-9.]*\)".*/\1 \2/p' | head -1)
  printf '<html><head><style>@page{size:%spx %spx;margin:0}html,body{margin:0}</style></head>' \
    "$width" "$height" > "$out/$name.html"
  printf '<body><img src="file:///%s" width="%s" height="%s"></body></html>' \
    "$(cygpath -m "$svg")" "$width" "$height" >> "$out/$name.html"
  "$edge" --headless --disable-gpu --no-pdf-header-footer \
    --print-to-pdf="$(cygpath -m "$out/$name.pdf")" "file:///$(cygpath -m "$out/$name.html")" \
    >/dev/null 2>&1
done
cd "$out"
pdflatex -interaction=nonstopmode -halt-on-error main.tex >/dev/null
pdflatex -interaction=nonstopmode -halt-on-error main.tex | grep -E "Output written|Warning" || true
