#!/bin/bash
# mk_highlighted.sh — IEEE Access "Highlighted PDF": paper_access_highlighted.pdf
# latexdiff(submitted_v1 -> current) with additions in blue, deletions omitted;
# tables/figures are treated as blocks and entirely-new floats are colored blue.
set -e
cd "$(dirname "$0")"
LD=${LATEXDIFF:-$HOME/.local/bin/latexdiff}
$LD --preamble=diffpreamble.tex --math-markup=0 --graphics-markup=0 \
    --exclude-textcmd="section,subsection,caption,title" \
    --config "PICTUREENV=(?:picture|DIFnomarkup|table\*?|tabular|figure\*?)[\w\d*@]*" \
    submitted_v1/paper_access.tex paper_access.tex > paper_access_highlighted.tex 2>latexdiff.log
python3 - <<'PY'
import re
p='paper_access_highlighted.tex'; s=open(p).read()
# color entirely-new floats: inside \DIFaddbegin ... \DIFaddend blocks, inject \color{blue}
# right after every \begin{table...}[..] / \begin{figure...}[..] opening line.
def fix(block):
    return re.sub(r'(\\begin\{(?:table|figure)\*?\}(?:\[[^\]]*\])?)', r'\1\\color{blue}', block.group(0))
s=re.sub(r'\\DIFaddbegin.*?\\DIFaddend', fix, s, flags=re.S)
# The author block and the biographies are not scientific changes: strip all DIF
# markup there by substituting the clean regions from the current manuscript, and
# reset the text color before the bibliography so no blue leaks into it.
cur=open('paper_access.tex').read()
def region(txt, a, b):
    i=txt.index(a); j=txt.index(b, i); return i, j
i,j=region(s, '\\author{', '\\tfootnote'); ic,jc=region(cur, '\\author{', '\\tfootnote')
s=s[:i]+cur[ic:jc]+s[j:]
i=s.index('\\begin{IEEEbiography}'); ic=cur.index('\\begin{IEEEbiography}')
s=s[:i]+'\\color{black}\n'+cur[ic:]
s=s.replace('\\bibliographystyle{IEEEtran}', '\\color{black}\n\\bibliographystyle{IEEEtran}')
open(p,'w').write(s)
PY
pdflatex -interaction=nonstopmode paper_access_highlighted.tex >/dev/null
bibtex paper_access_highlighted >/dev/null 2>&1 || true
pdflatex -interaction=nonstopmode paper_access_highlighted.tex >/dev/null
pdflatex -interaction=nonstopmode paper_access_highlighted.tex >/dev/null
echo "errors: $(grep -c '^!' paper_access_highlighted.log)   pages: $(pdfinfo paper_access_highlighted.pdf | awk '/Pages/{print $2}')"
