#!/usr/bin/env python3
"""mk_response.py — build response.tex (+pdf) from response.md (single source)."""
import re, subprocess, os
md = open("response.md", encoding="utf-8").read()

U = {"σ̃": r"$\tilde\sigma$", "σ": r"$\sigma$", "Σ": r"$\Sigma$", "δ": r"$\delta$", "ω": r"$\omega$", "β": r"$\beta$",
     "≈": r"$\approx$", "±": r"$\pm$", "≤": r"$\le$", "≥": r"$\ge$", "≳": r"$\gtrsim$", "→": r"$\to$", "×": r"$\times$",
     "⊗": r"$\otimes$", "·": r"$\cdot$", "²": r"$^2$", "³": r"$^3$", "⁻¹": r"$^{-1}$", "⁻": r"$^{-}$", "ᵀ": r"$^{\top}$",
     "—": "---", "–": "--", "“": "``", "”": "''", "‘": "`", "’": "'", "…": r"\ldots{}", "√": r"$\surd$", "∈": r"$\in$",
     "Cramér–Rao": "Cram\\'er--Rao", "é": "\\'e", "á": "\\'a", "í": "\\'i", "ó": "\\'o", "ú": "\\'u", "ü": '\\"u', "ñ": "\\~n",
     "%": r"\%", "&": r"\&", "_": r"\_", "#": r"\#", "~": r"$\sim$"}
SUP = str.maketrans("⁰¹²³⁴⁵⁶⁷⁸⁹⁻", "0123456789-")
def tex(t):
    # segments written as $...$ in the md are raw LaTeX math: protect them
    parts = re.split(r"(\$[^$]*\$)", t)
    return "".join(pp if pp.startswith("$") else _tex(pp) for pp in parts)
def _tex(t):
    t = t.replace("\\", "\\textbackslash{}")
    t = re.sub(r'"([^"]+)"', r"``\1''", t)
    t = re.sub(r"[⁰¹²³⁴⁵⁶⁷⁸⁹⁻]+", lambda m: "$^{" + m.group(0).translate(SUP) + "}$", t)
    t = t.replace("−", "$-$").replace("m\u0302", "$\\hat m$").replace("d\u0302", "$\\hat d$").replace("\u0302", "")
    for k, v in U.items(): t = t.replace(k, v)
    # inline math-ish tokens that read better in math: q_d, q_m, r_s, T/m, m/s etc. handled via \_ already
    t = re.sub(r"\bq\\_d\b", r"$q_d$", t); t = re.sub(r"\bq\\_m\b", r"$q_m$", t); t = re.sub(r"\br\\_s\b", r"$r_s$", t)
    t = t.replace("m/s$^2$", "m/s$^2$")
    return t

out = [r"""\documentclass[11pt]{article}
\usepackage[a4paper,margin=2.3cm]{geometry}
\usepackage[T1]{fontenc}\usepackage[utf8]{inputenc}\usepackage{lmodern}
\usepackage{amsmath,amssymb}\usepackage{xcolor}\usepackage{enumitem}
\usepackage[hidelinks]{hyperref}
\setlength{\parindent}{0pt}\setlength{\parskip}{6pt}
\newcommand{\concern}[1]{\par\medskip\noindent\colorbox{gray!12}{\parbox{\dimexpr\linewidth-2\fboxsep}{\textbf{#1}}}\par\smallskip}
\newcommand{\rlabel}[1]{\par\textbf{#1}\ }
\begin{document}
"""]
lines = md.split("\n"); i = 0
while i < len(lines):
    L = lines[i].rstrip()
    if L.startswith("====="):
        title = lines[i+1].strip(); i += 3
        out.append("\\section*{%s}\n" % tex(title)); continue
    if L.startswith("-----"): i += 1; continue
    m = re.match(r"^(Reviewer#\d+, [^(:]+?)\s*\((.*)\):\s*$", L)
    m2 = re.match(r"^(Reviewer#\d+, [^(:]+?)\s*\((.*)\):\s*(.+)$", L)
    m3 = re.match(r"^(Reviewer#\d+, [^(:]+?):\s*$", L)
    if not m and not m2 and m3:
        out.append("\\concern{%s}\n" % tex(m3.group(1))); i += 1; continue
    if m: out.append("\\concern{%s (%s)}\n" % (tex(m.group(1)), tex(m.group(2)))); i += 1; continue
    if m2:
        out.append("\\concern{%s (%s)}\n" % (tex(m2.group(1)), tex(m2.group(2)))); out.append(tex(m2.group(3)) + "\n"); i += 1; continue
    if L.startswith("Author response and action:"):
        out.append("\\rlabel{Author response and action:}" + tex(L[len("Author response and action:"):].strip()) + "\n"); i += 1; continue
    if L.startswith("Author response:"):
        out.append("\\rlabel{Author response:}" + tex(L[len("Author response:"):].strip()) + "\n"); i += 1; continue
    if L.startswith("Author action:"):
        out.append("\\rlabel{Author action:}" + tex(L[len("Author action:"):].strip()) + "\n"); i += 1; continue
    if L.startswith("> "):
        q = []
        while i < len(lines) and lines[i].startswith("> "):
            q.append(tex(lines[i][2:].strip())); i += 1
        out.append("\\begin{quote}\\itshape " + "\\par ".join(q) + "\\end{quote}\n"); continue
    if L.startswith("$$") and L.rstrip().endswith("$$") and len(L.strip()) > 4:
        out.append("\\[" + L.strip()[2:-2].strip() + "\\]\n"); i += 1; continue
    if L.startswith("- "):
        items = []
        while i < len(lines) and lines[i].startswith("- "): items.append(tex(lines[i][2:].strip())); i += 1
        out.append("\\begin{itemize}[leftmargin=1.4em]\n" + "".join("\\item %s\n" % x for x in items) + "\\end{itemize}\n"); continue
    if L.startswith("Original Manuscript ID") or L.startswith("Original Article Title") or L.startswith("(revised title") or L.startswith("To:") or L.startswith("Re:"):
        out.append(tex(L) + "\\\\\n"); i += 1; continue
    out.append(tex(L) + "\n"); i += 1
out.append("\\end{document}\n")
open("response.tex", "w", encoding="utf-8").write("".join(out))
r = subprocess.run(["pdflatex", "-interaction=nonstopmode", "response.tex"], capture_output=True, text=True)
r = subprocess.run(["pdflatex", "-interaction=nonstopmode", "response.tex"], capture_output=True, text=True)
log = open("response.log", errors="ignore").read()
print("errors:", log.count("\n!"), "| pages:", re.findall(r"Output written on response.pdf \((\d+) page", log))
