#!/usr/bin/env python3
"""mk_docx.py — response.md -> response.docx (minimal WordprocessingML, no external deps)."""
import re, zipfile, html
md=open('response.md',encoding='utf-8').read()
def esc(t): return html.escape(t.replace('$',''), quote=False)
def para(runs, style=None, shade=False):
    ppr='<w:pPr>'+(f'<w:pStyle w:val="{style}"/>' if style else '')+('<w:shd w:val="clear" w:color="auto" w:fill="E7E6E6"/>' if shade else '')+'<w:spacing w:after="120"/></w:pPr>'
    body=''.join(f'<w:r><w:rPr>{"<w:b/>" if b else ""}</w:rPr><w:t xml:space="preserve">{esc(t)}</w:t></w:r>' for t,b in runs)
    return f'<w:p>{ppr}{body}</w:p>'
def bullet(t): return f'<w:p><w:pPr><w:ind w:left="540" w:hanging="270"/><w:spacing w:after="80"/></w:pPr><w:r><w:t xml:space="preserve">•  {esc(t)}</w:t></w:r></w:p>'
P=[]; lines=md.split('\n'); i=0
while i<len(lines):
    L=lines[i].rstrip()
    if L.startswith('====='): P.append(para([(lines[i+1].strip(),True)],'Heading1')); i+=3; continue
    if L.startswith('-----'): i+=1; continue
    m=re.match(r'^(Reviewer#\d+, [^(:]+?)(?:\s*\((.*)\))?:\s*(.*)$', L)
    if m:
        P.append(para([(m.group(1)+(f" ({m.group(2)})" if m.group(2) else ""),True)],shade=True))
        if m.group(3): P.append(para([(m.group(3),False)]))
        i+=1; continue
    hit=False
    for lab in ('Author response and action:','Author response:','Author action:'):
        if L.startswith(lab): P.append(para([(lab+' ',True),(L[len(lab):].strip(),False)])); hit=True; break
    if hit: i+=1; continue
    if L.startswith('- '):
        while i<len(lines) and lines[i].startswith('- '): P.append(bullet(lines[i][2:])); i+=1
        continue
    if L.strip(): P.append(para([(L,False)]))
    i+=1
doc=f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?><w:document xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main"><w:body>{"".join(P)}<w:sectPr><w:pgSz w:w="11906" w:h="16838"/><w:pgMar w:top="1300" w:right="1300" w:bottom="1300" w:left="1300"/></w:sectPr></w:body></w:document>'
styles='<?xml version="1.0" encoding="UTF-8" standalone="yes"?><w:styles xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main"><w:docDefaults><w:rPrDefault><w:rPr><w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman"/><w:sz w:val="22"/></w:rPr></w:rPrDefault></w:docDefaults><w:style w:type="paragraph" w:styleId="Heading1"><w:name w:val="heading 1"/><w:pPr><w:spacing w:before="360" w:after="120"/></w:pPr><w:rPr><w:b/><w:sz w:val="30"/></w:rPr></w:style></w:styles>'
ct='<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="xml" ContentType="application/xml"/><Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/><Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/></Types>'
rels='<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/></Relationships>'
drels='<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/></Relationships>'
with zipfile.ZipFile('response.docx','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('[Content_Types].xml',ct); z.writestr('_rels/.rels',rels); z.writestr('word/document.xml',doc); z.writestr('word/styles.xml',styles); z.writestr('word/_rels/document.xml.rels',drels)
print("response.docx written:", len(P), "paragraphs")
