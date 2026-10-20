"""Extract per-bit definitions of H1/H2 data flash parameters from the TRM text.

    python tools/trm_parse_bits.py trm.txt docs/df_table_4800_0_04.json docs/df_bits_4800_0_04.json

trm.txt is TI SLUUBT5C converted to text with pypdf (one PdfReader page.extract_text()
per page). Output: {address: [{name, lo, width, desc, values}]}, used by
tools/gen_df_catalog.py to give the app bit names and meanings.
"""
import json
import re
import sys

trm = open(sys.argv[1], encoding='utf-8').read().split('\n')
table = json.load(open(sys.argv[2], encoding='utf-8'))['params']

def norm(s):
    return s.replace('–', '-').replace('−', '-').replace('—', '-')

start = next(i for i, l in enumerate(trm) if l.startswith('19 Data Flash Values') or l.strip() == '19 Data Flash Values')
end = next(i for i, l in enumerate(trm) if l.strip() == '19.17 Data Flash Summary' and i > start)
heading = re.compile(r'^19\.\d+(?:\.\d+)+ (.+)$')
row = re.compile(r'^(.*?) (H1|H2) (0[xX][0-9A-Fa-f]+) (0[xX][0-9A-Fa-f]+) (0[xX][0-9A-Fa-f]+)')
pair = re.compile(r'^([A-Za-z0-9_]+) \(Bit (\d+)\), ([A-Za-z0-9_]+) \(Bit (\d+)\)\s*:?\s*(.*)$')
value = re.compile(r'^((?:[01],)*[01]|0x[0-9A-Fa-f]+|[01]{2,}|\d+) = (.+)$')
bitline = re.compile(r'^([A-Za-z0-9_/\[\]]+(?:_[A-Za-z0-9]+)*) \(Bits? (\d+)(?:\s*-\s*(\d+))?\)\s*:?\s*(.*)$')
noise = re.compile(r'^(www\.ti\.com|SLUUBT5C|Submit Document|Copyright|=====PAGE|BQ40Z80 \d+|\d+ BQ40Z80|Data Flash Values)')

# Split the chapter into sections.
sections, cur = [], None
for l in (norm(x).strip() for x in trm[start:end]):
    if not l or noise.match(l):
        continue
    m = heading.match(l)
    if m:
        cur = {'title': m.group(1), 'lines': []}
        sections.append(cur)
    elif cur is not None:
        cur['lines'].append(l)

by_name = {}
for p in table:
    if p['type'] in ('H1', 'H2'):
        by_name.setdefault(p['name'].lower(), []).append(p)

out = {}
unmatched = []
for sec in sections:
    rowtext = next((l for l in sec['lines'] if row.match(l)), None)
    if not rowtext:
        continue
    # Bit descriptions: "NAME (Bit n): text" (text may continue on the next line).
    bits = []
    lines = sec['lines']
    for i, l in enumerate(lines):
        mp = pair.match(l)
        m = None if mp else bitline.match(l)
        if mp:
            hi, lo = max(int(mp.group(2)), int(mp.group(4))), min(int(mp.group(2)), int(mp.group(4)))
            name, desc = mp.group(1) + '/' + mp.group(3), mp.group(5).strip()
        elif m:
            hi = int(m.group(2))
            lo = int(m.group(3)) if m.group(3) else hi
            if lo > hi:
                hi, lo = lo, hi
            name, desc = m.group(1), m.group(4).strip()
        else:
            continue
        # "1 = ... / 0 = ..." (or field values) until the next bit.
        values = []
        for nxt in lines[i + 1:]:
            if pair.match(nxt) or bitline.match(nxt):
                break
            v = value.match(nxt)
            if v:
                values.append(f'{v.group(1)} = {v.group(2).strip()}')
            elif values:
                values[-1] += ' ' + nxt          # wrapped value text
            elif not desc:
                desc = nxt
            elif len(desc) < 200 and not desc.endswith('.'):
                desc += ' ' + nxt                # wrapped description
        if value.match(desc):
            values.insert(0, desc)
            desc = ''
        bits.append({'name': name, 'lo': lo, 'width': hi - lo + 1,
                     'desc': desc.rstrip('.'), 'values': values[:8]})
    if not bits:
        continue
    cands = by_name.get(sec['title'].lower(), [])
    if len(cands) > 1:
        cands = ([p for p in cands if p['cls'] in rowtext and p['sub'] in rowtext]
                 or [p for p in cands if (' ' + p['sub'] + ' ') in (' ' + rowtext + ' ')] or cands)
    if len(cands) != 1:
        unmatched.append((sec['title'], len(cands)))
        continue
    p = cands[0]
    seen = set()
    clean = []
    for b in sorted(bits, key=lambda b: -b['lo']):
        if b['lo'] in seen:
            continue
        seen.add(b['lo'])
        clean.append(b)
    out[p['addr']] = clean

json.dump(out, open(sys.argv[3], 'w', encoding='utf-8'), indent=1, ensure_ascii=False)
print('registers with bits:', len(out))
print('unmatched:', unmatched)
