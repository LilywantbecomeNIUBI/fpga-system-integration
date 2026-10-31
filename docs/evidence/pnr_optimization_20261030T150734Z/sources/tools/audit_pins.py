"""Offline consistency check only; no hardware or toolchain acceptance."""
from pathlib import Path
import csv
import re

root = Path(__file__).resolve().parents[1]
rows = list(csv.DictReader((root / 'hardware/pins.csv').open(encoding='utf-8-sig')))
errors = []
for field in ('signal', 'ball'):
    values = [r[field] for r in rows]
    if len(values) != len(set(values)):
        errors.append(f'duplicate {field}')
p7 = ['A13','A14','A15','A16','A18','A19','F13','F14','E13','E14',None,None,'C13','B13','D14','D15','C14','C15','B15','B16','D17','C17','E16','D16','B17','B18','C18','C19',None,None,'F16','E17','D20','C20','E19','D19','F18','E18','F19','F20']
assigned_p7 = [r['p7'] for r in rows if r['p7']]
if len(assigned_p7) != len(set(assigned_p7)):
    errors.append('duplicate P7 position')
if {int(n) for n in assigned_p7} != {i+1 for i,b in enumerate(p7) if b}:
    errors.append('P7 signal coverage incomplete')
for r in rows:
    if r['p7'] and p7[int(r['p7'])-1] != r['ball']:
        errors.append(f'P7 mismatch: {r["signal"]}')
expected = {r['signal']: r['ball'] for r in rows if r['group']=='chassis'}
cst = (root/'fpga/constraints/chassis_candidate.cst').read_text(encoding='utf-8')
locations = re.findall(r'IO_LOC\s+"([^"]+)"\s+(\w+);', cst)
if len(locations) != len(expected) or dict(locations) != expected:
    errors.append('candidate CST does not match chassis inventory')
if errors:
    raise SystemExit('\n'.join(errors))
print(f'PASS: {len(rows)} distinct signal/ball reservations; 36 P7 GPIOs; {len(expected)} chassis CST ports.')
print('Scope: pin consistency only. Voltage compatibility, synthesis, timing, and vehicle tests remain pending.')
