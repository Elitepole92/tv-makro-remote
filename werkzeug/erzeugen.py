import sys
tpl, html, out = sys.argv[1:4]
h = open(html, encoding='utf-8').read().rstrip('\n')
lines = h.split('\n')
lit = []
for i, l in enumerate(lines):
    e = l.replace('\\', '\\\\').replace('"', '\\"').replace('??', '?\\?')
    lit.append('    "' + e + ('\\n' if i < len(lines) - 1 else '') + '"')
block = 'static const char INDEX_HTML[] PROGMEM =\n' + '\n'.join(lit) + ';\n'
t = open(tpl, encoding='utf-8').read()
assert t.count('@@HTML@@') == 1
open(out, 'w', encoding='utf-8').write(t.replace('@@HTML@@\n', block + '\n'))
