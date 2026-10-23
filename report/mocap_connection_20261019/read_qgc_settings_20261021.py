from pathlib import Path
import os
for root in (Path(os.environ['APPDATA']),Path(os.environ['LOCALAPPDATA'])):
    for p in root.glob('QGroundControl*/*.ini'):
        print('FILE',p)
        section=''
        for line in p.read_text(encoding='utf-8',errors='replace').splitlines():
            if line.startswith('['): section=line
            if any(x in section.lower() for x in ('link','mavlink','autoconnect')):
                print(line)
