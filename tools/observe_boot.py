"""Read-only serial observer. Optional RTS reset; never supplies UTC or serial data."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import serial

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--port',default='COM10')
p.add_argument('--seconds',type=int,default=900)
p.add_argument('--log',type=Path,required=True)
p.add_argument('--reset',action='store_true')
a=p.parse_args()
a.log.parent.mkdir(parents=True,exist_ok=True)
s=serial.Serial(port=None,baudrate=115200,timeout=.25)
s.dtr=False;s.rts=False;s.port=a.port;s.open()
try:
    if a.reset:
        s.rts=True;time.sleep(.15);s.rts=False
    started=time.monotonic();pending=b'';synced=False;dates=0
    with a.log.open('w',encoding='utf-8') as output:
        while time.monotonic()-started<a.seconds:
            pending+=s.read(s.in_waiting or 1)
            while b'\n' in pending:
                line,pending=pending.split(b'\n',1)
                text=line.decode(errors='replace').strip()
                row={'received_at':datetime.now(timezone.utc).isoformat(),
                     'observer_elapsed_s':round(time.monotonic()-started,3),'line':text}
                output.write(json.dumps(row)+'\n');output.flush()
                print(text,flush=True)
                if 'UTC synchronized.' in text:synced=True
                if synced and text.startswith('202'):dates+=1
            if synced and dates>=3:break
finally:
    s.close()
print(json.dumps({'synced':synced,'serial_data_bytes_sent':0,'reset':a.reset}),flush=True)
raise SystemExit(0 if synced else 1)
