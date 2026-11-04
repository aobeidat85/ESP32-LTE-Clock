# SPDX-License-Identifier: GPL-3.0-only
from datetime import datetime,timedelta,timezone
import re

def utc_from_table(gps_calendar,table_text):
    gps=datetime.fromisoformat(gps_calendar)
    expires=datetime.strptime(re.search(r'expires on (\d+ \w+ \d+)',table_text).group(1),'%d %B %Y')
    entries=[]
    for line in table_text.splitlines():
        fields=line.split()
        if len(fields)==5 and not line.lstrip().startswith('#'):
            _,day,month,year,offset=fields
            entries.append((datetime(int(year),int(month),int(day)),int(offset)))
    for effective,tai_utc in reversed(entries):
        candidate=gps-timedelta(seconds=tai_utc-19)
        if candidate>=effective:
            if candidate>=expires:raise ValueError('Leap-second table expired for decoded date')
            return candidate.replace(tzinfo=timezone.utc),tai_utc-19,expires.date().isoformat()
    raise ValueError('Decoded date precedes leap-second table')
