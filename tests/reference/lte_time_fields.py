"""Interpret typed SIB8 time fields without borrowing any host date or clock.

TS 36.331 SystemTimeInfoCDMA2000; GPS epoch/time scale per C.S0002-E 1.3.
GPS calendar is explicitly separate from UTC and any local time zone.
"""
from datetime import datetime,timedelta
from fractions import Fraction


def sib8_time(block):
    info=block.get('systemTimeInfo')
    if info is None:return dict(system_time_present=False,gps_calendar=None,utc_time=None,time_zone=None)
    choice,(ticks,width)=info['cdma-SystemTime']
    if choice=='synchronousSystemTime':expected_width=39;unit=Fraction(1,100)
    elif choice=='asynchronousSystemTime':expected_width=49;unit=Fraction(1,153600)
    else:raise ValueError('Unknown CDMA system time representation')
    if width!=expected_width or not 0<=ticks<2**width:raise ValueError('Invalid time bit string')
    seconds=ticks*unit;whole=seconds.numerator//seconds.denominator
    micros=round((seconds-whole)*1000000)
    calendar=datetime(1980,1,6)+timedelta(seconds=whole,microseconds=micros)
    return dict(system_time_present=True,representation=choice,raw_ticks=ticks,bit_width=width,
        tick_seconds_numerator=unit.numerator,tick_seconds_denominator=unit.denominator,
        gps_seconds_numerator=seconds.numerator,gps_seconds_denominator=seconds.denominator,
        gps_calendar=calendar.isoformat(timespec='microseconds'),calendar_time_scale='GPS, not UTC',
        cdma_eutra_synchronised=info['cdma-EUTRA-Synchronisation'],
        reference='SFN boundary at or after the end of the containing SI window',
        utc_time=None,utc_offset=None,time_zone=None,host_clock_used=False)
