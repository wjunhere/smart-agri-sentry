import pytest
from sentry_bringup.gps_node import parse_nmea_line


def test_parse_gga_valid():
    line = "$GNGGA,123519,4807.038,N,01131.000,E,1,8,0.9,545.4,M,46.9,M,,*47"
    result = parse_nmea_line(line)
    assert result is not None
    assert result['lat'] > 48.0
    assert result['lon'] > 11.0
    assert result['fix_quality'] == 1


def test_parse_rmc_valid():
    line = "$GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"
    result = parse_nmea_line(line)
    assert result is not None
    assert result['speed_knots'] == 22.4
    assert result['track_angle'] == 84.4


def test_parse_invalid_checksum():
    line = "$GNGGA,123519,4807.038,N,01131.000,E,1,8,0.9,545.4,M,46.9,M,,*00"
    result = parse_nmea_line(line)
    assert result is None


def test_parse_unsupported_sentence():
    line = "$GNGSV,1,1,04,01,40,083,46*4D"
    result = parse_nmea_line(line)
    assert result is None
