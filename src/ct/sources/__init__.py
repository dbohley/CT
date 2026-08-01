"""Signal sources. Importing this package registers every built-in source."""

from ct.sources.base import SyntheticSource
from ct.sources.csv_source import CSVSource, write_csv
from ct.sources.lujan import LujanSource
from ct.sources.rc_piecewise import RCPiecewiseSource
from ct.sources.sinusoid import SinusoidSource

__all__ = [
    "SyntheticSource",
    "SinusoidSource",
    "LujanSource",
    "RCPiecewiseSource",
    "CSVSource",
    "write_csv",
]
