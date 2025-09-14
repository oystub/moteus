from dataclasses import dataclass
from enum import Enum
from typing import List
import struct
import pandas as pd


class FieldType(Enum):
    FLOAT32 = "f"
    INT32 = "i"
    UINT32 = "I"
    INT16 = "h"
    UINT16 = "H"
    INT8 = "b"
    UINT8 = "B"


@dataclass
class LoggedField:
    name: str
    type: FieldType


def row_size(fields: List[LoggedField]) -> int:
    return sum(struct.calcsize("<" + f.type.value) for f in fields)


def decode_speedlog(hex_str: str, fields: List[LoggedField]) -> pd.DataFrame:
    """Decode a hex string from the moteus speed logger into a pandas DataFrame."""
    data = bytes.fromhex(hex_str)
    row_len = row_size(fields)
    if row_len == 0:
        return pd.DataFrame()

    # Discard the first (oldest) row if it is incomplete.
    excess = len(data) % row_len
    if excess:
        data = data[excess:]

    n_rows = len(data) // row_len
    rows = []

    offset = 0
    for _ in range(n_rows):
        row = {}
        for f in fields:
            fmt = "<" + f.type.value  # little-endian
            size = struct.calcsize(fmt)
            chunk = data[offset:offset+size]
            value, = struct.unpack(fmt, chunk)
            row[f.name] = value
            offset += size
        rows.append(row)

    return pd.DataFrame(rows)
