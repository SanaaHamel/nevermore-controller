import random
import gdb

__all__ = []

_DOUBLE_TYPE = gdb.lookup_type("double")


def _pprint_fix16(raw):
    return raw.cast(_DOUBLE_TYPE) / (1 << 16)


class Fix16StructPrinter:
    """Print a fix16 struct numeric"""

    def __init__(self, val):
        self._val = val

    def to_string(self):
        return _pprint_fix16(self._val['raw'])

    def display_hint(self):
        return 'string'


class Fix16TypeDefPrinter:
    """Print a fix16 typedef numeric"""

    def __init__(self, val):
        self._val = val

    def to_string(self):
        return _pprint_fix16(self._val)

    def display_hint(self):
        return 'string'


def build_pretty_printer():
    # HACK: random name allows reloading in the same gdb session
    # (https://sourceware.org/pipermail/gdb/2019-May/047905.html)
    pp = gdb.printing.RegexpCollectionPrettyPrinter(
        "fix16.%d" % (random.randint(0, 1000000),)
    )
    pp.add_printer(
        "fix16::fix16_t",
        "^fix16::fix16_t$",
        Fix16StructPrinter,
    )
    pp.add_printer(
        "fix16_t",
        "^fix16_t$",
        Fix16TypeDefPrinter,
    )
    return pp


# executed at script load
gdb.printing.register_pretty_printer(gdb.current_objfile(), build_pretty_printer())
