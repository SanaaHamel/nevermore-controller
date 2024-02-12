#!/bin/bash
"true" '''\'
set -eu
set -o pipefail

FILE="$(readlink -f "$0")"
ROOT_DIR="$(dirname "$FILE")"

"$ROOT_DIR/setup-tool-env.bash"
"$ROOT_DIR/.venv/bin/python" "$FILE" "$@"

exit 0 # required to stop shell execution here
'''

# Script for regenerating IDs in saved SquareLine Studio.
# Useful for manually merging in new UIs.
#
# Copyright (C) 2024       Sanaa Hamel
#
# This file may be distributed under the terms of the GNU AGPLv3 license.

__doc__ = """
"""

import asyncio
import datetime
import enum
import json
import logging
import math
import os
import pickle
import random
import re
import shlex
import socket
import struct
import sys
import tempfile
import uuid
from collections import defaultdict
from dataclasses import dataclass
from typing import Any, Callable, Coroutine, Dict, List, Optional, Set, Tuple, Union

import typed_argparse as tap
from nevermore_utilities import *
from typing_extensions import override

_A = TypeVar("_A")


class CmdLnArgs(tap.TypedArgs):
    file: str = tap.arg(
        positional=True,
        help="seconds between samples",
    )
    in_place: bool = tap.arg(
        help="apply in place",
        default=False,
    )


def mk_remap(gen: Callable[[], _A]) -> Callable[[_A], _A]:
    known: Set[_A] = set()
    memo: Dict[_A, _A] = {}

    def apply(x: _A) -> _A:
        if x in memo:
            return memo[x]

        while True:
            y = gen()
            if y in known:
                continue

            memo[x] = y
            known.add(y)
            return y

    return apply


def main(args: CmdLnArgs):
    # no idea if `deepid`/`nid` are shared or not. assume they are for now
    nid = mk_remap(lambda: random.randint(-0x8000_0000, 0x7FFF_FFFF))
    key_mappers = {
        "guid": mk_remap(lambda: str(uuid.uuid4())),
        "deepid": nid,
        "nid": nid,
    }

    def apply(x: Any):
        if isinstance(x, list):
            return [apply(y) for y in x]

        if isinstance(x, dict):
            return {
                k: key_mappers[k](v) if k in key_mappers else apply(v)
                for k, v in x.items()
            }

        return x

    with open(args.file, 'r') as f:
        data = json.load(f)
        data_remapped = apply(data)
        data_rendered = json.dumps(data_remapped, indent=2)

    if args.in_place:
        with open(args.file, 'w') as f:
            f.write(data_rendered)
    else:
        print(data_rendered)


if __name__ == "__main__":
    tap.Parser(CmdLnArgs).bind(main).run()
