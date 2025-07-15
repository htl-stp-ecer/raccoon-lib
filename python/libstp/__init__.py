import sys
from abc import abstractmethod
from enum import Enum

# from libstp.logging import info, debug


class TableSide(Enum):
    DEFAULT = "DEFAULT"
    TABLE_1A = "1A"
    TABLE_1B = "1B"
    TABLE_2A = "2A"
    TABLE_2B = "2B"
    TABLE_3A = "3A"
    TABLE_3B = "3B"


def index_args():
    local_args = {}
    debug(f"Arguments passed to script: {' '.join(sys.argv[1:])}")
    for args in sys.argv:
        if args.startswith("--"):
            values = args.split("=")
            local_args[values[0][2:]] = values[1] if len(values) > 1 else True

    debug(f"Detected {len(local_args.keys())} / {len(sys.argv[1:])} arguments passed to script")
    return local_args


#arguments = index_args()


def get_table() -> TableSide:
    return TableSide(arguments.get("table", "DEFAULT"))

def get_bool_argument(name: str) -> bool:
    return arguments.get(name) is not None


def run_as_module(name: str, callback, *args, **kwargs):
    if arguments.get(f"no-{name}") is not None:
        info(f"Skipping {name} module")
        return

    info(f"Calling {name} module")
    callback(*args, **kwargs)


class Module:
    def __init__(self, name: str):
        run_as_module(name, self.run)

    @abstractmethod
    def run(self):
        raise NotImplementedError("Method run() not implemented")
