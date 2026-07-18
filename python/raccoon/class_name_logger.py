from __future__ import annotations

from raccoon.log import debug, error, info, trace, warn


class ClassNameLogger:
    """Logging mixin that tags each record with the instance's runtime class.

    The class name is carried as the ``_cls`` hook rather than prefixed onto the
    message, so it lands in the JSONL ``func`` field as ``"ClassName.method"``
    instead of cluttering ``msg``.
    """

    def debug(self, msg: str) -> None:
        """
        Print a debug message.

        Args:
            msg: The message to print
        """
        debug(msg, _stacklevel=2, _cls=self.__class__.__name__)

    def info(self, msg: str) -> None:
        """
        Print an info message.

        Args:
            msg: The message to print
        """
        info(msg, _stacklevel=2, _cls=self.__class__.__name__)

    def warn(self, msg: str) -> None:
        """
        Print a warn message.

        Args:
            msg: The message to print
        """
        warn(msg, _stacklevel=2, _cls=self.__class__.__name__)

    def error(self, msg: str) -> None:
        """
        Print an error message.

        Args:
            msg: The message to print
        """
        error(msg, _stacklevel=2, _cls=self.__class__.__name__)

    def trace(self, msg: str) -> None:
        """
        Print a trace message.

        Args:
            msg: The message to print
        """
        trace(msg, _stacklevel=2, _cls=self.__class__.__name__)
