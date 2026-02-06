from libstp.log import debug, info, error, warn

class ClassNameLogger:
    def debug(self, msg: str) -> None:
        """
        Print a debug message.

        Args:
            msg: The message to print
        """
        debug(f"[{self.__class__.__name__}]: {msg}", _stacklevel=2)

    def info(self, msg: str) -> None:
        """
        Print an info message.

        Args:
            msg: The message to print
        """
        info(f"[{self.__class__.__name__}]: {msg}", _stacklevel=2)

    def warn(self, msg: str) -> None:
        """
        Print a warn message.

        Args:
            msg: The message to print
        """
        warn(f"[{self.__class__.__name__}]: {msg}", _stacklevel=2)


    def error(self, msg: str) -> None:
        """
        Print an error message.

        Args:
            msg: The message to print
        """
        error(f"[{self.__class__.__name__}]: {msg}", _stacklevel=2)
