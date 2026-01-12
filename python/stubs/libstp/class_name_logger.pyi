from __future__ import annotations
from libstp.foundation import debug
from libstp.foundation import error
from libstp.foundation import info
from libstp.foundation import warn
__all__: list[str] = ['ClassNameLogger', 'debug', 'error', 'info', 'warn']
class ClassNameLogger:
    def debug(self, msg: str) -> None:
        """
        
                Print a debug message.
        
                Args:
                    msg: The message to print
                
        """
    def error(self, msg: str) -> None:
        """
        
                Print an error message.
        
                Args:
                    msg: The message to print
                
        """
    def info(self, msg: str) -> None:
        """
        
                Print an info message.
        
                Args:
                    msg: The message to print
                
        """
    def warn(self, msg: str) -> None:
        """
        
                Print an info message.
        
                Args:
                    msg: The message to print
                
        """
