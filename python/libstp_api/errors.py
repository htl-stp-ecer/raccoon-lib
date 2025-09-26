class ConfigError(ValueError):
    """Raised when the YAML/Dict configuration is invalid."""


class UnsupportedDriveError(ValueError):
    """Raised when a drive type is not supported by the current build."""