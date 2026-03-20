"""Qt binding compatibility layer.

The app prefers PyQt6 and falls back to PySide6.
"""

try:
    from PyQt6 import QtCore, QtGui, QtWidgets

    Signal = QtCore.pyqtSignal
    Slot = QtCore.pyqtSlot
    QT_API = "PyQt6"
except ImportError:
    try:
        from PySide6 import QtCore, QtGui, QtWidgets

        Signal = QtCore.Signal
        Slot = QtCore.Slot
        QT_API = "PySide6"
    except ImportError as exc:
        raise ImportError(
            "Install PyQt6 or PySide6 before running this GUI."
        ) from exc
