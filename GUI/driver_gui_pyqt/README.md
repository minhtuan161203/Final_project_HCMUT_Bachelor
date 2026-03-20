# ASD04 Python GUI

This folder contains a lightweight desktop GUI for the ASD04 servo driver.

## Features

- serial port connect / disconnect
- automatic monitor polling
- servo on / servo off / fault acknowledge
- read driver parameters
- read motor parameters
- write driver parameters
- write motor parameters
- write current parameters to flash
- raw command sender
- communication log

## Protocol source

The implementation follows:

- legacy PC app sources:
  - `USBApplication.h/.cpp`
  - `USBCommunication.h/.cpp`
- firmware sources:
  - `Library/USBComunication.c/.h`
  - `Src/main.c`

## Install

```powershell
python -m pip install -r requirements.txt
```

If you prefer, you can install `PySide6` instead of `PyQt6`.
The code supports both bindings.

## Run

```powershell
python main.py
```

## Notes

- The GUI implements only the basic communication and commissioning workflow.
- Plotting for trace / FFT / auto tuning packets is not implemented yet.
- The monitor packet sent by the current firmware scales `Vdc` by `0.1`, so the GUI multiplies it back by `10`.
- Motor parameter writes are chunked in groups of 20 entries to match the current firmware parser.
