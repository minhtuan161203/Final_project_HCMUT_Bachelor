# ASD04 Python GUI

Desktop commissioning GUI for the ASD04 servo driver.

## What It Covers

- serial connect / disconnect
- servo on / servo off / fault acknowledge
- live monitor polling
- driver parameter read / write
- motor parameter read / write
- flash save commands
- open-loop V/F control
- current tuning trace capture
- firmware trace scope
- drive snapshot / debug terminal views

## Commissioning Workflow

The current GUI is organized around a small set of commissioning tasks instead of showing every value at once.

### Main Windows

- `Drive Monitor`: compact dashboard for the values you usually need first
- `Trend Charts`: continuous plots for currents, D/Q currents, D/Q voltages, and speed
- `Commissioning / Scope`: rotor alignment and Id tuning workflow
- `Drive Snapshot`: readable one-shot summary for monitor and debug values
- `Communication Log`: raw command / response history

### Encoder Alignment Commissioning

The commissioning window now exposes a dedicated `Run Encoder Alignment Only` action and a matching firmware state machine state `ENCODER_ALIGN`. This flow is separate from `Id Square Wave` so that electrical-zero capture is explicit and easy to validate before current-loop tuning.

Alignment panel summary:

- `Policy`: shows whether the active encoder is treated as incremental or absolute
- `Status`: `Idle`, `Requested`, `Running`, `Done`, or `Fault`
- `Active Offset`: the offset currently used by the firmware for theta calculation
- `Captured Offset`: the most recent offset captured by the alignment routine
- `Save Offset to Flash`: stores the latest motor/driver parameters, including the new encoder offset

Policy handling in firmware:

1. `Incremental encoder / power-on policy`
   - selected automatically for encoder types that lose position at power-up
   - servo start runs current-sensor offset calibration first, then enters `ENCODER_ALIGN`, captures the electrical-zero offset, and only then continues to `START -> RUN`
   - no flash save is required because the alignment is expected on every start

2. `Absolute encoder / manual-save policy`
   - selected automatically for absolute encoder IDs
   - normal servo start reuses the offset already stored in flash and goes directly through the normal start path
   - `Run Encoder Alignment Only` is used as a commissioning step when you install the motor, replace hardware, or intentionally re-zero the system
   - after a successful capture, use `Save Offset to Flash` so the stored value is reused on the next power cycle

Recommended alignment workflow:

1. stop the drive and mechanically lock the rotor
2. run `Run Encoder Alignment Only`
3. verify `Id`, `Iq`, and `Vq` in the commissioning scope
4. if the encoder is absolute, press `Save Offset to Flash`
5. continue with `Alignment Hold` or `Id Square Wave`

### Current Tuning Modes

Two commissioning modes are available after the encoder offset is known.

1. `Alignment Hold`
   - runs the encoder-alignment capture step when needed
   - then holds a constant d-axis current
   - use this to verify electrical zero and check that `Iq` / `Vq` stay small

2. `Id Square Wave`
   - runs the same alignment capture step first when needed
   - then applies a bipolar d-axis current command for locked-rotor current-loop tuning
   - use this after alignment looks correct

Recommended current-loop order:

1. stop the drive and mechanically lock the rotor
2. finish encoder alignment first
3. run `Alignment Hold`
4. confirm `Id` is behaving sensibly and `Iq` / `Vq` stay small
5. switch to `Id Square Wave`
6. tune `Kp` / `Ki`

### Safety Notes

- `Alignment Hold` and `Id Square Wave` both assume a stationary rotor
- start from about `5-10%` of rated current
- keep square-wave frequency low during early tuning
- use `Trend Charts` or `Drive Snapshot` for deeper debug instead of overloading the main monitor

## Protocol Source

The implementation follows these sources:

- legacy PC app sources:
  - `USBApplication.h/.cpp`
  - `USBCommunication.h/.cpp`
- firmware sources:
  - `Firmware_final_project/MDK-ARM/Library/USBComunication.c/.h`
  - `Firmware_final_project/Src/main.c`

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

- the main monitor intentionally shows only the most useful commissioning signals
- `Trend Charts` and `Drive Snapshot` keep the deeper debug data available without cluttering the main screen
- the monitor packet sent by the current firmware scales `Vdc` by `0.1`, so the GUI multiplies it back by `10`
- motor parameter writes are chunked in groups of 20 entries to match the current firmware parser
