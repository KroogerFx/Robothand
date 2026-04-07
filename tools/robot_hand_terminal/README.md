# Robot Hand Serial Terminal

Small standalone Windows-native serial terminal for talking to the Robot Hand over USB serial.
It opens as a desktop window by default.

## Features

- scans available serial ports
- lets you choose which port to open
- splits incoming serial data and outgoing/system messages into separate panes
- sends input only after you press Enter
- shows serial data coming back from the board
- includes quick buttons for fixed firmware commands
- includes command builders for jog, position, finger, and single-motor stop commands
- supports line ending modes similar to a serial monitor

## Requirements

- Windows PowerShell, included with Windows

## Setup

No extra runtime or package install is required for the PowerShell terminal.

## Run

Start the terminal:

```powershell
cd tools/robot_hand_terminal
.\run_terminal.cmd
```

Start the text-only fallback:

```powershell
cd tools/robot_hand_terminal
.\run_terminal.cmd -Cli
```

Open a specific port directly:

```powershell
cd tools/robot_hand_terminal
.\run_terminal.cmd COM3
```

Start with explicit settings:

```powershell
cd tools/robot_hand_terminal
.\run_terminal.cmd -BaudRate 115200 -LineEnding lf
```

## Usage

The terminal starts by scanning serial ports and letting you pick one if you do not pass a port name.

After connecting:
- type a robot command such as `h`, `e`, `j2f`, or `sa`
- press Enter to send it
- or use the quick command buttons on the right
- or use the command builder tab to generate parameterized commands without typing the raw syntax

GUI notes:

- `Incoming Data (RX)` shows data returned by the ESP32
- `Outgoing Data / System (TX)` shows sent commands and terminal status messages
- `Quick Commands` includes one-click buttons for `h`, `cd`, `cp`, `e`, `ra`, `mz`, `sa`, `fd`, `fp`, `fr`, and `fm`
- the `Stop All` quick button is styled as a red emergency stop control
- `Command Builders` can assemble `j...`, `p...`, `f...`, and single `s...` commands
- multi-command forms such as `p...;...` and `f...;f...` can still be entered directly in the raw input box

Local terminal commands:

- `:help`
- `:ports`
- `:open`
- `:open COM3`
- `:close`
- `:baud 115200`
- `:ending none`
- `:ending lf`
- `:ending cr`
- `:ending crlf`
- `:quit`

## Recommended Settings

For your current firmware, start with:

- baud rate: `115200`
- line ending: `lf`

## Notes

- `run_terminal.cmd` launches `terminal.ps1` with `ExecutionPolicy Bypass` and `-STA`, so the GUI works even if direct `.ps1` execution is disabled in your shell.
