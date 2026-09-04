
Authoritative references:

- [WaveForms SDK Reference Manual](https://digilent.com/reference/_media/reference/software/waveforms/waveforms-sdk/waveformssdk_reference_manual.pdf)
- [Digilent WaveForms SDK getting-started guide](https://digilent.com/reference/test-and-measurement/guides/waveforms-sdk-getting-started)

## Local environment
- Python used for validation:
  `C:\Users\ngwei\AppData\Local\Programs\Python\Python312\python.exe`
- The bare `python` launcher may be unavailable to sandboxed sessions. Use the
  explicit interpreter path above when validating.
- The program searches for `dwf.dll` at:
  - `C:\Program Files\Digilent\WaveForms3\dwf.dll`
  - `C:\Program Files\Digilent\WaveFormsSDK\lib\dwf.dll`

WaveForms must be installed and closed before starting the program; a device
can be opened by only one application at a time.

## Hardware connections

Before making any hardware-facing change or measurement, the human must state
which physical pin is connected to which signal, device, or instrument channel.
