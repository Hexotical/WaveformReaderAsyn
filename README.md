# Abstract
This is an ASYN port driver that takes a stream of data and pushes it to an EPICS waveform record

## Usage
### WaveformStreamInit
Usage: **WaveformStreamInit** "<Path to stream>" "<Waveform record asyn Identifier>"

E.g. **WaveformStreamInit** "/Stream0" "WAVEFORM:0"

This command initializes a thread that connects to your specified stream, reads from the stream and proceeds to write the data it receives to the waveform record you specify.

### WaveformStatus
Usage: **WaveformStatus**

Currently doesn't really do anything will eventually tell you what streams have been initialized, what they're connected to and will provide a health check

