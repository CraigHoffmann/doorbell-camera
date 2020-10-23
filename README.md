# ESP32-cam Multiclient Doorbell Camera
by Craig Hoffmann

I made this doorbell camera because I wasn't happy with what I could find already (and I just like making things).  There are a number of key features that make this different from most other (at time of writing) implementations available.  It is based on code from XXXXXXX however at this point the code has been largely restructured and has deviated  significantly from the original - however without the original, this version would probably have never eventuated, so thank you.  

Features
* Designed specifically for use with home assistant
* No cloud subscriptions required
* Upto 4 simultanious clients (mjpeg streams) - this allows for example a recording stream with additional clients watching.
* Maintain approx 12fps for upto 3 simultaniuous streams dropping to about 10fps when the 4th connects.  Note this is *highly dependent* on a good wifi connection and will drop if the wifi is poor quality, conjested or interrupted.
* Request different (lower) frame rates via the stream url parameters.  Maybe use for a time lapse?
* Identify a *priority stream* via the stream url parameters.  For example prioritise the recording stream.
* Capture stream with upto 2sec of history selected via the stream url parameters.  For example when capturing a recording start recording from 2sec back in time.
* MQTT json for configuration of key camera settings.  Ideal for changing to low light settings at sunset via home assistant.
* MQTT inputs (3 general purpose) for doorbell button, IR sensor, etc.


