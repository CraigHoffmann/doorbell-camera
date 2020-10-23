# ESP32-cam Multiclient Doorbell Camera
by Craig Hoffmann

I made this doorbell camera because I wasn't happy with what I could find already (and I just like making things).  There are a number of key features that make this different from most other (at time of writing) implementations available.  It is based on code from XXXXXXX however at this point the code has been largely restructured and has deviated  significantly from the original - however without the original, this version would probably have never eventuated, so thank you.  

Features
* Designed specifically for use with home assistant
* Upto 4 simultanious clients - this allows for example a recording stream with additional clients watching
* Maintain approx 12fps for upto 3 simultaniuous streams dropping to about 10fps when the 4th connects.  Note this is *highly dependent* on a good wifi connection.
* Request different (lower) frame rates via the stream url parameters
* Identify a *priority stream* via the stream url parameters
* Capture stream with upto 2sec of history selected via the stream url parameters.  For example when capturing a recording start recording from 2sec back in time.


