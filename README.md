# ESP32-cam Multiclient Doorbell Camera
by Craig Hoffmann

I made this doorbell camera because I wasn't happy with what I could find already, and I just like making things!  There are a number of key features that make this different (at time of writing) from most other implementations available.  It is based on code from [Anatoli Arkhipenko](https://www.hackster.io/anatoli-arkhipenko/multi-client-mjpeg-streaming-from-esp32-47768f) although at this point the code has been largely restructured and has deviated  significantly from the original - however without the original, this version would probably have never eventuated, so thank you.  

Camera    |  Wide Lens
:-------------------------:|:-------------------------:
![doorbell](https://github.com/CraigHoffmann/doorbell-camera/blob/main/Images/doorbell.jpg?raw=true) |  ![doorbell](https://github.com/CraigHoffmann/doorbell-camera/blob/main/Images/ov2640wide.jpg?raw=true)

## Features ##
* Designed specifically for use with home assistant
* No cloud subscriptions required
* Upto 4 simultanious clients (mjpeg streams) - this allows for example a recording stream with additional clients watching.
* Maintain approx 12fps for upto 3 simultaniuous streams XGA:1024x768, dropping to about 10fps when the 4th connects.  Note this is *highly dependent* on a good wifi connection and will drop if the wifi is poor quality, conjested or interrupted.
* Request different (lower) frame rates via the stream url parameters.  Maybe use for a time lapse?
* Identify a *priority stream* via the stream url parameters.  For example prioritise the recording stream.
* Capture stream with upto 2sec of history selected via the stream url parameters.  For example when capturing a recording start recording from 2sec back in time.
* MQTT json for configuration of key camera settings.  Ideal for changing to low light settings at sunset via home assistant.
* MQTT inputs (3 general purpose) for doorbell button, IR sensor, etc.

Video quality is average but acceptable, jpeg compression had to be set such that frame rates could be achieved achieved and maintained while not running out of memory.  A balance was made between the image size, fps, and number of streams that could be maintained... the result is pretty much everything is on the limit of the ESP32-cam module.  I originally had a Chinese 1080p HD video dooorbell and while the image was much sharper, it suffered constant disconnects, stuttering and missed rings due the the cloud connection required and in the end it just wasn't what I wanted.  While the video quality of this design is not as good, that is far less important to me than having the camera reliable with almost instant connection response via home assistant (and no cloud subscription required). 

Sample video...

## Build Notes ##

The camera is based on an ESP32-cam board, however the camera/lense has been replaced by a wide angle version from banggood.  I bough 3 wide angle lenses for about $15.  The original camera lense just unplugs and the new one plugged in.  The unit uses "wired 5V" power and an external antenna - this makes a **big difference** to the consistency of the video streams.

I designed a 3D printed case that uses a piece of clear plastic from an old CD case to provide a weather proof window for the camera.

## MQTT ##

MQTT is used to provide notification of when any of the three inputs are triggered, one typically used for the doorbell button.  It is also used to send camera configuration to the doorbell with a json payload.  I use this to change the settings for low light at dusk/night and then back to normal in the morning.  If only one or two parameters need to be changed then just include those required in the json payload.

```JSON
{
  "brightness":0,
  "contrast":0,
  "saturation":0,
  "effect":0,
  "whitebal":1,
  "awb_gain":1,
  "wb_mode":0,
  "exposure_ctrl":1,
  "aec2":0,
  "ae_level":0,
  "aec_value":300,
  "gain_ctrl":0,
  "agc_gain":3,
  "gainceiling":1
}
```

## Home Assistant setup notes ##

### Camera Setup ###

Add the following to configuration.yaml
```YAML
camera:
  - platform: mjpeg
    name: esp32 doorbell camera
    still_image_url: http://10.0.0.43/jpg        # replace ip with your camera ip
    mjpeg_url: http://10.0.0.43/mjpeg/1          # replace ip with your camera ip
    verify_ssl: false  
```

There are a few url parameters you can add to the mjpeg stream as follows:

```
http://10.0.0.43/mjpeg/1?back=10      # This will display the stream but 10 frames 'back in time' upto 24 frames (approx 2sec) back
http://10.0.0.43/mjpeg/1?fps=3        # Request a specific FPS between 1 and 12
http://10.0.0.43/mjpeg/1?priority=1   # Prioritise this stream - ie for recording
```

### Recording Video and Media Browser ###

Prerequisites
1. make sure you have ffmpeg installed in home assistant
2. whitelist the directory to be used for saving recordings (I use www/cam_captures in the home assistant config directory) 

```
# Allow shell command access to directory for saving video
homeassistant:
  whitelist_external_dirs:
    - /config/www/cam_captures           # This is so we can record them
  media_dirs:
    cameras: /config/www/cam_captures    # This is so media browser can acces them
```

Because this camera uses mjep streams we use **ffmpeg** to make the recordings.  If just want a raw capture of mjpeg stream use:

In configuration.yaml you can call ffmpeg directly using a shell_command (change IP and path to yours)
```YAML
shell_command:
  record_cam1: 'ffmpeg -r 12 -i http://10.0.0.43/mjpeg/1 -c copy -y -frames:v 180 /config/www/cam_captures/recording.avi'
```

and then call record_cam1 in your automation.yaml as required.

**Note** I have noticed while this is fine for playback via most dedicated media players, it doesn't play back in the new home assistant media browser, I suspect something is missing from the stream information?

The solution to enable playback using home assistant media browser is to use ffmpeg to recode the stream in a format that will work.  Now you can do this in one go with ffmeg however I have noticed that on my home assistant rpi3b+ it struggles to recode the stream without dropping frames.  This was easily fixed by just doing it in two steps 1. capture the mjpeg stream as is, and 2. when capture complete recode the stream.  I use a bash shell script to do this as it was easier to edit the bash script and retest without reloading the configuration.yaml.

Create a file called record_mjpeg.sh in the config/shell_scripts directory (create dir if not there)
```BASH
#!/bin/bash

folder=/config/www/cam_captures
tmp_file=tmp_$(date +"%y%m%d%H%M%S")
mpg_file=door_$(date +"%y-%m-%d_%H-%M-%S")
http_url=http://10.0.0.43/mjpeg/1?back=24

ffmpeg -i $http_url -c:v copy -y -frames:v 180 $folder/$tmp_file.avi
ffmpeg -r 12 -i $folder/$tmp_file.avi -c:v libx264 -preset veryfast -y -vf "fps=12,format=yuv420p" $folder/$mpg_file.mp4
rm $folder/$tmp_file.avi
```
and in configuration.yaml use
```YAML
shell_command:
  record_cmd: '/config/shell_scripts/record_mjpeg.sh'
```



