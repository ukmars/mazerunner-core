# Reporting

The UKMARSBOT controller is an Arduino nan. That means it has a built-in serial to USB bridge that makes communication with a host computer quite simple. Having the robot tethered with a bulky USB cable is less convenient. Even so there are some calibration tests - the sensor calibration in particular - that really benefit from the ability to display results on a host computer.
The UKMARSBOT controller is an Arduino nan. That means it has a built-in serial to USB bridge that makes communication with a host computer quite simple. Having the robot tethered with a bulky USB cable is less convenient. Even so there are some calibration tests - the sensor calibration in particular - that really benefit from the ability to display results on a host computer.

For moving tests, it is convenient to attach a Bluetooth Classic adaptor like the HC-05 and view telemetry remotely.
For moving tests, it is convenient to attach a Bluetooth Classic adaptor like the HC-05 and view telemetry remotely.

Several reporting functions are provided in the code as examples of the kinds of information that can be sent. Sometimes, a basic one-line response is all that is needed. At other times a stream of data from sensors or motion control variables is desirable. Data that is streamed out can be recorded at the host and displayed in tools like Excel to give a better understanding of the robot behaviour.
Several reporting functions are provided in the code as examples of the kinds of information that can be sent. Sometimes, a basic one-line response is all that is needed. At other times a stream of data from sensors or motion control variables is desirable. Data that is streamed out can be recorded at the host and displayed in tools like Excel to give a better understanding of the robot behaviour.

Look at the sample reports in reports.cpp to get some ideas of what might be possible.

If you would rather not have the robot stream data out during tests, there is a flag in config.h that lets you disable the reporting without having to edit the test code.

## Custom reports

When writing your own reporting functions you might like to bear in mind a number of points.

### Serial buffering

The Arduino HardwareSerial object has an output buffer of 64 characters. All serial output comes from this buffer and when you use statements like `Serial.print(42);` the characters are actually placed in the buffer and the buffer is transferred one character at a time under interrupt control. All this means that you can send up to 64 characters in one go with very little impact on the timing of your code but only so long as the buffer is empty when you begin to send.

### Serial baud rates

The default baud rate for the serial port in this code is 115200. That equates to a maximum of roughly 10,000 characters per second. At that speed, it would take the Arduino nearly 6.5ms to send out the entire 64 character buffer. It should be clear that trying to send longer lines of telemetry any faster than about 100 lines per second is likely to fill up the transmit buffer and characters will be lost. For many requirements, shorter lines, sent less frequently are going to be adequate.
The default baud rate for the serial port in this code is 115200. That equates to a maximum of roughly 10,000 characters per second. At that speed, it would take the Arduino nearly 6.5ms to send out the entire 64 character buffer. It should be clear that trying to send longer lines of telemetry any faster than about 100 lines per second is likely to fill up the transmit buffer and characters will be lost. For many requirements, shorter lines, sent less frequently are going to be adequate.

If, on the other hand, you are stuck with a BT module running at a default 9600 baud, how much telemetry can you manage? At 9600 baud, characters are sent at about 1000 characters per second. You might be able to send a 64 character buffer in 64ms or 15 lines per second. Not too shabby but that is risking data loss. For lower data rates, it pays to be mindful of how often you send telemetry data and how much you send.
If, on the other hand, you are stuck with a BT module running at a default 9600 baud, how much telemetry can you manage? At 9600 baud, characters are sent at about 1000 characters per second. You might be able to send a 64 character buffer in 64ms or 15 lines per second. Not too shabby but that is risking data loss. For lower data rates, it pays to be mindful of how often you send telemetry data and how much you send.

### Regular reporting

For some purposes, it is fine just to send out new data when you have it. If all you want to know is the battery voltage, for example, there is probably no great urgency. Quite often though, you want to be sending data out at very regular intervals. For example, you might want to know the battery voltage at specific instants so that you can gauge the effect of high speed and acceleration on the battery drain. Or you might want to observe the sensor response as you pass a maze post at speed.
For some purposes, it is fine just to send out new data when you have it. If all you want to know is the battery voltage, for example, there is probably no great urgency. Quite often though, you want to be sending data out at very regular intervals. For example, you might want to know the battery voltage at specific instants so that you can gauge the effect of high speed and acceleration on the battery drain. Or you might want to observe the sensor response as you pass a maze post at speed.

In these cases, getting timely reports at fixed intervals is important. It is common to see code like this in Arduino programs:

```
void loop() {
  Serial.println(data);
  delay(10);
}
```

Presumably, the intention is to send out data every 10ms but what happens as you add more code to the loop? Now your delay will get longer. Or what if you are sending data out while waiting for some other task to complete? You cannot then just do nothing for 10ms - the other code must continue to run.

A better technique is illustrated in many of the reporting functions in the code. You will see that reports have a fixed interval and that the reporting code is called as often as possible. Instead of just writing out data, the function checks to see if the interval has passed and, if it has, a new report time is calculated and then the data is sent. This method is much more reliable than a simple delay.
A better technique is illustrated in many of the reporting functions in the code. You will see that reports have a fixed interval and that the reporting code is called as often as possible. Instead of just writing out data, the function checks to see if the interval has passed and, if it has, a new report time is calculated and then the data is sent. This method is much more reliable than a simple delay.

### Reporting by distance

Although this method is not used in the current code, you might like to consider a variation on reporting that is distance based rather than time based. Normally data is sent out at fixed time intervals. That makes interpretation in charts reasonably simple to understand. Sometimes though the key feature is _distance_. For example, if you are looking at the response of the sensors as the robot passes through a cell, it might be better to have the reporting function look not at the elapsed _time_ since the last report but at the elapsed _distance_ since the last report. Move slowly and sent out new data every few mm of travel.
Although this method is not used in the current code, you might like to consider a variation on reporting that is distance based rather than time based. Normally data is sent out at fixed time intervals. That makes interpretation in charts reasonably simple to understand. Sometimes though the key feature is _distance_. For example, if you are looking at the response of the sensors as the robot passes through a cell, it might be better to have the reporting function look not at the elapsed _time_ since the last report but at the elapsed _distance_ since the last report. Move slowly and sent out new data every few mm of travel.

Alternatively, make sure that your time-based report has a column with the current robot position and plot the data in Excel using the position as the x-axis instead of time.
Alternatively, make sure that your time-based report has a column with the current robot position and plot the data in Excel using the position as the x-axis instead of time.
