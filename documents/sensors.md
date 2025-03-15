# Sensors

UKMARSBOT is a multi-purpose robot and is designed so that you can plug in a variety of sensors. For the mazerunner variant, all the code and documentation assumes that the basic wall sensor board is fitted. This has three reflective sensors that use visible or infra-red light emitters and matching phototransistors. One reflective sensor points directly forwards and can measure reflections from a wall ahead. Two more sensor point out and forwards at about 45 degrees to sense walls on the left and right of the robot. The sensors are normally aligned so that they point at the posts when the robot is positioned centrally in a maze cell.

## Wall Sensors

The wall sensors are used to measure the distance to a wall. They are not able to measure the actual distance though. Instead, they measure the amount of light reflected from the wall when the emitter is illuminating the wall. The amount of reflected light is a function of both the distance to the wall and the angle between the wall and the sensor. This relationship is quite complex. Fortunately, for most practical purposes, the software only needs to know that close walls appear brighter and that the response is always bigger as the wall gets closer.

There are four issues to be aware of that spoil this simple interpretation.

 - First, when the sensor is very close to a wall, the circuit board shields the detector and prevents it from seeing all the reflected light. Thus the apparent brightness will increase sharply when very close to a wall. This is not normally a problem but you should take care when approaching a wall ahead that you do not get too close.
 - Second, walls can be shiny and reflect light much more strongly when the illumination is close to perpendicular to a wall. This mostly affects the front sensor which nearly always points directly at right angles to the wall ahead. The consequence of this is that the value seen by the front sensor can fall off much more than you might expect for quite small alignment errors.
 - Third, a corner in the maze can confuse the side sensors. The corners form a box reflector and reflect the light strongly and may cause the robot to see readings that indicate that it is closer to a wall than it actually is.
 - Finally, and related to the previous issue, because the side sensors are not pointing straight out to the sides, they will get a return from a wall ahead if they are close enough. Not only can that look like there is a wall where none exists, it can cause a serious error in the steering. Returns from a wall ahead will reverse the sense of the steering control and can cause positive feedback. This reveals itself as a robot that approaches a wall ahead and then suddenly veers off to one side - possibly losing control completely. The front sensor can be used to provide a warning that this is about to happen.

## Pulsed operation

The environment in which the robot runs is not always friendly. That is, there may be variable amounts of ambient illumination and, worse, it may be strong _and_ directional. Think sun coming in from a window. If the sensors just measured the light coming from a wall illuminated by the emitter, that reading would change along with the ambient illumination. For this reason, the sensors are always run in pulsed, differential mode. First a reading is taken with the emitter off - that is the 'dark' value and serves to indicate the level of ambient illumination. Then the emitter is turned on and another reading is taken - the 'lit' value. The difference between these two reading (lit-dark) is the reflection causes only by the emitter and is a much more reliable measure of the wall distance. Note that the scheme can still fail if there is so much background light that the detector is nearly saturated even with the emitter off.

## Wall presence.

As well as giving a measure of the distance to a wall, the sensors must indicate whether or not a wall is present. For the side sensors this is reasonably simple. A typical method is to take note of the sensor reading when the robot is correctly positioned and can clearly illuminate a wall on either side. From that, the detection threshold can just be set to 50% of that nominal value. That corresponds to the emitter illumination spot falling half on and half off a wall. If you want to be a little more sophisticated, you can add some hysteresis and/or sample the wall several times to be sure. Note that, for more advanced operations, it is also important to know the _position_ that the robot acquired or lost a wall.

The front wall can be more tricky. The size of the response from a wall ahead can be quite small until you get rather close. And, of course, small angular errors may change that response quite a lot. The trick is to not try and identify a wall ahead until you are definitely close enough to be sure of doing so reliably. For UKMARSBOT and the suggested sensor components, that may mean you should be no further way than about 200mm.

## Calibration

So that the software can be as general purpose as possible, the sensor readings should be _normalised_. All sensors vary so a reading is taken in a redefined calibration location and that reading is used to automatically adjust the sensor readings so that, for example, the side sensors always give a normalised reading of 100 when the robot is correctly positioned with walls either side. If only normalised readings are used, then you can easily calibrate for different mazes and still have some confidence that the robot will run reliably.

## Other analogue inputs

As well as the thee wall sensors, there are two more analogue inputs used in UKMARSBOT. One of these is connected to the battery supply through a pair of resistors that create a voltage divider. This channel is used to monitor the battery voltage.

Another channel is connected to a network of resistors and switches so that a single channel can be used to identify 16 different settings of four DIP-switches at the rear of the robot as well as detecting presses of the single pushbutton found adjacent to the DIP switches.

Special functions in the sensors module return a values that represents the state of these switches.
