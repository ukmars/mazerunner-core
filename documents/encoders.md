# Encoders
Attached to each motor is an encoder which generates a series of pulses as the motor shaft turns. Each encoder generates two sets of pulses that are in quadrature. That is, the pulses on one of the channels is delayed compared to the other channel - they do not change state at the same time:
```
            ___     ___     ___     __
channel A  |   |___|   |___|   |___|
              ___     ___     ___     _
channel B  __|   |___|   |___|   |___|

```

The electronics of UKMARSBOT modifies these signals slightly so that they are suitable for generating interrupts on two of the pins of the ATmega328p in the Arduino nano.

Each encoder generates interrupts using one channel and feeds the other channel to another pin. Thus there are two interrupt sources and two additional inputs used by the encoders in total.

Each interrupt keeps track of one encoder. By comparing the interrupting signal with the signal from the other channel, the service routine can tell if the motor has moved forward or backwards. The movement is always one step per interrupt.

Every change in state of the two encoder channels will generate an interrupt. That means there will be four changes per magnet pole per revolution of the motor shaft.

The stock encoders from Pololu have just three magnet poles and so they will give 12 counts per rotation of the motor shaft.

## Encoder resolution

Consider a motor with one of these encoders connected to a 22:1 gearbox. Now a single turn of the output shaft will give 264 counts. If the drive wheel has a diameter of 28mm then one turn of the wheel is approximately 88mm of linear travel (28 * PI, or about 28*22/7).

So, for 88mm of travel, the wheel will give 264 counts and the basic resolution of the encoder is going to be 264/88 = 3 counts per mm of travel. The robot has two drive wheels so, for forward motion, both encoders can be combined to give an overall resolution of 6 counts per mm.

In the `config.h` file, you will be able to enter values that describe the encoders and drive train in _your_ robot. Because real robots have some small variations in things like actual wheel diameter, you can use the calibration test code to fine tune those values for the best possible accuracy.

Once the encoder resolution is known, the software will have little difficulty in recording distances and speeds in real world units (mm and mm/s) rather than some abstract number of counts and counts per tick.

## Encoder pulse frequency

Each encoder count generates an interrupt. Suppose the robot in this example is travelling at 2000mm/s. With a combined resolution of 6 counts per mm, you can see that there will be about 6 * 2000 = 12000 interrupts per second from the encoders. That is quite a high rate and, even if the processor had nothing else to do, each interrupt would have a maximum of 80 microseconds available for its work.

Of course,the processor is always busy with several tasks including the systick event that must run 500 times per second.

In this code, the systick interrupt uses the option to enable other interrupts while it runs. That is not normally the case. Each encoder interrupt is carefully written to take as little time as possible - about 4 microseconds typically.

It should be clear that there is a lot going on behind the scenes in the processor. You will see quite a lot of examples in the code where special precautions are taken to make sure that critical operations are not held up and that variables modified by interrupt routines are not corrupted when used by other operations. Look for `volatile` declarations and `ATOMIC_BLOCK` statement blocks.
