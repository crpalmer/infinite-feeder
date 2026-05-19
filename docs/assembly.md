# Assembly

## Prepare the switches

If you are using D2F-L switches for all the switches, move the levers to have two D2F-L and 5 D2F switches.  You can either solder wires directly to the switches or (recommended) solder JST-XH connectors to the switches.  I find the JST-XH connectors make a much more robust and durable connection.

'''Option 1: wires''' You'll need to size you wires now so that they are long enough to reach the board that will be mounted on the back of the main body.  First, crimp JST pins to each wire and then solder wires to the outer two pins of each switch.

'''Option 2: JST-XH connects''' solder a 5 pin JST-XH male header to the outer 2 pins of each switch.  You don't need to worry about preparing the wires at this point and you can more easily cut them to length by doing so when you have a full assembled feeder.  Note: for the absolute nicest build, try to orient the JST-XH connectors so that they are all facing the right direction when the switches are mounted.  You can check the orientation by matching the hole used to trigger the switch.  If they aren't all oriented correctly, it will be perfect fine (I screwed one up, it happens).

## Add the heatset inserts

Insert 8 heatset inserts into the feeder body.

* **4x** inserts to mount the top of the filament buffer to the main body
* **4x** inserts to mount the extruders
* **4x** inserts in the board mount to attach it to the main body.

![heatset locations](images/heatset-inserts.png)

## Switch Assembly (x3)

* Insert an ECAS-04 connector in each end
* Insert a bearing into the slot
* Attach the switch oriented over the bearing using two M2x8mm SHCS.

## Slide Assembly

Insert an ECAS-04 connector in the top end (note arrows) of the slide.

![switches and slide](images/switch-and-slide.png)

## Buffer Assembly

'''(optional)''' Use a M2 tap to slightly start the M2 holes for the switches.  I tap about 1mm deep as this makes it easier to start the screw.

Mount the two lever switches in the correct orientation (see picture) using two M2x8mm SHCS.  Note that you may have to bend the JST-XH headers slightly to get the two switches to fit.

Place an EACS connector at the inlet to the buffer and insert a length of PTFE tube that butts up against the inlet and is as long as possible while still allowing the slide to reach the bottom of the channel.

Cut a length of ptfe tubing to the slide which will extend past the opening at the top of the buffer when the slide is at the bottom of the channel.

Place the spring over the ptfe tube and insert it into the slide, place the slide and the spring in the channel and use four M3x8mm SHCS to close up the buffer.
Add an inline filament sensor to the output (which I stupidly did here before adding the spring).

![buffer partial assembly](images/buffer-partial-assembly.png)
![buffer assembled](images/buffer-assembled.png)

See the second picture for the finished assembly.

At this point you should test that everything seems okay.  Take a length of filament and run it from one of the extruder filament holes until it exits at the inline filament sensor.  It should be relatively easy to feed the filament through.  If you have a hard time feeding the filament, verify that your ptfe tube is nicely seated at the inlet of the buffer, that it is as long as possible (inside the slide) and that the filament sensor is completely pressed onto the ptfe tube.

Make sure that you can move the buffer manually and there is no binding.

## Electronics mounting

* '''(optional)''' use a M2.5 tap to start the 4 holes for the SKR Pico board and tap the 4 M3 holes on the back side of the mounting plate.
* Mount the DC power inlet with the wires exiting on the board side.
* Mount the SKR Pico board with the power inlet at the bottom.
* Connect the power wires as shown (note the location of +24V and GND).
* Create a sandwich of the spacer, neopixel board and cover and attach to the back side of the mounting plate using four M3x10mm button head screws.
* '''(optional)''' Attach the USB cable to the SKR Pico board
* Attach the mounting plate to the feeder body using four M3x8mm SHCS.

![board mounted](images/board-mounted.png)
![neopixels mounted](images/neopixels-mounted.png)

## Extruder mounting

For Orbiter extruders:
* Attach the two extruders using four M3x10mm SHCS

For the BMG extruder:
* Attach the two brackets to the feeder body using four M3x6mm button head screws.
* Insert ptfe tubing (~ 28mm) into the feeder body for each extruder
* Attach the BMG body to the steppers using the BMG screws
* Attach the last two inline filament sensors to the two extruders using a short length of ptfe tubing.

The hardware assembly should now be complete and it should like the picture.

![assembly complete](images/assembly-complete.png)
