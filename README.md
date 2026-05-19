# Infinite Feeder:

![Infinite Feeder Assembly](docs/images/infinite-feeder.png)

## Table of Contents:

* [Bill of Material (BOM)](docs/bom.md)
* [3D Printed Parts](docs/3d-printed-parts.md)
* [Hardware Assembly](docs/hardware.md)
* [Wiring](docs/wiring.md)
* [Software](docs/software.md)

## Introduction to the Infinite Feeder

The infinite-feeder project was started because of my dissatisfaction with
all the available options.

I tried the original Infinity Flow S1 and it wasn't able to push the filament all the way to my extruder on a large printer.

I tried the Nightowl (with the standalone firmware).  The first hurdle was that I had to create a new filament buffer (the turtleneck) because the existing design would not always end up on `empty` when the filament ran out.  I also found that it was possible for the filament to get caught up in the transition points when loading.

Therefore, I created this project to fill in the gaps.  The key differences are:

* **minimalist design**: the entire assembly is a single part that can bolt directly on to v-slot extrusion.  You preserve your existing spool mount and reuse it and if you need to move it to another printer it is trivial.

* **simple assembly**: a natural consequence of the minimalist design, the assembly is relatively straightforward with just a handful of printed parts.

* **reliable**: make use of the different sensor (switches) which detect the filement at various points in the path to detect when the filament didn't feed correctly and then initiate corrective measures to ensure it does eventually feed through the whole tube.  To test the final design, I cut the filament being fed into the system about 50 times to simulate 50 spools running out of filament.  While this isn't the same as actually running out of filament, it was a solid test of the reliability of feeding new filament.  I also have since had numerous actually spools that ran out of filament.  There was never a problem with the feeding of the new filament.
