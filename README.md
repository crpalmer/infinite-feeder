Infinite Feeder:

BOM:
====

I broke this into "main components" which are items that you aren't likely
to have laying around and "minor components" which are items that you may
have from other projects.

Main components:
----------------

* BTT SKR-Pico Control Board
* 2x extruder:
   * designed for Orbiter V2.5 (or V2)
   * adapter for Bondtech BMG (e.g. 3DMan clone if you want cheap)
* 1/2in x 1.5in spring (1mm wire diameter)
  * e.g. https://www.amazon.com/gp/product/B09N6NJR3B?smid=A1BCNX405R6I8Q&th=1
* 90 degree USB-C to USB-C adapter or cable (low profile)
  * e.g. https://www.amazon.com/dp/B0CCJLS4R5?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1

Minor components:
-----------------

* 7x D2F switches (2 lever, 5 non lever, just buy levers and remove them)
* 5x MR63-ZZ bearings (3x6x2.5mm)
* 8x ECAS04 (buy a bigger pack since they are so cheap)
* 2.5mm ID 4mm OD PTFE bowden tube
* JST XH connector set (2, 3 and 5 pin connectors and crimp pins)
* M2 SHCS
* M2.5 SHCS
* M3 button head screws
* 12x M3 heatset inserts (3x5x4 voron style)

Buying all this from amazon.com comes to around $150.  If you just need
some of the less common items it could cost around $100-110.

I have built one version with the Orbiter extruders for my largest printer
which I want to be the most reliable.  This printer will often print 2-5KG
jobs and it's expensive (time and money) to have it fail.

For my more moderately sized printer that doesn't tend to print anything
larger than 1KG, I made a cheaper version using the BMG clones.


Note:
-----

This project originally started as a custom firmware for the
NightOwl project, but as I got further into the project, things
diverged and I ended up with a completely new project.

This code started as a fork of:

https://github.com/sebas1984x/standalone-nightowl

but bears no meaningful resemblance to that code at this point.
