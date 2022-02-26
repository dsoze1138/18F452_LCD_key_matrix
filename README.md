# PIC18F452 LCD and key matrix
=====================

This demo code for post in the Microchip forum.

See: https://www.microchip.com/forums/FindPost/1200456

The forum member that started this topic seem lack useful experience with embedded controllers and circuit design.

The fragment of schematic attached to the first post is not complete as it lacks a crystal or external oscillator so it's unlikely that a matching prototype was built . How the GPIO pins are assigned to the LCD module and key matrix reveals there is little or no understanding the relationship between the bit position in a port register can impact the code implementation.

In my view this schematic looks like something created using the Lab Center Proteus simulation tools.

Even without useful details about the goals the original poster is trying to achieve this demo code does work as expected in the MPLAB v8.92 simulator. The MPLABX v5.50 simulator will also run it but on my Windows 7 OS MPLABX is not working reliably and fails in odd and mysterious ways.