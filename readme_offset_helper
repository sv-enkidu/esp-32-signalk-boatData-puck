Compass Offset/Calibration Tool/Helper (python)

### What it is:
Utility to use debug feature QMC_RAW_DEBUG (set to 1 to use) in the ESP firmware.
Written for the QMC compass in the current puck hardware, which is the compass most
susceptable to hard/soft iron interference (the BNO dynamically adjusts for much of this)

### To setup/run:
Setup (one time): pip install numpy matplotlib pyserial

### Two ways to use it:

From a saved log (a copy-paste of the puck output from the Arduino serial monitor works as is):
- python qmc_cal_plot.py capture.txt

Record directly from the puck, then analyse:
- python qmc_cal_plot.py --serial COM5 --seconds 60 capture.txt
- Close the serial monitor and Signal K first, because only one program can use the port at a time. 
- On a Mac the port looks like /dev/cu.usbmodem….

## What the utility gives you:
A text report with:
- the fitted circle centre, radius and offset size, with a warning if (0,0) is outside the circle (your desk case);
the QMC-vs-BNO error before and after correction;
- a ready-to-paste block of qmcHard/qmcSoft lines, plus the offsetQmcYaw that lines the QMC up with the BNO.

Three plots, each titled with what “good” looks like:
- Horizontal field: red is raw and blue is corrected. You want a round blue circle centred on the black +.
- Headings over the capture: the BNO as a black line and the QMC as dots. Your desk log clearly shows the red 
- QMC stalling around 85° and turning back.
- Error vs heading: QMC minus BNO at each heading. You want a flat line near 0. Your corrected data still shows a 
- gentle hump, about +25° around 90–150° and about −10° around 270–330°. A once-per-turn wave like that usually 
- means some offset is left over, or the field was distorted where you were, so it’s a good thing to recheck on the bench
 or on the boat.

See
<img width="887" height="1027" alt="puck_signalk_pathView" src="https://github.com/sv-enkidu/esp-32-signalk-boatData-puck/blob/main/offset_helper_plot_example.png" />

Previewing values before flashing:
python qmc_cal_plot.py capture.txt --hard 0.233 0.143 0 --soft 1.0 1.0

This shows exactly what those numbers would do to the same capture. It’s useful for comparing options, or for 
checking whether bench values still hold on a capture taken on the boat.

### Tips for good captures:

Turn slowly, two or three full turns over 30–60 s. The report tells you if you didn’t cover the full 360°.
Tilt the puck well over in several directions while turning, if you want the Z offset. Otherwise the script 
reports Z as “not determined” rather than guessing.
Compare runs. Captures from the desk, the sill, battery vs laptop power, and GPS unplugged will show directly 
whether the centre moves. That’s the hardware question you’re weighing: if the centre stays put, a one-time 
calibration holds; if it moves, something on the board needs relocating or isolating.
