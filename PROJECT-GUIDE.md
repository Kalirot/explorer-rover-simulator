# Coordinate-Based Intelligent Line Following Robot Simulator

A software simulation for classic Turbo C / Turbo C++ using Borland BGI `graphics.h`. The user creates the track by entering coordinates. A four-wheel Explorer Rover follows every segment with real geometric virtual sensors, incremental movement, and guided recovery.

## Part A — Complete source code

The complete, self-contained source is **ROBOT.C**. No external runtime libraries beyond the standard Turbo C/BGI installation are used. The optional `HEADLESS` section is a modern-host verification harness; the normal Turbo C build excludes it automatically.

Features: four-choice graphical menu, bounded coordinate editor, custom preview, numbered points and coordinate labels, start/finish markers, direction arrows, grid, rotated cyan Explorer Rover with four wheels, an R badge, yellow direction arrow, status light and three visible sensors, path analysis, countdown, live data, speed control, pause/resume, trail, and an honest completion/stopped report.

## Part B — Required Turbo C setup/configuration

1. Use a licensed classic Turbo C 2.0 or Turbo C++ 3.0 installation. On a modern operating system, run that DOS compiler through DOSBox or a compatible DOS environment. This is a DOS BGI application, not a modern Windows `graphics.h` port.
2. The installation needs its `INCLUDE`, `LIB`, and `BGI` directories, including **EGAVGA.BGI** and **GRAPHICS.LIB**.
3. In the IDE, set **Options → Directories** to your installation's include and library paths. For example: `C:\TC\INCLUDE` and `C:\TC\LIB`. Actual folder names may differ.
4. Enable the graphics library in the linker's library settings. A command-line Turbo C build can explicitly link it: `TCC -ms ROBOT.C GRAPHICS.LIB`. Use the normal floating-point/math support supplied with Turbo C.
5. Use the **small memory model**, with the standard stack setting (4 KB is adequate). The fixed arrays are sized to fit the conventional 16-bit data segment. No large frame buffers or heap allocations are required.
6. The program explicitly selects **VGA / VGAHI, 640 × 480, 16 colors**. DOSBox must provide VGA compatibility.
7. The default driver folder in the source is `C:\TURBOC3\BGI`. Change `BGI_DIR` if needed, or supply the directory when running: `ROBOT C:\TURBOC3\BGI`. Paths here are DOS paths inside the emulator, not Windows host paths.

If graphics initialization fails, the program prints the BGI error and the directory it attempted. Check the driver file and path first. Do not replace the headers with an unrelated modern graphics package.

## Part C — How to run

1. Copy `ROBOT.C` into the DOS compiler's working folder. Open it in Turbo C/C++.
2. Compile and link using the configuration above; run the resulting `ROBOT.EXE`.
3. Choose **1** for your own track or **2** for the demonstration track.
4. For custom input, select **3 points**, **5 points**, **10 points**, or **Choose another number** (2–30 total). **P1 = (0,0)** is supplied automatically; enter only P2 onward. Coordinates are signed whole numbers of up to nine digits, independent of the display size. Type one value and press Enter. Backspace edits; Escape cancels. Invalid numbers stay in the editor. A repeated coordinate must be re-entered.
5. Review the generated track and its calculated analysis. Press **Enter** to start the countdown, or **Escape** to return.
6. During movement: **P** pauses, **R** resumes, **S** stops, **Q** returns through the stopped report, **+** increases speed, and **−** decreases it. Speed is clamped to 0.5–4.0 pixels per controller tick. It is the requested cruise step; corners and recovery use smaller steps.
7. Review the final report. Press a key for the menu; choose **4** to exit.

Coordinates use a Cartesian convention: the robot starts at (0,0), X increases rightward, and Y increases upward. The entire route is uniformly scaled and centered to fit the viewport, including negative coordinates. The visible screen origin may move to accommodate the route. There is no screen-sized input range; numeric storage supports -999999999 through 999999999. The shape and aspect ratio are preserved.

## Part D — Algorithm

1. Initialize VGA graphics; show the menu.
2. Read a bounded point count. Set the first world point to (0,0), read the remaining signed coordinates into `worldX[]` and `worldY[]`, and uniformly map them into floating-point screen arrays `xPoints[]` and `yPoints[]`. Alternatively, load the sample.
3. Connect adjacent points. Sum segment lengths and classify changes in segment direction.
4. Show a preview with labels, arrows, start/finish and analysis.
5. Initialize the robot at the first point, facing the first segment. This is the only initial position assignment.
6. After the countdown, repeatedly process keys and, unless paused, perform a controller tick.
7. Compute the distance and desired heading to the active segment endpoint.
8. Transform three sensor locations from robot-local coordinates to screen coordinates. Measure their distance to every finite path segment.
9. Use the waypoint heading as guidance. If the center is off and only one side sensor detects line, apply a steering correction toward that side. If the heading error is large, turn in place before advancing. With no detection, use slower, guided recovery toward the active target.
10. Advance with `X += step*cos(heading)` and `Y += step*sin(heading)`. Cap the step by the remaining distance. Near the endpoint, remove side-sensor bias and align the final approach.
11. Once the robot is within floating-point tolerance of the endpoint, advance the segment index. At the final endpoint, stop and mark completion.
12. Display the final report; return to the menu.

The implementation was constructed with the movement core first, followed by geometric sensors and steering, then the graphical interface. The host check directly executes the same controller used by the graphics build.

## Part E — Flowchart explanation

```text
START → Initialize graphics → Menu
                               ├─ Instructions → Menu
                               ├─ Exit → Close graphics → END
                               ├─ Sample path ─────────────┐
                               └─ Custom point input       │
                                    ↓                      │
                               Validate / retry            │
                                    └──────────────────────┤
                                                           ↓
                                            Analyze → Draw path preview
                                                           ↓ Enter
                                              Robot ready → 3,2,1,GO
                                                           ↓
                    ┌────────────────────── Read controls ←┘
                    │                              ↓
                    │                    Stop? → Stopped report → Menu
                    │                              ↓ No
                    │                    Paused? → Wait / redraw
                    │                              ↓ No
                    │              Calculate target + read sensors
                    │                              ↓
                    │               Steer / recover / move a small step
                    │                              ↓
                    │                   Endpoint reached?
                    │                     ↓ No         ↓ Yes
                    └─────────────────────┘       Final endpoint?
                                                   ↓ No    ↓ Yes
                                             Next segment  Completed report
                                                   ↓             ↓
                                             Read controls      Menu
```

The preview makes the user's role visible. The simulation loop makes the sensing, steering and movement visible. Completion is recorded only after all ordered segment endpoints are reached.

## Part F — Important functions in simple language

| Function | Purpose |
|---|---|
| `initGraphics` | Opens the requested VGA mode and reports driver errors. |
| `showMainMenu` | Offers custom input, sample, instructions and exit. |
| `readNumber` | Reads a short number safely inside the graphics screen. |
| `getCoordinates` | Supplies (0,0), then collects signed world coordinates with duplicate checks. |
| `fitPath` | Preserves the route shape while scaling it to fit the screen. |
| `validateCoordinates` | Legacy screen-space validation used by host tests; user input checks duplicates in world coordinates before auto-fitting. |
| `calculateDistance` | Uses the Pythagorean formula to measure pixel distance. |
| `calculateAngle` / `wrapAngle` | Finds segment direction and keeps angle differences within ±180 degrees. |
| `analyzePath` | Calculates real track length, turns and sharp turns. |
| `drawGrid`, `drawPath`, `drawPoints` | Shows the user's track, coordinates and direction arrows. |
| `drawRobot` / `robotBox` | Draws the Explorer Rover body, four wheels, R badge, status lamp and front bar rotated with the robot. |
| `distanceToSegment` | Finds distance to a finite line segment, including its endpoints. |
| `detectSensors` | Positions the three sensors and computes their real ON/OFF readings. |
| `updateRobot` | Makes one steering or movement step and handles waypoint arrival. |
| `completion` | Measures ordered progress along the current segment and completed segments. |
| `rememberTrail` | Stores spaced actual positions in a bounded history. |
| `simulateRobot` | Runs preview, countdown, controls, movement and reporting. |
| `showFinalReport` | Reports the actual outcome and final position, even after an early stop. |

## Part G — Exactly how coordinates create the route

For N total points, world arrays store N long-integer X values and N long-integer Y values, with the first point fixed at zero. Screen arrays hold their uniformly scaled floating-point positions. Adjacent entries define **N−1 ordered segments**. The robot's current segment starts at index `segment` and ends at index `segment+1`.

For a target `(tx,ty)`, compute `dx=tx−robotX`, `dy=ty−robotY`, `distance=sqrt(dx*dx+dy*dy)`, and `desired=atan2(dy,dx)`. The heading approaches this direction, and the position changes by a bounded incremental step. It never assigns the robot position to a later waypoint. Within 14 pixels of a target, the robot slows down and aligns its final approach; the last movement is limited to the remaining distance. Arrival tolerance is 0.00000000001 screen pixel. Extremely unequal segment sizes can be visually indistinguishable at screen resolution.

All points must be distinct, including nonconsecutive ones. Crossings and overlapping segments are allowed, but repeating exactly the starting point as a final point is rejected by this duplicate policy. Crossing lines do not change the waypoint order.

Track length is the sum of all segment lengths, converted back into user coordinate units for display. A direction change greater than 10 degrees counts as a turn; 70 degrees or more counts as sharp. Internally screen Y increases downward, so a positive internal heading change is a right turn. Displayed coordinates and angles are converted back to Cartesian values. The live turn label describes the turn entering the current segment. Motion slows near every endpoint, including sharp corners.

Completion is based on completed segment lengths plus the robot's clamped projected progress on the active segment, divided by total track length. Actual movement distance separately sums the lengths of all movement steps, so steering corrections can make it longer than track length. Angle is displayed in signed screen degrees; zero points right and positive displayed angles turn counterclockwise.

## Part H — Exactly how virtual sensors work

Each sensor is **6 pixels forward** of the robot center. Left/center/right lateral offsets are **+5/0/−5 pixels**. With forward distance `f`, left offset `l`, and heading `h`:

```text
sensorX = robotX + f*cos(h) + l*sin(h)
sensorY = robotY + f*sin(h) - l*cos(h)
```

For each segment from A to B, project the sensor S onto the segment:

```text
t = dot(S-A, B-A) / dot(B-A, B-A)
t = clamp(t, 0, 1)
nearest = A + t*(B-A)
distance = length(S-nearest)
```

Clamping is essential: a sensor beyond an endpoint must not detect the infinite extension of that line. A sensor is ON when its distance to **any** actual track segment is at most **4.5 pixels**: the virtual line radius is 3 pixels and the sensor's footprint radius is 1.5 pixels. Raster drawing approximates this mathematical band. Detection is calculated from geometry, not screen colors, so labels, grid and trail cannot create false readings.

When center is off, left-only detection subtracts 0.055 radians from steering; right-only adds 0.055 radians. Center detection needs no side correction. Equal/ambiguous side readings leave waypoint guidance in control. At large heading error the robot rotates in place, at up to 0.12 radians per tick. Normal heading correction is limited to 0.09 radians per tick before the side correction. Near corners, waypoint alignment takes priority over sensor bias.

With all sensors off, the robot turns toward the active target and moves at 45% of the current step when aligned. This is **guided recovery**, not a blind oscillating search. The simulation is deliberately a **hybrid waypoint-and-sensor controller**, not a physical sensor-only differential-drive model. Sensor readings influence motion, while known coordinate targets disambiguate crossings and make endpoint arrival reliable. The front sensors can legitimately all be OFF at the final endpoint because they extend beyond the finite track.

The sensor circles and live L/C/R labels use the same computed readings. Green means detection, red means no detection. There are no scripted sensor patterns or sample-specific statistics.

## Part I — Demonstration coordinates

Choose menu option 2, or choose eight total points and enter P2 through P8 below:

| Point | X | Y |
|---|---:|---:|
| P1 (automatic) | 0 | 0 |
| P2 | 100 | 0 |
| P3 | 180 | 70 |
| P4 | 260 | 70 |
| P5 | 310 | 170 |
| P6 | 210 | 125 |
| P7 | 285 | 20 |
| P8 | 345 | 200 |

This route includes straight travel, diagonals, left/right changes and sharp turns. Its analysis is calculated at runtime. For a quick second demonstration, choose five total points: `(0,0)` is automatic, then enter `(100,0), (180,100), (270,40), (340,190)`. The different preview immediately shows the custom-coordinate concept.

Suggested demonstration: create the second track, point to the labels and analysis, start, pause to explain the front sensors, resume, change speed, then show the final report. Run the built-in sample afterward to show that the controller adapts without code changes.

## Part J — 15 likely viva questions

1. **What is the main idea?** The user enters coordinates that become a track followed by a simulated robot.
2. **Is this an Arduino project?** No. It is a software simulation using Turbo C graphics.
3. **What makes the project different?** The route is created by the user instead of being limited to one fixed track.
4. **How are coordinates stored?** In `worldX[]` and `worldY[]`; scaled positions go into `xPoints[]` and `yPoints[]`.
5. **How many segments do N points create?** N−1, connecting each point to the next.
6. **Why use `atan2`?** It finds the correct target direction in all four quadrants.
7. **How does the robot move without teleporting?** Each tick adds a small X/Y displacement calculated with cosine and sine.
8. **How does a sensor detect the line?** It measures its distance to the finite track segments and compares that distance with a detection threshold.
9. **Are sensor readings real?** They are simulated geometric measurements that also affect steering; they are not scripted values.
10. **What happens when only the left sensor detects?** Away from a corner, it adds a small left correction to waypoint steering.
11. **What happens when no sensor detects?** The robot uses a slower guided recovery toward the current target.
12. **Is this a sensor-only physical robot model?** No. It combines waypoint guidance with sensor corrections for reliable custom-route completion.
13. **How are invalid inputs handled?** The editor accepts signed whole numbers up to nine digits and rejects duplicate points. The route automatically fits the screen.
14. **How are sharp turns detected?** Consecutive segment headings differ by at least 70 degrees; the robot slows near endpoints and can turn in place.
15. **How do you know the mission is complete?** Every ordered segment endpoint has been reached and the final endpoint is within numerical tolerance.

## Verification and practical limits

The movement/sensor core was compiled as strict ANSI C89 on the host and run over 1,007 paths, including random paths, tight reversals and one-pixel spacing. Checks cover ordered waypoint arrival, bounded displacement, final coordinates, completion, geometric readings, steering response and validation. See `VERIFICATION.txt` for commands and results.

The actual Borland compiler, DOS BGI driver and VGA screen were **not available for an end-to-end run here**. The DOS executable must be compiled and visually checked in your Turbo C environment. The delivered host check executable, if present locally, is not the DOS simulator.

VGAHI normally offers one graphics page. The animation no longer clears and redraws the full screen. It saves a 33x33-pixel BGI patch beneath the rover, restores that patch, extends the actual movement trail, stamps reached waypoints green, captures the next patch, and redraws the rover. The live panel replaces only small text rows about every five controller ticks. A mission progress bar fills from calculated completion. Controller ticks use a 20 ms delay; actual frame rate also depends on rendering and emulator speed. A fixed 2 KB image buffer is checked with imagesize before use. This greatly reduces redraw work but actual DOSBox smoothness still needs visual checking. A bounded 800-point history remains available, while the live trail is painted directly onto the persistent scene, so its visible history does not disappear when stored samples are thinned. Dense coordinates can cause point-label overlap. Simulation speed is pixels per tick, not calibrated real-world velocity. This is an educational kinematic simulation with in-place turns, not a motor/traction/physics model.
