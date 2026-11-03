# Explorer Rover Simulator

An interactive line-following robot simulator written in C for Borland Turbo C/C++ and BGI graphics. Choose a point count, enter Cartesian coordinates, and watch a four-wheel rover follow your route from (0,0).

## Demo

[Watch the DOSBox demonstration (MP4)](explorer-rover-demo.mp4)

The recording captures an earlier build. The current source restores only the small area beneath the rover during movement to reduce flicker.

## Run locally

1. Open `ROBOT.C` in Turbo C/C++ in DOSBox.
2. Link the graphics library and ensure `EGAVGA.BGI` is in the BGI folder. The source defaults to `C:\TURBOC3\BGI`.
3. Press **Alt+F9** to compile, then **Ctrl+F9** to run.
4. Choose **Create Custom Path**, select **5 points**, then enter (100,0), (100,100), (200,100), and (250,200). The simulator supplies the starting point.
5. Press **Enter** on the preview to start.

## What it demonstrates

- Automatic scaling of custom routes with positive and negative coordinates.
- Three geometric sensors that contribute steering corrections as the rover follows waypoints.
- Live position, progress, reached-point markers, and a movement trail.
- A final report with distance, completion, and the rover's last position.

See the [project guide](PROJECT-GUIDE.md) for setup, controls, and the algorithm. The [verification notes](VERIFICATION.txt) record the controller and coordinate-scaling checks.
