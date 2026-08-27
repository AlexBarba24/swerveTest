# swerveTest

WPILib 2026 robot project that exercises the Monster eight-motor board over CAN.
Team number is **811**. Routines are selected on the dashboard and started from
the Driver Station in Autonomous.

## Deploy

1. Power the roboRIO and connect it to the same network as the programming PC
   (radio, USB, or Ethernet).
2. Open this project in WPILib VS Code.
3. Deploy:
   - Command Palette → **WPILib: Deploy Robot Code**, or
   - `./gradlew deploy`

Wait until the deploy finishes and the Driver Station shows the roboRIO
connected. The Monster vendordep is already in `vendordeps/`; a first deploy
will download it if the Maven cache is empty.

## Choose and run a routine

Routines are published as a `SendableChooser` named **Test Routine**. Progress
is logged to the console (`[TEST] …`) and to dashboard keys **Test/Name** and
**Test/Status**.

1. Open **SmartDashboard** or **Shuffleboard** (WPILib tools, or the buttons in
   the Driver Station).
2. Find the **Test Routine** dropdown.
   - SmartDashboard shows it automatically.
   - Shuffleboard: if it is missing, add it from NetworkTables (`SmartDashboard`
     → `Test Routine`).
3. Open the **FRC Driver Station**. Confirm the robot is connected and
   **Disabled**.
4. Pick a routine from **Test Routine**. Selection is read when Autonomous
   starts, so change it while disabled.
5. Set the mode to **Autonomous** and click **Enable**.
6. Watch the robot and the `Test/Status` line. Click **Disable** to stop; every
   motion step also stops the motors when it ends.

Routines **1–16** are meant to be run with the robot on blocks. Routines
**17–19** drive the chassis — they need clear floor space, and their results
assume the modules were homed with zero pointing forward.

| Chooser label | What it does |
|---------------|--------------|
| 1. Range guards (no motion) | Software: out-of-range arguments are rejected |
| 2. Cached state checks (no motion) | Software: `get` / `getInverted` / `getTarget` |
| 3. Identify drive motors | Spin each drive motor alone |
| 4. Identify steer motors | Spin each steer motor alone |
| 5. Direction check | Each drive motor forward, then reverse |
| 6. Inversion test | Invert FL drive while all four run |
| 7. Stop vs disable | Brake vs coast |
| 8. Steer calibration | Small absolute move to measure counts/rev |
| 9. Absolute position sweep | Steer through 0, ¼, ½, ¾, 0 |
| 10. Relative position steps | Four quarter-turns out and back |
| 11. Position speed test | Same half-turn at slow then max speed |
| 12. Position interrupt | Cancel a long move with `stopMotor()` |
| 13. Command coalescing | Two speeds in one cycle; only the last should run |
| 14. Diagnostic requests | Status + encoder requests, no motion |
| 15. Module sweep | One module at a time, steer then drive |
| 16. All motors at once | All eight commanded every cycle |
| 17. Drive straight (ROBOT MOVES) | Square to zero, drive forward and back |
| 18. Drive while steering (ROBOT MOVES) | Drive while crabbing |
| 19. Rotate in place (ROBOT MOVES) | X-pattern spin |
| Run all (1-19) | Full suite, several minutes |

To run another routine, **Disable**, change the dropdown, then **Enable**
Autonomous again.
