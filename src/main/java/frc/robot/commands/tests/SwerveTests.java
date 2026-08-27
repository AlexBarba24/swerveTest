// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.TestConstants;
import com.vendor.MonsterController;
import com.vendor.MonsterProtocol;
import frc.robot.subsystems.ExampleSubsystem;
import java.util.ArrayList;
import java.util.List;

/**
 * The Monster controller API test suite: one static factory per routine, ordered from software-only
 * checks through single-motor identification and eight-motor integration to whole-robot motion.
 *
 * <p>Routines 1 through 16 are safe with the robot on blocks. Routines 17 through 19 drive the
 * chassis, so they need clear floor space, and they are the only ones whose results depend on the
 * modules having been homed with zero pointing forward.
 *
 * <p>The suite is open loop. Nothing reads encoder or status replies back from the board, so only
 * {@link #rangeGuards} and {@link #cachedStateChecks} can report PASS/FAIL on their own. Every
 * other routine is verified by watching the robot while reading the lines each step writes to the
 * console and to {@code Test/Status} on SmartDashboard.
 *
 * <p>Every {@link TimedTestCommand} requires the subsystem, so routines are chained with {@code
 * Commands.sequence} and never composed in parallel. A step that has to command several motors
 * inside one 20 ms cycle is written as a single command for that reason.
 */
public final class SwerveTests {
  private static final String[] CORNERS = {"FL", "FR", "BL", "BR"};

  private static final String[] CORNER_NAMES = {
    "front-left", "front-right", "back-left", "back-right"
  };

  private static final int EIGHTH_TURN = TestConstants.STEER_COUNTS_PER_REV / 8;
  private static final int QUARTER_TURN = TestConstants.STEER_COUNTS_PER_REV / 4;
  private static final int HALF_TURN = TestConstants.STEER_COUNTS_PER_REV / 2;
  private static final int THREE_QUARTER_TURN = 3 * TestConstants.STEER_COUNTS_PER_REV / 4;

  /** Number of routines in the suite, used for the {@link #runAll} progress markers. */
  private static final int ROUTINE_COUNT = 19;

  /**
   * Steer targets that put the modules in the X pattern needed to rotate about the robot's centre,
   * in FL, FR, BL, BR order. Each wheel ends up tangent to the circle around the centre, which for
   * a square chassis is an eighth turn off straight, alternating sign across the diagonals.
   */
  private static final int[] ROTATION_STEER_TARGETS = {
    -EIGHTH_TURN, EIGHTH_TURN, EIGHTH_TURN, -EIGHTH_TURN
  };

  /**
   * Drive directions that pair with {@link #ROTATION_STEER_TARGETS}, in FL, FR, BL, BR order. The
   * left pair runs against the right pair, since the X pattern leaves each side's wheels pointing
   * along the same line but facing opposite ways around the circle.
   */
  private static final double[] ROTATION_DRIVE_SIGNS = {-1.0, 1.0, -1.0, 1.0};

  /** How long each whole-robot motion step drives for. */
  private static final double ROBOT_DRIVE_SECONDS = 2.0;

  /**
   * Dwell held after commanding a position move. The board owns the motion profile and {@link
   * TimedTestCommand} stops the motors when a step ends, so this has to outlast the travel or the
   * move is cut short.
   */
  private static final double MOVE_DWELL_SECONDS = 3.0;

  /**
   * Counts commanded by {@link #steerCalibration}. Deliberately small, since nothing is known
   * about the gearing until that routine has been run.
   */
  private static final int CALIBRATION_COUNTS = 1000;

  private static final int SLOW_POSITION_SPEED = 50;
  private static final int FAST_POSITION_SPEED = 250;
  private static final double SOFTWARE_TEST_SECONDS = 0.5;
  private static final double DIAGNOSTIC_PERIOD_SECONDS = 1.0;
  private static final double DIAGNOSTIC_SECONDS = 5.0;

  /**
   * Checks that the protocol rejects out-of-range arguments instead of clipping them and sending
   * something the board did not ask for.
   *
   * <p>Nothing moves: every call under test throws before it reaches the CAN buffer. Safe to run
   * first on a robot that is not on blocks, and self-verifying, so watch the log for PASS/FAIL
   * rather than the robot.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command rangeGuards(ExampleSubsystem subsystem) {
    return new RangeGuardsCommand(subsystem);
  }

  /**
   * Checks the state {@link MonsterController} caches on the roboRIO side: {@code get()},
   * {@code getInverted()}, and {@code getTarget()}.
   *
   * <p>Nothing moves. Each write is followed by a stop inside the same scheduler cycle, and the
   * buffer holds one command per motor, so the stop overwrites the motion before the flush. Self
   * verifying, so watch the log for PASS/FAIL.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command cachedStateChecks(ExampleSubsystem subsystem) {
    return new CachedStateChecksCommand(subsystem);
  }

  /**
   * Spins each drive motor alone so CAN IDs 0, 2, 4 and 6 can be matched to the corners they
   * actually sit on.
   *
   * <p>Watch for exactly one wheel turning per step, and confirm it is the corner the log names. If
   * two wheels turn, or the wrong one does, the CAN IDs in {@code Constants} are wrong.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command identifyDriveMotors(ExampleSubsystem subsystem) {
    return identify(
            subsystem,
            subsystem.driveMotors(),
            "drive",
            TestConstants.DRIVE_TEST_SPEED,
            "wheel to spin")
        .withName("identifyDriveMotors");
  }

  /**
   * Spins each steer motor alone so CAN IDs 1, 3, 5 and 7 can be matched to the corners they
   * actually sit on.
   *
   * <p>Watch for exactly one module rotating per step, and confirm it is the corner the log names.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command identifySteerMotors(ExampleSubsystem subsystem) {
    return identify(
            subsystem,
            subsystem.steerMotors(),
            "steer",
            TestConstants.STEER_TEST_SPEED,
            "module to rotate")
        .withName("identifySteerMotors");
  }

  /**
   * Runs each drive motor forward and then in reverse to establish the sign convention per corner.
   *
   * <p>Watch which way each wheel turns on the positive step. Corners that disagree need
   * {@code setInverted(true)} in the real drivetrain code; note which ones here.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command directionCheck(ExampleSubsystem subsystem) {
    MonsterController[] motors = subsystem.driveMotors();
    List<Command> steps = new ArrayList<>();
    steps.add(
        new LogCommand(
            "directionCheck: each drive motor forward then reverse; note the corners whose "
                + "positive direction disagrees with the rest"));
    for (int corner = 0; corner < motors.length; corner++) {
      String label = tag(corner, "drive", motors[corner]);
      steps.add(new LogCommand("  next: " + label + " FORWARD at " + TestConstants.DRIVE_TEST_SPEED));
      steps.add(
          new RunMotorsCommand(
              subsystem,
              "direction " + label + " forward",
              TestConstants.DRIVE_TEST_SPEED,
              TestConstants.STEP_SECONDS,
              motors[corner]));
      steps.add(dwell());
      steps.add(new LogCommand("  next: " + label + " REVERSE, expect the same wheel the other way"));
      steps.add(
          new RunMotorsCommand(
              subsystem,
              "direction " + label + " reverse",
              -TestConstants.DRIVE_TEST_SPEED,
              TestConstants.STEP_SECONDS,
              motors[corner]));
      steps.add(dwell());
    }
    return Commands.sequence(steps.toArray(new Command[0])).withName("directionCheck");
  }

  /**
   * Inverts FL drive, runs all four drive motors together, then restores it.
   *
   * <p>Watch the front-left wheel turn opposite the other three during the run, and confirm it
   * turns with them again on the final step once inversion is cleared.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command inversionTest(ExampleSubsystem subsystem) {
    MonsterController fl = subsystem.fl_drive;
    String label = tag(0, "drive", fl);
    return Commands.sequence(
            new LogCommand(
                "inversionTest: " + label + " inverted while all four drive motors run the same "
                    + "speed; expect the front-left wheel to oppose the other three"),
            action("  setInverted(true) on " + label, () -> fl.setInverted(true)),
            new RunMotorsCommand(
                subsystem,
                "inversion inverted",
                TestConstants.DRIVE_TEST_SPEED,
                TestConstants.STEP_SECONDS,
                subsystem.driveMotors()),
            dwell(),
            action("  setInverted(false) on " + label, () -> fl.setInverted(false)),
            new LogCommand("  next: all four again, expect every wheel the same way now"),
            new RunMotorsCommand(
                subsystem,
                "inversion restored",
                TestConstants.DRIVE_TEST_SPEED,
                TestConstants.STEP_SECONDS,
                subsystem.driveMotors()))
        .withName("inversionTest");
  }

  /**
   * Separates the two ways of ending motion: STOP leaves the driver energized, DISABLE
   * de-energizes it.
   *
   * <p>After the STOP the wheel should brake and resist being turned by hand. After the DISABLE it
   * should coast down and spin freely. The last step then sends a plain speed command with no
   * explicit re-enable, which answers whether the board wakes itself back up on motion traffic.
   * Judge each behavior during the step, because the suite stops every motor when a step ends.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command stopVsDisable(ExampleSubsystem subsystem) {
    MonsterController fl = subsystem.fl_drive;
    double hold = 2 * TestConstants.DWELL_SECONDS;
    double releaseAt = TestConstants.STEP_SECONDS;
    double retryAt = releaseAt + hold;
    return Commands.sequence(
            new LogCommand(
                "stopVsDisable: " + tag(0, "drive", fl) + " spun up twice, released with STOP the "
                    + "first time and DISABLE the second"),
            new StopVsDisableCommand(
                subsystem,
                "stopVsDisable STOP",
                fl,
                TestConstants.DRIVE_TEST_SPEED,
                releaseAt,
                false,
                0,
                releaseAt + hold),
            dwell(),
            new StopVsDisableCommand(
                subsystem,
                "stopVsDisable DISABLE",
                fl,
                TestConstants.DRIVE_TEST_SPEED,
                releaseAt,
                true,
                retryAt,
                retryAt + TestConstants.STEP_SECONDS),
            new LogCommand(
                "stopVsDisable done: a braked stop that resists by hand plus a free coast means "
                    + "the two protocol paths are wired the way the comments claim"))
        .withName("stopVsDisable");
  }

  /**
   * Commands FL steer a small fixed number of counts so the real counts per revolution can be
   * measured.
   *
   * <p>Mark the module, run this, and measure how far it actually rotated, then correct
   * {@code TestConstants.STEER_COUNTS_PER_REV} before trusting any other position routine. The
   * target is absolute, so it is measured from wherever the board considers zero, normally the
   * position the module was in at power-up.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command steerCalibration(ExampleSubsystem subsystem) {
    MonsterController steer = subsystem.fl_steer;
    return Commands.sequence(
            new LogCommand(
                "steerCalibration: " + tag(0, "steer", steer) + " to absolute "
                    + CALIBRATION_COUNTS + " counts; measure the angle it actually sweeps"),
            PositionMoveCommand.absolute(
                subsystem,
                "steerCalibration",
                CALIBRATION_COUNTS,
                MonsterController.DEFAULT_POSITION_SPEED,
                MOVE_DWELL_SECONDS,
                steer),
            new LogCommand(
                "  set STEER_COUNTS_PER_REV = " + CALIBRATION_COUNTS + " * 360 / (degrees "
                    + "measured); it is currently the placeholder "
                    + TestConstants.STEER_COUNTS_PER_REV))
        .withName("steerCalibration");
  }

  /**
   * Walks all four steer motors through zero, a quarter, a half, three quarters and back to zero as
   * absolute targets.
   *
   * <p>Watch for four equal quarter-turn arcs and a return to the starting angle. Unequal arcs mean
   * {@code STEER_COUNTS_PER_REV} is still wrong; a module that keeps rotating the same way instead
   * of coming back means the board is treating absolute targets as relative. Because every module
   * gets the same target in the same cycle, a corner that lags or overshoots the other three is
   * showing you a mechanical or gearing difference rather than a protocol problem.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command absolutePositionSweep(ExampleSubsystem subsystem) {
    MonsterController[] steer = subsystem.steerMotors();
    int[] targets = {0, QUARTER_TURN, HALF_TURN, THREE_QUARTER_TURN, 0};
    List<Command> steps = new ArrayList<>();
    steps.add(
        new LogCommand(
            "absolutePositionSweep: " + steerGroup(steer) + " through 0, quarter, half, "
                + "three-quarter and back to 0; expect four equal arcs on every module, all "
                + "ending where they started"));
    for (int target : targets) {
      steps.add(
          PositionMoveCommand.absolute(
              subsystem,
              "sweep to " + target,
              target,
              MonsterController.DEFAULT_POSITION_SPEED,
              MOVE_DWELL_SECONDS,
              steer));
      steps.add(new LogCommand(() -> "  getTarget() reports " + targetReport(steer)));
    }
    return Commands.sequence(steps.toArray(new Command[0])).withName("absolutePositionSweep");
  }

  /**
   * Steps all four steer motors four quarter turns forward and four back using relative moves.
   *
   * <p>Each group of four should add up to one full revolution and leave every module where it
   * started. If the modules stop after the first step and never move again, the board is treating
   * {@code MOVE_RELATIVE} as an absolute target rather than accumulating it. Drift that only shows
   * up on some corners after the eight steps points at counts being lost on those boards.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command relativePositionSteps(ExampleSubsystem subsystem) {
    MonsterController[] steer = subsystem.steerMotors();
    List<Command> steps = new ArrayList<>();
    steps.add(
        new LogCommand(
            "relativePositionSteps: " + steerGroup(steer) + " by " + QUARTER_TURN
                + " counts at a time; four forward should equal one revolution, four back should "
                + "undo it"));
    for (int step = 1; step <= 4; step++) {
      steps.add(relativeStep(subsystem, steer, QUARTER_TURN, "forward " + step + "/4"));
    }
    steps.add(new LogCommand("  now the same four in reverse"));
    for (int step = 1; step <= 4; step++) {
      steps.add(relativeStep(subsystem, steer, -QUARTER_TURN, "reverse " + step + "/4"));
    }
    return Commands.sequence(steps.toArray(new Command[0])).withName("relativePositionSteps");
  }

  /**
   * Runs the same half-turn move at a slow board speed and then at the maximum, to confirm the
   * speed argument reaches the board.
   *
   * <p>Both passes get the same dwell, so the fast pass should visibly finish sooner, and the slow
   * pass may not reach the target before the step ends. If the two passes look identical, the speed
   * argument is not reaching the board.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command positionSpeedTest(ExampleSubsystem subsystem) {
    MonsterController steer = subsystem.fl_steer;
    return Commands.sequence(
            new LogCommand(
                "positionSpeedTest: " + tag(0, "steer", steer) + " half turn at speed "
                    + SLOW_POSITION_SPEED + " then at " + FAST_POSITION_SPEED
                    + ", same dwell both times"),
            PositionMoveCommand.absolute(
                subsystem,
                "speed test park",
                0,
                MonsterController.DEFAULT_POSITION_SPEED,
                MOVE_DWELL_SECONDS,
                steer),
            new LogCommand("  slow pass, expect it to crawl and possibly run out of dwell"),
            PositionMoveCommand.absolute(
                subsystem, "speed test slow out", HALF_TURN, SLOW_POSITION_SPEED,
                MOVE_DWELL_SECONDS, steer),
            PositionMoveCommand.absolute(
                subsystem, "speed test slow back", 0, SLOW_POSITION_SPEED, MOVE_DWELL_SECONDS,
                steer),
            new LogCommand("  fast pass, same two targets, expect both to complete early"),
            PositionMoveCommand.absolute(
                subsystem, "speed test fast out", HALF_TURN, FAST_POSITION_SPEED,
                MOVE_DWELL_SECONDS, steer),
            PositionMoveCommand.absolute(
                subsystem, "speed test fast back", 0, FAST_POSITION_SPEED, MOVE_DWELL_SECONDS,
                steer))
        .withName("positionSpeedTest");
  }

  /**
   * Starts a long position move on FL steer and cancels it mid-flight with {@code stopMotor()}.
   *
   * <p>This is the one routine that checks the arbitration claim behind the STOP path: a class-0
   * STOP is supposed to beat motion already queued on the board. Watch the module start turning,
   * halt part way, and stay put for the rest of the step. If it resumes and finishes the move, STOP
   * does not cancel a position profile.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command positionInterrupt(ExampleSubsystem subsystem) {
    return new PositionInterruptCommand(subsystem);
  }

  /**
   * Issues two different speeds to the same motor inside one scheduler cycle.
   *
   * <p>The protocol buffers one command per motor, so only the second speed should ever reach the
   * board. Watch for the wheel coming straight up to the higher speed with no blip at the lower
   * one. Both calls have to happen in a single command, since sequenced commands run on different
   * cycles and would each get their own flush.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command commandCoalescing(ExampleSubsystem subsystem) {
    return new CommandCoalescingCommand(subsystem);
  }

  /**
   * Sends status requests to all eight motors and an encoder request to the board at roughly 1 Hz.
   *
   * <p>There is no read path in this code, so verify with a CAN sniffer or the board's own
   * indicators; nothing moves. Also note that the encoder request borrows motor 0's buffer slot,
   * so motor 0's status frame is the one that gets dropped each cycle.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command diagnosticRequests(ExampleSubsystem subsystem) {
    return new DiagnosticRequestsCommand(subsystem);
  }

  /**
   * Exercises one module at a time the way a swerve drive uses it: steer to zero, drive, steer a
   * quarter turn, drive again.
   *
   * <p>Watch that the wheel that spins belongs to the module that just rotated, and that the second
   * drive step pushes in a direction a quarter turn away from the first. Mismatches here mean a
   * drive and steer motor from different corners share a module in {@code Constants}.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command moduleSweep(ExampleSubsystem subsystem) {
    MonsterController[] drive = subsystem.driveMotors();
    MonsterController[] steer = subsystem.steerMotors();
    List<Command> steps = new ArrayList<>();
    steps.add(
        new LogCommand(
            "moduleSweep: one module at a time, steer then drive at two angles; expect the wheel "
                + "that spins to belong to the module that just rotated"));
    for (int corner = 0; corner < drive.length; corner++) {
      String moduleLabel = CORNER_NAMES[corner] + " module";
      steps.add(
          new LogCommand(
              "  next: " + moduleLabel + ", steer " + tag(corner, "steer", steer[corner])
                  + " and drive " + tag(corner, "drive", drive[corner])));
      for (int target : new int[] {0, QUARTER_TURN}) {
        steps.add(
            PositionMoveCommand.absolute(
                subsystem,
                "moduleSweep " + CORNERS[corner] + " steer to " + target,
                target,
                MonsterController.DEFAULT_POSITION_SPEED,
                MOVE_DWELL_SECONDS,
                steer[corner]));
        steps.add(
            new RunMotorsCommand(
                subsystem,
                "moduleSweep " + CORNERS[corner] + " drive at " + target,
                TestConstants.DRIVE_TEST_SPEED,
                TestConstants.STEP_SECONDS,
                drive[corner]));
        steps.add(dwell());
      }
    }
    return Commands.sequence(steps.toArray(new Command[0])).withName("moduleSweep");
  }

  /**
   * Drives all four wheels while all four modules run position moves, filling every buffer slot on
   * every cycle.
   *
   * <p>Eight frames per 20 ms is the heaviest load the suite puts on the bus. Watch for wheels that
   * stutter or modules that lag the others, which would point at dropped or delayed frames. This
   * cannot be a parallel composition, since two commands that require the subsystem interrupt each
   * other, so all eight motors are commanded from one command instead.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command allMotorsAtOnce(ExampleSubsystem subsystem) {
    return new AllMotorsAtOnceCommand(subsystem);
  }

  /**
   * Squares every module to zero and drives the whole robot forward, then back.
   *
   * <p>The first whole-robot routine, so give it clear floor space or leave the robot on blocks. A
   * robot that veers instead of tracking straight has a module that did not reach zero or a drive
   * motor running backwards, which routines 4 and 5 identify. Straight-but-crooked travel, where
   * all four wheels agree but the robot crabs, means the modules are square to each other but their
   * shared zero is not pointing forward, so they need mechanical homing or an offset per module.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command driveStraight(ExampleSubsystem subsystem) {
    MonsterController[] drive = subsystem.driveMotors();
    return Commands.sequence(
            new LogCommand(
                "driveStraight: modules squared to 0, then the whole robot forward and back at "
                    + TestConstants.DRIVE_TEST_SPEED + " for " + ROBOT_DRIVE_SECONDS
                    + "s each; THE ROBOT WILL TRANSLATE, clear the floor or use blocks"),
            PositionMoveCommand.absolute(
                subsystem,
                "square the modules",
                0,
                MonsterController.DEFAULT_POSITION_SPEED,
                MOVE_DWELL_SECONDS,
                subsystem.steerMotors()),
            new LogCommand(
                "  modules should all be pointing the same way now; next is forward, expect the "
                    + "robot to track a straight line"),
            new RunMotorsCommand(
                subsystem, "drive forward", TestConstants.DRIVE_TEST_SPEED, ROBOT_DRIVE_SECONDS,
                drive),
            dwell(),
            new LogCommand("  next is reverse, expect it to retrace the same line backwards"),
            new RunMotorsCommand(
                subsystem, "drive reverse", -TestConstants.DRIVE_TEST_SPEED, ROBOT_DRIVE_SECONDS,
                drive),
            new LogCommand(
                "driveStraight done: equal travel out and back with no veer means the four "
                    + "modules agree with each other"))
        .withName("driveStraight");
  }

  /**
   * Drives the whole robot while sweeping every module through straight, an eighth turn one way,
   * straight again, then an eighth turn the other way.
   *
   * <p>All four modules always share a target, so the robot should crab diagonally rather than
   * curve: the heading stays put while the direction of travel swings side to side. Watch that the
   * change of direction happens on all four corners together and that the wheels keep rolling
   * through it. A module that stalls while turning under load is drawing more current than it can
   * when stationary, which the earlier stationary position routines cannot reveal.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command driveWhileSteering(ExampleSubsystem subsystem) {
    return new DriveWhileSteeringCommand(subsystem);
  }

  /**
   * Turns the robot in place by putting the modules in the X pattern an eighth turn off straight
   * and running the left pair against the right pair.
   *
   * <p>Expect the chassis to rotate about its own centre, first one way and then the other, with
   * the frame staying over the same spot on the floor. This is the routine that proves the drive
   * and steer conventions agree with each other, so read a failure carefully: a robot that
   * translates instead of spinning has a steer sign wrong, and one that fights itself and barely
   * moves has a drive sign wrong. Both assume the modules were homed with zero pointing forward
   * and that counts increase counter-clockwise, neither of which this code can check.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command rotateInPlace(ExampleSubsystem subsystem) {
    return new RotateInPlaceCommand(subsystem);
  }

  /**
   * Chains every routine in its intended order, safest first, with a marker between each.
   *
   * <p>This takes several minutes and ends with the three whole-robot routines, so run it only with
   * the robot on blocks and after the individual routines have been run once each. On blocks the
   * last three still exercise every motor, you just watch the wheels rather than the chassis; put
   * the robot on the floor with space around it and run 17 through 19 on their own to see it drive.
   *
   * @param subsystem the subsystem owning the motors
   * @return the command
   */
  public static Command runAll(ExampleSubsystem subsystem) {
    return Commands.sequence(
            new LogCommand(
                "runAll: " + ROUTINE_COUNT + " routines, software checks first, whole-robot "
                    + "motion last"),
            separator(1, "rangeGuards"),
            rangeGuards(subsystem),
            separator(2, "cachedStateChecks"),
            cachedStateChecks(subsystem),
            separator(3, "identifyDriveMotors"),
            identifyDriveMotors(subsystem),
            separator(4, "identifySteerMotors"),
            identifySteerMotors(subsystem),
            separator(5, "directionCheck"),
            directionCheck(subsystem),
            separator(6, "inversionTest"),
            inversionTest(subsystem),
            separator(7, "stopVsDisable"),
            stopVsDisable(subsystem),
            separator(8, "steerCalibration"),
            steerCalibration(subsystem),
            separator(9, "absolutePositionSweep"),
            absolutePositionSweep(subsystem),
            separator(10, "relativePositionSteps"),
            relativePositionSteps(subsystem),
            separator(11, "positionSpeedTest"),
            positionSpeedTest(subsystem),
            separator(12, "positionInterrupt"),
            positionInterrupt(subsystem),
            separator(13, "commandCoalescing"),
            commandCoalescing(subsystem),
            separator(14, "diagnosticRequests"),
            diagnosticRequests(subsystem),
            separator(15, "moduleSweep"),
            moduleSweep(subsystem),
            separator(16, "allMotorsAtOnce"),
            allMotorsAtOnce(subsystem),
            separator(17, "driveStraight"),
            driveStraight(subsystem),
            separator(18, "driveWhileSteering"),
            driveWhileSteering(subsystem),
            separator(19, "rotateInPlace"),
            rotateInPlace(subsystem),
            new LogCommand("runAll complete"))
        .withName("runAll");
  }

  private static Command identify(
      ExampleSubsystem subsystem,
      MonsterController[] motors,
      String role,
      double speed,
      String expectation) {
    List<Command> steps = new ArrayList<>();
    steps.add(
        new LogCommand(
            "identify " + role + " motors: one at a time at " + speed + "; expect exactly one "
                + expectation + " per step"));
    for (int corner = 0; corner < motors.length; corner++) {
      String label = tag(corner, role, motors[corner]);
      steps.add(
          new LogCommand(
              "  next: " + label + ", expect only the " + CORNER_NAMES[corner] + " "
                  + expectation));
      steps.add(
          new RunMotorsCommand(
              subsystem, "identify " + label, speed, TestConstants.STEP_SECONDS, motors[corner]));
      steps.add(dwell());
    }
    return Commands.sequence(steps.toArray(new Command[0]));
  }

  private static Command relativeStep(
      ExampleSubsystem subsystem, MonsterController[] steer, int offset, String label) {
    return Commands.sequence(
        new LogCommand("  next: relative " + offset + " counts, " + label),
        PositionMoveCommand.relative(
            subsystem,
            "relative " + label,
            offset,
            MonsterController.DEFAULT_POSITION_SPEED,
            MOVE_DWELL_SECONDS,
            steer),
        new LogCommand(() -> "  getTarget() reports " + targetReport(steer)));
  }

  private static Command separator(int index, String name) {
    return new LogCommand("===== " + index + "/" + ROUTINE_COUNT + " " + name + " =====");
  }

  private static Command dwell() {
    return Commands.waitSeconds(TestConstants.DWELL_SECONDS);
  }

  private static Command action(String message, Runnable body) {
    return Commands.runOnce(
        () -> {
          TimedTestCommand.log(message);
          body.run();
        });
  }

  private static String tag(int corner, String role, MonsterController motor) {
    return CORNERS[corner] + " " + role + " (CAN " + motor.getMotorID() + ")";
  }

  private static String steerGroup(MonsterController[] motors) {
    StringBuilder text = new StringBuilder("all four steer motors (CAN ");
    for (int corner = 0; corner < motors.length; corner++) {
      text.append(corner == 0 ? "" : ", ").append(motors[corner].getMotorID());
    }
    return text.append(")").toString();
  }

  private static String targetReport(MonsterController[] motors) {
    StringBuilder text = new StringBuilder();
    for (int corner = 0; corner < motors.length; corner++) {
      text.append(corner == 0 ? "" : ", ")
          .append(CORNERS[corner])
          .append("=")
          .append(motors[corner].getTarget());
    }
    return text.toString();
  }

  private static boolean check(String what, boolean condition) {
    TimedTestCommand.log((condition ? "  PASS " : "  FAIL ") + what);
    return condition;
  }

  private static boolean expectIllegalArgument(String what, Runnable body) {
    try {
      body.run();
    } catch (IllegalArgumentException e) {
      return check(what + " rejected: " + e.getMessage(), true);
    } catch (RuntimeException e) {
      return check(
          what + " threw " + e.getClass().getSimpleName() + " instead of IllegalArgumentException",
          false);
    }
    return check(what + " was accepted, so nothing guards it", false);
  }

  /** Reports how many assertions of a self-verifying routine passed. */
  private static void summarize(String name, int passed, int total) {
    TimedTestCommand.log(
        (passed == total ? "  RESULT PASS " : "  RESULT FAIL ")
            + name
            + " "
            + passed
            + "/"
            + total
            + " assertions");
  }

  private static final class RangeGuardsCommand extends TimedTestCommand {
    private RangeGuardsCommand(ExampleSubsystem subsystem) {
      super(subsystem, "rangeGuards", SOFTWARE_TEST_SECONDS);
    }

    @Override
    protected void onStart() {
      MonsterController motor = m_subsystem.fl_drive;
      log("  no motion expected: each call below is rejected before anything is buffered");
      int passed = 0;
      if (expectIllegalArgument(
          "set(1.5) on " + tag(0, "drive", motor) + ", which scales to 375, past the board limit "
              + "of 250",
          () -> motor.set(1.5))) {
        passed++;
      }
      if (expectIllegalArgument(
          "setPosition(" + QUARTER_TURN + ", 300) with a speed past the board limit",
          () -> motor.setPosition(QUARTER_TURN, 300))) {
        passed++;
      }
      if (expectIllegalArgument(
          "constructing a controller for motor ID 8 when the board has 0 through 7",
          () -> new MonsterController(8))) {
        passed++;
      }

      // set(1.5) cached its argument before throwing; this clears it and leaves a stop buffered.
      motor.set(0);
      summarize("rangeGuards", passed, 3);
    }
  }

  private static final class CachedStateChecksCommand extends TimedTestCommand {
    private static final double SAMPLE_SPEED = 0.42;
    private static final int SAMPLE_TARGET = 777;

    private CachedStateChecksCommand(ExampleSubsystem subsystem) {
      super(subsystem, "cachedStateChecks", SOFTWARE_TEST_SECONDS);
    }

    @Override
    protected void onStart() {
      MonsterController motor = m_subsystem.fl_drive;
      log(
          "  no motion expected: these writes and the closing stop share one cycle, and the "
              + "buffer keeps only the last command per motor");
      int passed = 0;

      motor.set(SAMPLE_SPEED);
      if (check("get() returns the last speed passed to set()", motor.get() == SAMPLE_SPEED)) {
        passed++;
      }

      motor.setInverted(true);
      if (check("getInverted() is true after setInverted(true)", motor.getInverted())) {
        passed++;
      }

      motor.setPosition(SAMPLE_TARGET);
      if (check(
          "getTarget() reports the requested target, not the inverted one it sent",
          motor.getTarget() == SAMPLE_TARGET)) {
        passed++;
      }

      motor.setInverted(false);
      if (check("getInverted() is false after setInverted(false)", !motor.getInverted())) {
        passed++;
      }

      motor.setPosition(-SAMPLE_TARGET);
      if (check(
          "getTarget() tracks the newest setPosition()", motor.getTarget() == -SAMPLE_TARGET)) {
        passed++;
      }

      motor.set(0);
      summarize("cachedStateChecks", passed, 5);
    }
  }

  /**
   * Spins one motor, releases it with STOP or DISABLE part way through, and optionally re-sends a
   * speed afterwards to see whether the board comes back on its own.
   *
   * <p>All of that lives in one command because the release and the observation window have to
   * happen before the base class stops every motor at the end of the step.
   */
  private static final class StopVsDisableCommand extends TimedTestCommand {
    private final MonsterController m_motor;
    private final double m_speed;
    private final double m_releaseAt;
    private final boolean m_disable;
    private final double m_retryAt;
    private boolean m_released;
    private boolean m_retried;

    private StopVsDisableCommand(
        ExampleSubsystem subsystem,
        String name,
        MonsterController motor,
        double speed,
        double releaseAt,
        boolean disable,
        double retryAt,
        double durationSeconds) {
      super(subsystem, name, durationSeconds);
      m_motor = motor;
      m_speed = speed;
      m_releaseAt = releaseAt;
      m_disable = disable;
      m_retryAt = retryAt;
    }

    @Override
    protected void onStart() {
      log(
          "  spinning " + tag(0, "drive", m_motor) + " at " + m_speed + " for " + m_releaseAt
              + "s, then " + (m_disable ? "DISABLE" : "STOP"));
      m_motor.set(m_speed);
    }

    @Override
    protected void onRun(double elapsedSeconds) {
      if (!m_released && elapsedSeconds >= m_releaseAt) {
        m_released = true;
        if (m_disable) {
          m_motor.disable();
          log("  DISABLE sent: driver de-energized, expect a free coast down with no braking");
        } else {
          m_motor.stopMotor();
          log("  STOP sent: driver still energized, expect a braked stop that resists by hand");
        }
      }
      if (m_retryAt > 0 && !m_retried && elapsedSeconds >= m_retryAt) {
        m_retried = true;
        m_motor.set(m_speed);
        log(
            "  speed re-sent with no explicit re-enable: spinning back up means the board "
                + "re-energizes on motion traffic, staying dead means it needs an enable first");
      }
    }
  }

  private static final class PositionInterruptCommand extends TimedTestCommand {
    private static final int LONG_TARGET = 2 * TestConstants.STEER_COUNTS_PER_REV;
    private static final int TRAVEL_SPEED = 60;
    private static final double STOP_AT_SECONDS = 1.0;

    private boolean m_stopped;

    private PositionInterruptCommand(ExampleSubsystem subsystem) {
      super(subsystem, "positionInterrupt", STOP_AT_SECONDS + 2 * TestConstants.DWELL_SECONDS);
    }

    @Override
    protected void onStart() {
      MonsterController steer = m_subsystem.fl_steer;
      log(
          "  " + tag(0, "steer", steer) + " to absolute " + LONG_TARGET + " at speed "
              + TRAVEL_SPEED + ", a move far too long to finish before the interrupt");
      steer.setPosition(LONG_TARGET, TRAVEL_SPEED);
    }

    @Override
    protected void onRun(double elapsedSeconds) {
      if (!m_stopped && elapsedSeconds >= STOP_AT_SECONDS) {
        m_stopped = true;
        m_subsystem.fl_steer.stopMotor();
        log(
            "  STOP sent mid-flight: expect the module to halt now and stay put; resuming the "
                + "move would mean STOP loses to queued position traffic");
      }
    }
  }

  private static final class CommandCoalescingCommand extends TimedTestCommand {
    private static final double FIRST_SPEED = 0.15;
    private static final double SECOND_SPEED = 0.6;

    private CommandCoalescingCommand(ExampleSubsystem subsystem) {
      super(subsystem, "commandCoalescing", TestConstants.STEP_SECONDS);
    }

    @Override
    protected void onStart() {
      MonsterController motor = m_subsystem.fl_drive;
      motor.set(FIRST_SPEED);
      motor.set(SECOND_SPEED);
      log(
          "  " + tag(0, "drive", motor) + " told " + FIRST_SPEED + " then " + SECOND_SPEED
              + " in the same cycle; expect it to come straight up to " + SECOND_SPEED
              + " with no blip at " + FIRST_SPEED + ", since the buffer holds one command per "
              + "motor");
    }
  }

  private static final class DiagnosticRequestsCommand extends TimedTestCommand {
    private double m_nextRequestAt;
    private int m_rounds;

    private DiagnosticRequestsCommand(ExampleSubsystem subsystem) {
      super(subsystem, "diagnosticRequests", DIAGNOSTIC_SECONDS);
    }

    @Override
    protected void onStart() {
      m_nextRequestAt = 0;
      m_rounds = 0;
      log("  no motion expected: status and encoder requests only, roughly once a second");
      log(
          "  nothing here reads replies, so confirm the frames on a CAN sniffer or the board's "
              + "own indicators");
      log(
          "  note: request_encoders() guards against motor 0 being unregistered, but "
              + "ExampleSubsystem registers CAN 0 in its constructor, so that exception cannot "
              + "happen once the robot is up and is not asserted here");
    }

    @Override
    protected void onRun(double elapsedSeconds) {
      if (elapsedSeconds < m_nextRequestAt) {
        return;
      }
      m_nextRequestAt = elapsedSeconds + DIAGNOSTIC_PERIOD_SECONDS;
      m_rounds++;
      for (MonsterController motor : m_subsystem.allMotors()) {
        motor.requestStatus();
      }
      MonsterProtocol.getInstance().request_encoders();
      log(
          "  round " + m_rounds + ": status requested on CAN 0-7 plus one encoder request, which "
              + "takes motor 0's buffer slot, so seven status frames and one encoder frame go out");
    }
  }

  /**
   * Runs every drive motor while every steer motor works between two angles.
   *
   * <p>One command rather than a composition: commands that require the subsystem interrupt each
   * other, and the point of the routine is to have all eight frames buffered in the same cycle.
   */
  private static final class AllMotorsAtOnceCommand extends TimedTestCommand {
    private static final int PHASES = 4;

    private int m_phase = -1;

    private AllMotorsAtOnceCommand(ExampleSubsystem subsystem) {
      super(subsystem, "allMotorsAtOnce", PHASES * TestConstants.STEP_SECONDS);
    }

    @Override
    protected void onStart() {
      log(
          "  all four wheels at " + TestConstants.DRIVE_TEST_SPEED + " while all four modules "
              + "alternate between 0 and " + QUARTER_TURN + " counts every "
              + TestConstants.STEP_SECONDS + "s");
      log(
          "  every motor is re-commanded each cycle, so eight frames go out per flush; watch for "
              + "a wheel that stutters or a module that lags the other three");
      commandAll(0);
    }

    @Override
    protected void onRun(double elapsedSeconds) {
      int phase = (int) (elapsedSeconds / TestConstants.STEP_SECONDS);
      if (phase != m_phase) {
        m_phase = phase;
        log("  phase " + phase + ": steer target " + targetFor(phase));
      }
      commandAll(phase);
    }

    private void commandAll(int phase) {
      int target = targetFor(phase);
      for (MonsterController motor : m_subsystem.driveMotors()) {
        motor.set(TestConstants.DRIVE_TEST_SPEED);
      }
      for (MonsterController motor : m_subsystem.steerMotors()) {
        motor.setPosition(target, MonsterController.DEFAULT_POSITION_SPEED);
      }
    }

    private static int targetFor(int phase) {
      return phase % 2 == 0 ? QUARTER_TURN : 0;
    }
  }

  /**
   * Holds every wheel rolling while the modules swing between straight and an eighth turn either
   * side of it.
   *
   * <p>One command rather than a composition, since the drive motors have to keep running across
   * the steering changes and two commands requiring the subsystem would interrupt each other. The
   * drive speed is re-sent every cycle to hold it through the phase changes; the steer target is
   * only re-sent when the phase changes, so the board is left to finish a move rather than being
   * handed the same target sixty times a second.
   */
  private static final class DriveWhileSteeringCommand extends TimedTestCommand {
    private static final int[] TARGETS = {0, EIGHTH_TURN, 0, -EIGHTH_TURN};
    private static final double PHASE_SECONDS = 2.0;

    private int m_phase = -1;

    private DriveWhileSteeringCommand(ExampleSubsystem subsystem) {
      super(subsystem, "driveWhileSteering", TARGETS.length * PHASE_SECONDS);
    }

    @Override
    protected void onStart() {
      log(
          "  every wheel at " + TestConstants.DRIVE_TEST_SPEED + " throughout, while all four "
              + "modules move together through 0, " + EIGHTH_TURN + ", 0 and " + -EIGHTH_TURN
              + " counts every " + PHASE_SECONDS + "s");
      log(
          "  THE ROBOT WILL MOVE: expect it to crab, changing the direction it travels without "
              + "changing the way it faces");
    }

    @Override
    protected void onRun(double elapsedSeconds) {
      for (MonsterController motor : m_subsystem.driveMotors()) {
        motor.set(TestConstants.DRIVE_TEST_SPEED);
      }

      int phase = Math.min((int) (elapsedSeconds / PHASE_SECONDS), TARGETS.length - 1);
      if (phase == m_phase) {
        return;
      }
      m_phase = phase;
      log("  phase " + (phase + 1) + "/" + TARGETS.length + ": steer to " + TARGETS[phase]);
      for (MonsterController motor : m_subsystem.steerMotors()) {
        motor.setPosition(TARGETS[phase], MonsterController.DEFAULT_POSITION_SPEED);
      }
    }
  }

  /**
   * Rotates the chassis in place: settle the modules into the X pattern, spin one way, pause, then
   * spin back.
   *
   * <p>One command rather than a composition because the modules have to hold the X pattern while
   * the drive motors run, and because each corner needs its own steer target, which a single {@link
   * PositionMoveCommand} cannot express.
   */
  private static final class RotateInPlaceCommand extends TimedTestCommand {
    private static final double SETTLE_SECONDS = MOVE_DWELL_SECONDS;
    private static final double SPIN_SECONDS = ROBOT_DRIVE_SECONDS;
    private static final double PAUSE_SECONDS = TestConstants.DWELL_SECONDS;

    private int m_phase = -1;

    private RotateInPlaceCommand(ExampleSubsystem subsystem) {
      super(
          subsystem,
          "rotateInPlace",
          SETTLE_SECONDS + SPIN_SECONDS + PAUSE_SECONDS + SPIN_SECONDS);
    }

    @Override
    protected void onStart() {
      log(
          "  modules to the X pattern (FL " + ROTATION_STEER_TARGETS[0] + ", FR "
              + ROTATION_STEER_TARGETS[1] + ", BL " + ROTATION_STEER_TARGETS[2] + ", BR "
              + ROTATION_STEER_TARGETS[3] + " counts), then the left pair against the right pair");
      log(
          "  THE ROBOT WILL SPIN about its own centre, " + SPIN_SECONDS + "s each way; expect the "
              + "frame to stay over the same spot on the floor");
      applySteer();
    }

    @Override
    protected void onRun(double elapsedSeconds) {
      int phase = phaseFor(elapsedSeconds);
      if (phase != m_phase) {
        m_phase = phase;
        applySteer();
        log("  " + describe(phase));
      }

      double[] signs = ROTATION_DRIVE_SIGNS;
      MonsterController[] drive = m_subsystem.driveMotors();
      for (int corner = 0; corner < drive.length; corner++) {
        drive[corner].set(driveScale(phase) * signs[corner] * TestConstants.DRIVE_TEST_SPEED);
      }
    }

    private void applySteer() {
      MonsterController[] steer = m_subsystem.steerMotors();
      for (int corner = 0; corner < steer.length; corner++) {
        steer[corner].setPosition(
            ROTATION_STEER_TARGETS[corner], MonsterController.DEFAULT_POSITION_SPEED);
      }
    }

    private static int phaseFor(double elapsedSeconds) {
      if (elapsedSeconds < SETTLE_SECONDS) {
        return 0;
      }
      if (elapsedSeconds < SETTLE_SECONDS + SPIN_SECONDS) {
        return 1;
      }
      if (elapsedSeconds < SETTLE_SECONDS + SPIN_SECONDS + PAUSE_SECONDS) {
        return 2;
      }
      return 3;
    }

    private static double driveScale(int phase) {
      switch (phase) {
        case 1:
          return 1.0;
        case 3:
          return -1.0;
        default:
          return 0.0;
      }
    }

    private static String describe(int phase) {
      switch (phase) {
        case 0:
          return "settling the modules into the X pattern, wheels stopped";
        case 1:
          return "spinning one way";
        case 2:
          return "pausing, holding the X pattern";
        default:
          return "spinning back the other way, expect it to return to where it started";
      }
    }
  }

  private SwerveTests() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
