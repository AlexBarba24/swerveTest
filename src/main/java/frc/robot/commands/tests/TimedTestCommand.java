// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * Base class for every motion test routine.
 *
 * <p>Timing is measured with a {@link Timer} restarted in {@link #initialize()}, so a command
 * built once at startup still runs for its full duration whenever it is scheduled. Subclasses
 * customize behavior through the {@link #onStart()}, {@link #onRun(double)}, and {@link
 * #onEnd(boolean)} hooks rather than by overriding the lifecycle methods, which guarantees that
 * the motors are stopped and the outcome is logged even when the command is interrupted.
 *
 * <p>Every test requires the subsystem, so two of them cannot be composed in parallel; chain them
 * with {@code Commands.sequence} instead.
 */
public abstract class TimedTestCommand extends Command {
  /** Dashboard key carrying the most recent log line. */
  public static final String STATUS_KEY = "Test/Status";

  /** Dashboard key carrying the name of the test that started most recently. */
  public static final String NAME_KEY = "Test/Name";

  /** The subsystem owning the motors under test. */
  protected final ExampleSubsystem m_subsystem;

  private final String m_name;
  private final double m_durationSeconds;
  private final Timer m_timer = new Timer();

  /**
   * Creates a timed test.
   *
   * @param subsystem the subsystem owning the motors under test
   * @param name label used in log lines
   * @param durationSeconds how long the test runs before finishing on its own
   */
  protected TimedTestCommand(ExampleSubsystem subsystem, String name, double durationSeconds) {
    m_subsystem = subsystem;
    m_name = name;
    m_durationSeconds = durationSeconds;
    setName(name);
    addRequirements(subsystem);
  }

  /**
   * Writes a line to the console and the dashboard.
   *
   * @param message the line to publish
   */
  public static void log(String message) {
    System.out.println("[TEST] " + message);
    SmartDashboard.putString(STATUS_KEY, message);
  }

  /** Issues the commands the test is here to exercise. Called once, after the timer starts. */
  protected void onStart() {}

  /**
   * Called every scheduler cycle while the test is running.
   *
   * @param elapsedSeconds seconds since {@link #initialize()}
   */
  protected void onRun(double elapsedSeconds) {}

  /**
   * Called after the motors have been stopped and the outcome logged.
   *
   * @param interrupted true if the test was cancelled rather than finishing on time
   */
  protected void onEnd(boolean interrupted) {}

  /**
   * How long the test runs before {@link #isFinished()} returns true.
   *
   * @return the duration in seconds
   */
  public double getDurationSeconds() {
    return m_durationSeconds;
  }

  /**
   * Seconds since the test started.
   *
   * @return the elapsed time in seconds
   */
  protected double elapsedSeconds() {
    return m_timer.get();
  }

  @Override
  public final void initialize() {
    m_timer.restart();
    SmartDashboard.putString(NAME_KEY, m_name);
    log("START " + m_name);
    onStart();
  }

  @Override
  public final void execute() {
    onRun(m_timer.get());
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_durationSeconds);
  }

  @Override
  public final void end(boolean interrupted) {
    m_subsystem.stopMotors();
    log((interrupted ? "INTERRUPTED " : "END ") + m_name);
    onEnd(interrupted);
  }
}
