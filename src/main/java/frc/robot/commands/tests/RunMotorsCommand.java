// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import com.vendor.MonsterController;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * Runs any set of motors at one speed for a fixed time.
 *
 * <p>The speed is issued once at the start, since the board holds the last speed it received. Pass
 * {@code continuous} to re-issue it every cycle instead, which is what the bus-loading tests want.
 */
public class RunMotorsCommand extends TimedTestCommand {
  private final MonsterController[] m_motors;
  private final double m_speed;
  private final boolean m_continuous;

  /**
   * Runs the given motors, issuing the speed once.
   *
   * @param subsystem the subsystem owning the motors
   * @param name label used in log lines
   * @param speed normalized speed, -1.0 to 1.0
   * @param durationSeconds how long to run before stopping
   * @param motors the motors to run
   */
  public RunMotorsCommand(
      ExampleSubsystem subsystem,
      String name,
      double speed,
      double durationSeconds,
      MonsterController... motors) {
    this(subsystem, name, speed, durationSeconds, false, motors);
  }

  /**
   * Runs the given motors, optionally re-issuing the speed every cycle.
   *
   * @param subsystem the subsystem owning the motors
   * @param name label used in log lines
   * @param speed normalized speed, -1.0 to 1.0
   * @param durationSeconds how long to run before stopping
   * @param continuous true to send a fresh speed frame every scheduler cycle
   * @param motors the motors to run
   */
  public RunMotorsCommand(
      ExampleSubsystem subsystem,
      String name,
      double speed,
      double durationSeconds,
      boolean continuous,
      MonsterController[] motors) {
    super(subsystem, name, durationSeconds);
    m_motors = motors;
    m_speed = speed;
    m_continuous = continuous;
  }

  @Override
  protected void onStart() {
    log(describe());
    setAll();
  }

  @Override
  protected void onRun(double elapsedSeconds) {
    if (m_continuous) {
      setAll();
    }
  }

  private void setAll() {
    for (MonsterController motor : m_motors) {
      motor.set(m_speed);
    }
  }

  private String describe() {
    StringBuilder ids = new StringBuilder();
    for (MonsterController motor : m_motors) {
      if (ids.length() > 0) {
        ids.append(", ");
      }
      ids.append(motor.getMotorID());
    }
    return "  motors [" + ids + "] at " + m_speed + " for " + getDurationSeconds() + "s";
  }
}
