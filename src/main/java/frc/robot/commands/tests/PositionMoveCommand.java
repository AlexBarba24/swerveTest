// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import frc.robot.lib.MonsterController;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * Issues one position move to a set of motors and then dwells so the travel can be observed.
 *
 * <p>The move is commanded once at the start; the rest of the command is the dwell. The board owns
 * the motion profile, so nothing here waits for the motor to arrive: the dwell has to be long
 * enough to cover the travel, because the base class stops the motors when the command ends.
 */
public class PositionMoveCommand extends TimedTestCommand {
  private final MonsterController[] m_motors;
  private final int m_target;
  private final int m_speed;
  private final boolean m_relative;

  /**
   * Creates a position move.
   *
   * @param subsystem the subsystem owning the motors
   * @param name label used in log lines
   * @param target absolute target, or offset from the current position when relative
   * @param speed board speed to travel at, -250 to 250
   * @param relative true for a relative move, false for an absolute one
   * @param dwellSeconds how long to hold before the command finishes
   * @param motors the motors to move
   */
  public PositionMoveCommand(
      ExampleSubsystem subsystem,
      String name,
      int target,
      int speed,
      boolean relative,
      double dwellSeconds,
      MonsterController... motors) {
    super(subsystem, name, dwellSeconds);
    m_motors = motors;
    m_target = target;
    m_speed = speed;
    m_relative = relative;
  }

  /**
   * Creates an absolute move.
   *
   * @param subsystem the subsystem owning the motors
   * @param name label used in log lines
   * @param target absolute encoder target in board counts
   * @param speed board speed to travel at, -250 to 250
   * @param dwellSeconds how long to hold before the command finishes
   * @param motors the motors to move
   * @return the command
   */
  public static PositionMoveCommand absolute(
      ExampleSubsystem subsystem,
      String name,
      int target,
      int speed,
      double dwellSeconds,
      MonsterController... motors) {
    return new PositionMoveCommand(subsystem, name, target, speed, false, dwellSeconds, motors);
  }

  /**
   * Creates a relative move.
   *
   * @param subsystem the subsystem owning the motors
   * @param name label used in log lines
   * @param offset encoder counts to travel from the current position
   * @param speed board speed to travel at, -250 to 250
   * @param dwellSeconds how long to hold before the command finishes
   * @param motors the motors to move
   * @return the command
   */
  public static PositionMoveCommand relative(
      ExampleSubsystem subsystem,
      String name,
      int offset,
      int speed,
      double dwellSeconds,
      MonsterController... motors) {
    return new PositionMoveCommand(subsystem, name, offset, speed, true, dwellSeconds, motors);
  }

  @Override
  protected void onStart() {
    for (MonsterController motor : m_motors) {
      if (m_relative) {
        motor.setRelativePosition(m_target, m_speed);
      } else {
        motor.setPosition(m_target, m_speed);
      }
      log(
          "  motor "
              + motor.getMotorID()
              + (m_relative ? " relative " : " absolute ")
              + m_target
              + " at speed "
              + m_speed);
    }
  }
}
