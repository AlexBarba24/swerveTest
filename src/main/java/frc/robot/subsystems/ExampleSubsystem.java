// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.MonsterController;

public class ExampleSubsystem extends SubsystemBase {

  public MonsterController fl_drive;
  public MonsterController fr_drive;
  public MonsterController bl_drive;
  public MonsterController br_drive;

  public MonsterController fl_steer;
  public MonsterController fr_steer;
  public MonsterController bl_steer;
  public MonsterController br_steer;

  private final MonsterController[] m_driveMotors;
  private final MonsterController[] m_steerMotors;
  private final MonsterController[] m_allMotors;

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    fl_drive = new MonsterController(Constants.FL_DRIVE_CAN);
    fr_drive = new MonsterController(Constants.FR_DRIVE_CAN);
    bl_drive = new MonsterController(Constants.BL_DRIVE_CAN);
    br_drive = new MonsterController(Constants.BR_DRIVE_CAN);
    fl_steer = new MonsterController(Constants.FL_STEER_CAN);
    fr_steer = new MonsterController(Constants.FR_STEER_CAN);
    bl_steer = new MonsterController(Constants.BL_STEER_CAN);
    br_steer = new MonsterController(Constants.BR_STEER_CAN);

    m_driveMotors = new MonsterController[] {fl_drive, fr_drive, bl_drive, br_drive};
    m_steerMotors = new MonsterController[] {fl_steer, fr_steer, bl_steer, br_steer};
    m_allMotors =
        new MonsterController[] {
          fl_drive, fr_drive, bl_drive, br_drive, fl_steer, fr_steer, bl_steer, br_steer
        };
    bl_drive.setInverted(true);
    br_drive.setInverted(true);
    fr_steer.setInverted(true);
    bl_steer.setInverted(true);
  }

  /**
   * The four drive motors in FL, FR, BL, BR order.
   *
   * @return the drive motor controllers
   */
  public MonsterController[] driveMotors() {
    return m_driveMotors;
  }

  /**
   * The four steer motors in FL, FR, BL, BR order.
   *
   * @return the steer motor controllers
   */
  public MonsterController[] steerMotors() {
    return m_steerMotors;
  }

  /**
   * All eight motors, drive motors first, each group in FL, FR, BL, BR order.
   *
   * @return every motor controller on the board
   */
  public MonsterController[] allMotors() {
    return m_allMotors;
  }

  /**
   * Runs every drive motor at the same speed.
   *
   * @param speed normalized speed, -1.0 to 1.0
   */
  public void setAllDrive(double speed) {
    for (MonsterController motor : m_driveMotors) {
      motor.set(speed);
    }
  }

  /**
   * Runs every steer motor at the same speed.
   *
   * @param speed normalized speed, -1.0 to 1.0
   */
  public void setAllSteer(double speed) {
    for (MonsterController motor : m_steerMotors) {
      motor.set(speed);
    }
  }

  /** De-energizes every motor driver, unlike {@link #stopMotors()} which leaves them powered. */
  public void disableAll() {
    for (MonsterController motor : m_allMotors) {
      motor.disable();
    }
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void stopMotors() {
    fl_drive.stopMotor();
    fr_drive.stopMotor();
    bl_drive.stopMotor();
    br_drive.stopMotor();
    fl_steer.stopMotor();
    fr_steer.stopMotor();
    bl_steer.stopMotor();
    br_steer.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
