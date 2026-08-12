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
