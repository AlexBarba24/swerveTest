// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.tests.SwerveTests;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SendableChooser<Command> m_testChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureTestChooser();
  }

  /**
   * Publishes the motor controller test suite to the dashboard. The routines are listed in the
   * order they are meant to be run, and the default selection is the one that commands no motion.
   * The last three drive the chassis rather than individual motors, so they are labelled to make
   * that obvious before anyone enables them.
   */
  private void configureTestChooser() {
    m_testChooser.setDefaultOption(
        "1. Range guards (no motion)", SwerveTests.rangeGuards(m_exampleSubsystem));
    m_testChooser.addOption(
        "2. Cached state checks (no motion)", SwerveTests.cachedStateChecks(m_exampleSubsystem));
    m_testChooser.addOption(
        "3. Identify drive motors", SwerveTests.identifyDriveMotors(m_exampleSubsystem));
    m_testChooser.addOption(
        "4. Identify steer motors", SwerveTests.identifySteerMotors(m_exampleSubsystem));
    m_testChooser.addOption("5. Direction check", SwerveTests.directionCheck(m_exampleSubsystem));
    m_testChooser.addOption("6. Inversion test", SwerveTests.inversionTest(m_exampleSubsystem));
    m_testChooser.addOption("7. Stop vs disable", SwerveTests.stopVsDisable(m_exampleSubsystem));
    m_testChooser.addOption(
        "8. Steer calibration", SwerveTests.steerCalibration(m_exampleSubsystem));
    m_testChooser.addOption(
        "9. Absolute position sweep", SwerveTests.absolutePositionSweep(m_exampleSubsystem));
    m_testChooser.addOption(
        "10. Relative position steps", SwerveTests.relativePositionSteps(m_exampleSubsystem));
    m_testChooser.addOption(
        "11. Position speed test", SwerveTests.positionSpeedTest(m_exampleSubsystem));
    m_testChooser.addOption(
        "12. Position interrupt", SwerveTests.positionInterrupt(m_exampleSubsystem));
    m_testChooser.addOption(
        "13. Command coalescing", SwerveTests.commandCoalescing(m_exampleSubsystem));
    m_testChooser.addOption(
        "14. Diagnostic requests", SwerveTests.diagnosticRequests(m_exampleSubsystem));
    m_testChooser.addOption("15. Module sweep", SwerveTests.moduleSweep(m_exampleSubsystem));
    m_testChooser.addOption(
        "16. All motors at once", SwerveTests.allMotorsAtOnce(m_exampleSubsystem));
    m_testChooser.addOption(
        "17. Drive straight (ROBOT MOVES)", SwerveTests.driveStraight(m_exampleSubsystem));
    m_testChooser.addOption(
        "18. Drive while steering (ROBOT MOVES)",
        SwerveTests.driveWhileSteering(m_exampleSubsystem));
    m_testChooser.addOption(
        "19. Rotate in place (ROBOT MOVES)", SwerveTests.rotateInPlace(m_exampleSubsystem));
    m_testChooser.addOption("Run all (1-19)", SwerveTests.runAll(m_exampleSubsystem));

    SmartDashboard.putData("Test Routine", m_testChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the test routine selected on the dashboard
   */
  public Command getAutonomousCommand() {
    return m_testChooser.getSelected();
  }
}
