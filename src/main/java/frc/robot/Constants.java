// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  /** Tunables for the motor controller API test routines. */
  public static class TestConstants {
    /** Normalized speed used when spinning a drive motor during a test. */
    public static final double DRIVE_TEST_SPEED = 0.3;

    /** Normalized speed used when spinning a steer motor during a test. */
    public static final double STEER_TEST_SPEED = 0.2;

    /** How long a single motion step of a test runs, in seconds. */
    public static final double STEP_SECONDS = 1.5;

    /** How long a test pauses between steps so the motion can be observed, in seconds. */
    public static final double DWELL_SECONDS = 1.0;

    /**
     * Encoder counts for one full revolution of a steer motor. This is a placeholder: run the
     * steerCalibration routine, measure the actual rotation, and correct this value before
     * relying on the other position tests.
     */
    public static final int STEER_COUNTS_PER_REV = 3600;
  }

  public static final int FL_DRIVE_CAN = 0;
  public static final int FR_DRIVE_CAN = 2;
  public static final int BL_DRIVE_CAN = 4;
  public static final int BR_DRIVE_CAN = 6;
  public static final int FL_STEER_CAN = 1;
  public static final int FR_STEER_CAN = 3;
  public static final int BL_STEER_CAN = 5;
  public static final int BR_STEER_CAN = 7;
}
