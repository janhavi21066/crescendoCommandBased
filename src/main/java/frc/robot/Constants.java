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
    public static final double rotationsToDegrees = 360 / 512 * 26 / 42 * 18 / 60 * 18 / 84;
    public static final double rotationsToFeet = (1.0 / 4096 * 6 * Math.PI / 12) / 3.281;
    public static final double trackWidthInMeters = 0.55;
    public static final double armOuttakePos = 1258;
    public static final double armIntakePos = 0;
    public static final double KP_PIDArm = 0.3;
    public static final double KI_PIDArm = 0;
    public static final double KD_PIDArm = 0;
    public static final double intakeInSpeed = -1;
    public static final double intakeOutSpeed = 1;
    public static final double outtakeOutSpeed = 0.3;
    public static final double outtakeInSpeed = -1;
    public static final double feederOuttake = 0.7;
  }
}
