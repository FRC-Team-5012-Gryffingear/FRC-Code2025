// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Unit;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // TRACKWIDTH is the distance between front 2 modules
  // WHEELBASE IS THE DISTANCE BETWEEN FRONT TO THE BACK MODULES
  public static final double trackWidth = Units.inchesToMeters(0);
  public static final double wheelBase = Units.inchesToMeters(0);

  // Max velo using RPM/Gear Ratio * pi * 4(diameter)/12(feet) * 1/60(min/seconds?)
  // Assuming we are using same Swerve
  public static final double Max_velo = 6380 / 6.75 * Math.PI * 4 / 12 / 60;
  public static final int Max_voltage = 12;
  public static final double Max_Angular = Max_velo / Math.hypot(trackWidth/2, wheelBase/2);

  // IDS for swerve drive
  public static final int FrontLeftDriveID = 0;
  public static final int FrontRightDriveID = 0;
  public static final int BackLeftDriveID = 0;
  public static final int BackRightDriveID = 0;

  public static final int FrontLeftSteerID = 0;
  public static final int FrontRightSteerID = 0;
  public static final int BackLeftSteerID = 0;
  public static final int BackRightSteerID = 0;

  public static final int FrontLeftEncoderID = 0;
  public static final int FrontRightEncoderID = 0;
  public static final int BackLeftEncoderID = 0;
  public static final int BackRightEncoderID = 0;

  // Offset should be grabbed through tunerX
  public static final double FrontLeftOffset = 0;
  public static final double FrontRightOffset = 0;
  public static final double BackLeftOffset = 0;
  public static final double BackRightOffset = 0;

  // True inverses motors
  public static final boolean FrontLeftInv = false;
  public static final boolean FrontRightInv = false;
  public static final boolean BackLeftInv = false;
  public static final boolean backRightInv = false;

  public static final int PigeonID = 0;

  // Gives position of modules on a 2d plane
  public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(wheelBase/2, -trackWidth/2), // FR
    new Translation2d(wheelBase/2, trackWidth/2), // FL
    new Translation2d(-wheelBase/2, -trackWidth/2), //BR
    new Translation2d(-wheelBase/2, trackWidth/2) //BL
  );

  public static class ModConstants{
    // Hopefully no need to adjust
    public static final double KP = 0.4;
    public static final double KI = 0;
    public static final double KD = 0;
  }

  public static class OperatorConstants{
    public static final int DriverContrlPort = 0;
    public static final double Deadband = 0.05;

    public static final int OperatorContrlPort = 1;
  } 

}
