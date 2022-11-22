// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
public final class DriveConstants {

    public static final double teleDriveMaxAccelerationPerSecond = 0.0;
    public static final double teleDriveMaxAngularAccelerationPerSecond = 0.0;

    public static final double teleDriveMaxSpeedMetersPerSecond = 0.0;
    public static final double teleDriveMaxAngularSpeedRadiansPerSecond = 0.0;

    public static final double trackWidth = Units.inchesToMeters(0);
    public static final double wheelBase = Units.inchesToMeters(0);

    public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(new Translation2d(wheelBase / 2, -trackWidth / 2) , new Translation2d(wheelBase / 2, trackWidth / 2), new Translation2d(-wheelBase / 2, -trackWidth / 2), new Translation2d(-wheelBase / 2, trackWidth / 2)); 

}