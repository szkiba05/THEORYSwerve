package frc.robot.subsystems;

import com.kauailabs.navx.frc.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;


public class SwerveDriveSubsystem extends SubsystemBase {

    private SwerveModule rightFront;
    private SwerveModule rightBack;

    private SwerveModule leftFront;
    private SwerveModule leftBack;

    private AHRS gyro;

    private SwerveDriveOdometry odometry;

    /** Creates a new ExampleSubsystem. */
    public SwerveDriveSubsystem() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    // Gyro Methods

    public void tareHeading()
    {

        gyro.reset();

    }

    public double getHeading()
    {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    // Odometry Getters

    public Rotation2d getRotation2d()
    {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose2d()
    {
        return odometry.getPoseMeters();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.teleDriveMaxSpeedMetersPerSecond);

        rightFront.setModuleState(desiredStates[0]);
        rightBack.setModuleState(desiredStates[1]);
        leftFront.setModuleState(desiredStates[2]);
        leftBack.setModuleState(desiredStates[3]);
    }

    // Reset Odometry

    public void resetOdometry(Pose2d pose)
    {
        odometry.resetPosition(pose, getRotation2d());
    }

    // Stop all modules

    public void stopAllModules()
    {
        rightFront.stop();
        rightBack.stop();

        leftFront.stop();
        leftBack.stop();
    }




}
