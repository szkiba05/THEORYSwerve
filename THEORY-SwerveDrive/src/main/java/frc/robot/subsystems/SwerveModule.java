package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.DriveConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule 
{

    // Motors for each module
    private WPI_TalonFX driveMotor;
    private WPI_TalonFX turnMotor;

    // Turning motor PID - Helps to correct for error in the turning motor and setting it to a desired postion accurately
    private PIDController turningPIDController;
    private PIDController velocityPIDController;


    private AnalogInput absoluteEncoder;

    private boolean driveMotorReversed;
    private boolean turnMotorReversed;

    private boolean absoluteEncoderReversed;


    public SwerveModule(int driveMotorID, int turnMotorID, int absoluteEncoderID, boolean driveMotorReversed, boolean turnMotorReversed, boolean absoluteEncoderReversed)
    {
        driveMotor = new WPI_TalonFX(driveMotorID);
        turnMotor = new WPI_TalonFX(turnMotorID);

        this.driveMotorReversed = driveMotorReversed;
        this.turnMotorReversed = turnMotorReversed;

        this.driveMotor.setInverted(this.driveMotorReversed);
        this.turnMotor.setInverted(this.turnMotorReversed);

        absoluteEncoder = new AnalogInput(absoluteEncoderID);
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        turningPIDController = new PIDController(0, 0, 0);
        velocityPIDController = new PIDController(0, 0, 0);

        this.turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    }

    public double getDrivePosition()
    {
       return this.driveMotor.getSelectedSensorPosition(0);
    }

    public double getTurnPosition()
    {
       return this.turnMotor.getSelectedSensorPosition(0);
    }

    public double getDriveVelocity()
    {
       return this.driveMotor.getSelectedSensorVelocity(0);
    }

    public double getTurnVelocity()
    {
       return this.turnMotor.getSelectedSensorVelocity(0);
    }

    public void resetEncoders()
    {
        this.driveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getModuleState()
    {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    public void setModuleState(SwerveModuleState state)
    {

        if(Math.abs(state.speedMetersPerSecond) < 0.001)
        {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getModuleState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.teleDriveMaxSpeedMetersPerSecond);
        driveMotor.set(this.turningPIDController.calculate(getTurnPosition(), state.angle.getRadians()));

    }

    public void stop()
    {
        this.driveMotor.set(TalonFXControlMode.PercentOutput, 0);
        this.turnMotor.set(TalonFXControlMode.PercentOutput, 0);
    }


}





