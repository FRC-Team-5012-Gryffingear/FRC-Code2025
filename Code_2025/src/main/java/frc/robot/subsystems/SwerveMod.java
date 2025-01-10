package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.util.concurrent.CancellationException;

import org.opencv.core.RotatedRect;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModConstants;

public class SwerveMod {
    private final TalonFX DriveMotor, SteerMotor;
    private final CANcoder SteerEncoder;

    // private final SlewRateLimiter drivelim = new SlewRateLimiter(1);

    // Incase Slew Rate Limiter doesn't work for drive
    private final PIDController driveCont = new PIDController(ModConstants.KP,0,0);
    
    private final PIDController steerContrl = new PIDController(ModConstants.KP, 0, 0);

    private final String moduleName;

    private  SwerveModuleState lastDesiredState = new SwerveModuleState();


    public SwerveMod(int DriveM_ID, int SteerM_ID, int encoderID, double encoderOffset, boolean inverted,String modName){
        // Bring the empty variables in here to create objects using the IDs required above
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        if(inverted){
            talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }
        else{
            talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
        
        DriveMotor = new TalonFX(DriveM_ID);
        SteerMotor = new TalonFX(SteerM_ID);

        DriveMotor.getConfigurator().apply(talonConfig);
        SteerMotor.getConfigurator().apply(new TalonFXConfiguration());

        DriveMotor.setNeutralMode(NeutralModeValue.Brake);
        SteerMotor.setNeutralMode(NeutralModeValue.Brake);

        SteerEncoder = new CANcoder(encoderID);
        

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = encoderOffset;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        SteerEncoder.getConfigurator().apply(config);
    

        steerContrl.enableContinuousInput(-Math.PI, Math.PI);
        driveCont.enableContinuousInput(-Math.PI, Math.PI);


        steerContrl.reset();
        driveCont.reset();

        moduleName = modName;

    }

    public void Info(){
        if(RobotBase.isReal()){
            SmartDashboard.putNumber(moduleName + "Turn rotation", getRotation().getDegrees());
            SmartDashboard.putNumber(moduleName + " Drive Speed", DriveMotor.get());
        }else{
            SmartDashboard.putNumber(moduleName + " Turn Rotation", lastDesiredState.angle.getDegrees());
            SmartDashboard.putNumber(moduleName + " Drive Speed", lastDesiredState.speedMetersPerSecond);
        }
    }

    public Rotation2d getRotation(){
        return Rotation2d.fromRotations(SteerEncoder.getAbsolutePosition().getValueAsDouble());
    }
    public double getDrivePos(){
        return DriveMotor.getPosition().getValueAsDouble();
    }
    public double getDriveVelo(){
        return DriveMotor.getVelocity().getValueAsDouble();
    }
    public SwerveModuleState getModState(){
        return new SwerveModuleState(getDriveVelo(),getRotation());
    }
    public SwerveModulePosition getModPos(){
        return new SwerveModulePosition(getDrivePos(),getRotation());
    }


    public void setModState(SwerveModuleState desiredState){
        SwerveModuleState optimizedState = new SwerveModuleState(desiredState.speedMetersPerSecond,getRotation());
       
        if(Math.abs(lastDesiredState.speedMetersPerSecond) < 0.02){
            stop();
            return;
        }

        //Might need massive change
        DriveMotor.set(driveCont.calculate(optimizedState.speedMetersPerSecond * 8, lastDesiredState.speedMetersPerSecond));
        SteerMotor.set(steerContrl.calculate(getRotation().getRadians(), optimizedState.angle.getRadians()));

        lastDesiredState = desiredState;
    }

    public void stop(){
        DriveMotor.set(0);
        SteerMotor.set(0);
    }
}
