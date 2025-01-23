package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModConstants;

public class SwerveMod {
    private final TalonFX DriveMotor, SteerMotor;
    private final CANcoder SteerEncoder;

    private final PIDController pidcontrl = new PIDController(ModConstants.KP, ModConstants.KI, ModConstants.KD);

    private final PIDController drivecont = new PIDController(ModConstants.KP, 0, 0);
    private final String Modname;

    private SwerveModuleState lastDesiredState = new SwerveModuleState();



    //Creates a new Swerve Mod using talons and cancoders
    public SwerveMod(int DriveID, int SteerID, int encoderID, double offset, boolean DriveInvr, String moduleName){
        //Use the declared empty motor variables and creates an object using the drive Id as port ID
        DriveMotor = new TalonFX(DriveID);
        SteerMotor = new TalonFX(SteerID);
        SteerEncoder = new CANcoder(encoderID);

        TalonFXConfiguration talon = new TalonFXConfiguration();
        talon.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        //Configurating the drive motor to default
        DriveMotor.getConfigurator().apply(talon);
        // DriveMotor.setInverted(DriveInvr);

        //config steer motor to default
        SteerMotor.getConfigurator().apply(new TalonFXConfiguration());

        //Have the motors immediately stop after no input on the joystick
        DriveMotor.setNeutralMode(NeutralModeValue.Brake);             
        SteerMotor.setNeutralMode(NeutralModeValue.Brake);  

        //add configurations for the Encoder
        //have the magnet offset = to the offset recorded 
        //Rotation towards the right is positive
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = offset;
        //Config to full 360 if problems set back to signed half
        // config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        
        //apply the config into the Encoder
        SteerEncoder.getConfigurator().apply(config);

        //Have the Steer PID controller recognize that there is continuation after 180 and to connect those
        //radians/degrees
        pidcontrl.enableContinuousInput(-Math.PI, Math.PI);
        
        //delete old data that is not used
        pidcontrl.reset();
        drivecont.reset();

        Modname = moduleName;

    }

    public void Info(){
        if(RobotBase.isReal()){
            //Gets the Rotation of the steer as well as the power from the Drive motor
            SmartDashboard.putNumber(Modname + " Turn Rotation", getRotation().getDegrees());
            SmartDashboard.putNumber(Modname + " Drive Speed", DriveMotor.get());
        } else {
            SmartDashboard.putNumber(Modname + " Turn Rotation", lastDesiredState.angle.getDegrees());
            SmartDashboard.putNumber(Modname + " Drive Speed", lastDesiredState.speedMetersPerSecond);
        }
    }
    
    //Gets the Current deg of the Encoder on a 2d plane
    public Rotation2d getRotation(){
        return Rotation2d.fromRotations(SteerEncoder.getAbsolutePosition().getValueAsDouble());
    } 

    //Get the position of the drive or how far did it drive
    public double getDrivePos(){
        return DriveMotor.getPosition().getValueAsDouble();
    }
    
    //Gets the velocity of the drive motor 
    public double getDriveVelo(){
        return DriveMotor.getVelocity().getValueAsDouble();
    }

    //gets the state of the drive motor and the steering motor
    public SwerveModuleState getModState(){
        return new SwerveModuleState(getDriveVelo(),getRotation());
    }

    //How far and which direction have we gone
    public SwerveModulePosition getModPos(){
        return new SwerveModulePosition(getDrivePos(),getRotation());
    }

    //Set the Modules to the desired State we want it to be in
    public void setModState(SwerveModuleState desiredState){
         lastDesiredState = desiredState;

        //If there is no input in where we want it to go then don't move
        if(Math.abs(lastDesiredState.speedMetersPerSecond) < 0.02){
            stop();
            return;
        }
        //else then calculate the best option to reach our desired state
        SwerveModuleState optimizedState = SwerveModuleState.optimize(lastDesiredState, getRotation());
        // SwerveModuleState optimizedState = new SwerveModuleState(lastDesiredState.speedMetersPerSecond, getRotation());
        // optimizedState.optimize(getRotation());

        //Using the optimized state to calculate convert it into motor power we can use it to move
        DriveMotor.set(drivecont.calculate(optimizedState.speedMetersPerSecond * 8, lastDesiredState.speedMetersPerSecond));

        SteerMotor.set(pidcontrl.calculate(getRotation().getRadians(), optimizedState.angle.getRadians()));

    }

    public void stop(){
        DriveMotor.set(0);
        SteerMotor.set(0);  
    }



}