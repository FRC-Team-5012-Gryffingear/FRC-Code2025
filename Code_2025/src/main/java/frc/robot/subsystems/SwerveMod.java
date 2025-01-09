package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModConstants;

public class SwerveMod {
    private final TalonFX DriveMotor, SteerMotor;
    private final CANcoder SteerEncoder;

    private final SlewRateLimiter drivelim = new SlewRateLimiter(1);

    // Incase Slew Rate Limiter doesn't work for drive
    // private final PIDController drivecont = new PIDController(ModConstants.KP,0,0);
    
    private final PIDController steerContrl = new PIDController(ModConstants.KP, 0, 0);

    private final String moduleName;

    private final SwerveModuleState lastDesiredState = new SwerveModuleState();


    public SwerveMod(int DriveM_ID, int SteerM_ID, int encoderID, double encoderOffset, String modName){
        // Bring the empty variables in here to create objects using the IDs required above
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        // TalonFXConfigurator talonConfigurator = DriveMotor.getConfigurator();

        var Motorconfigs = new MotorOutputConfigs();
        Motorconfigs.Inverted = InvertedValue.Clockwise_Positive;
        
        // talonConfigurator.refresh(talonConfig);

        
        DriveMotor = new TalonFX(DriveM_ID);
        SteerMotor = new TalonFX(SteerM_ID);


        

    }



    public void setModState(){

    }
}
