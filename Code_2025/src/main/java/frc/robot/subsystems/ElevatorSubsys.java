// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsys extends SubsystemBase {
    private TalonSRX elevatorTalon = new TalonSRX(012012);
    private CANcoder elevEncoder = new CANcoder(2121211);
    private double offset = Constants.elevOffset;


    private PIDController elevHold = new PIDController(.01, 0, 0); // might not work because of gravity variable not taken into count

    public ElevatorSubsys() {
        elevatorTalon.configFactoryDefault();
        
        elevatorTalon.setNeutralMode(NeutralMode.Brake);
        
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = offset;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        elevEncoder.getConfigurator().apply(config);     
    }
    
    //Gather data and determine what value and unit we are using to determine our movement
    public void elevMovement(double goal){ // each call of this function will have a different goal value
        double currentPosition = elevEncoder.getPosition().getValueAsDouble();
        double percent = elevHold.calculate(currentPosition,goal);
        
        elevatorTalon.set(ControlMode.PercentOutput, percent);
    }

    public void elevUpAndDown(double power){
      elevatorTalon.set(ControlMode.PercentOutput, power);
    }



    public void resetEncoderPos(){
        elevEncoder.setPosition(0);
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder pos", elevEncoder.getPosition().getValueAsDouble());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
