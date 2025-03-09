// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class intakeCombined extends SubsystemBase {
  /** Creates a new intakeCombined. */
  private TalonSRX intakeTalon2 = new TalonSRX(Constants.intakeHook);
    private TalonSRX intakeTalon3 = new TalonSRX(Constants.intakeCoral);
    private TalonSRX intakeTalon = new TalonSRX(Constants.intakeLift);  

    private double motorPower = 1; // Initially closed
    private double absoluteState = -1;

  public intakeCombined() {
    intakeTalon.configFactoryDefault();
    intakeTalon2.configFactoryDefault();
    intakeTalon3.configFactoryDefault();

    intakeTalon.setNeutralMode(NeutralMode.Brake);
    intakeTalon2.setNeutralMode(NeutralMode.Brake);
    intakeTalon3.setNeutralMode(NeutralMode.Brake);

    intakeTalon2.setInverted(InvertType.InvertMotorOutput);

    intakeTalon.set(ControlMode.PercentOutput, -1);
    intakeTalon2.set(ControlMode.PercentOutput, -1);
    intakeTalon3.set(ControlMode.PercentOutput, -1);
  }


  public void toggleHook(){
    absoluteState *= -1;
    motorPower = 1;
  
    intakeTalon.set(ControlMode.PercentOutput, absoluteState);
    intakeTalon2.set(ControlMode.PercentOutput, absoluteState);
    intakeTalon3.set(ControlMode.PercentOutput, absoluteState);
  }
  
  public void reverseHook(){
    if(motorPower != absoluteState){
      motorPower = absoluteState * -1;
    }else{
      motorPower *= -1;
    }
    
    intakeTalon2.set(ControlMode.PercentOutput, motorPower);
  }
  public double getAbsolutePower(){
    return absoluteState;
  }

  public double getMotorPower() {
    return motorPower;
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ABSOLUTE INTAKE POSITION POWER", getAbsolutePower());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
