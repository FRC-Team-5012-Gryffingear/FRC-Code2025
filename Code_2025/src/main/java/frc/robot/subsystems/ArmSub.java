// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSub extends SubsystemBase {
  /** Creates a new ArmSub. */
  
  PIDController move;
  // ArmFeedforward armfeed;
  // If it is a kraken/Falcon
  TalonFX arm = new TalonFX(Constants.armPort);



  // private final int targetAngle1 = 15;
  private final int ticks_per_Rev = 4096;
  
  //Bottom 2 values might need change
  private final double ArmGearRatio = 6.75;
  private final double Acceptable_error = .5;
  

  public ArmSub() {
    var talon = new TalonFXConfiguration();
    // var invert = talon.MotorOutput.Inverted.Clockwise_Positive;
    
    arm.getConfigurator().apply(talon);
    arm.setNeutralMode(NeutralModeValue.Brake);
    

    //Change PID and FeedFwrd vals if needed
    move = new PIDController(.2, 0, 0);
    move.setTolerance(Acceptable_error);

    // armfeed = new ArmFeedforward(2,10,2);
  }

 //All Functions below is an attempt in using PID with Angles instead of ArmFowardFeedback cuz its awful 
 public void setArmAngle(double targetAngle){

  double targetInRevs = degToRot(targetAngle); // MULTIPLY BY THE CORRECT GEAR RATIO WHEN IN ARM

  double power = move.calculate(arm.getPosition().getValueAsDouble(), (targetInRevs));
  System.out.println(power);

  
  arm.set(power);
}


  public boolean armAtTarget(double targetAngle){

    double targetRevs = degToRot(targetAngle); // MULTIPLY BY THE CORRECT GEAR RATIO

    return (Math.abs(arm.getPosition().getValueAsDouble() - targetRevs)) <= move.getErrorTolerance();
  }

  public double getCurrentAngle(){
    return rotToDeg(arm.getPosition().getValueAsDouble());
  }
 
  private double rotToDeg(double CurrentRev){
    
     return (CurrentRev) * 360; // CURRENT REV DIVIDE BY CORRECT GEAR RATIO
  }

  private double degToRot(double targetAngle1){
    return (targetAngle1 / 360); // MULTIPLY BY THE CORRECT GEAR RATIO 
  } 

  public void armStop(){
    arm.set(0);
  }
  public void resetEncoder(){
    arm.setPosition(0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("target in Angle", 45);
    SmartDashboard.putNumber("Current Angle", getCurrentAngle());
    SmartDashboard.putNumber("Target in Tick", degToRot(45));
    SmartDashboard.putNumber("Current in tick", arm.getPosition().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
