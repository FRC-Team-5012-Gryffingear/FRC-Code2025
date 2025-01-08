// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
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
  ArmFeedforward armfeed;
  // If it is a kraken/Falcon
  TalonFX arm = new TalonFX(Constants.armPort);



  // private final int targetAngle1 = 15;
  private final int ticks_per_Rev = 4096;
  
  //Bottom 2 values might need change
  private final double ArmGearRatio = 6.75;
  private final int Acceptable_error = 2;

  public ArmSub() {
    arm.getConfigurator().apply(new TalonFXConfiguration());
    arm.setNeutralMode(NeutralModeValue.Brake);

    //Change PID and FeedFwrd vals if needed
    move = new PIDController(.02, 0, 0);
    move.setTolerance(2);

    armfeed = new ArmFeedforward(2,10,2);
  }

 //All Functions below is an attempt in using PID with Angles instead of ArmFowardFeedback cuz its awful 
  public void armMovement(double targetAngle){
    arm.set(setArmAngle(targetAngle));
  }

  public boolean armAtTarget(double targetAngle){
    double targetTicks = degToTick(targetAngle);

    return (Math.abs(arm.getPosition().getValueAsDouble() - targetTicks)) <= .5;
  }

  public double getCurrentAngle(){
    return Math.abs(tickToDeg(arm.getPosition().getValueAsDouble()));
  }


  public double setArmAngle(double targetAngle){
    double targetInTicks = degToTick(targetAngle); 

    double power = move.calculate(arm.getPosition().getValueAsDouble(), targetInTicks);

    return power;
  }

 
  private double tickToDeg(double CurrentTick){
    //return (CurrentTick/(ticks_per_Rev*ArmGearRatio))* 360;
     return CurrentTick * 360;
  }

  private double degToTick(double targetAngle1){
    //AUTOMATICALLY SET TO 15 deg
    //this means we are converting 15 deg to ticks since that is our "goal"

    return (targetAngle1 / 360); // * ticks_per_Rev * ArmGearRatio;
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
