// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArcadeSubsystem extends SubsystemBase {
  /** Creates a new ArcadeSubsystem. */

  TalonSRX FLM = new TalonSRX(Constants.FL);
  TalonSRX FRM = new TalonSRX(Constants.FR);
  TalonSRX BLM = new TalonSRX(Constants.BL);
  TalonSRX BRM = new TalonSRX(Constants.BR);
  TalonSRX BTB = new TalonSRX(Constants.coralouttake);
  
  Pigeon2 pigeon = new Pigeon2(Constants.pigeon);

  public ArcadeSubsystem() {
    FLM.configFactoryDefault();
    FRM.configFactoryDefault();
    BLM.configFactoryDefault();
    BRM.configFactoryDefault();
    BTB.configFactoryDefault();

    FLM.setNeutralMode(NeutralMode.Brake);
    FRM.setNeutralMode(NeutralMode.Brake);
    BLM.setNeutralMode(NeutralMode.Brake);
    BRM.setNeutralMode(NeutralMode.Brake);
    BTB.setNeutralMode(NeutralMode.Brake);

    BLM.follow(FLM);
    BRM.follow(FRM);

    FRM.setInverted(InvertType.InvertMotorOutput);
    BRM.setInverted(InvertType.FollowMaster);
  }


  public double getVoltage(){
    return FLM.getBusVoltage();
  }

  public void moveAndTurn(double power, double turn){
    FLM.set(ControlMode.PercentOutput, (power + turn));
    FRM.set(ControlMode.PercentOutput, (power - turn));
  }
  /* When a button is pressed/held the motor outtakes the coral
   * when released power is 0
   */
  public void coralouttake(boolean a){
    if(a) {
      BTB.set(ControlMode.PercentOutput, -.5);
    }
    else{
      BTB.set(ControlMode.PercentOutput, 0);
    }
  }
  
  public void resetYaw(){
    pigeon.reset();
    pigeon.setYaw(0);
  }

  public double getYaw(){

   return -pigeon.getYaw().getValueAsDouble() % 360;
  }

 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Yaw", getYaw());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
