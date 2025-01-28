// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArcadeSubsystem extends SubsystemBase {
  /** Creates a new ArcadeSubsystem. */

  TalonSRX FLM = new TalonSRX(Constants.FL);
  TalonSRX FRM = new TalonSRX(Constants.FR);
  TalonSRX BLM = new TalonSRX(Constants.BL);
  TalonSRX BRM = new TalonSRX(Constants.BR);

  public ArcadeSubsystem() {
    FLM.configFactoryDefault();
    FRM.configFactoryDefault();
    BLM.configFactoryDefault();
    BRM.configFactoryDefault();

    FLM.setNeutralMode(NeutralMode.Brake);
    FRM.setNeutralMode(NeutralMode.Brake);
    BLM.setNeutralMode(NeutralMode.Brake);
    BRM.setNeutralMode(NeutralMode.Brake);

    BLM.follow(FLM);
    BRM.follow(FRM);

    FRM.setInverted(InvertType.InvertMotorOutput);
    BRM.setInverted(InvertType.FollowMaster);
  }



  public void moveAndTurn(double power, double turn){
    FLM.set(ControlMode.PercentOutput, power + turn);
    FRM.set(ControlMode.PercentOutput, power - turn);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
