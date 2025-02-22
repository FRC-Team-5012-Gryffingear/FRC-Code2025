// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.constant.Constable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsysLift extends SubsystemBase {
    private TalonSRX intakeTalon = new TalonSRX(Constants.intakeLift);

    public IntakeSubsysLift() {
        intakeTalon.configFactoryDefault();
        
        intakeTalon.setNeutralMode(NeutralMode.Brake);
    }


    public void up(double num){
        System.out.println("This is the value of intake movement: " + num);
        intakeTalon.set(ControlMode.PercentOutput, num);
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
