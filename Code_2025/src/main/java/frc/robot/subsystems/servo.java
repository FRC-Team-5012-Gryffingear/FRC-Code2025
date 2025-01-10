// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier.PWMChannel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class servo extends SubsystemBase {
  /** Creates a new servo. */
  PIDController servoContrl = new PIDController(.001, 0, 0);
  PWM servo1 = new PWM(Constants.servo1);
  

  public servo() {
    servo1.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
  }

  public void settingServo45(){
    double pulseProportion = 45 / 180; // 
    servo1.setPosition(Math.max(0, Math.min(pulseProportion, 1)));
    System.out.println("Current servo angle: " + servo1.getPosition());
  }

  public void settingServo(){
  
    double pow = Math.abs(servoContrl.calculate(servo1.getPosition(), .5));
    double limitedpow = Math.max(0, Math.min(1, pow));

    System.out.println(servo1.getPosition() + "\n" + " Power: " + limitedpow);

    if(limitedpow < 0.05){
      servo1.setSpeed(0);
    }
    else{
      servo1.setSpeed(servoContrl.calculate(servo1.getPosition(), .5));
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    settingServo45();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
