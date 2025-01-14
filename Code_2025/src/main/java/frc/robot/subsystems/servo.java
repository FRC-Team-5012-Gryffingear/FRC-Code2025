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
  PWM servo1 = new PWM(Constants.servo1);
  Servo servo2 = new Servo(Constants.servo2);
  

  public servo() {
    // servo1.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

    // servo2.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    
  }

  public void settingServo45(){
    servo2.setAngle(45);
  }

  public void settingServo15(){
   servo2.setAngle(15);
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
