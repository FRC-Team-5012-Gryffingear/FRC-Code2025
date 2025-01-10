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
  Servo servo2 = new Servo(Constants.servo2);
  Servo servo3 = new Servo(Constants.servo3);

  public servo() {
  }


  public void settingServo(){
    System.out.println(servo1.getPosition());
    if(Math.abs(servoContrl.calculate(servo1.getPosition(), .5)) < 0.05){
      servo1.setSpeed(0);
    }
    else{
      servo1.setSpeed(servoContrl.calculate(servo1.getPosition(), .5));
    }
    
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
