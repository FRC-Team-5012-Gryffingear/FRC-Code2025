// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class test extends SubsystemBase {
  /** Creates a new test. */
  private final TalonSRX potent = new TalonSRX(15); // Set CAN ID
    private boolean toggleState = false; // Tracks last state (true = open, false = closed)
    private double motorPower = -1; // Initially closed
    private boolean manuallyReversed = false; // Tracks if Button 2 reversed the direction
  public test() {
    potent.configFactoryDefault();
    potent.setNeutralMode(NeutralMode.Brake);

    potent.set(ControlMode.PercentOutput, motorPower);
  }

  public void toggleHook() {
    if (manuallyReversed) {
        manuallyReversed = false; // Reset manual reverse so Button 1 has authority
        return; // Don't allow toggle if it was manually reversed
    }

    toggleState = !toggleState; // Flip toggle state
    motorPower = toggleState ? 1 : -1; // 1 for open, -1 for closed
    potent.set(ControlMode.PercentOutput, motorPower);
}

public void reverseHook() {
  motorPower = -motorPower; // Reverse the current motor direction
  manuallyReversed = true; // Flag that a manual override happened
  potent.set(ControlMode.PercentOutput, motorPower);
}

public double getMotorPower() {
  return motorPower;
}

//  public void tester(double power){
    
//     potent.set(ControlMode.PercentOutput, power);
//  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
