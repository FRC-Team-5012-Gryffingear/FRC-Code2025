// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsysCoral extends SubsystemBase {

    private TalonSRX intakeTalon2 = new TalonSRX(Constants.intakeHook);
    private TalonSRX intakeTalon3 = new TalonSRX(Constants.intakeCoral);
    private TalonSRX intakeTalon = new TalonSRX(Constants.intakeLift);  

    private double global = 0;

    public IntakeSubsysCoral() {

        intakeTalon2.configFactoryDefault();
        intakeTalon3.configFactoryDefault();
        intakeTalon.configFactoryDefault();
        
        intakeTalon2.setNeutralMode(NeutralMode.Brake);
        intakeTalon3.setNeutralMode(NeutralMode.Brake);
        intakeTalon.setNeutralMode(NeutralMode.Brake);

        // intakeTalon2.setInverted(InvertType.InvertMotorOutput);
        // intakeTalon3.setInverted(InvertType.InvertMotorOutput);
    }



    public void coralIntake(double power){
      // negative pow makes it close
      // global = power;
      //    intakeTalon2.set(ControlMode.PercentOutput, -power);
         intakeTalon3.set(ControlMode.PercentOutput, power);
          intakeTalon.set(ControlMode.PercentOutput, power);

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
