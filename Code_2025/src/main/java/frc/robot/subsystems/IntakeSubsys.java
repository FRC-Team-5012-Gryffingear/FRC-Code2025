// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsys extends SubsystemBase {
    private TalonSRX intakeTalon = new TalonSRX(202020);
    private TalonSRX intakeTalon2 = new TalonSRX(202012);
    

    private double num = 0;

    public IntakeSubsys() {
        num = 0;

        intakeTalon.configFactoryDefault();
        intakeTalon2.configFactoryDefault();
        
        intakeTalon.setNeutralMode(NeutralMode.Brake);
        intakeTalon2.setNeutralMode(NeutralMode.Brake);

        intakeTalon2.follow(intakeTalon);
        
    }


    public void up(boolean a, boolean b){
        if(a){
            num = 1;
        }
        else if(b){
            num = -1;
        }
        else if(!a && !b){
          num = 0;
        }
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
