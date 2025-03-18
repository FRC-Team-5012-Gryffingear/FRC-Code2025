// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.temporal.ValueRange;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intakePneumatics extends SubsystemBase {
  DoubleSolenoid hookPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 1);
  DoubleSolenoid liftPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);
  DoubleSolenoid flapPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6,7);

  private Value absoluteVal = Value.kReverse;
  private boolean canReverse = true; 

  public intakePneumatics() {

    hookPiston.set(Value.kReverse);
    liftPiston.set(Value.kReverse);
    flapPiston.set(Value.kReverse);
  }

  public void toggle(){
    absoluteVal = (absoluteVal == Value.kForward) ? Value.kReverse : Value.kForward;

    hookPiston.set(absoluteVal);
    liftPiston.set(absoluteVal);
    flapPiston.set(absoluteVal);
    
    canReverse = true;
  }

  public void reverseHook(){
    if(canReverse){
        hookPiston.toggle();
        canReverse = false;
    }
  }

  public Value SolenoidState(){
    return absoluteVal;
  }

  public Value hookState(){
    return hookPiston.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Absolute Solenoid State", SolenoidState().toString());
    SmartDashboard.putString("Hook Solenoid state", hookState().toString());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
