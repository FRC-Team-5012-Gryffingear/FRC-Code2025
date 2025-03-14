// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class pneumatic extends SubsystemBase {
  /** Creates a new pneumatic. */

  DoubleSolenoid hooktest1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 7);
  DoubleSolenoid test2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 6);

  Value absoluteval = Value.kReverse;
  private boolean canReverse = true;

  Timer time = new Timer();

  public pneumatic() {
    hooktest1.set(Value.kReverse);
    test2.set(Value.kReverse);
  }

  public void toggle(){
    absoluteval = (absoluteval == Value.kForward) ? Value.kReverse : Value.kForward;
    hooktest1.set(absoluteval);
    test2.set(absoluteval);
    canReverse = true;
  }

  public void reverseHook(){
    if(canReverse){
      hooktest1.toggle();
      canReverse = false;
    }
  }

  public Value getSolenoidState(){
    return absoluteval;
  }
  
  public Value getHookState(){
    return hooktest1.get();
  }

  // public void toggle(){
  //   absoluteval = test2.get();
  // }


  // public void pneumaticMove(boolean a,boolean b){
  //   if(a){
  //       if(test2.get() == Value.kReverse){
  //           hooktest1.set(Value.kForward);
  //           test2.set(Value.kForward);
            
  //       }else if(test2.get() == Value.kForward){
  //           hooktest1.set(Value.kReverse);
  //           test2.set(Value.kReverse);
  //       }
  //   }else if(b){
  //       hooktest1.toggle();
  //   }
  // }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Solenoid State", getSolenoidState().toString());
    SmartDashboard.putString("Hook State", getHookState().toString());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
