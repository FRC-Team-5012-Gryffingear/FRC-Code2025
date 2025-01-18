// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveMod;
import frc.robot.subsystems.SwerveSubsys;
import frc.robot.subsystems.limey;

import javax.sound.sampled.LineEvent;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.UnitBuilder;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class com extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final ExampleSubsystem m_subsystem;
  private final SwerveSubsys swerve;
  private final limey lime;
  private final SwerveMod mod;
  private PIDController rot_PID = new PIDController(.2, 0, 0);
  private PIDController fwd_PID = new PIDController(.1, 0, 0);

  private boolean needs_move_fwrd = false;
  private boolean needs_rotate = false;
  private double april_tag_rotation = -10000;
  private double initial_gyro_yaw = -10000;
  private double remaining_fwrd_distance = -10000;

  /**
   * Creates a new com.
   *
   * @param subsystem The subsystem used by this command.
   */
  public com(SwerveSubsys swerve, limey lime, SwerveMod mod) {
    this.swerve = swerve;
    this.lime = lime;
    this.mod = mod;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve,lime);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(LimelightHelpers.getTV("")){
        swerve.drive3(0,0,.2,true);
    }
    else{
        needs_rotate = true;
        if(needs_rotate && april_tag_rotation == -10000 && initial_gyro_yaw == -10000){
            april_tag_rotation = LimelightHelpers.getCameraPose3d_TargetSpace("").getY();
            initial_gyro_yaw = swerve.getYaw();
            if(needs_rotate){
                double final_gyro_yaw = initial_gyro_yaw + april_tag_rotation;
                rot_PID.calculate(swerve.getYaw(), final_gyro_yaw);
                if(rot_PID.getError() < 0.01){
                    needs_rotate = false;
                    april_tag_rotation = -10000;
                    needs_move_fwrd = true;
                }
                
                }
            }
        if(needs_move_fwrd && remaining_fwrd_distance == -10000){
            remaining_fwrd_distance = lime.getTZ();
        }
        while(needs_move_fwrd){
            fwd_PID.calculate(mod.getDrivePos(), remaining_fwrd_distance);
            if(fwd_PID.getError() < 0.01){
                needs_move_fwrd = false;
                remaining_fwrd_distance = -10000;
            }

        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rot_PID.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
