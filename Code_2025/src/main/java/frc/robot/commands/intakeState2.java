package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intakeCombined;

public class intakeState2 extends InstantCommand {
    private final intakeCombined hookSubsystem;

    public intakeState2(intakeCombined hookSubsystem) {
        this.hookSubsystem = hookSubsystem;
        addRequirements(hookSubsystem);
    }

    @Override
    public void initialize() {
        hookSubsystem.reverseHook();
    }
}