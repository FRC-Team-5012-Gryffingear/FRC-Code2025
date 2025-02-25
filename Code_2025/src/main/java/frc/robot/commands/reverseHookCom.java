package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.test;

public class reverseHookCom extends InstantCommand {
    private final test hookSubsystem;

    public reverseHookCom(test hookSubsystem) {
        this.hookSubsystem = hookSubsystem;
        addRequirements(hookSubsystem);
    }

    @Override
    public void initialize() {
        hookSubsystem.reverseHook();
    }
}
