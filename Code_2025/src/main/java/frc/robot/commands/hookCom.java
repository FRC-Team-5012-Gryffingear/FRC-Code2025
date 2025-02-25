package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.test;

public class hookCom extends InstantCommand {
    private final test hookSubsystem;

    public hookCom(test hookSubsystem) {
        this.hookSubsystem = hookSubsystem;
        addRequirements(hookSubsystem);
    }

    @Override
    public void initialize() {
        hookSubsystem.toggleHook();
    }
}
