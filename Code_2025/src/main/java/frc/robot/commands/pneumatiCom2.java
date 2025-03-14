package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.pneumatic;

public class pneumatiCom2 extends InstantCommand {
    private final pneumatic pneumaticSubsys;

    public pneumatiCom2(pneumatic pneumaticSubsys) {
        this.pneumaticSubsys = pneumaticSubsys;
        addRequirements(pneumaticSubsys);
    }

    @Override
    public void initialize() {
        pneumaticSubsys.reverseHook(); // Reverse only if allowed in the current state
    }
}
