package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeMove;

public class RunIntakeOut extends Command {

    private final IntakeMove out;

    public RunIntakeOut(IntakeMove out) {
        this.out = out;
        addRequirements(out);
    }

    @Override
    public void initialize() {
        out.out();
    }

    @Override
    public void end(boolean interrupted) {
        out.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
