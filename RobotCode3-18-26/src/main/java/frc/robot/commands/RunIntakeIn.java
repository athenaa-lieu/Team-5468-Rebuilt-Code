package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeMove;

public class RunIntakeIn extends Command {

    private final IntakeMove in;

    public RunIntakeIn(IntakeMove in) {
        this.in = in;
        addRequirements(in);
    }

    @Override
    public void initialize() {
        in.in();
    }

    @Override
    public void end(boolean interrupted) {
        in.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
