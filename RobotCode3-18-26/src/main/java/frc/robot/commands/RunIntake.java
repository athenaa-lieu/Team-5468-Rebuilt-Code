package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends Command {

    private final IntakeSubsystem intake;

    public RunIntake(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intake();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
