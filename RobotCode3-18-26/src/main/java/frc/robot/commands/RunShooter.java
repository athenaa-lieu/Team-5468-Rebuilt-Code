package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooter extends Command {

    private final ShooterSubsystem shooter;

    public RunShooter(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.runAtRPM(-4500);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}