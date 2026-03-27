package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.Constants;

public class ShootDelay {

    public static Command create(ShooterSubsystem shooter, IndexerSubsystem indexer) {
    return Commands.sequence(
        // 1. Start the shooter and wait 2 seconds
        // Using runOnce keeps the motor spinning during the wait
        Commands.runOnce(() -> shooter.runAtRPM(Constants.SHOOTER_RPM), shooter),
        Commands.waitSeconds(2.0),

        // 2. Now start the indexer while keeping the shooter running
        Commands.run(
            () -> {
                shooter.runAtRPM(Constants.SHOOTER_RPM);
                indexer.feed();
            },
            shooter, indexer
        )
    )
    // 3. This runs as soon as you release the Y button
    .finallyDo((interrupted) -> {
        shooter.stop();
        indexer.stop();
    });
    }
}
