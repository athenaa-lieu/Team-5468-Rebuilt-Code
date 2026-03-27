package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class RunIndexer extends Command {

    private final IndexerSubsystem indexer;

    public RunIndexer(IndexerSubsystem indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.feed();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
