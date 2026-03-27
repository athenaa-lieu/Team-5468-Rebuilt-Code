package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

    private final SparkMax indexerMotor = new SparkMax(13, MotorType.kBrushless);
    private final DigitalInput beamBreak = new DigitalInput(0);

    public void feed() {
        indexerMotor.set(0.6); //This is ur indexer speed
    }

    public void reverse() {
        indexerMotor.set(-0.3);
    }

    public void stop() {
        indexerMotor.stopMotor();
    }

    public boolean hasPiece() {
        return !beamBreak.get();
    }
}