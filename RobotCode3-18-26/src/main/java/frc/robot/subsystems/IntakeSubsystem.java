package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax intakeMotor = new SparkMax(12, MotorType.kBrushless);

    public void intake() {
        intakeMotor.set(-0.3); //This is your intake speed
    }

    public void outtake() {
        intakeMotor.set(0.25);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }
}