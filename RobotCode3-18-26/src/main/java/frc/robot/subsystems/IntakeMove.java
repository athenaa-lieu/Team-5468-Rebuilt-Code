package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController; // Updated for 2025/2026 API
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeMove extends SubsystemBase {

    private final SparkMax intakeMoveMotor = new SparkMax(15, MotorType.kBrushless);
    private final SparkClosedLoopController pidController = intakeMoveMotor.getClosedLoopController();
    
    // Adjust this value to your desired RPM (e.g., 500 is quite slow for a NEO)
    private final double targetRPM = 750.0;

    public IntakeMove() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Basic PID constants - these may need tuning for your specific intake
        // P (Proportional) is the most important one to start with
        config.closedLoop.p(0.0001); 
        config.closedLoop.i(0);
        config.closedLoop.d(0);
        config.closedLoop.velocityFF(0.00017); // Feed-Forward helps hit the target speed

        intakeMoveMotor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    public void in() {
        // Sets the motor to maintain a specific velocity (RPM)
        pidController.setReference(targetRPM, SparkMax.ControlType.kVelocity);
    }

    public void out() {
        pidController.setReference(-targetRPM, SparkMax.ControlType.kVelocity);
    }

    public void stop() {
        intakeMoveMotor.stopMotor();
    }
}