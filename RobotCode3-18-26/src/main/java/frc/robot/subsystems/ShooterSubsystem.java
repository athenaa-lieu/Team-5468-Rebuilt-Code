
package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax shooterMotor;
    private SparkMax shooterFollower; // Optional second motor
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pid;

    //private final SimpleMotorFeedforward feedforward; //This is commented out for now

    private double targetRPM = 4500.0;

    // Feedforward constants (tune for your robot)
    //private static final double kS = 0.2;      // Volts to overcome static friction
    private static final double kV = 0.000195;  // Volts per RPM
   // private static final double kA = 0.0;      // Optional acceleration feedforward

    // PID constants (tune for your robot)
    private static final double kP = 0.0005;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public ShooterSubsystem() {

        shooterMotor = new SparkMax(11, MotorType.kBrushless);
        encoder = shooterMotor.getEncoder();
        pid = shooterMotor.getClosedLoopController();

        //feedforward = new SimpleMotorFeedforward(kS, kV, kA);

        SparkMaxConfig config = new SparkMaxConfig();

        // Encoder conversion factor (RPM)
        config.encoder.velocityConversionFactor(1.0);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .outputRange(-1, 1)
            .feedForward.kV(kV);

        // Apply config
        shooterMotor.configure(
            config,
            com.revrobotics.spark.SparkMax.ResetMode.kResetSafeParameters,
            com.revrobotics.spark.SparkMax.PersistMode.kPersistParameters
        );

        // Optional: add follower motor
        // shooterFollower = new SparkMax(11, MotorType.kBrushless);
        // shooterFollower.follow(shooterMotor);
    }

    /** Run shooter at a specific target RPM */
    public void runAtRPM(double rpm) {
        targetRPM = rpm;
        // Feedforward + PID
       // double ffVolts = feedforward.calculate(rpm, (rpm - getCurrentRPM()) / 0.02);
        // REV SparkMax velocity setpoint (PID takes care of control internally)
        pid.setSetpoint(rpm, com.revrobotics.spark.SparkMax.ControlType.kVelocity);
        // Note: voltage feedforward can also be applied via motor.setVoltage(ffVolts);
    }

    /** Stop the shooter */
    public void stop() {
        targetRPM = 0.0;
        shooterMotor.stopMotor();
        if (shooterFollower != null) {
            shooterFollower.stopMotor();
        }
    }

    /** Get current shooter RPM */
    public double getCurrentRPM() {
        return encoder.getVelocity();
    }

    /** Check if the shooter has reached target RPM within tolerance */
    public boolean atSpeed() {
        return Math.abs(getCurrentRPM() - targetRPM) < 50; // ±50 RPM tolerance
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", getCurrentRPM());
        SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
        SmartDashboard.putBoolean("Shooter At Speed", atSpeed());
    }
}