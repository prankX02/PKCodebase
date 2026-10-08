package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;

/**
 * Constants for the intake subsystem.
 */
public final class IntakeConstants {
    // CAN IDs for the pivot and roller motors
    public static final int kPivotMotorCANId = 0; // TODO: Set actual CAN ID
    public static final int kRollerMotorCANId = 0; // TODO: Set actual CAN ID

    // CAN Coder (absolute encoder) CAN ID for pivot position feedback
    public static final int kPivotEncoderCANId = 0; // TODO: Set actual CAN ID

    // CAN Bus
    public static final CANBus kCANBus = new CANBus("Intake", "./logs/intake.hoot");

    // Gear ratio for pivot mechanism
    public static final double kPivotGearRatio = 0.0; // TODO: Set actual gear ratio

    // PID gains for pivot position control
    public static final double kPivotKP = 0.0; // TODO: Tune
    public static final double kPivotKI = 0.0; // TODO: Tune
    public static final double kPivotKD = 0.0; // TODO: Tune

    // Setpoints for pivot positions (in rotations)
    public static final double kDeployedPosition = 0.0; // TODO: Set deployed position
    public static final double kStowedPosition = 0.0; // TODO: Set stowed position

    // Roller speeds (as percentage output -1.0 to 1.0)
    public static final double kIntakeSpeed = 0.0; // TODO: Set intake speed
    public static final double kReverseSpeed = 0.0; // TODO: Set reverse speed

    // Tolerance for considering the pivot at setpoint (in rotations)
    public static final double kPivotTolerance = 0.05;

    // Current limits
    public static final double kPivotCurrentLimit = 40.0; // Amps
    public static final double kRollerCurrentLimit = 40.0; // Amps

    // Motor inversion flags
    public static final boolean kPivotInverted = false;
    public static final boolean kRollerInverted = false;
}
