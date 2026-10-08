package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for intake subsystem IO.
 */
public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        // Pivot motor inputs
        public boolean pivotConnected = false;
        public double pivotPositionRotations = 0.0;
        public double pivotVelocityRotPerSec = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;
        public double pivotEncoderAbsolutePosition = 0.0;

        // Roller motor inputs
        public boolean rollerConnected = false;
        public double rollerVelocityRotPerSec = 0.0;
        public double rollerAppliedVolts = 0.0;
        public double rollerCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {}

    /** Set the pivot motor to a specified voltage. */
    public default void setPivotVoltage(double volts) {}

    /** Set the pivot motor to a specified position (in rotations). */
    public default void setPivotPosition(double positionRotations) {}

    /** Set the roller motor to a specified voltage. */
    public default void setRollerVoltage(double volts) {}

    /** Stop the pivot motor. */
    public default void stopPivot() {}

    /** Stop the roller motor. */
    public default void stopRoller() {}

    /** Reset the pivot encoder to the absolute position. */
    public default void resetPivotEncoder() {}
}
