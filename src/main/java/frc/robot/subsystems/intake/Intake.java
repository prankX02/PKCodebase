package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Intake subsystem for managing the intake mechanism.
 * Handles both pivot (deploy/stow) and roller (intake/reverse) control.
 */
public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // Log processed outputs
        Logger.recordOutput("Intake/PivotAtSetpoint", isPivotAtSetpoint());
    }

    // ============ PIVOT CONTROLS ============

    /**
     * Deploy the intake (move pivot to deployed position).
     */
    public Command deployCmd() {
        return Commands.run(() -> {
            io.setPivotPosition(IntakeConstants.kDeployedPosition);
        }, this)
            .withName("DeployIntake");
    }

    /**
     * Stow the intake (move pivot to stowed position).
     */
    public Command stowCmd() {
        return Commands.run(() -> {
            io.setPivotPosition(IntakeConstants.kStowedPosition);
        }, this)
            .withName("StowIntake");
    }

    /**
     * Move the pivot to a specific position.
     */
    public Command setPivotPositionCmd(double positionRotations) {
        return Commands.run(() -> {
            io.setPivotPosition(positionRotations);
        }, this)
            .withName("SetPivotPosition");
    }

    /**
     * Stop the pivot motor.
     */
    public Command stopPivotCmd() {
        return Commands.runOnce(io::stopPivot, this)
            .withName("StopPivot");
    }

    // ============ ROLLER CONTROLS ============

    /**
     * Run the intake roller (intake balls).
     */
    public Command intakeCmd() {
        return Commands.run(() -> {
            double volts = IntakeConstants.kIntakeSpeed * 12.0; // Convert percentage to volts
            io.setRollerVoltage(volts);
        }, this)
            .withName("Intake");
    }

    /**
     * Run the roller in reverse (eject balls).
     */
    public Command reverseCmd() {
        return Commands.run(() -> {
            double volts = IntakeConstants.kReverseSpeed * 12.0; // Convert percentage to volts
            io.setRollerVoltage(volts);
        }, this)
            .withName("ReverseIntake");
    }

    /**
     * Run the roller at a specified voltage.
     */
    public Command setRollerVoltageCmd(double volts) {
        return Commands.run(() -> {
            io.setRollerVoltage(volts);
        }, this)
            .withName("SetRollerVoltage");
    }

    /**
     * Stop the roller motor.
     */
    public Command stopRollerCmd() {
        return Commands.runOnce(io::stopRoller, this)
            .withName("StopRoller");
    }

    /**
     * Stop both pivot and roller.
     */
    public Command stopCmd() {
        return Commands.runOnce(() -> {
            io.stopPivot();
            io.stopRoller();
        }, this)
            .withName("StopIntake");
    }

    // ============ STATE QUERIES ============

    /**
     * Check if the pivot is at the deployed position (within tolerance).
     */
    public boolean isDeployed() {
        return isPivotAtPosition(IntakeConstants.kDeployedPosition);
    }

    /**
     * Check if the pivot is at the stowed position (within tolerance).
     */
    public boolean isStowed() {
        return isPivotAtPosition(IntakeConstants.kStowedPosition);
    }

    /**
     * Check if the pivot is at a specific position (within tolerance).
     */
    public boolean isPivotAtPosition(double positionRotations) {
        double error = Math.abs(inputs.pivotPositionRotations - positionRotations);
        return error < IntakeConstants.kPivotTolerance;
    }

    /**
     * Check if the pivot is at its setpoint (used for deployment).
     */
    @AutoLogOutput(key = "Intake/PivotAtSetpoint")
    public boolean isPivotAtSetpoint() {
        // This is a placeholder - update based on your actual control logic
        return false;
    }

    /**
     * Get the current pivot position in rotations.
     */
    @AutoLogOutput(key = "Intake/PivotPosition")
    public double getPivotPosition() {
        return inputs.pivotPositionRotations;
    }

    /**
     * Get the current pivot velocity in rotations per second.
     */
    @AutoLogOutput(key = "Intake/PivotVelocity")
    public double getPivotVelocity() {
        return inputs.pivotVelocityRotPerSec;
    }

    /**
     * Get the absolute encoder position in rotations.
     */
    @AutoLogOutput(key = "Intake/EncoderAbsolutePosition")
    public double getPivotEncoderAbsolutePosition() {
        return inputs.pivotEncoderAbsolutePosition;
    }

    /**
     * Get the current roller velocity in rotations per second.
     */
    @AutoLogOutput(key = "Intake/RollerVelocity")
    public double getRollerVelocity() {
        return inputs.rollerVelocityRotPerSec;
    }

    /**
     * Check if the pivot motor is connected.
     */
    public boolean isPivotConnected() {
        return inputs.pivotConnected;
    }

    /**
     * Check if the roller motor is connected.
     */
    public boolean isRollerConnected() {
        return inputs.rollerConnected;
    }

    /**
     * Reset the pivot encoder to the absolute position.
     */
    public void resetPivotEncoder() {
        io.resetPivotEncoder();
    }
}
