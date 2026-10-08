package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * Hardware implementation of the intake subsystem using CTRE TalonFX motors and CANcoder.
 */
public class IntakeIOHardware implements IntakeIO {
    private final TalonFX pivotMotor;
    private final TalonFX rollerMotor;
    private final CANcoder pivotEncoder;
    private final PIDController pivotPIDController;

    // Status signals for efficient logging
    private final StatusSignal<Angle> pivotPosition;
    private final StatusSignal<AngularVelocity> pivotVelocity;
    private final StatusSignal<Voltage> pivotAppliedVoltage;
    private final StatusSignal<Current> pivotCurrent;
    private final StatusSignal<Angle> pivotEncoderAbsolute;

    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerAppliedVoltage;
    private final StatusSignal<Current> rollerCurrent;

    public IntakeIOHardware() {
        // Initialize motors
        pivotMotor = new TalonFX(IntakeConstants.kPivotMotorCANId, IntakeConstants.kCANBus.getName());
        rollerMotor = new TalonFX(IntakeConstants.kRollerMotorCANId, IntakeConstants.kCANBus.getName());
        pivotEncoder = new CANcoder(IntakeConstants.kPivotEncoderCANId, IntakeConstants.kCANBus.getName());

        // Initialize PID controller for pivot
        pivotPIDController = new PIDController(
            IntakeConstants.kPivotKP,
            IntakeConstants.kPivotKI,
            IntakeConstants.kPivotKD
        );
        pivotPIDController.setTolerance(IntakeConstants.kPivotTolerance);
        pivotPIDController.enableContinuousInput(-0.5, 0.5); // For rotations wrapping

        // Configure pivot motor
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.kPivotCurrentLimit;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.MotorOutput.Inverted = IntakeConstants.kPivotInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.Feedback.FeedbackRemoteSensorID = IntakeConstants.kPivotEncoderCANId;
        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        pivotMotor.getConfigurator().apply(pivotConfig);

        // Configure roller motor
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.kRollerCurrentLimit;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.MotorOutput.Inverted = IntakeConstants.kRollerInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerMotor.getConfigurator().apply(rollerConfig);

        // Configure CANcoder
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        pivotEncoder.getConfigurator().apply(encoderConfig);

        // Get status signals for efficient updates
        pivotPosition = pivotMotor.getPosition();
        pivotVelocity = pivotMotor.getVelocity();
        pivotAppliedVoltage = pivotMotor.getMotorVoltage();
        pivotCurrent = pivotMotor.getStatorCurrent();
        pivotEncoderAbsolute = pivotEncoder.getAbsolutePosition();

        rollerVelocity = rollerMotor.getVelocity();
        rollerAppliedVoltage = rollerMotor.getMotorVoltage();
        rollerCurrent = rollerMotor.getStatorCurrent();

        // Optimize update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            pivotPosition,
            pivotVelocity,
            pivotAppliedVoltage,
            pivotCurrent,
            pivotEncoderAbsolute,
            rollerVelocity,
            rollerAppliedVoltage,
            rollerCurrent
        );

        pivotMotor.optimizeBusUtilization();
        rollerMotor.optimizeBusUtilization();
        pivotEncoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.pivotConnected = BaseStatusSignal.refreshAll(
            pivotPosition,
            pivotVelocity,
            pivotAppliedVoltage,
            pivotCurrent,
            pivotEncoderAbsolute
        ).isOK();

        // Convert Phoenix6 units to rotations
        inputs.pivotPositionRotations = pivotPosition.getValueAsDouble();
        inputs.pivotVelocityRotPerSec = pivotVelocity.getValueAsDouble();
        inputs.pivotAppliedVolts = pivotAppliedVoltage.getValueAsDouble();
        inputs.pivotCurrentAmps = pivotCurrent.getValueAsDouble();
        inputs.pivotEncoderAbsolutePosition = pivotEncoderAbsolute.getValueAsDouble();

        inputs.rollerConnected = BaseStatusSignal.refreshAll(
            rollerVelocity,
            rollerAppliedVoltage,
            rollerCurrent
        ).isOK();

        inputs.rollerVelocityRotPerSec = rollerVelocity.getValueAsDouble();
        inputs.rollerAppliedVolts = rollerAppliedVoltage.getValueAsDouble();
        inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivotMotor.setVoltage(volts);
    }

    @Override
    public void setPivotPosition(double positionRotations) {
        double pidOutput = pivotPIDController.calculate(
            pivotPosition.getValueAsDouble(),
            positionRotations
        );
        // Clamp to reasonable voltage range
        double voltage = Math.max(-12.0, Math.min(12.0, pidOutput));
        setPivotVoltage(voltage);
    }

    @Override
    public void setRollerVoltage(double volts) {
        rollerMotor.setVoltage(volts);
    }

    @Override
    public void stopPivot() {
        pivotMotor.setVoltage(0);
    }

    @Override
    public void stopRoller() {
        rollerMotor.setVoltage(0);
    }

    @Override
    public void resetPivotEncoder() {
        pivotMotor.setPosition(pivotEncoderAbsolute.getValueAsDouble());
    }
}
