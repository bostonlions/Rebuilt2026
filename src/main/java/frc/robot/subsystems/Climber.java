package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static edu.wpi.first.units.Units.Meters;

import frc.robot.Robot.Ports;

import static frc.robot.Robot.kCANBusJustice;
import static frc.robot.RobotContainer.ControlBoard.CustomXboxController.scaleWithDeadband;

/**
 * Elevator open-loop from operator left stick. Up / down unless CANrange is within tolerance of top/bottom;
 * centered stick commands 0 duty immediately.
 */
public final class Climber extends SubsystemBase {
    private static Climber instance = null;

    public static final double elevatorCANrangeZero = 0.022;
    public static final double elevatorCANrangeTop = 0.27;
    /** Fraction of reference distance for top and bottom stop bands. */
    public static final double elevatorCANrangeTolerancePercent = 0.03;

    private static final double elevatorCANrangeMax = 0.35;
    private static final double elevatorStdDevDistThreshold = 0.12;
    private static final double elevatorMaxStdDev = 0.015;

    private static final double kStickDeadband = 0.25;
    private static final double kStickThreshold = 0.15;
    /** Open-loop duty magnitude while moving (sign from direction). */
    private static final double kOpenLoopDuty = 0.95;

    private final TalonFX elevatorMotor = new TalonFX(Ports.ELEVATOR_MOTOR, kCANBusJustice);
    private final CANrange canRange = new CANrange(Ports.CANRANGE, kCANBusJustice);

    private double elevatorMinTorque = 0, elevatorMaxTorque = 0;
    private double lastCommandedDuty = 0.0;

    public static Climber getInstance() {
        if (instance == null) instance = new Climber();
        return instance;
    }

    private Climber() {
        canRange.getConfigurator().apply(new CANrangeConfiguration()
            .withFovParams(new FovParamsConfigs().withFOVRangeX(7).withFOVRangeY(7)));
        elevatorMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(new Slot0Configs().withKP(0.025))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(500)
                .withMotionMagicCruiseVelocity(80))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(20)
                .withPeakReverseTorqueCurrent(-20)));
    }

    private double toleranceFor(double referenceM) {
        return Math.max(0.001, referenceM * elevatorCANrangeTolerancePercent);
    }

    private boolean withinToleranceOfTop(double distM) {
        return Math.abs(distM - elevatorCANrangeTop) <= toleranceFor(elevatorCANrangeTop);
    }

    private boolean withinToleranceOfBottom(double distM) {
        return Math.abs(distM - elevatorCANrangeZero) <= toleranceFor(elevatorCANrangeZero);
    }

    private boolean canRangeFault() {
        double d = getElevatorCANrangeMeters();
        double std = canRange.getDistanceStdDev().getValueAsDouble();
        return d > elevatorCANrangeMax
            || ((std == 0 || std > elevatorMaxStdDev) && d < elevatorStdDevDistThreshold);
    }

    private double getElevatorCANrangeMeters() {
        return canRange.getDistance().getValue().in(Meters);
    }

    /**
     * Call each teleop loop with operator left stick Y (raw). Centered stick → motor off immediately.
     * Xbox: up is negative; up runs {@code +kOpenLoopDuty} unless near top, down runs negative unless near bottom.
     */
    public void driveFromStick(double rawOperatorLeftStickY) {
        if (!DriverStation.isEnabled()) {
            return;
        }
        double y = scaleWithDeadband(rawOperatorLeftStickY, kStickDeadband);
        double dist = getElevatorCANrangeMeters();

        if (canRangeFault()) {
            lastCommandedDuty = 0.0;
            elevatorMotor.setControl(new DutyCycleOut(0));
            return;
        }

        if (Math.abs(y) < kStickThreshold) {
            // hold: stop immediately
            lastCommandedDuty = 0.0;
            elevatorMotor.setControl(new DutyCycleOut(0));
            return;
        }

        if (y < -kStickThreshold) {
            // stick up → move up
            if (withinToleranceOfTop(dist)) {
                lastCommandedDuty = 0.0;
                elevatorMotor.setControl(new DutyCycleOut(0));
            } else {
                lastCommandedDuty = kOpenLoopDuty;
                elevatorMotor.setControl(new DutyCycleOut(kOpenLoopDuty));
            }
            return;
        }

        // stick down → move down
        if (withinToleranceOfBottom(dist)) {
            lastCommandedDuty = 0.0;
            elevatorMotor.setControl(new DutyCycleOut(0));
        } else {
            lastCommandedDuty = -kOpenLoopDuty;
            elevatorMotor.setControl(new DutyCycleOut(-kOpenLoopDuty));
        }
    }

    public SequentialCommandGroup elevatorUp() {
        return new InstantCommand(() -> elevatorMotor.setControl(new DutyCycleOut(kOpenLoopDuty)), this).andThen(
            new WaitUntilCommand(() -> withinToleranceOfTop(getElevatorCANrangeMeters())).andThen(
                () -> elevatorMotor.setControl(new DutyCycleOut(0)), this
            )
        );
    }

    public SequentialCommandGroup elevatorDown() {
        return new InstantCommand(() -> elevatorMotor.setControl(new DutyCycleOut(-kOpenLoopDuty)), this).andThen(
            new WaitUntilCommand(() -> withinToleranceOfBottom(getElevatorCANrangeMeters())).andThen(
                () -> elevatorMotor.setControl(new DutyCycleOut(0)), this
            )
        );
    }

    public void resetZerosAndTargetState() {
        elevatorMotor.setControl(new DutyCycleOut(0));
        lastCommandedDuty = 0.0;
    }

    @Override
    public void simulationPeriodic() {
        canRange.getSimState().setDistance(0.1);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            elevatorMotor.setControl(new DutyCycleOut(0));
            lastCommandedDuty = 0.0;
            return;
        }
        double elevatorTorque = elevatorMotor.getTorqueCurrent().getValueAsDouble();
        if (elevatorTorque < elevatorMinTorque) elevatorMinTorque = elevatorTorque;
        if (elevatorTorque > elevatorMaxTorque) elevatorMaxTorque = elevatorTorque;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Climber");
        builder.setActuator(true);

        builder.addDoubleProperty("Elevator CANrange (m)", this::getElevatorCANrangeMeters, null);
        builder.addDoubleProperty("Elevator CANrange stdev", () -> canRange.getDistanceStdDev().getValueAsDouble(), null);
        builder.addDoubleProperty("elevatorCANrangeZero", () -> elevatorCANrangeZero, null);
        builder.addDoubleProperty("elevatorCANrangeTop", () -> elevatorCANrangeTop, null);
        builder.addDoubleProperty("elevatorCANrangeTolerancePercent", () -> elevatorCANrangeTolerancePercent, null);
        builder.addDoubleProperty("lastCommandedDuty", () -> lastCommandedDuty, null);
        builder.addBooleanProperty("withinTopBand", () -> withinToleranceOfTop(getElevatorCANrangeMeters()), null);
        builder.addBooleanProperty("withinBottomBand", () -> withinToleranceOfBottom(getElevatorCANrangeMeters()), null);
        builder.addBooleanProperty("canRangeFault", this::canRangeFault, null);
        builder.addDoubleProperty("elevatorMinTorque", () -> elevatorMinTorque, null);
        builder.addDoubleProperty("elevatorMaxTorque", () -> elevatorMaxTorque, null);
        builder.addDoubleProperty("elevatorVelocity", () -> elevatorMotor.getVelocity().getValueAsDouble(), null);
    }
}
