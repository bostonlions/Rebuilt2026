package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;

import frc.robot.Robot.Ports;

import static frc.robot.Robot.kCANBusJustice;
import static frc.robot.RobotContainer.ControlBoard.CustomXboxController.scaleWithDeadband;

import java.util.Map;

public final class Climber extends SubsystemBase {
    private static Climber instance = null;
    private enum Position {Clinch, Grab, Stow, Bottom, L1, Top}
    private enum Request {Stow, Ready, L1, L3}
    private Request request = Request.Ready;
    private final double lowerClosedLoopErrorTolerance = 0.006944;
    private final double upperClosedLoopErrorTolerance = Millimeters.of(5).magnitude() / (24.0/45.0 *
        Inches.of(0.375).in(Millimeters));
    private final double elevatorClosedLoopErrorTolerance = 1.71455;
    private final double elevatorForceTorque = 20;
    private final double elevatorForceVelocityLimit = 0.1;
    private final double hookForceTorque = -20;
    private final double hookForceVelocityLimit = -0.1;
    private final double hookLimitRotations = -0.5; // FIXME
    // private final CANrange canRange = new CANrange(Ports.CANRANGE, kCANBus);
    private final Map<Position, Angle> elevatorPositions = Map.of( // rotations are motor rotations;
        Position.Stow, Rotations.of(0),
        Position.Bottom, Rotations.of(0),                       // 15 elevator motor rotations = 4 cm height change
        Position.L1, Rotations.of(-61.8),      //FIXME
        Position.Top, Rotations.of(-81.8)
    );
    private final Map<Position, Angle> lowerHookPositions = Map.of( // rotations are of the hook itself
        Position.Stow, Rotations.of(0.3698),
        Position.Clinch, Rotations.of(0.5890), //FIXME
        Position.Grab, Rotations.of(0.6282)
    );
    private final Map<Position, Angle> upperHookPositions = Map.of( // rotations are motor rotations
        Position.Stow, Rotations.of(0),
        Position.Bottom, Rotations.of(83.9),
        Position.L1, Rotations.of(42.2),      //FIXME
        Position.Top, Rotations.of(22.2)
    );

    private final MotionMagicVoltage motion = new MotionMagicVoltage(0);
    private final TalonFX lowerHookMotor = new TalonFX(Ports.LOWER_HOOK_MOTOR, kCANBusJustice);
    private final TalonFX upperHookMotor = new TalonFX(Ports.UPPER_HOOK_MOTOR, kCANBusJustice);
    private final TalonFX elevatorMotor = new TalonFX(Ports.ELEVATOR_MOTOR, kCANBusJustice);
    private final CANcoder lowerHookCANcoder = new CANcoder(Ports.LOWER_HOOK_CANCODER, kCANBusJustice);

    public static Climber getInstance() {
        if (instance == null) instance = new Climber();
        return instance;
    }

    private Climber() {
        // canRange.getConfigurator().apply(new CANrangeConfiguration());
        lowerHookCANcoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(0).withAbsoluteSensorDiscontinuityPoint(1));
        // Force the cancoder into the 0-1 range
        lowerHookCANcoder.setPosition(MathUtil.inputModulus(lowerHookCANcoder.getPosition().getValueAsDouble(), 0, 1));
        lowerHookMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
            .withSlot0(new Slot0Configs()
                .withKP(1.65)
                .withKI(0.04)
                .withKD(0.012)
                .withKV(0.12)
                .withKA(0.01)
                .withKS(0.05))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(10)
                .withMotionMagicCruiseVelocity(5))
            .withFeedback(new FeedbackConfigs()
                .withFeedbackRemoteSensorID(Ports.LOWER_HOOK_CANCODER)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withSensorToMechanismRatio(1)
                .withRotorToSensorRatio(5 * 9 * 72 / 20.0))
        );
        upperHookMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(0.025))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(30)
                .withMotionMagicCruiseVelocity(30))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(30)
                .withPeakReverseTorqueCurrent(-30))
        );
        elevatorMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
            .withSlot0(new Slot0Configs().withKP(0.025))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(30)
                .withMotionMagicCruiseVelocity(30))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(10)
                .withPeakReverseTorqueCurrent(-10))
        );

        // if (!request(Request.Stow).isOK()) throw new IllegalStateException("Couldn't stow climber; it began in state " + request);
    }

    /** used for testing only */
    private boolean manualLH = false, manualUH = false, manualElevator = false;
    public void move(double joyValueUpHook, double joyValueLowHook, double joyValueElevate) {
        double scaledLH = scaleWithDeadband(joyValueLowHook, 0.25);
        double scaledUH = scaleWithDeadband(joyValueUpHook, 0.25);
        double scaledElevator = scaleWithDeadband(joyValueElevate, 0.25);
        if (scaledLH != 0 || manualLH) {
            lowerHookMotor.setControl(new MotionMagicVelocityDutyCycle(25 * scaledLH));
            manualLH = scaledLH != 0;
        }
        if (scaledUH != 0 || manualUH) {
            upperHookMotor.setControl(new MotionMagicVelocityDutyCycle(25 * scaledUH));
            manualUH = scaledUH != 0;
        }
        if (scaledElevator != 0 || manualElevator) {
            elevatorMotor.setControl(new MotionMagicVelocityDutyCycle(25 * scaledElevator));
            manualElevator = scaledElevator != 0;
        }
    }

    public StatusCode request(final Request request) {
        if (request == Request.Stow && this.request == Request.Ready) {
            CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                    new InstantCommand(() -> elevatorMotor.setControl(motion.withPosition(elevatorPositions.get(Position.Bottom))))
                        .andThen(
                            new WaitCommand(2.5),
                            new InstantCommand(() ->
                                upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Stow))), this)
                        ),
                    new InstantCommand(() -> lowerHookMotor.setControl(motion.withPosition(lowerHookPositions.get(Position.Stow))))
                )
             );
             return StatusCode.OK;
        } else if (request == Request.Ready && this.request == Request.Stow) {
            return upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Top))).isOK()
                && elevatorMotor.setControl(motion.withPosition(elevatorPositions.get(Position.Top))).isOK() ?
                StatusCode.OK : StatusCode.GeneralError;
        } else if (request == Request.L1 && this.request == Request.Ready) {
            return upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.L1))).isOK()
                && elevatorMotor.setControl(motion.withPosition(elevatorPositions.get(Position.L1))).isOK() ?
                StatusCode.OK : StatusCode.GeneralError;
        } else if (request == Request.Ready && this.request == Request.L1) {
            return upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Top))).isOK()
                && elevatorMotor.setControl(motion.withPosition(elevatorPositions.get(Position.Top))).isOK() ?
                StatusCode.OK : StatusCode.GeneralError;
        } else if (request == Request.L3 && this.request == Request.Ready) {
            CommandScheduler.getInstance().schedule(
                new InstantCommand(() -> {
                    upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Bottom)));
                    elevatorMotor.setControl(motion.withPosition(elevatorPositions.get(Position.Bottom)));
                }, this).andThen(
                new WaitUntilCommand(
                    () -> upperHookMotor.getClosedLoopError().getValue() < upperClosedLoopErrorTolerance &&
                    elevatorMotor.getClosedLoopError().getValue() < elevatorClosedLoopErrorTolerance
                ).andThen(() -> lowerHookMotor.setControl(motion.withPosition(lowerHookPositions.get(Position.Grab))), this),
                new WaitUntilCommand(() -> lowerHookMotor.getClosedLoopError().getValue() < lowerClosedLoopErrorTolerance)
                    .andThen(() -> {
                        elevatorMotor.setControl(motion.withPosition(elevatorPositions.get(Position.Top)));
                        upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Top)));
                    }, this),
                new WaitUntilCommand(
                    () -> upperHookMotor.getClosedLoopError().getValue() < upperClosedLoopErrorTolerance &&
                        elevatorMotor.getClosedLoopError().getValue() < elevatorClosedLoopErrorTolerance
                ).andThen(() -> lowerHookMotor.setControl(motion.withPosition(lowerHookPositions.get(Position.Clinch))), this),
                new WaitUntilCommand(() -> lowerHookMotor.getClosedLoopError().getValue() < lowerClosedLoopErrorTolerance)
                    .andThen(new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Bottom)));
                            elevatorMotor.setControl(motion.withPosition(elevatorPositions.get(Position.Bottom)));
                        }, this),
                        new WaitUntilCommand(
                            () -> elevatorMotor.getPosition().getValueAsDouble() + upperHookMotor.getPosition().getValueAsDouble() < -1 // FIXME
                        ).andThen(() -> lowerHookMotor.setControl(motion.withPosition(lowerHookPositions.get(Position.Stow))), this)
                    )),
                new WaitUntilCommand(
                    () -> upperHookMotor.getClosedLoopError().getValue() < upperClosedLoopErrorTolerance &&
                    lowerHookMotor.getClosedLoopError().getValue() < lowerClosedLoopErrorTolerance &&
                    elevatorMotor.getClosedLoopError().getValue() < elevatorClosedLoopErrorTolerance
                ).andThen(() -> lowerHookMotor.setControl(motion.withPosition(lowerHookPositions.get(Position.Grab))), this),
                new WaitUntilCommand(() -> lowerHookMotor.getClosedLoopError().getValue() < lowerClosedLoopErrorTolerance)
                    .andThen(() -> {
                        elevatorMotor.setControl(motion.withPosition(elevatorPositions.get(Position.Top)));
                        upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Top)));
                    }, this),
                new WaitUntilCommand(
                    () -> upperHookMotor.getClosedLoopError().getValue() < upperClosedLoopErrorTolerance &&
                    elevatorMotor.getClosedLoopError().getValue() < elevatorClosedLoopErrorTolerance
                ).andThen(() -> lowerHookMotor.setControl(motion.withPosition(lowerHookPositions.get(Position.Clinch))), this),
                new WaitUntilCommand(() -> lowerHookMotor.getClosedLoopError().getValue() < lowerClosedLoopErrorTolerance)
                    .andThen(new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Bottom)));
                            elevatorMotor.setControl(motion.withPosition(elevatorPositions.get(Position.Bottom)));
                        }, this),
                        new WaitUntilCommand(
                            () -> elevatorMotor.getPosition().getValueAsDouble() + upperHookMotor.getPosition().getValueAsDouble() < -1 // FIXME
                        ).andThen(() -> lowerHookMotor.setControl(motion.withPosition(lowerHookPositions.get(Position.Stow))), this)
                    )))
            );
            return StatusCode.OK;
        } else return StatusCode.GeneralError;
    }

    private void setElevator(Position target) {
        Angle pos = elevatorPositions.get(target);
        if (pos == null) pos = elevatorPositions.get(Position.Stow);
        elevatorMotor.setControl(new MotionMagicDutyCycle(pos));
    }

    private void setUpperHooks(Position target) {
        Angle pos = upperHookPositions.get(target);
        if (pos == null) pos = upperHookPositions.get(Position.Stow);
        upperHookMotor.setControl(new MotionMagicDutyCycle(pos));
    }

    private void setLowerHooks(Position target) {
        Angle pos = lowerHookPositions.get(target);
        if (pos == null) pos = lowerHookPositions.get(Position.Stow);
        System.out.println("setLowerHooks " + pos);
        lowerHookMotor.setControl(new MotionMagicDutyCycle(pos.in(Rotations)));
    }

    private double elevatorMinTorque = 0, elevatorMaxTorque = 0, hookMinTorque = 0, hookMaxTorque = 0;

    private boolean forcingElevatorDown = false;
    private boolean forcingHooksUp = false;
    public void forceStow() {
        forcingElevatorDown = true;
        forcingHooksUp = true;
        elevatorMotor.setControl(new DutyCycleOut(0.05));
        upperHookMotor.setControl(new DutyCycleOut(-0.1));
        setLowerHooks(Position.Stow);
    }

    @Override
    public void periodic() {
        double elevatorTorque = elevatorMotor.getTorqueCurrent().getValueAsDouble();
        if (elevatorTorque < elevatorMinTorque) elevatorMinTorque = elevatorTorque;
        if (elevatorTorque > elevatorMaxTorque) elevatorMaxTorque = elevatorTorque;

        double upperHooksTorque = upperHookMotor.getTorqueCurrent().getValueAsDouble();
        if (upperHooksTorque < hookMinTorque) hookMinTorque = upperHooksTorque;
        if (upperHooksTorque > hookMaxTorque) hookMaxTorque = upperHooksTorque;

        if (
            forcingElevatorDown &&
            (elevatorTorque > elevatorForceTorque) &&
            (elevatorMotor.getVelocity().getValueAsDouble() < elevatorForceVelocityLimit)
        ) {
            System.out.println("Elevator bottom limit hit, marking min height");
            elevatorMotor.setPosition(0);
            forcingElevatorDown = false;
            setElevator(Position.Stow);
        } else if (forcingElevatorDown) {
            System.out.println("Still forcing elevator down");
        }
        if (
            forcingHooksUp &&
            (upperHooksTorque < hookForceTorque) &&
            (upperHookMotor.getVelocity().getValueAsDouble() > hookForceVelocityLimit)
        ) {
            System.out.println("Upper hooks stow limit hit, marking max height");
            upperHookMotor.setPosition(hookLimitRotations);
            forcingHooksUp = false;
            setUpperHooks(Position.Stow);
        } else if (forcingHooksUp) {
            System.out.println("Still forcing hooks up");
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Climber");
        builder.setActuator(true);

        builder.addDoubleProperty("Elevator rot", () -> elevatorMotor.getPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("Upper hook rot", () -> upperHookMotor.getPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("Lower hook rot", () -> lowerHookMotor.getPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("Lower hook cc", () -> lowerHookCANcoder.getPosition().getValueAsDouble(), null);

        builder.addDoubleProperty("elevatorMinTorque", () -> elevatorMinTorque, null);
        builder.addDoubleProperty("elevatorMaxTorque", () -> elevatorMaxTorque, null);
        builder.addDoubleProperty("elevatorVelocity", () -> elevatorMotor.getVelocity().getValueAsDouble(), null);
        builder.addDoubleProperty("hookMinTorque", () -> hookMinTorque, null);
        builder.addDoubleProperty("hookMaxTorque", () -> hookMaxTorque, null);
        builder.addDoubleProperty("hookVelocity", () -> upperHookMotor.getVelocity().getValueAsDouble(), null);
        builder.addBooleanProperty("forcingElevatorDown", () -> forcingElevatorDown, null);
        builder.addBooleanProperty("forcingHooksUp", () -> forcingHooksUp, null);
        builder.addBooleanProperty("manualLH", () -> manualLH, null);
        builder.addBooleanProperty("manualUH", () -> manualUH, null);
        builder.addBooleanProperty("manualElevator", () -> manualElevator, null);
    }
}
