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
    /** Target position for lower hook from CANcoder (rotations, 0–1). From Phoenix Tuner. */
    public static final double kLowerHookTargetCanCoderPosition = 0.557;

    /**
     * Target position for upper hook in motor rotations from the zero point (position when teleop enabled).
     * On left stick click, upper hook goes to (zero + this value). Tune to get the desired location.
     */
    public static final double kUpperHookTargetRotationsFromZero = 24.0;

    /** Tolerance in motor rotations to consider upper hook "at" the target (avoids re-running on double-click). */
    private static final double kUpperHookAtTargetToleranceRotations =1;

    /** Upper hook motor position when teleop was enabled; used as zero for target calculation. */
    private double upperHookZeroPosition = 0.0;

    /**
     * Target position for elevator in motor rotations from the zero point (position when teleop enabled).
     * Same pattern as upper hook; call moveElevatorToTarget() to go to (zero + this value).
     */
    public static final double kElevatorTargetRotationsFromZero = -83.5; //(roughly 1" per 10 rotation)

    /** Tolerance in motor rotations to consider elevator "at" the target. */
    private static final double kElevatorAtTargetToleranceRotations 
    /** Elevator motor position when teleop was enabled; used as zero for target calculation. */
    private double elevatorZeroPosition = 0.0;

    /**
     * Left stick pull-down targets (rotations from zero). Upper hook goes here first, then elevator.
     * Push-up uses kUpperHookTargetRotationsFromZero and kElevatorTargetRotationsFromZero (same as right-stick click).
     */
    public static final double kUpperHookPullDownTargetRotationsFromZero = 60.0;
    public static final double kElevatorPullDownTargetRotationsFromZero = -40.0;

    /** Left stick Y must exceed this to enter target mode (push up or pull down); else motion stops. */
    private static final double kLeftStickTargetModeThreshold = 0.15;

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
    /** True when left stick was in target mode last cycle; used to hold position on release instead of cutting power. */
    private boolean leftStickWasInTargetMode = false;
    /** Last position target we sent to upper hook (motor rotations); only re-send when target changes to avoid glitches. */
    private double lastUpperHookTargetRotations = Double.NaN;
    /** Last position target we sent to elevator (motor rotations). */
    private double lastElevatorTargetRotations = Double.NaN;
    /** Treat targets within this many rotations as "same" so we don't constantly re-command. */
    private static final double kTargetChangeEpsilonRotations = 0.02;
    /** Debug: print every this many move() calls when in target mode (~1 sec). Set 0 to disable. */
    private static final int kElevatorDebugPrintInterval = 50;
    private int moveCallCount = 0;

    /**
     * Left stick Y: pull down = go to initial/right-stick position; push up = upper to 30 from zero, then elevator to -40.
     * We only send a new position command when the target actually changes (encoder-based targets); on release we hold current position.
     */
    public void move(double joyValueUpHook, double joyValueLowHook, double joyValueElevate) {
        double scaledUH = scaleWithDeadband(joyValueUpHook, 0.25);
        double scaledElevator = scaleWithDeadband(joyValueElevate, 0.25);

        if (Math.abs(scaledUH) > kLeftStickTargetModeThreshold) {
            if (scaledUH < 0) {
                // Pull down: both to initial/right-stick position
                double targetUpper = upperHookZeroPosition + kUpperHookTargetRotationsFromZero;
                double targetElevator = elevatorZeroPosition + kElevatorTargetRotationsFromZero;
                if (targetChanged(targetUpper, lastUpperHookTargetRotations)) {
                    upperHookMotor.setControl(new MotionMagicDutyCycle(Rotations.of(targetUpper)));
                    lastUpperHookTargetRotations = targetUpper;
                }
                elevatorMotor.setControl(new MotionMagicDutyCycle(Rotations.of(targetElevator)));
                lastElevatorTargetRotations = targetElevator;
                if (kElevatorDebugPrintInterval > 0 && (++moveCallCount % kElevatorDebugPrintInterval == 0)) {
                    double curElev = elevatorMotor.getPosition().getValueAsDouble();
                    System.out.println("[Climber] PULL DOWN: elevatorZero=" + elevatorZeroPosition + " targetElev=" + targetElevator + " currentElev=" + curElev + " (commanding elevator every cycle)");
                }
            } else {
                // Push up: upper hook to target first, then elevator to target once upper is there
                double targetUpper = upperHookZeroPosition + kUpperHookPullDownTargetRotationsFromZero;
                if (targetChanged(targetUpper, lastUpperHookTargetRotations)) {
                    upperHookMotor.setControl(new MotionMagicDutyCycle(Rotations.of(targetUpper)));
                    lastUpperHookTargetRotations = targetUpper;
                }
                double currentUpper = upperHookMotor.getPosition().getValueAsDouble();
                double upperErr = Math.abs(currentUpper - targetUpper);
                boolean upperAtTarget = upperErr <= kUpperHookAtTargetToleranceRotations;
                if (upperAtTarget) {
                    double targetElevator = elevatorZeroPosition + kElevatorPullDownTargetRotationsFromZero;
                    elevatorMotor.setControl(new MotionMagicDutyCycle(Rotations.of(targetElevator)));
                    lastElevatorTargetRotations = targetElevator;
                }
                if (kElevatorDebugPrintInterval > 0 && (++moveCallCount % kElevatorDebugPrintInterval == 0)) {
                    double curElev = elevatorMotor.getPosition().getValueAsDouble();
                    double targetElevator = elevatorZeroPosition + kElevatorPullDownTargetRotationsFromZero;
                    System.out.println("[Climber] PUSH UP: upperCur=" + currentUpper + " upperTgt=" + targetUpper + " upperErr=" + String.format("%.3f", upperErr) + " tol=" + kUpperHookAtTargetToleranceRotations + " upperAtTarget=" + upperAtTarget
                        + " | elevatorZero=" + elevatorZeroPosition + " targetElev=" + targetElevator + " currentElev=" + curElev + " commandingElevator=" + upperAtTarget);
                }
            }
            manualUH = false;
            manualElevator = false;
            leftStickWasInTargetMode = true;
        } else {
            moveCallCount = 0;
            if (leftStickWasInTargetMode) {
                double holdUpper = upperHookMotor.getPosition().getValueAsDouble();
                double holdElevator = elevatorMotor.getPosition().getValueAsDouble();
                upperHookMotor.setControl(new MotionMagicDutyCycle(Rotations.of(holdUpper)));
                elevatorMotor.setControl(new MotionMagicDutyCycle(Rotations.of(holdElevator)));
                lastUpperHookTargetRotations = Double.NaN;
                lastElevatorTargetRotations = Double.NaN;
            }
            leftStickWasInTargetMode = false;
            manualUH = false;
            manualElevator = false;
        }

        if (Math.abs(scaledUH) <= kLeftStickTargetModeThreshold && (scaledUH != 0 || manualUH)) {
            upperHookMotor.setControl(new MotionMagicVelocityDutyCycle(25 * scaledUH));
            manualUH = scaledUH != 0;
            lastUpperHookTargetRotations = Double.NaN;
        }
        if (Math.abs(scaledUH) <= kLeftStickTargetModeThreshold && (scaledElevator != 0 || manualElevator)) {
            elevatorMotor.setControl(new MotionMagicVelocityDutyCycle(25 * scaledElevator));
            manualElevator = scaledElevator != 0;
            lastElevatorTargetRotations = Double.NaN;
        }
    }

    private static boolean targetChanged(double newTarget, double lastTarget) {
        if (Double.isNaN(lastTarget)) return true;
        return Math.abs(newTarget - lastTarget) > kTargetChangeEpsilonRotations;
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

    /**
     * Sets the current upper hook motor position as the zero point (call when teleop is enabled).
     */
    public void setUpperHookZeroFromCurrentPosition() {
        upperHookZeroPosition = upperHookMotor.getPosition().getValueAsDouble();
    }

    /**
     * Sets the current elevator motor position as the zero point (call when teleop is enabled).
     */
    public void setElevatorZeroFromCurrentPosition() {
        elevatorZeroPosition = elevatorMotor.getPosition().getValueAsDouble();
    }

    /**
     * Resets zero points from current encoder positions and clears saved target state.
     * Call whenever the robot is enabled (e.g. in teleop) so "starting position" is always where you are now.
     */
    public void resetZerosAndTargetState() {
        upperHookZeroPosition = upperHookMotor.getPosition().getValueAsDouble();
        elevatorZeroPosition = elevatorMotor.getPosition().getValueAsDouble();
        lastUpperHookTargetRotations = Double.NaN;
        lastElevatorTargetRotations = Double.NaN;
        leftStickWasInTargetMode = false;
    }

    /**
     * On right stick click: moves upper hook, lower hook, and elevator to their targets.
     * Always sends the position commands so the button reliably runs the mechanism to the preset position.
     */
    public void moveUpperHookToTargetAndLowerHookToConstant() {
        double targetUpper = upperHookZeroPosition + kUpperHookTargetRotationsFromZero;
        double targetElevator = elevatorZeroPosition + kElevatorTargetRotationsFromZero;
        upperHookMotor.setControl(new MotionMagicDutyCycle(Rotations.of(targetUpper)));
        lowerHookMotor.setControl(new MotionMagicDutyCycle(Rotations.of(kLowerHookTargetCanCoderPosition)));
        elevatorMotor.setControl(new MotionMagicDutyCycle(Rotations.of(targetElevator)));
        lastUpperHookTargetRotations = targetUpper;
        lastElevatorTargetRotations = targetElevator;
    }

    /**
     * Moves elevator to (zero + kElevatorTargetRotationsFromZero). If already at target (within tolerance),
     * does nothing. Call from a button or command when you want to go to the preset elevator height.
     */
    public void moveElevatorToTarget() {
        double targetElevator = elevatorZeroPosition + kElevatorTargetRotationsFromZero;
        double currentElevator = elevatorMotor.getPosition().getValueAsDouble();
        if (Math.abs(currentElevator - targetElevator) <= kElevatorAtTargetToleranceRotations) {
            return;
        }
        elevatorMotor.setControl(new MotionMagicDutyCycle(Rotations.of(targetElevator)));
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
