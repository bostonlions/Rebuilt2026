package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;

import frc.robot.Robot.Ports;

import static frc.robot.Robot.kCANBusJustice;
import static frc.robot.RobotContainer.ControlBoard.CustomXboxController.scaleWithDeadband;

import java.util.Map;

public final class Climber extends SubsystemBase {
    private static Climber instance = null;
    public enum Position {Clinch, Hook, Prepare, Grab, Stow, Bottom, L1, Top, ClearBumper}
    private enum Request {Stow, Ready, L1, L3}
    private Request request = Request.Ready;
    private final double lowerClosedLoopErrorTolerance = 0.006944;
    private final double upperClosedLoopErrorTolerance = Millimeters.of(5).magnitude() / (24.0/45.0 *
        Inches.of(0.375).in(Millimeters));

    /** CANrange distance (m) for elevator zero/bottom. Within 2% = at position. */
    private static final double elevatorCANrangeZero = 0.04;
    /** CANrange distance (m) for elevator top. Within 2% = at position. */
    private static final double elevatorCANrangeTop = 0.245;
    // Max range before we assume the canrange sees no reflector so we shouldn't trust it
    private static final double elevatorCANrangeMax = 0.35;
    // Seeing nothing at all looks like a small distance and large StdDev
    private static final double elevatorStdDevDistThreshold = 0.12;
    private static final double elevatorMaxStdDev = 0.015;
    /** Tolerance: within this fraction of target = at target. */
    private static final double elevatorCANrangeTolerancePercent = 0.02;
    /** Max duty cycle for elevator when driving toward CANrange target. */
    private static final double elevatorDutyMax = 0.5;
    /** Min duty when approaching target—avoids slow crawl near goal. */
    private static final double elevatorDutyMin = 0.2;
    private final double hookForceTorque = 5;
    private final double hookForceVelocityLimit = 0.5;
    private final double hookLimitRotations = 15; // FIXME
    private final double hookTargetTolerance = 2;
    /** Elevator positions from CANrange distance (m). Stow/Bottom = zero, Top = top. */
    private final Map<Position, Double> elevatorPositions = Map.of(
        Position.Stow, elevatorCANrangeZero,
        Position.Bottom, elevatorCANrangeZero,
        Position.L1, (elevatorCANrangeZero + elevatorCANrangeTop) / 2,  // intermediate
        Position.Top, elevatorCANrangeTop
    );

    /** Left stick Y must exceed this to enter target mode (push up or pull down); else motion stops. */
    private static final double leftStickTargetModeThreshold = 0.15;

    /** When pulling down, if lower hook CANCoder changes by this much (rotations), move to Hook position. */
    private static final double lowerHookChangeThreshold = 0.01;

    private final Map<Position, Angle> lowerHookPositions = Map.of( // rotations are CANCoder absolute positions (0–1)
        Position.Stow, Rotations.of(0.370),
        Position.Prepare, Rotations.of(0.547),
        Position.Clinch, Rotations.of(0.680),
        Position.Grab, Rotations.of(0.715),
        Position.Hook, Rotations.of(0.715),
        Position.ClearBumper, Rotations.of(0.62)  
    );
    private final Map<Position, Angle> upperHookPositions = Map.of( // rotations are motor rotations
        Position.Stow, Rotations.of(0),
        Position.Bottom, Rotations.of(-260),
        Position.L1, Rotations.of(-100),      //FIXME
        Position.Top, Rotations.of(-64)
    );

    private final MotionMagicVoltage motion = new MotionMagicVoltage(0);
    private final TalonFX lowerHookMotor = new TalonFX(Ports.LOWER_HOOK_MOTOR, kCANBusJustice);
    private final TalonFX upperHookMotor = new TalonFX(Ports.UPPER_HOOK_MOTOR, kCANBusJustice);
    private final TalonFX elevatorMotor = new TalonFX(Ports.ELEVATOR_MOTOR, kCANBusJustice);
    private final CANcoder lowerHookCANcoder = new CANcoder(Ports.LOWER_HOOK_CANCODER, kCANBusJustice);
    private final CANrange canRange = new CANrange(Ports.CANRANGE, kCANBusJustice);

    public static Climber getInstance() {
        if (instance == null) instance = new Climber();
        return instance;
    }

    private Climber() {
        canRange.getConfigurator().apply(new CANrangeConfiguration()
            .withFovParams(new FovParamsConfigs().withFOVRangeX(7).withFOVRangeY(7))
        );
        lowerHookCANcoder.getConfigurator().apply(new MagnetSensorConfigs()
            .withMagnetOffset(0)
            .withAbsoluteSensorDiscontinuityPoint(1));
        // Do NOT call setPosition() on the CANcoder here; we want to keep its factory absolute reference
        // stable across power cycles and use absolute position (0–1 rotations) for our targets.
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
                .withMotionMagicAcceleration(100)
                .withMotionMagicCruiseVelocity(100))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(30)
                .withPeakReverseTorqueCurrent(-30))
            .withFeedback(new FeedbackConfigs()
                .withFeedbackRemoteSensorID(Ports.LOWER_HOOK_CANCODER)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withSensorToMechanismRatio(1)
                .withRotorToSensorRatio(5 * 9 * 72 / 20.0))
        );
        upperHookMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(new Slot0Configs().withKP(0.025))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(500)
                .withMotionMagicCruiseVelocity(80))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(30)
                .withPeakReverseTorqueCurrent(-30))
        );
        elevatorMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(new Slot0Configs().withKP(0.025))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(500)
                .withMotionMagicCruiseVelocity(80))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(20)
                .withPeakReverseTorqueCurrent(-20))
        );

        // if (!request(Request.Stow).isOK()) throw new IllegalStateException("Couldn't stow climber; it began in state " + request);
    }

    /** used for testing only */
    private boolean manualLH = false, manualUH = false, manualElevator = false;

    /**
     * Left stick Y: push up = both to Top; pull down = both to Bottom. Both move at the same time.
     * Right stick Y: same as left, but pull down moves lower hooks to ClearBumper (and holds) instead of neutral.
     * On release we hold current position.
     */
    public void move(double joyValueUpHook, double joyValueLowHook, double joyValueElevate) {
        double scaledUH = scaleWithDeadband(joyValueUpHook, 0.25);
        double scaledRightY = scaleWithDeadband(joyValueElevate, 0.25);

        if (forcingElevatorDown || forcingHooksUp) return;

        boolean leftPullDown = scaledUH > leftStickTargetModeThreshold;
        boolean rightPullDown = scaledRightY > leftStickTargetModeThreshold;
        boolean leftPushUp = scaledUH < -leftStickTargetModeThreshold;
        boolean rightPushUp = scaledRightY < -leftStickTargetModeThreshold;
        boolean anyPullDown = leftPullDown || rightPullDown;
        boolean anyPushUp = leftPushUp || rightPushUp;

        if (anyPushUp) {
            // Push up: both move to their targets at the same time (upper Top, elevator Top)
            setUpperHooks(Position.Top);
            setElevator(Position.Top);
            lowerHookBaselineWhenPullingDown = Double.NaN;
            manualUH = true;
        } else if (anyPullDown) {
            // Pull down: upper Bottom, elevator Bottom. Lower hooks depend on which stick.
            setUpperHooks(Position.Bottom);
            setElevator(Position.Bottom);
            if (rightPullDown) {
                // Right stick pull down: lower hooks to ClearBumper and hold
                lowerHookBaselineWhenPullingDown = Double.NaN;
                setLowerHooks(Position.ClearBumper);
            } else {
                // Left stick pull down only: lower hooks neutral, or Hook if CANCoder starts to change
                double currentLowerHookPos = lowerHookCANcoder.getPosition().getValueAsDouble();
                if (Double.isNaN(lowerHookBaselineWhenPullingDown)) {
                    lowerHookBaselineWhenPullingDown = currentLowerHookPos;
                    setLowerHooksNeutral();
                } else if (Math.abs(currentLowerHookPos - lowerHookBaselineWhenPullingDown) > lowerHookChangeThreshold) {
                    setLowerHooks(Position.Hook);
                } else {
                    setLowerHooksNeutral();
                }
            }
            manualUH = true;
        } else if (manualUH) {
            setUpperHooksHold();
            setElevatorHold();
            lowerHookBaselineWhenPullingDown = Double.NaN;
            manualUH = false;
        }
    }

    public StatusCode request(final Request request) {
        if (request == Request.Stow && this.request == Request.Ready) {
            CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                    new InstantCommand(() -> setElevator(Position.Bottom), this)
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
            setElevator(Position.Top);
            return upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Top))).isOK() ?
                StatusCode.OK : StatusCode.GeneralError;
        } else if (request == Request.L1 && this.request == Request.Ready) {
            setElevator(Position.L1);
            return upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.L1))).isOK() ?
                StatusCode.OK : StatusCode.GeneralError;
        } else if (request == Request.Ready && this.request == Request.L1) {
            setElevator(Position.Top);
            return upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Top))).isOK() ?
                StatusCode.OK : StatusCode.GeneralError;
        } else if (request == Request.L3 && this.request == Request.Ready) {
            CommandScheduler.getInstance().schedule(
                new InstantCommand(() -> {
                    setElevator(Position.Bottom);
                    upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Bottom)));
                }, this).andThen(
                new WaitUntilCommand(
                    () -> upperHookMotor.getClosedLoopError().getValue() < upperClosedLoopErrorTolerance &&
                    elevatorAtTarget()
                ).andThen(() -> lowerHookMotor.setControl(motion.withPosition(lowerHookPositions.get(Position.Grab))), this),
                new WaitUntilCommand(() -> lowerHookMotor.getClosedLoopError().getValue() < lowerClosedLoopErrorTolerance)
                    .andThen(() -> {
                        setElevator(Position.Top);
                        upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Top)));
                    }, this),
                new WaitUntilCommand(
                    () -> upperHookMotor.getClosedLoopError().getValue() < upperClosedLoopErrorTolerance &&
                    elevatorAtTarget()
                ).andThen(() -> lowerHookMotor.setControl(motion.withPosition(lowerHookPositions.get(Position.Clinch))), this),
                new WaitUntilCommand(() -> lowerHookMotor.getClosedLoopError().getValue() < lowerClosedLoopErrorTolerance)
                    .andThen(new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            setElevator(Position.Bottom);
                            upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Bottom)));
                        }, this),
                        new WaitUntilCommand(
                            () -> elevatorAtTarget() && upperHookMotor.getClosedLoopError().getValue() < upperClosedLoopErrorTolerance
                        ).andThen(() -> lowerHookMotor.setControl(motion.withPosition(lowerHookPositions.get(Position.Stow))), this)
                    )),
                new WaitUntilCommand(
                    () -> upperHookMotor.getClosedLoopError().getValue() < upperClosedLoopErrorTolerance &&
                    lowerHookMotor.getClosedLoopError().getValue() < lowerClosedLoopErrorTolerance &&
                    elevatorAtTarget()
                ).andThen(() -> lowerHookMotor.setControl(motion.withPosition(lowerHookPositions.get(Position.Grab))), this),
                new WaitUntilCommand(() -> lowerHookMotor.getClosedLoopError().getValue() < lowerClosedLoopErrorTolerance)
                    .andThen(() -> {
                        setElevator(Position.Top);
                        upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Top)));
                    }, this),
                new WaitUntilCommand(
                    () -> upperHookMotor.getClosedLoopError().getValue() < upperClosedLoopErrorTolerance &&
                    elevatorAtTarget()
                ).andThen(() -> lowerHookMotor.setControl(motion.withPosition(lowerHookPositions.get(Position.Clinch))), this),
                new WaitUntilCommand(() -> lowerHookMotor.getClosedLoopError().getValue() < lowerClosedLoopErrorTolerance)
                    .andThen(new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            setElevator(Position.Bottom);
                            upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Bottom)));
                        }, this),
                        new WaitUntilCommand(
                            () -> elevatorAtTarget() && upperHookMotor.getClosedLoopError().getValue() < upperClosedLoopErrorTolerance
                        ).andThen(() -> lowerHookMotor.setControl(motion.withPosition(lowerHookPositions.get(Position.Stow))), this)
                    )))
            );
            return StatusCode.OK;
        } else return StatusCode.GeneralError;
    }

    private double elevatorTarget = Double.NaN, upperHookTarget = Double.NaN, lowerHookTarget = Double.NaN;

    /** Lower hook CANCoder value when we entered pull-down mode; used to detect hook engagement. */
    private double lowerHookBaselineWhenPullingDown = Double.NaN;

    public void setElevator(Position target) {
        Double pos = elevatorPositions.get(target);
        if (pos == null) pos = elevatorPositions.get(Position.Stow);
        System.out.println("[Climber] setElevator " + target + " - CANrange " + pos + " m");
        elevatorTarget = pos;
    }

    public void setUpperHooks(Position target) {
        Angle pos = upperHookPositions.get(target);
        if (pos == null) pos = upperHookPositions.get(Position.Stow);
        System.out.println("[Climber] setUpperHooks " + target + " - " + pos);
        upperHookMotor.setControl(new MotionMagicDutyCycle(pos));
        upperHookTarget = pos.magnitude();
    }

    public void setLowerHooks(Position target) {
        Angle pos = lowerHookPositions.get(target);
        if (pos == null) pos = lowerHookPositions.get(Position.Stow);
        System.out.println("[Climber] setLowerHooks " + target + " - " + pos);
        lowerHookMotor.setControl(new MotionMagicDutyCycle(pos.in(Rotations)));
        lowerHookTarget = pos.magnitude();
    }

    /** Lower hooks neutral: stop actively targeting a position (no MotionMagic holding). */
    private void setLowerHooksNeutral() {
        System.out.println("[Climber] setLowerHooksNeutral");
        lowerHookMotor.setControl(new DutyCycleOut(0.0));
        lowerHookTarget = Double.NaN;
    }

    private double upperHooksHoldPosition = -1000;
    private void setUpperHooksHold() {
        System.out.println("[Climber] setUpperHooksHold");
        double pos = upperHookMotor.getPosition().getValueAsDouble();
        if (!MathUtil.isNear(pos, upperHooksHoldPosition, hookTargetTolerance)) {
            upperHooksHoldPosition = pos;
            upperHookMotor.setControl(new MotionMagicDutyCycle(pos));
            upperHookTarget = pos;
        }
    }

    private void setElevatorHold() {
        System.out.println("[Climber] setElevatorHold");
        double pos = getElevatorCANrangeMeters();
        // Don't overwrite Top target when releasing—let elevator finish reaching top
        if (!Double.isNaN(elevatorTarget) && Math.abs(elevatorTarget - elevatorCANrangeTop) < 0.01
                && pos < elevatorCANrangeTop - 0.005) {
            return;  // Keep driving toward Top
        }
        elevatorTarget = pos;
    }

    public boolean upperHooksAtTarget() {
        return MathUtil.isNear(upperHookMotor.getPosition().getValueAsDouble(), upperHookTarget, hookTargetTolerance);
    }

    public boolean elevatorAtTarget() {
        if (Double.isNaN(elevatorTarget)) return true;
        return isElevatorWithinTolerance(elevatorTarget);
    }

    private double getElevatorCANrangeMeters() {
        // Phoenix Pro "Distance" signal = same value in meters (0.035=zero, 0.245=top)
        return canRange.getDistance().getValue().in(Meters);
    }

    private boolean isElevatorWithinTolerance(double targetM) {
        double current = getElevatorCANrangeMeters();
        double tol = Math.max(0.001, targetM * elevatorCANrangeTolerancePercent);
        return Math.abs(current - targetM) <= tol;
    }

    /**
     * Resets zero points from current encoder positions and clears saved target state.
     * Call whenever the robot is enabled (e.g. in teleop) so "starting position" is always where you are now.
     * Sets both motors' reported position to 0 so they read zero at teleop entry.
     */
    public void resetZerosAndTargetState() {
        upperHookMotor.setPosition(0);
        elevatorTarget = Double.NaN;
    }

    /**
     * Prepare to climb: Elevator and upper hooks to top, lower hooks to Prepare.
     * All move together at the same time.
     * WARNING: If elevator is already up and upper hooks are stowed, this will breach the rules.
     * Call this only with the elevator down, or with the upper hooks already out.
     */
    public void prepToClimb() {
        setUpperHooks(Position.Top);
        setElevator(Position.Top);
        setLowerHooks(Position.Prepare);
    }

    /**
     * Moves elevator and upper hooks to zero rotations (the position when teleop was enabled).
     * Call from left stick click.
     */
    public void stow() {
        setElevator(Position.Stow);
        setLowerHooks(Position.Stow);
        CommandScheduler.getInstance().schedule(
            new WaitUntilCommand(() -> elevatorAtTarget())
                .andThen(new InstantCommand(() -> setUpperHooks(Position.Stow)))
        );
    }

    private double elevatorMinTorque = 0, elevatorMaxTorque = 0, hookMinTorque = 0, hookMaxTorque = 0, hookSpeed = 0;

    private boolean forcingElevatorDown = false;
    private boolean forcingHooksUp = false;
    public void forceStow() {
        forcingElevatorDown = true;
        forcingHooksUp = true;
        elevatorMotor.setControl(new DutyCycleOut(-0.05));
        upperHookMotor.setControl(new DutyCycleOut(0.05));
        setLowerHooks(Position.Stow);
    }

    @Override
    public void simulationPeriodic() {
        // CANrange returns 0 in sim by default; set a value so elevator logic and dashboard work
        // TODO - what's this for?
        canRange.getSimState().setDistance(0.1);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            elevatorTarget = getElevatorCANrangeMeters();
            double elevCurrent = getElevatorCANrangeMeters();
            if (isElevatorWithinTolerance(elevatorTarget)) {
                elevatorMotor.setControl(new DutyCycleOut(0));
            } else {
                double error = elevatorTarget - elevCurrent;
                double duty = MathUtil.clamp(error * 5.0, -elevatorDutyMax, elevatorDutyMax);
                if (Math.abs(duty) > 0 && Math.abs(duty) < elevatorDutyMin) {
                    duty = Math.signum(duty) * elevatorDutyMin;
                }
                elevatorMotor.setControl(new DutyCycleOut(duty));
            }
            upperHookMotor.setControl(new MotionMagicDutyCycle(upperHookMotor.getPosition().getValueAsDouble()));
            return;
        }

        double elevatorTorque = elevatorMotor.getTorqueCurrent().getValueAsDouble();
        if (elevatorTorque < elevatorMinTorque) elevatorMinTorque = elevatorTorque;
        if (elevatorTorque > elevatorMaxTorque) elevatorMaxTorque = elevatorTorque;

        double upperHooksTorque = upperHookMotor.getTorqueCurrent().getValueAsDouble();
        if (upperHooksTorque < hookMinTorque) hookMinTorque = upperHooksTorque;
        if (upperHooksTorque > hookMaxTorque) hookMaxTorque = upperHooksTorque;

        double elevatorCurrent = getElevatorCANrangeMeters();
        double elevatorStdDev = canRange.getDistanceStdDev().getValueAsDouble();
        if (
            elevatorCurrent > elevatorCANrangeMax ||
            ((elevatorStdDev == 0 || elevatorStdDev > elevatorMaxStdDev) && elevatorCurrent < elevatorStdDevDistThreshold)
        ) {
            System.out.println("Elevator CANrange failure: dist " + elevatorCurrent + ", stdDev " + elevatorStdDev + " - disabling elevator");
            elevatorMotor.setControl(new DutyCycleOut(0));
        } else if (forcingElevatorDown) {
            if (isElevatorWithinTolerance(elevatorCANrangeZero)) {
                System.out.println("Elevator bottom limit hit (CANrange at zero)");
                forcingElevatorDown = false;
                elevatorTarget = elevatorCANrangeZero;
                elevatorMotor.setControl(new DutyCycleOut(0));
            } else {
                elevatorMotor.setControl(new DutyCycleOut(-0.05));
            }
        } else if (!Double.isNaN(elevatorTarget)) {
            if (isElevatorWithinTolerance(elevatorTarget)) {
                elevatorMotor.setControl(new DutyCycleOut(0));
            } else {
                double error = elevatorTarget - elevatorCurrent;
                double duty = MathUtil.clamp(error * 5.0, -elevatorDutyMax, elevatorDutyMax);
                if (Math.abs(duty) > 0 && Math.abs(duty) < elevatorDutyMin) {
                    duty = Math.signum(duty) * elevatorDutyMin;
                }
                elevatorMotor.setControl(new DutyCycleOut(duty));
            }
        } else {
            elevatorMotor.setControl(new DutyCycleOut(0));
        }
        hookSpeed = upperHookMotor.getVelocity().getValueAsDouble();
        if (forcingHooksUp && (upperHooksTorque > hookForceTorque) && (hookSpeed < hookForceVelocityLimit)) {
            System.out.println("Upper hooks stow limit hit, marking max position");
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

        builder.addDoubleProperty("Elevator CANrange (m)", () -> getElevatorCANrangeMeters(), null);
        builder.addDoubleProperty("Elevator CANrange stdev", () -> canRange.getDistanceStdDev().getValueAsDouble(), null);
        builder.addDoubleProperty("Upper hook rot", () -> upperHookMotor.getPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("Lower hook rot", () -> lowerHookMotor.getPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("Lower hook CANCoder", () -> lowerHookCANcoder.getPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("Lower hook baseline (pull down)", () -> lowerHookBaselineWhenPullingDown, null);
        builder.addDoubleProperty("Elevator target", () -> elevatorTarget, null);
        builder.addDoubleProperty("Upper hook target", () -> upperHookTarget, null);
        builder.addDoubleProperty("Lower hook target", () -> lowerHookTarget, null);

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
