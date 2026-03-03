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
    public enum Position {Clinch, Grab, Stow, Bottom, L1, Top}
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
        Position.Top, Rotations.of(-250)
    );
    /** Lower hook CANcoder targets (rotations 0–1). From Phoenix Tuner. */
    public static final double kLowerHookStowCancoderPosition   = 0.399;
    public static final double kLowerHookClinchCancoderPosition = 0.555;
    public static final double kLowerHookGrabCancoderPosition   = 0.663;

    /** Tolerance in motor rotations to consider upper hook "at" the target (avoids re-running on double-click). */
    private static final double kUpperHookAtTargetToleranceRotations = 2;

    /** Upper hook motor position when teleop was enabled; used as zero for target calculation. */
    private double upperHookZeroPosition = 0.0;

    /** Tolerance in motor rotations to consider elevator "at" the target. */
    private static final double kElevatorAtTargetToleranceRotations = 2;

    /** Elevator motor position when teleop was enabled; no longer used for absolute targets but retained for compatibility. */
    private double elevatorZeroPosition = 0.0;

    /**
     * Left stick targets: push-up uses Top (same as right-stick click); pull-down uses Bottom.
     * Both upper hook and elevator move at the same time.
     */
    public static final double kElevatorPullDownTargetRotationsFromZero = 0.0;

    /** Left stick Y must exceed this to enter target mode (push up or pull down); else motion stops. */
    private static final double kLeftStickTargetModeThreshold = 0.15;

    private final Map<Position, Angle> lowerHookPositions = Map.of( // rotations are CANCoder absolute positions (0–1)
        Position.Stow, Rotations.of(kLowerHookStowCancoderPosition),
        Position.Clinch, Rotations.of(kLowerHookClinchCancoderPosition),
        Position.Grab, Rotations.of(kLowerHookGrabCancoderPosition)
    );
    private final Map<Position, Angle> upperHookPositions = Map.of( // rotations are motor rotations
        Position.Stow, Rotations.of(0),
        Position.Bottom, Rotations.of(260),
        Position.L1, Rotations.of(42.2),      //FIXME
        Position.Top, Rotations.of(64)
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
                .withMotionMagicAcceleration(160)
                .withMotionMagicCruiseVelocity(80))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(30)
                .withPeakReverseTorqueCurrent(-30))
        );
        elevatorMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
            .withSlot0(new Slot0Configs().withKP(0.025))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(160)
                .withMotionMagicCruiseVelocity(80))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(20)
                .withPeakReverseTorqueCurrent(-20))
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
    /** Hold positions when stick released mid-move; re-sent every cycle so motors don't revert to old target. */
    private double lastHoldUpperRotations = Double.NaN;
    private double lastHoldElevatorRotations = Double.NaN;
    /** Treat targets within this many rotations as "same" so we don't constantly re-command. */
    private static final double kTargetChangeEpsilonRotations = 0.02;
    /** Debug: print every this many move() calls when in target mode (~1 sec). Set 0 to disable. */
    private static final int kElevatorDebugPrintInterval = 50;
    private int moveCallCount = 0;
    /** Debug: stream upper hook and elevator positions every this many periodic() calls (~0.5 sec). Set 0 to disable. */
    private static final int kPositionStreamInterval = 25;
    private int periodicCount = 0;

    /**
     * Left stick Y: push up = both to Top; pull down = both to Bottom. Both move at the same time.
     * On release we hold current position.
     */
    public void move(double joyValueUpHook, double joyValueLowHook, double joyValueElevate) {
        double scaledUH = scaleWithDeadband(joyValueUpHook, 0.25);
        double scaledElevator = scaleWithDeadband(joyValueElevate, 0.25);

        if (Math.abs(scaledUH) > kLeftStickTargetModeThreshold) {
            if (scaledUH < 0) {
                // Push up: both move to their targets at the same time (upper Top, elevator Top)
                double targetUpper = upperHookPositions.get(Position.Top).in(Rotations);
                double targetElevator = elevatorPositions.get(Position.Top).in(Rotations);
                upperHookMotor.setControl(new MotionMagicDutyCycle(Rotations.of(targetUpper)));
                elevatorMotor.setControl(new MotionMagicDutyCycle(Rotations.of(targetElevator)));
                lastUpperHookTargetRotations = targetUpper;
                lastElevatorTargetRotations = targetElevator;
                if (kElevatorDebugPrintInterval > 0 && (++moveCallCount % kElevatorDebugPrintInterval == 0)) {
                    double curElev = elevatorMotor.getPosition().getValueAsDouble();
                    System.out.println("[Climber] PUSH UP: elevatorZero=" + elevatorZeroPosition + " targetElev=" + targetElevator + " currentElev=" + curElev + " (commanding elevator every cycle)");
                }
            } else {
                // Pull down: both move to their targets at the same time (upper Bottom, elevator Bottom)
                double targetUpper = upperHookPositions.get(Position.Bottom).in(Rotations);
                double targetElevator = elevatorPositions.get(Position.Bottom).in(Rotations);
                upperHookMotor.setControl(new MotionMagicDutyCycle(Rotations.of(targetUpper)));
                elevatorMotor.setControl(new MotionMagicDutyCycle(Rotations.of(targetElevator)));
                // Whenever we pull down on the left stick, lower hooks are neutral (not targeting a position).
                setLowerHooksNeutral();
                lastUpperHookTargetRotations = targetUpper;
                lastElevatorTargetRotations = targetElevator;
                if (kElevatorDebugPrintInterval > 0 && (++moveCallCount % kElevatorDebugPrintInterval == 0)) {
                    double curElev = elevatorMotor.getPosition().getValueAsDouble();
                    System.out.println("[Climber] PULL DOWN: elevatorZero=" + elevatorZeroPosition + " targetElev=" + targetElevator + " currentElev=" + curElev + " (commanding elevator every cycle)");
                }
            }
            manualUH = false;
            manualElevator = false;
            leftStickWasInTargetMode = true;
        } else {
            moveCallCount = 0;
            if (leftStickWasInTargetMode || manualUH || manualElevator) {
                // Transition from target or manual: capture current position to hold
                lastHoldUpperRotations = upperHookMotor.getPosition().getValueAsDouble();
                lastHoldElevatorRotations = elevatorMotor.getPosition().getValueAsDouble();
            }
            // Re-send hold every cycle when released so motors don't revert to a previous target
            if (!Double.isNaN(lastHoldUpperRotations) && !Double.isNaN(lastHoldElevatorRotations)) {
                upperHookMotor.setControl(new MotionMagicDutyCycle(Rotations.of(lastHoldUpperRotations)));
                elevatorMotor.setControl(new MotionMagicDutyCycle(Rotations.of(lastHoldElevatorRotations)));
            }
            lastUpperHookTargetRotations = Double.NaN;
            lastElevatorTargetRotations = Double.NaN;
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

    /** Lower hooks helpers: move to Stow/Clinch/Grab using absolute CANCoder positions (0–1 rotations). */
    public void moveLowerHooksToStow() {
        System.out.println("[Climber] moveLowerHooksToStow (CANCoder=" + kLowerHookStowCancoderPosition + ")");
        setLowerHooks(Position.Stow);
    }

    public void moveLowerHooksToClinch() {
        System.out.println("[Climber] moveLowerHooksToClinch (CANCoder=" + kLowerHookClinchCancoderPosition + ")");
        setLowerHooks(Position.Clinch);
    }

    public void moveLowerHooksToGrab() {
        System.out.println("[Climber] moveLowerHooksToGrab (CANCoder=" + kLowerHookGrabCancoderPosition + ")");
        setLowerHooks(Position.Grab);
    }

    /** Lower hooks neutral: stop actively targeting a position (no MotionMagic holding). */
    private void setLowerHooksNeutral() {
        System.out.println("[Climber] setLowerHooksNeutral");
        lowerHookMotor.setControl(new DutyCycleOut(0.0));
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
     * Sets both motors' reported position to 0 so they read zero at teleop entry.
     */
    public void resetZerosAndTargetState() {
        upperHookMotor.setPosition(0);
        elevatorMotor.setPosition(0);
        upperHookZeroPosition = 0;
        // Elevator now uses absolute positions from elevatorPositions (Top/Bottom), so zero is not used.
        lastUpperHookTargetRotations = Double.NaN;
        lastElevatorTargetRotations = Double.NaN;
        lastHoldUpperRotations = Double.NaN;
        lastHoldElevatorRotations = Double.NaN;
        leftStickWasInTargetMode = false;
    }

    /**
     * On right stick click: moves upper hook, lower hook, and elevator to their targets.
     * Lower hooks go to the "Clinch" CANCoder position (absolute 0–1 rotations),
     * elevator goes to Top using elevatorPositions.
     */
    public void moveUpperHookToTargetAndLowerHookToConstant() {
        double targetUpper = upperHookPositions.get(Position.Top).in(Rotations);
        double targetElevator = elevatorPositions.get(Position.Top).in(Rotations);
        upperHookMotor.setControl(new MotionMagicDutyCycle(Rotations.of(targetUpper)));
        // Lower hook to Clinch position
        lowerHookMotor.setControl(new MotionMagicDutyCycle(Rotations.of(kLowerHookClinchCancoderPosition)));
        elevatorMotor.setControl(new MotionMagicDutyCycle(Rotations.of(targetElevator)));
        lastUpperHookTargetRotations = targetUpper;
        lastElevatorTargetRotations = targetElevator;
    }

    /**
     * Moves elevator and upper hooks to zero rotations (the position when teleop was enabled).
     * Call from left stick click.
     */
    public void moveElevatorAndUpperHooksToZero() {
        double zero = 0;
        upperHookMotor.setControl(new MotionMagicDutyCycle(Rotations.of(zero)));
        elevatorMotor.setControl(new MotionMagicDutyCycle(Rotations.of(zero)));
        lastUpperHookTargetRotations = zero;
        lastElevatorTargetRotations = zero;
        lastHoldUpperRotations = zero;
        lastHoldElevatorRotations = zero;
    }

    /**
     * Moves elevator to Top (as defined in elevatorPositions). If already at target (within tolerance),
     * does nothing. Call from a button or command when you want to go to the preset elevator height.
     */
    public void moveElevatorToTarget() {
        double targetElevator = elevatorPositions.get(Position.Top).in(Rotations);
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

        // Stream upper hook and elevator positions (rotations) for debugging
        if (kPositionStreamInterval > 0 && (++periodicCount % kPositionStreamInterval == 0)) {
            double upperHookRot = upperHookMotor.getPosition().getValueAsDouble();
            double elevatorRot = elevatorMotor.getPosition().getValueAsDouble();
            System.out.printf("[Climber] upperHook=%.3f rot  elevator=%.3f rot%n", upperHookRot, elevatorRot);
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
