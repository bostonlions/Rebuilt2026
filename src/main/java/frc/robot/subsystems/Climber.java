package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;

import frc.robot.Robot.Ports;

import static frc.robot.Robot.kCANBusGronk;
import static frc.robot.Robot.kCANBusJustice;
import static frc.robot.RobotContainer.ControlBoard.CustomXboxController.scaleWithDeadband;

import java.util.Map;

public final class Climber extends SubsystemBase {
    private static Climber instance = null;
    private enum Position {Clinch, Grab, Stow, Bottom, L1, Top}
    private enum Request {Stow, Ready, L1, L3}
    private Request request = Request.Ready;
    private final MotionMagicVoltage motion = new MotionMagicVoltage(0);
    private final TalonFX lowerHookMotor = new TalonFX(Ports.LOWER_HOOK_MOTOR, kCANBusGronk);
    private final TalonFX upperHookMotor = new TalonFX(Ports.UPPER_HOOK_MOTOR, kCANBusJustice);
    private final TalonFX elevatorMotor = new TalonFX(Ports.ELEVATOR_MOTOR, kCANBusJustice);
    private final double lowerClosedLoopErrorTolerance = 0.006944444444444444444444444444444444444444444444444444444444444;
    private final double UpperClosedLoopErrorTolerance = Millimeters.of(5).magnitude() / (24.0/45.0 *
        Inches.of(0.375).in(Millimeters));
    private final double ElevatorClosedLoopErrorTolerance = 1.714555555555555555555555555555555555555555555555555555555555;
    // private final CANrange canRange = new CANrange(Ports.CANRANGE, kCANBus);
    private final Map<Position, Angle> elevatorPositions = Map.of( // rotations are motor rotations;
        Position.Bottom, Rotations.of(-1),                         // 15 elevator motor rotations = 4 cm height change
        Position.L1, Rotations.of(-1),      //FIXME
        Position.Top, Rotations.of(-1)
    );
    private final Map<Position, Angle> lowerHookPositions = Map.of( // rotations are of the hook itself
        Position.Stow, Rotations.of(-1),
        Position.Clinch, Rotations.of(-1),  //FIXME
        Position.Grab, Rotations.of(-1)
    );
    private final Map<Position, Angle> upperHookPositions = Map.of( // rotations are motor rotations
        Position.Stow, Rotations.of(-1),
        Position.Bottom, Rotations.of(-1),
        Position.L1, Rotations.of(-1),      //FIXME
        Position.Top, Rotations.of(-1)
    );

    public static Climber getInstance() {
        if (instance == null) instance = new Climber();
        return instance;
    }

    private Climber() {
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().registerSubsystem(this);
        // canRange.getConfigurator().apply(new CANrangeConfiguration());
        lowerHookMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(0.025))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(30)
                .withMotionMagicJerk(1000)
                .withMotionMagicCruiseVelocity(30))
            .withFeedback(new FeedbackConfigs()
                .withFeedbackRemoteSensorID(14)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withSensorToMechanismRatio(1)
                .withRotorToSensorRatio(5 * 9 * 72 / 20.0)));
        upperHookMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(0.025))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(30)
                .withMotionMagicJerk(1000)
                .withMotionMagicCruiseVelocity(30)));
        elevatorMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
            .withSlot0(new Slot0Configs().withKP(0.025))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(300)
                .withMotionMagicJerk(1000)
                .withMotionMagicCruiseVelocity(30)));
        if (!request(Request.Stow).isOK()) throw new IllegalStateException("Couldn't stow climber; it began in state " + request);
    }

    /** used for testing only */
    public void move(double joyValueUpHook, double joyValueLowHook, double joyValueElevate) {
        lowerHookMotor.setControl(new MotionMagicVelocityDutyCycle(25 * scaleWithDeadband(joyValueLowHook, 0.25)));
        upperHookMotor.setControl(new MotionMagicVelocityDutyCycle(25 * scaleWithDeadband(joyValueUpHook, 0.25)));
        elevatorMotor.setControl(new MotionMagicVelocityDutyCycle(25 * scaleWithDeadband(joyValueElevate, 0.25)));
    }

    public StatusCode request(final Request request) {
        if (request == Request.Stow && this.request == Request.Ready) {
            new ParallelCommandGroup(
                new InstantCommand(() -> elevatorMotor.setControl(motion.withPosition(elevatorPositions.get(Position.Bottom))))
                    .andThen(
                        new WaitCommand(2.5),
                        new InstantCommand(() ->
                            upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Stow))), this)
                    ),
                new InstantCommand(() -> lowerHookMotor.setControl(motion.withPosition(lowerHookPositions.get(Position.Stow))))
            ).schedule(); return StatusCode.OK;
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
            new InstantCommand(() -> {
                upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Bottom)));
                elevatorMotor.setControl(motion.withPosition(elevatorPositions.get(Position.Bottom)));
            }, this).andThen(
            new WaitUntilCommand(
                () -> upperHookMotor.getClosedLoopError().getValue() < UpperClosedLoopErrorTolerance &&
                elevatorMotor.getClosedLoopError().getValue() < ElevatorClosedLoopErrorTolerance
            ).andThen(() -> lowerHookMotor.setControl(motion.withPosition(lowerHookPositions.get(Position.Grab))), this),
            new WaitUntilCommand(() -> lowerHookMotor.getClosedLoopError().getValue() < lowerClosedLoopErrorTolerance)
                .andThen(() -> {
                    elevatorMotor.setControl(motion.withPosition(elevatorPositions.get(Position.Top)));
                    upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Top)));
                }, this),
            new WaitUntilCommand(
                () -> upperHookMotor.getClosedLoopError().getValue() < UpperClosedLoopErrorTolerance &&
                    elevatorMotor.getClosedLoopError().getValue() < ElevatorClosedLoopErrorTolerance
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
                () -> upperHookMotor.getClosedLoopError().getValue() < UpperClosedLoopErrorTolerance &&
                lowerHookMotor.getClosedLoopError().getValue() < lowerClosedLoopErrorTolerance &&
                elevatorMotor.getClosedLoopError().getValue() < ElevatorClosedLoopErrorTolerance
            ).andThen(() -> lowerHookMotor.setControl(motion.withPosition(lowerHookPositions.get(Position.Grab))), this),
            new WaitUntilCommand(() -> lowerHookMotor.getClosedLoopError().getValue() < lowerClosedLoopErrorTolerance)
                .andThen(() -> {
                    elevatorMotor.setControl(motion.withPosition(elevatorPositions.get(Position.Top)));
                    upperHookMotor.setControl(motion.withPosition(upperHookPositions.get(Position.Top)));
                }, this),
            new WaitUntilCommand(
                () -> upperHookMotor.getClosedLoopError().getValue() < UpperClosedLoopErrorTolerance &&
                elevatorMotor.getClosedLoopError().getValue() < ElevatorClosedLoopErrorTolerance
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
                ))).schedule();
            return StatusCode.OK;
        } else return StatusCode.GeneralError;
    }

    @Override
    public void periodic() {}
}
