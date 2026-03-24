package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import static frc.robot.Robot.kCANBusJustice;
import static frc.robot.Robot.kCANBusRio;

import frc.robot.Robot.Ports;

/**
 * Intake spinner is either Neo Vortex (Spark) or Kraken X60 (TalonFX) on CAN 44 — only the selected motor is constructed.
 * Set preference {@code IntakeSpinnerUseKraken} in Driver Station → Preferences (or change {@code initBoolean} default below), then reboot robot code.
 */
public final class Intake extends SubsystemBase {
    /** Set false to silence intake spinner diagnostics (console + SmartDashboard Intake/SpinDebug*). */
    private static final boolean DEBUG_INTAKE_SPINNER = true;

    private static final String PREF_SPINNER_KRAKEN = "IntakeSpinnerUseKraken";

    static {
        Preferences.initBoolean(PREF_SPINNER_KRAKEN, true);
    }

    private static Intake instance = null;
    private final TalonFX extendMotor = new TalonFX(Ports.INTAKE_EXTEND, kCANBusJustice);
    private final CANcoder extendCANcoder = new CANcoder(15, kCANBusJustice);

    /** Non-null only when using Neo Vortex (Spark) on CAN 44. */
    private final SparkFlex spinMotorNeo;
    /** Non-null only when using Kraken X60 (Phoenix) on CAN 44. */
    private final TalonFX spinMotorKraken;
    private final StaticBrake krakenBrake;
    private final DutyCycleOut krakenSpinDuty;
    private final boolean spinnerIsKraken;

    private final double inPosition = 0.1;
    private final double outPosition = 0.9355;
    private final double intakeSpeed = 0.48;

    private StatusCode lastKrakenConfigStatus = StatusCode.OK;
    private StatusCode lastKrakenSetControlStatus = StatusCode.OK;
    private int spinDebugTick = 0;

    public static Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }

    private Intake() {
        spinnerIsKraken = Preferences.getBoolean(PREF_SPINNER_KRAKEN, false);

        extendCANcoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs()
            .withMagnetOffset(0.118) // add 0.1 to measured value, and then add 0.1 to in and out positions; offset all values by 0.1
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(1)));

        if (spinnerIsKraken) {
            spinMotorNeo = null;
            krakenBrake = new StaticBrake();
            krakenSpinDuty = new DutyCycleOut(0.44);
            spinMotorKraken = new TalonFX(Ports.INTAKE_SPIN, kCANBusRio);
            lastKrakenConfigStatus = spinMotorKraken.getConfigurator().apply(new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLowerLimit(20)
                    .withSupplyCurrentLowerTime(0.1))
                .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive)));
            lastKrakenSetControlStatus = spinMotorKraken.setControl(krakenBrake);
            if (DEBUG_INTAKE_SPINNER) {
                System.out.println("[IntakeSpin] Kraken path | CAN id=" + Ports.INTAKE_SPIN + " bus=\"" + kCANBusRio.getName()
                    + "\" | configApply=" + lastKrakenConfigStatus + " | brake setControl=" + lastKrakenSetControlStatus);
            }
        } else {
            spinMotorKraken = null;
            krakenBrake = null;
            krakenSpinDuty = null;
            spinMotorNeo = new SparkFlex(Ports.INTAKE_SPIN, MotorType.kBrushless);
            final SparkFlexConfig config = new SparkFlexConfig();
            config.smartCurrentLimit(40);
            config.idleMode(IdleMode.kBrake);
            config.inverted(true);
            spinMotorNeo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            spinMotorNeo.clearFaults();
            if (DEBUG_INTAKE_SPINNER) {
                System.out.println("[IntakeSpin] Neo Spark path | CAN id=" + Ports.INTAKE_SPIN + " (roboRIO CAN)");
            }
        }

        extendMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLowerLimit(20)
                .withSupplyCurrentLowerTime(0.1))
            .withSlot0(new Slot0Configs()
                .withKP(1.65)
                .withKI(0.04)
                .withKD(0.012)
                .withKV(0.12)
                .withKA(0.01)
                .withKS(0.05))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(6)
                .withMotionMagicAcceleration(10))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive))
            .withFeedback(new FeedbackConfigs()
                .withFeedbackRemoteSensorID(extendCANcoder.getDeviceID())
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withRotorToSensorRatio(3 * 48 / 16.0)
                .withSensorToMechanismRatio(1)));

        // Force the position into the 0-1 range, in case the initial reading is <0
        // so gets pushed up to near 1
        extendCANcoder.setPosition(MathUtil.inputModulus(extendCANcoder.getPosition().getValueAsDouble(), 0, 1));

        setExtension(false);
        setSpinner(false);
    }

    @Override
    public void periodic() {
        if (!DEBUG_INTAKE_SPINNER || !spinnerIsKraken || spinMotorKraken == null) return;
        if ((++spinDebugTick % 25) != 0) return;
        BaseStatusSignal.refreshAll(
            spinMotorKraken.getMotorVoltage(),
            spinMotorKraken.getDutyCycle(),
            spinMotorKraken.getStatorCurrent(),
            spinMotorKraken.getDeviceTemp(),
            spinMotorKraken.getFaultField());
        double motorVolts = spinMotorKraken.getMotorVoltage().getValueAsDouble();
        double duty = spinMotorKraken.getDutyCycle().getValueAsDouble();
        double stator = spinMotorKraken.getStatorCurrent().getValueAsDouble();
        long faults = spinMotorKraken.getFaultField().getValue();
        boolean alive = spinMotorKraken.isAlive();
        String line = String.format(
            "spinning=%b alive=%b setCtrl=%s motorV=%.2f duty=%.2f statorA=%.2f faults=0x%X",
            spinning, alive, lastKrakenSetControlStatus, motorVolts, duty, stator, faults);
        SmartDashboard.putString("Intake/SpinDebugKraken", line);
        SmartDashboard.putBoolean("Intake/SpinDebugKrakenAlive", alive);
        SmartDashboard.putString("Intake/SpinDebugLastSetControl", lastKrakenSetControlStatus.toString());
        if (DriverStation.isEnabled()) {
            System.out.println("[IntakeSpin Kraken] " + line);
        }
    }

    public boolean isRetracted() {
        return edu.wpi.first.math.MathUtil.isNear(inPosition,
            extendMotor.getPosition().getValueAsDouble(), 0.028);
    }

    private boolean extended = false; // extender starts in

    public void setExtension(final boolean extend) {
        extended = extend;
        extendMotor.setControl(new MotionMagicDutyCycle(extend ? outPosition : inPosition));
    }

    public void toggleExtension() {
        setExtension(!extended);
    }

    private boolean spinning = false; // not spinning initially

    public void setSpinner(final boolean spin) {
        spinning = spin;
        if (spinnerIsKraken) {
            lastKrakenSetControlStatus = spinMotorKraken.setControl(spin ? krakenSpinDuty : krakenBrake);
            if (DEBUG_INTAKE_SPINNER) {
                System.out.println("[IntakeSpin] setSpinner requested=" + spin + " mode=Kraken setControl=" + lastKrakenSetControlStatus
                    + " dutyCmd=0.44");
            }
        } else {
            if (spin) spinMotorNeo.set(intakeSpeed);
            else spinMotorNeo.stopMotor();
            if (DEBUG_INTAKE_SPINNER) {
                System.out.println("[IntakeSpin] setSpinner requested=" + spin + " mode=Neo output=" + (spin ? intakeSpeed : 0.0));
            }
        }
        SmartDashboard.putBoolean("Intake/SpinDebugWantsSpin", spin);
        SmartDashboard.putBoolean("Intake/SpinDebugSessionKraken", spinnerIsKraken);
    }

    public void toggleSpin() {
        if (DEBUG_INTAKE_SPINNER) {
            System.out.println("[IntakeSpin] toggleSpin (operator **LB** in RobotContainer, not RB) | wasSpinning=" + spinning
                + " | sessionKraken=" + spinnerIsKraken);
        }
        setSpinner(!spinning);
    }

    /** Whether the intake spinner is currently running. */
    public boolean isSpinning() {
        return spinning;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Intake");
        builder.setActuator(true);

        builder.addBooleanProperty("Extended", () -> extended, null);
        builder.addBooleanProperty("Spinning", () -> spinning, null);
        builder.addBooleanProperty("Pref: Kraken spinner (reboot to apply)", () -> Preferences.getBoolean(PREF_SPINNER_KRAKEN, false), v -> {
            Preferences.setBoolean(PREF_SPINNER_KRAKEN, v);
            DriverStation.reportWarning("Reboot robot code after changing intake spinner preference.", false);
        });
        builder.addBooleanProperty("Active session: Kraken spinner", () -> spinnerIsKraken, null);
        builder.addDoubleProperty("Extension", () -> extendMotor.getPosition().getValueAsDouble(), null);
    }
}
