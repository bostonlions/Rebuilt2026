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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import static frc.robot.Robot.kCANBusJustice;

import frc.robot.Robot.Ports;

public final class Intake extends SubsystemBase {
    /**
     * {@code false} = Neo Vortex (SparkFlex, CAN {@link Ports#INTAKE_SPIN}).<br>
     * {@code true} = Kraken X60 (TalonFX, same ID on roboRIO CAN via {@link frc.robot.Robot#kCANBusRio}).<br>
     * Only the selected motor is constructed; recompile after changing.
     */
    private static final boolean INTAKE_SPINNER_USE_KRAKEN = true;
    private final double intakeSpeedNeo = 0.48;

    private static Intake instance = null;
    private final TalonFX extendMotor = new TalonFX(Ports.INTAKE_EXTEND, kCANBusJustice);
    private final CANcoder extendCANcoder = new CANcoder(15, kCANBusJustice);

    private final SparkFlex spinMotorNeo;
    private final TalonFX spinMotorKraken;
    private final StaticBrake krakenBrake;
    private final DutyCycleOut krakenSpinDuty;
    private final boolean spinnerIsKraken;

    private final double inPosition = 0.086; //flush with bumper
    private final double partialOut = 0.1; // for agitation (was .37)
    private final double partialIn = 0.02; // for agitation (was .8_)
    private final double outPosition = 0.685; // can't exceed one

    public static Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }

    private Intake() {
        spinnerIsKraken = INTAKE_SPINNER_USE_KRAKEN;

        extendCANcoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs()
            .withMagnetOffset(0.32)
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(1)));

        if (spinnerIsKraken) {
            spinMotorNeo = null;
            krakenBrake = new StaticBrake();
            krakenSpinDuty = new DutyCycleOut(0.75);
            spinMotorKraken = new TalonFX(Ports.INTAKE_SPIN, kCANBusJustice);
            spinMotorKraken.getConfigurator().apply(new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLowerLimit(20)
                    .withSupplyCurrentLowerTime(0.1))
                .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    // Swap Clockwise_ / CounterClockwise_ to reverse spin direction vs positive duty.
                    .withInverted(InvertedValue.Clockwise_Positive)));
            spinMotorKraken.setControl(krakenBrake);
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

    private boolean agitating = false; // start not agitating
    private double agitationRateMS = 100;

    /** Returns agitation state incase we need it */
    public boolean toggleAgitation() {
        agitating = !agitating;
        return agitating;
    }

    /** Returns agitation state incase we need it */
    public boolean setAgitation(boolean agitating) {
        this.agitating = agitating;
        return agitating;
    }

    @Override
    public void periodic() {
        if (agitating) {
            if (System.currentTimeMillis() % (2 * agitationRateMS) < agitationRateMS) {
                setExtension(extended);
                agitating = true;
            } else {
                extendMotor.setControl(new MotionMagicDutyCycle(extended ? partialIn : partialOut));
            }
        }
    }

    public boolean isRetracted() {
        return edu.wpi.first.math.MathUtil.isNear(inPosition,
            extendMotor.getPosition().getValueAsDouble(), 0.028);
    }

    private boolean extended = false; // extender starts in

    public void setExtension(final boolean extend) {
        agitating = false;
        extended = extend;
        extendMotor.setControl(new MotionMagicDutyCycle(extend ? outPosition : inPosition));
    }

    // public void toggleExtension() {
    //     setExtension(!extended);
    // }

    private boolean spinning = false; // not spinning initially

    public void setSpinner(final boolean spin) {
        spinning = spin;
        if (spinnerIsKraken) {
            spinMotorKraken.setControl(spin ? krakenSpinDuty : krakenBrake);
        } else {
            if (spin) spinMotorNeo.set(intakeSpeedNeo);
            else spinMotorNeo.stopMotor();
        }
    }

    public void toggleSpin() {
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
        builder.addBooleanProperty("Spinner is Kraken (compile-time)", () -> spinnerIsKraken, null);
        builder.addDoubleProperty("Extension", () -> extendMotor.getPosition().getValueAsDouble(), null);
    }
}
