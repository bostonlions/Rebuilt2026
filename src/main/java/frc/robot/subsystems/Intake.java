package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
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
    private static Intake instance = null;
    private final TalonFX extendMotor = new TalonFX(Ports.INTAKE_EXTEND, kCANBusJustice);
    private final CANcoder extendCANcoder = new CANcoder(15, kCANBusJustice);
    private final SparkFlex spinMotor = new SparkFlex(Ports.INTAKE_SPIN, MotorType.kBrushless);
    private final double inPosition = 0.1;
    private final double outPosition = 0.9355;
    private final double intakeSpeed = 0.4;

    public static Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }

    private Intake() {
        extendCANcoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs()
            .withMagnetOffset(0.12)
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(1)));
        final SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(40);
        config.idleMode(IdleMode.kBrake);
        config.inverted(true);
        spinMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        spinMotor.clearFaults();
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
                .withMotionMagicCruiseVelocity(4)
                .withMotionMagicAcceleration(4))
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
    public void periodic() {}

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
        if (spin) spinMotor.set(intakeSpeed); else spinMotor.stopMotor();
    }

    public void toggleSpin() {
        setSpinner(!spinning);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Intake");
        builder.setActuator(true);

        builder.addBooleanProperty("Extended", () -> extended, null);
        builder.addBooleanProperty("Spinning", () -> spinning, null);
        builder.addDoubleProperty("Extension", () -> extendMotor.getPosition().getValueAsDouble(), null);
    }
}
