package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Rotations;

import static frc.robot.Robot.kCANBus;

public final class Intake implements edu.wpi.first.wpilibj2.command.Subsystem {
    private final TalonFX extendMotor = new TalonFX(-1, kCANBus); // FIXME
    private final TalonFX spinMotor = new TalonFX(-1, kCANBus); // FIXME
    private final StaticBrake brake = new StaticBrake();
    private final DutyCycleOut spinRequest = new DutyCycleOut(1);
    private final MotionMagicVoltage in = new MotionMagicVoltage(-1); // FIXME
    private final MotionMagicVoltage out = new MotionMagicVoltage(-1); // FIXME

    public Intake() {
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().registerSubsystem(this);
        spinMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentLowerLimit(10)
                .withSupplyCurrentLowerTime(0.1))
            .withSlot0(new Slot0Configs()
                .withKP(0.6)
                .withKI(0.0)
                .withKD(0.0)
                .withKV(0.0))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive)));
        extendMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20.)
                .withSupplyCurrentLowerLimit(10.)
                .withSupplyCurrentLowerTime(0.1))
            .withSlot0(new Slot0Configs()
                .withKP(1.65)
                .withKI(0.04)
                .withKD(0.012))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(8.3)
                .withMotionMagicExpo_kA(0.3)
             // .withMotionMagicJerk(1600)
                .withMotionMagicAcceleration(1.5))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive))
            .withFeedback(new FeedbackConfigs()
                .withFeedbackRemoteSensorID(-1) // FIXME
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)));
        setExtension(false);
    }

    @Override
    public void periodic() {}

    public boolean isRetracted() {
        return edu.wpi.first.math.MathUtil.isNear(in.getPositionMeasure().in(Rotations),
            extendMotor.getPosition().getValueAsDouble(), 0.028); // FIXME: get the tolerance right
    }

    private boolean extended;

    public StatusCode setExtension(final boolean extend) {
        extended = extend;
        return extendMotor.setControl(extend ? out : in);
    }

    public StatusCode toggleExtension() {
        return setExtension(!extended);
    }

    private boolean spinning;

    public StatusCode setSpinner(final boolean spin) {
        spinning = spin;
        return spinMotor.setControl(spin ? spinRequest : brake);
    }

    public StatusCode toggleSpin() {
        return setSpinner(!spinning);
    }
}
