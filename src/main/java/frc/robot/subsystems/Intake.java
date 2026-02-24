package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
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

import static edu.wpi.first.units.Units.Rotations;

import static frc.robot.Robot.kCANBusJustice;

import frc.robot.Robot.Ports;

public final class Intake implements edu.wpi.first.wpilibj2.command.Subsystem {
    private final TalonFX extendMotor = new TalonFX(Ports.INTAKE_EXTEND, kCANBusJustice);
    private final CANcoder extendCANcoder = new CANcoder(15, kCANBusJustice);
    private final TalonFX spinMotor = new TalonFX(Ports.INTAKE_SPIN, kCANBusJustice);
    private final StaticBrake brake = new StaticBrake();
    private final DutyCycleOut spinRequest = new DutyCycleOut(0.4);
    private final MotionMagicDutyCycle in = new MotionMagicDutyCycle(0.055);
    private final MotionMagicDutyCycle out = new MotionMagicDutyCycle(0.86);

    public Intake() {
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().registerSubsystem(this);
        extendCANcoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs()
            .withMagnetOffset(0.018555)
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(1)));
        spinMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentLowerLimit(10)
                .withSupplyCurrentLowerTime(0.1))
            .withSlot0(new Slot0Configs().withKP(0.6))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive)));
        extendMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentLowerLimit(10)
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
        // System.out.println("CAN" + extendCANcoder.getAbsolutePosition().getValueAsDouble() + "; motor: " + extendMotor.getPosition().getValueAsDouble());
        // return extendMotor.setControl(new DutyCycleOut(0.05));
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
