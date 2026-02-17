package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Robot.Ports;

import static frc.robot.Robot.kCANBus;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Amps;

public final class Launcher implements Subsystem {
    private static Launcher instance = null;
    private Translation3d shootTarget = null; // updated in launcher constructor based on prescence of alliance color
    private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(null);
    private final Slot0Configs gains = new Slot0Configs()
        .withKP(4.8).withKD(0.1).withKV(0.12).withKS(0.2).withKA(0.01); // TODO: tune this, maybe different for each motor
    private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(60)
        .withMotionMagicAcceleration(100)
        .withMotionMagicJerk(1000);
    private final TalonFX launchMotor = new TalonFX(Ports.LAUNCHER, kCANBus);
    private final TalonFX pitchMotor = new TalonFX(Ports.PITCH_MOTOR, kCANBus);
    private final TalonFX yawMotor = new TalonFX(Ports.YAW_MOTOR, kCANBus);
    private final CANcoder yaw10cancoder = new CANcoder(Ports.YAW_CANCODER_10, kCANBus);
    private final CANcoder yaw11cancoder = new CANcoder(Ports.YAW_CANCODER_11, kCANBus);

    public static Launcher getInstance() {
        if (instance == null) instance = new Launcher();
        return instance;
    }

    private Launcher() {
        yaw10cancoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(-1/*FIXME*/).withAbsoluteSensorDiscontinuityPoint(1));
        yaw11cancoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(-1/*FIXME*/).withAbsoluteSensorDiscontinuityPoint(1));

        DriverStation.getAlliance().ifPresentOrElse((color) -> shootTarget = new Translation3d(
            color == Alliance.Blue ? 4.625594 : 11.915394, 4.034536, 1.8288), () -> {
                throw new IllegalArgumentException("Is this code happening too early and the alliance color isn't available yet?");
            });

        launchMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withSlot0(gains)
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8)))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(Amps.of(40)).withPeakReverseTorqueCurrent(Amps.of(-40))));
        pitchMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withSlot0(gains).withMotionMagic(motionMagicConfigs)
            .withFeedback(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withFeedbackRemoteSensorID(Ports.PITCH_CANCODER)
                .withSensorToMechanismRatio(-1) // FIXME
                .withRotorToSensorRatio(-1))); // FIXME

        double t10 = yaw10cancoder.getAbsolutePosition().getValue().in(Units.Degrees);
        if (yawMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withSlot0(gains).withMotionMagic(motionMagicConfigs)
            .withFeedback(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(-1) // FIXME
                .withRotorToSensorRatio(-1))) // FIXME
        .isOK()) yawMotor.setPosition((t10 + 360 * (int)((630.253574644 * edu.wpi.first.math.MathUtil.angleModulus(t10 * 0.01745329251 - yaw11cancoder.getAbsolutePosition().getValue().in(Units.Radians)) - t10) / 360)) / 2520);
        else DriverStation.reportError("Yaw motor config apply failed so the motor couldn't be zeroed", true);
    }

    private Launcher setSpeed(double rps) {
        launchMotor.setControl(new VelocityVoltage(rps));
        return this;
    }

    private Launcher setPitch(double degrees) {
        pitchMotor.setControl(motionRequest.withPosition(degrees / 360));
        return this;
    }

    private Launcher setYaw(double degrees) {
        yawMotor.setControl(motionRequest.withPosition(degrees / 360));
        return this;
    }

    public void shoot(Pose2d robotPosition/*(x,y,yaw)*/) {
        this.setSpeed(shootTarget.getX()).setPitch(shootTarget.getY()).setYaw(shootTarget.getZ()); // FIXME
    }
}
