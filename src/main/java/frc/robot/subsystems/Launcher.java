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
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private final Translation2d turretPosRobotRel = new Translation2d(-0.033147, 0.1394968); // coordinates are in meters following wpiblue coordinate convention
    private final double kAirRes = 0.01; // unit is inverse meters; this is quadratic coefficient of air resistance scaling equation
    private final double projectionTime = 0.1; // unit is seconds; this is the time into the future to project robot motion for shooting
    private final double vScale = 1.2; // (used elsewhere besides just the below line)
    private final double vMin = vScale * 7.7;
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

    private Pair<Translation2d, Translation2d> targetVectorAndSpeeds() {
        SwerveDriveState state = Drive.Drivetrain.getInstance().getState();
        Pose2d currentPose = state.Pose;
        double rvx = state.Speeds.vxMetersPerSecond;
        double rvy = state.Speeds.vyMetersPerSecond;
        double yaw = currentPose.getRotation().getRadians();
        ChassisSpeeds speedsFieldCentric = new ChassisSpeeds(rvx * Math.cos(yaw) - rvy * Math.sin(yaw),
            rvx * Math.sin(yaw) + rvy * Math.cos(yaw), state.Speeds.omegaRadiansPerSecond);
        Pose2d finalRobotPose = new Pose2d(currentPose.getX() + speedsFieldCentric.vxMetersPerSecond * projectionTime,
            currentPose.getY() + speedsFieldCentric.vyMetersPerSecond * projectionTime,
            new Rotation2d(currentPose.getRotation().getRadians() + speedsFieldCentric.omegaRadiansPerSecond * projectionTime));
        yaw = finalRobotPose.getRotation().getRadians();
        Pose2d turretPose = new Pose2d(finalRobotPose.getX() + turretPosRobotRel.getX() * Math.cos(yaw) -
            turretPosRobotRel.getY() * Math.sin(yaw), finalRobotPose.getY() + turretPosRobotRel.getX() * Math.sin(yaw) +
            turretPosRobotRel.getY() * Math.cos(yaw), finalRobotPose.getRotation());
        Translation2d shootTarget2d = new Translation2d(shootTarget.getMeasureX(), shootTarget.getMeasureY());
        double turretVx = speedsFieldCentric.vxMetersPerSecond - speedsFieldCentric.omegaRadiansPerSecond *
            (turretPosRobotRel.getX() * Math.sin(yaw) + turretPosRobotRel.getY() * Math.cos(yaw));
        double turretVy = speedsFieldCentric.vyMetersPerSecond + speedsFieldCentric.omegaRadiansPerSecond *
            (turretPosRobotRel.getX() * Math.cos(yaw) - turretPosRobotRel.getY() * Math.sin(yaw));

        final double tRadians = Math.PI / 2 - shootTarget2d.getAngle().getRadians() - yaw;
        final double vR = turretVx * Math.cos(tRadians + yaw) + turretVy * Math.sin(tRadians + yaw);
        final double vT = turretVy * Math.cos(tRadians + yaw) - turretVx * Math.sin(tRadians + yaw);
        return new Pair<>(new Translation2d(turretPose.getTranslation().minus(shootTarget2d).getNorm(),
            new Rotation2d(tRadians)), new Translation2d(vR, vT));
    }

    public void shoot() {
    }
}
