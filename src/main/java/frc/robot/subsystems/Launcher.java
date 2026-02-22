package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.MathUtil;

import frc.robot.Robot.Ports;

import static frc.robot.Robot.kCANBus;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Amps;

public final class Launcher implements Subsystem {
    public static enum Mode {OFF, STANDBY, FIRE}
    private Mode mode = Mode.OFF;
    private boolean blueAlliance, hurling, shooterSpeedReady;
    private Translation3d shootTarget = null; // updated in launcher constructor based on alliance color; units are meters
    private Translation2d hurlTargetXZ = null; // updated in launcher constructor based on alliance color; units are meters
    private static Launcher instance = null;
    private final Pair<Double, Double> pitchBounds = new Pair<Double, Double>(0.2618, 0.872); // min, max -- in radians; FIXME: get these right
    private final Pair<Double, Double> yawBounds = new Pair<Double, Double>(-5.0/18, 13.0/18); // in rotations
    private final Translation2d turretPosRobotRel = new Translation2d(-0.033147, 0.1394968); // coordinates are in meters following wpiblue coordinate convention
    private final double turretHeight = 0.6096; // in meters; TODO: get this right
    private final double kAirRes = 0.01; // unit is inverse meters; this is quadratic coefficient of air resistance scaling equation
    private final double projectionTime = 0.1; // unit is seconds; this is the time into the future to project robot motion for shooting
    private final double vScale = 1.2; // (used elsewhere besides just the below line)
    private final double vMin = vScale * Math.sqrt(2 * 9.80665 * 3); // speed to make ball launch 3m high fired straight up
    private final double launchToTurretSpeedScale = Double.NaN; // FIXME
    private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0);
    private final DutyCycleOut feederMotionRequest = new DutyCycleOut(1);
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
    private final TalonFX feeder_spinner = new TalonFX(-1, kCANBus); // FIXME
    private final TalonFX feeder_roller = new TalonFX(-1, kCANBus); // FIXME
    private final TalonFXConfiguration feederMotorConfig = new TalonFXConfiguration()
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
            .withInverted(InvertedValue.CounterClockwise_Positive));
    public final boolean shooterIsGoingFastEnough() {return shooterSpeedReady;}

    public static Launcher getInstance() {
        if (instance == null) instance = new Launcher();
        return instance;
    }

    private Launcher() {
        CommandScheduler.getInstance().registerSubsystem(this);

        feeder_spinner.getConfigurator().apply(feederMotorConfig);
        feeder_roller.getConfigurator().apply(feederMotorConfig);

        yaw10cancoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(-1/*FIXME*/).withAbsoluteSensorDiscontinuityPoint(1));
        yaw11cancoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(-1/*FIXME*/).withAbsoluteSensorDiscontinuityPoint(1));

        DriverStation.getAlliance().ifPresentOrElse((color) -> {
            shootTarget = new Translation3d(color == Alliance.Blue ? 4.625594 : 11.915394, 4.034536, 1.8288);
            hurlTargetXZ = new Translation2d(color == Alliance.Blue ? 3.048 : 13.492988, 1.016);
            blueAlliance = color == Alliance.Blue;
        }, () -> {
            throw new IllegalArgumentException("Is this code happening too early and the alliance color isn't available yet?");
        });

        launchMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withSlot0(gains).withMotionMagic(motionMagicConfigs)
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
        .isOK()) yawMotor.setPosition((t10 + 360 * (int)((630.253574644 * MathUtil.angleModulus(t10 * 0.01745329251 - yaw11cancoder.getAbsolutePosition().getValue().in(Units.Radians)) - t10) / 360)) / 2520);
        else DriverStation.reportError("Yaw motor config apply failed so the motor couldn't be zeroed", true);
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
        hurling = blueAlliance ? (finalRobotPose.getX() > 4.625594 &&! MathUtil.isNear(4.034536,
            finalRobotPose.getY(), 1.2192)) : (finalRobotPose.getX() < 11.915394 &&!
            MathUtil.isNear(4.034536, finalRobotPose.getY(), 1.2192));
        Translation2d shootTarget2d = hurling ? new Translation2d(hurlTargetXZ.getX(), finalRobotPose.getY() > 4.034536 ?
            6.0519945 : 2.017268) : new Translation2d(shootTarget.getMeasureX(), shootTarget.getMeasureY());
        double turretVx = speedsFieldCentric.vxMetersPerSecond - speedsFieldCentric.omegaRadiansPerSecond *
            (turretPosRobotRel.getX() * Math.sin(yaw) + turretPosRobotRel.getY() * Math.cos(yaw));
        double turretVy = speedsFieldCentric.vyMetersPerSecond + speedsFieldCentric.omegaRadiansPerSecond *
            (turretPosRobotRel.getX() * Math.cos(yaw) - turretPosRobotRel.getY() * Math.sin(yaw));

        final double r0 = turretPose.getTranslation().minus(shootTarget2d).getNorm();
        final double tRadians = Math.PI / 2 - shootTarget2d.getAngle().getRadians() - yaw;
        final double vR = turretVx * Math.cos(tRadians + yaw) + turretVy * Math.sin(tRadians + yaw);
        final double vT = turretVy * Math.cos(tRadians + yaw) - turretVx * Math.sin(tRadians + yaw);
        return new Pair<>(new Translation2d(r0 + kAirRes * r0 * r0, new Rotation2d(tRadians)), new Translation2d(vR, vT));
    }

    private double simError(final double h, final double r, final double t, final double b, final double sinPhi) {
        final double a = r + Math.sqrt(sinPhi * sinPhi - t * t);
        return h * a * a + b / 2 - a * Math.sqrt(1 - sinPhi * sinPhi);
    }

    /**(Yaw, Pitch) in rotations*/
    private Pair<Double, Double> getAngles(final double h, final double r, final double t, final double b) {
        final double minSine = Math.sin(pitchBounds.getFirst());
        final double maxSine = Math.sin(pitchBounds.getSecond());
        double end = Math.max(1.2 * Math.abs(t), minSine);
        double start = 0;
        for (int i = 0; MathUtil.isNear(start, end, 1e-6) && i < 10; i++) {
            start = end;
            end = start - 1e-6 / (simError(h, r, t, b, start + 1e-6) / simError(h, r, t, b, start) - 1);
            if (end > maxSine) {
                System.out.println("calculated pitch exceeded max pitch!");
                end = maxSine;
            }
            else if (end < minSine) {
                System.out.println("calculated pitch is below min pitch!");
                end = minSine;
            }
        }
        if (Math.abs(simError(h, r, t, b, end)) > 1e-4) {
            shooterSpeedReady = false;
            return new Pair<Double, Double>(0.0, pitchBounds.getFirst() / 2 / Math.PI);
        }
        shooterSpeedReady = true;
        return new Pair<Double, Double>(Math.asin(-t / end) / (2 * Math.PI), Math.asin(end) / (2 * Math.PI));
    }

    private void prepToShoot() {
        final Pair<Translation2d, Translation2d> targetVectorAndSpeeds = targetVectorAndSpeeds();
        final double h = hurling ? 1.016 : shootTarget.getZ() - turretHeight;
        final double r = targetVectorAndSpeeds.getFirst().getNorm();
        final double vDesired = Math.max(vMin, vScale * Math.sqrt(9.80665 * (h + Math.hypot(h, r))));
        launchMotor.setControl(new MotionMagicVelocityDutyCycle(launchToTurretSpeedScale * vDesired));
        final double vNow = launchMotor.getVelocity().getValueAsDouble() / launchToTurretSpeedScale;
        final double dv = launchMotor.getAcceleration().getValueAsDouble() * projectionTime / launchToTurretSpeedScale;
        final double vProjected = dv / (vDesired - vNow) > 1 ? vDesired : (vNow + dv);
        final double heightCoefficient = h / r;
        final double radialCoefficient = targetVectorAndSpeeds.getSecond().getX() / vProjected;
        final double tangentialCoefficient = targetVectorAndSpeeds.getSecond().getY() / vProjected;
        final double distanceCoefficient = 9.80665 * r / vProjected / vProjected;
        final Pair<Double, Double> angles = getAngles(heightCoefficient, radialCoefficient, tangentialCoefficient, distanceCoefficient);
        shooterSpeedReady = vProjected / vDesired > 0.89;
        yawMotor.setControl(motionRequest.withPosition(MathUtil.inputModulus(angles.getFirst() +
            targetVectorAndSpeeds.getFirst().getAngle().getRotations(), yawBounds.getFirst(), yawBounds.getSecond())));
        final double x = Drive.Drivetrain.getInstance().getState().Pose.getX();
        if (
            MathUtil.isNear(4.625594, x, 0.61) || MathUtil.isNear(11.915394, x, 0.61)
            ||!shooterSpeedReady
        ) {
            System.out.println("In/near a trench or low speed; can't shoot");
            pitchMotor.setControl(motionRequest.withPosition(pitchBounds.getFirst() / 2 / Math.PI));
            setMode(Mode.STANDBY);
        } else pitchMotor.setControl(motionRequest.withPosition(angles.getSecond()));
    }

    private boolean setFeeder(boolean on) {
        return feeder_spinner.setControl(on ? feederMotionRequest : new StaticBrake()).isOK() &&
            feeder_roller.setControl(on ? feederMotionRequest : new StaticBrake()).isOK();
    }

    public void setMode (Mode newMode) {
        if (mode == newMode) return; else
        if (newMode == Mode.OFF) if (launchMotor.setControl(new StaticBrake()).isOK())
        if (yawMotor.setControl(motionRequest).isOK())
        if (pitchMotor.setControl(motionRequest.withPosition(pitchBounds.getFirst() / 2 / Math.PI)).isOK()) {
            mode = newMode; return;
        }
        if (mode == Mode.OFF) prepToShoot();
        setFeeder(newMode == Mode.FIRE);
        mode = newMode;
    }

    @Override
    public void periodic() {
        if (mode != Mode.OFF) prepToShoot();
    }
}
