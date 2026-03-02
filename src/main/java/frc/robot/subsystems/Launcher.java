package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.math.MathUtil;

import frc.robot.Robot.Ports;

import static frc.robot.Robot.kCANBusJustice;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Amps;

public final class Launcher extends SubsystemBase {
    public static enum Mode {OFF, STANDBY, FIRE}
    private Mode mode = Mode.OFF;
    private boolean blueAlliance, hurling, shooterSpeedReady;
    private Translation3d shootTarget = null; // updated in launcher constructor based on alliance color; units are meters
    private Translation2d hurlTargetXZ = null; // updated in launcher constructor based on alliance color; units are meters
    private static Launcher instance = null;

    private final Pair<Double, Double> pitchBounds = new Pair<Double, Double>(15., 55.); // min, max -- in radians; FIXME: get these right
    private final double pitchGearRatio = 40. / 6.5; // degrees pitch per rotation TODO: get this right
    private final double pitchForceTorque = -10;
    private final double pitchForceVelocityLimit = -0.1;
    private final double pitchLimitRotations = -0.25;

    private final Pair<Double, Double> yawBounds = new Pair<Double, Double>(-100., 260.); // in degrees
    private final double c11Offset = .6225; // 11-tooth cancoder value at 0 degrees
    private final double c12Offset = .4840; // 12-tooth cancoder value at 0 degrees
    private final double yawGearRatio = 21; // rotations of yaw motor to give a full rotation of turret

    private final Translation2d turretPosRobotRel = new Translation2d(-0.033147, 0.1394968); // coordinates are in meters following wpiblue coordinate convention
    private final double turretHeight = 0.6096; // in meters; TODO: get this right
    private final double kAirRes = 0.01; // unit is inverse meters; this is quadratic coefficient of air resistance scaling equation
    private final double projectionTime = 0.1; // unit is seconds; this is the time into the future to project robot motion for shooting
    private final double vScale = 1.2; // (used elsewhere besides just the below line)
    private final double vMin = vScale * Math.sqrt(2 * 9.80665 * 3); // speed to make ball launch 3m high fired straight up
    private final double launchToTurretSpeedScale = Double.NaN; // FIXME -- launch motor revs * resulting fly wheel revs  * flywheel circumference / 2  ???

    private final StaticBrake brake = new StaticBrake();
    private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0);
    private final DutyCycleOut feederMotionRequest = new DutyCycleOut(1);
    // Separate test speeds: spinner 25%, roller 75%, launcher 10%
    private final DutyCycleOut feederSpinnerTestRequest = new DutyCycleOut(0.30);
    private final DutyCycleOut feederRollerTestRequest  = new DutyCycleOut(0.40);
    private final DutyCycleOut launcherTestRequest      = new DutyCycleOut(0.60);
    private final Slot0Configs gains = new Slot0Configs()
        .withKP(4.8).withKD(0.1).withKV(0.12).withKS(0.2).withKA(0.01); // TODO: tune this, maybe different for each motor
    private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(60)
        .withMotionMagicAcceleration(100)
        .withMotionMagicJerk(1000);
    private final TalonFX launchMotor = new TalonFX(Ports.LAUNCHER, kCANBusJustice);
    private final TalonFX pitchMotor = new TalonFX(Ports.PITCH_MOTOR, kCANBusJustice);
    private final TalonFX yawMotor = new TalonFX(Ports.YAW_MOTOR, kCANBusJustice);
    private final CANcoder yaw12cancoder = new CANcoder(Ports.YAW_CANCODER_12, kCANBusJustice);
    private final CANcoder yaw11cancoder = new CANcoder(Ports.YAW_CANCODER_11, kCANBusJustice);
    private final TalonFX feeder_spinner = new TalonFX(Ports.FEEDER_SPINNER, kCANBusJustice);
    private final TalonFX feeder_roller = new TalonFX(Ports.FEEDER_ROLLER, kCANBusJustice);
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

    // Simple-toggle launcher RPM target, adjustable via Trimmer at runtime
    private double simpleLaunchRpm = 2200.0;

    public static Launcher getInstance() {
        if (instance == null) instance = new Launcher();
        return instance;
    }

    private Launcher() {
        feeder_spinner.getConfigurator().apply(feederMotorConfig);
        feeder_roller.getConfigurator().apply(feederMotorConfig);

        yaw12cancoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(0).withAbsoluteSensorDiscontinuityPoint(1));
        yaw11cancoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(0).withAbsoluteSensorDiscontinuityPoint(1));

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
                .withPeakForwardTorqueCurrent(Amps.of(30)).withPeakReverseTorqueCurrent(Amps.of(-30))));
        pitchMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withSlot0(gains).withMotionMagic(motionMagicConfigs)
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(Amps.of(15)).withPeakReverseTorqueCurrent(Amps.of(-15)))
        );

        if (yawMotor.getConfigurator().apply(
            new TalonFXConfiguration()
            .withSlot0(gains).withMotionMagic(motionMagicConfigs)
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(Amps.of(40)).withPeakReverseTorqueCurrent(Amps.of(-40)))
        ).isOK()) {
            yawMotor.setPosition(-calcYawDegrees() * yawGearRatio / 360.);
        }
        else DriverStation.reportError("Yaw motor config apply failed so the motor couldn't be zeroed", true);

        forcePitchDown();

        setYaw(0);
        setPitch(20);

        initTrimmer();
    }

    private double getPitch() {
        return pitchMotor.getPosition().getValueAsDouble() * pitchGearRatio + pitchBounds.getFirst();
    }

    private void setPitch(double degrees) {
        if (degrees < pitchBounds.getFirst() || degrees > pitchBounds.getSecond()) {
            System.out.println("Invalid pitch target: " + degrees);
            return;
        }
        pitchMotor.setControl(motionRequest.withPosition((degrees - pitchBounds.getFirst()) / pitchGearRatio));
    }

    private double calcYawDegrees() {
        double t12 = ccDegrees(yaw12cancoder, c12Offset);
        double t11 = ccDegrees(yaw11cancoder, c11Offset);
        int N = (int) Math.round((12 * MathUtil.inputModulus(t11 - t12, -180, 180) - t11) / 360.);
        return (t11 + 360 * N) * 11. / 70.;
    }

    private double ccDegrees(CANcoder cc, double ccOffset) {
        return 360 * MathUtil.inputModulus(cc.getAbsolutePosition().getValue().in(Units.Rotations) - ccOffset, 0, 1);
    }

    private void setYaw(double degrees) {
        yawMotor.setControl(motionRequest.withPosition(-MathUtil.inputModulus(degrees, yawBounds.getFirst(), yawBounds.getSecond()) * yawGearRatio / 360.));
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
        final double minSine = Math.sin(pitchBounds.getFirst() * Math.PI / 180);
        final double maxSine = Math.sin(pitchBounds.getSecond() * Math.PI / 180);
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
            return new Pair<Double, Double>(0.0, pitchBounds.getFirst() / 360);
        }
        shooterSpeedReady = true;
        return new Pair<Double, Double>(Math.asin(-t / end) / (2 * Math.PI), Math.asin(end) / (2 * Math.PI));
    }

    // diagnostics for shooter targeting
    private double vDesired, vNow, vProjected, heightCoefficient, radialCoefficient, tangentialCoefficient, distanceCoefficient;
    private double yawStraight, yawOffset, yawTarget, pitchTarget;
    private double yawMinTorque=0, yawMaxTorque=0, pitchMinTorque=0, pitchMaxTorque=0;
    private boolean nearTrench;

    private void prepToShoot() {
        final Pair<Translation2d, Translation2d> targetVectorAndSpeeds = targetVectorAndSpeeds();
        final double h = hurling ? 1.016 : shootTarget.getZ() - turretHeight;
        final double r = targetVectorAndSpeeds.getFirst().getNorm();
        vDesired = Math.max(vMin, vScale * Math.sqrt(9.80665 * (h + Math.hypot(h, r))));
        launchMotor.setControl(new MotionMagicVelocityDutyCycle(launchToTurretSpeedScale * vDesired));
        vNow = launchMotor.getVelocity().getValueAsDouble() / launchToTurretSpeedScale;
        final double dv = launchMotor.getAcceleration().getValueAsDouble() * projectionTime / launchToTurretSpeedScale;
        vProjected = dv / (vDesired - vNow) > 1 ? vDesired : (vNow + dv);
        heightCoefficient = h / r;
        radialCoefficient = targetVectorAndSpeeds.getSecond().getX() / vProjected;
        tangentialCoefficient = targetVectorAndSpeeds.getSecond().getY() / vProjected;
        distanceCoefficient = 9.80665 * r / vProjected / vProjected;
        final Pair<Double, Double> angles = getAngles(heightCoefficient, radialCoefficient, tangentialCoefficient, distanceCoefficient);
        yawOffset = angles.getFirst();
        yawStraight = targetVectorAndSpeeds.getFirst().getAngle().getRotations();
        yawTarget = yawOffset + yawStraight;
        pitchTarget = angles.getSecond();
        shooterSpeedReady = vProjected / vDesired > 0.89;
        setYaw(yawTarget);

        final double x = Drive.Drivetrain.getInstance().getState().Pose.getX();
        nearTrench = MathUtil.isNear(4.625594, x, 0.61) || MathUtil.isNear(11.915394, x, 0.61);
        if (nearTrench || !shooterSpeedReady) {
            System.out.println("In/near a trench or low speed; can't shoot");
            setPitch(pitchBounds.getFirst());
            setMode(Mode.STANDBY);
        } else setPitch(pitchTarget * 360);
    }

    private boolean setFeeder(boolean on) {
        return feeder_spinner.setControl(on ? feederMotionRequest : brake).isOK() &&
            feeder_roller.setControl(on ? feederMotionRequest : brake).isOK();
    }

    /** Test helper: spin only the feeder spinner at 25% duty while on is true. */
    public void setFeederTest(boolean on) {
        feeder_spinner.setControl(on ? feederSpinnerTestRequest : brake);
    }

    /** Test helper: spin only the feeder roller at 75% duty while on is true. */
    public void setFeederRollerTest(boolean on) {
        feeder_roller.setControl(on ? feederRollerTestRequest : brake);
    }

    /** Test helper: spin only the launcher at test duty while on is true. */
    public void setLauncherTest(boolean on) {
        launchMotor.setControl(on ? launcherTestRequest : brake);
    }

    private boolean toggledOn = false;

    /** Accessor for the simple-toggle launcher RPM target. Used by Trimmer and RobotContainer. */
    public double getSimpleLaunchRpm() {
        return simpleLaunchRpm;
    }

    /** Simple shooting mode toggle, using the internal simpleLaunchRpm and a fixed pitch. */
    public void simpleToggle() {
        simpleToggle(simpleLaunchRpm, 25.0);
    }

    public void simpleToggle(double launchSpeedRpm, double pitchDegrees) {
        toggledOn = !toggledOn;
        System.out.println("launcher simpleToggle, toggledOn: " + toggledOn + ", target RPM=" + launchSpeedRpm);
        if (toggledOn) {
            // Closed-loop velocity control: interpret launchSpeedRpm as motor RPM
            double targetRps = launchSpeedRpm / 60.0; // rotations per second
            launchMotor.setControl(new MotionMagicVelocityDutyCycle(targetRps));
            setYaw(0);
            setPitch(pitchDegrees);
            CommandScheduler.getInstance().schedule(
                // Wait until either launcher is at speed (within tolerance) OR 3 seconds have passed,
                // then start the feeder.
                new WaitUntilCommand(() -> {
                    double currentRps = launchMotor.getVelocity().getValueAsDouble();
                    // Treat "at speed" as within 5% of target or an absolute small epsilon for low targets
                    double tol = Math.max(2.0, Math.abs(targetRps) * 0.05); // RPS tolerance
                    boolean atSpeed = Math.abs(currentRps - targetRps) <= tol;
                    // Optional debug
                    // System.out.println("Launcher wait: currentRps=" + currentRps + " targetRps=" + targetRps + " atSpeed=" + atSpeed);
                    return atSpeed;
                }).withTimeout(3.0).andThen(new InstantCommand(() -> setFeeder(true)))
            );
        } else {
            setFeeder(false);
            CommandScheduler.getInstance().schedule(
                new WaitCommand(1.5).andThen(new InstantCommand(() -> {
                    launchMotor.setControl(brake);
                    forcePitchDown();
                }))
            );
        }
    }

    public void setMode (Mode newMode) {
        if (mode == newMode) return;

        if (newMode == Mode.OFF) {
            launchMotor.setControl(brake);
            yawMotor.setControl(motionRequest);
            setPitch(pitchBounds.getFirst());
        }
        else {
            if (mode == Mode.OFF) prepToShoot();
            setFeeder(newMode == Mode.FIRE);
        }
        mode = newMode;
    }

    private boolean forcingDown = false;
    private void forcePitchDown() {
        forcingDown = true;
        pitchMotor.setControl(new DutyCycleOut(-0.1));
    }

    @Override
    public void periodic() {
        if (mode != Mode.OFF) prepToShoot();
        double pitchTorque = pitchMotor.getTorqueCurrent().getValueAsDouble();
        if (pitchTorque < pitchMinTorque) pitchMinTorque = pitchTorque;
        if (pitchTorque > pitchMaxTorque) pitchMaxTorque = pitchTorque;

        double yawTorque = yawMotor.getTorqueCurrent().getValueAsDouble();
        if (yawTorque < yawMinTorque) yawMinTorque = yawTorque;
        if (yawTorque > yawMaxTorque) yawMaxTorque = yawTorque;

        if (
            forcingDown &&
            (pitchTorque < pitchForceTorque) &&
            (pitchMotor.getVelocity().getValueAsDouble() > pitchForceVelocityLimit)
        ) {
            System.out.println("Pitch bottom limit hit, marking min pitch");
            pitchMotor.setPosition(pitchLimitRotations);
            forcingDown = false;
            setPitch(pitchBounds.getFirst());
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Launcher");
        builder.setActuator(true);

        builder.addStringProperty("Mode", () -> mode.toString(), null);
        builder.addBooleanProperty("blueAlliance", () -> blueAlliance, null);
        builder.addBooleanProperty("hurling", () -> hurling, null);
        builder.addBooleanProperty("shooterSpeedReady", () -> shooterSpeedReady, null);
        builder.addDoubleProperty("shootTargetX", () -> shootTarget.getX(), null);
        builder.addDoubleProperty("shootTargetY", () -> shootTarget.getY(), null);
        builder.addDoubleProperty("hurlTargetX", () -> hurlTargetXZ.getX(), null);

        builder.addDoubleProperty("vDesired", () -> vDesired, null);
        builder.addDoubleProperty("vNow", () -> vNow, null);
        builder.addDoubleProperty("vProjected", () -> vProjected, null);
        builder.addDoubleProperty("heightCoefficient", () -> heightCoefficient, null);
        builder.addDoubleProperty("radialCoefficient", () -> radialCoefficient, null);
        builder.addDoubleProperty("tangentialCoefficient", () -> tangentialCoefficient, null);
        builder.addDoubleProperty("distanceCoefficient", () -> distanceCoefficient, null);
        builder.addDoubleProperty("yawStraight", () -> yawStraight, null);
        builder.addDoubleProperty("yawOffset", () -> yawOffset, null);
        builder.addDoubleProperty("yawTarget", () -> yawTarget, null);
        builder.addDoubleProperty("pitchTarget", () -> pitchTarget, null);
        builder.addBooleanProperty("nearTrench", () -> nearTrench, null);

        builder.addDoubleProperty("cancoder12Raw", () -> yaw12cancoder.getAbsolutePosition().getValue().in(Units.Rotations), null);
        builder.addDoubleProperty("cancoder11Raw", () -> yaw11cancoder.getAbsolutePosition().getValue().in(Units.Rotations), null);
        builder.addDoubleProperty("calcYawDegrees", () -> calcYawDegrees(), null);
        builder.addDoubleProperty("motorRotations", () -> yawMotor.getPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("yawFromMotor", () -> -yawMotor.getPosition().getValueAsDouble() * 360. / yawGearRatio, null);
        builder.addDoubleProperty("yawMinTorque", () -> yawMinTorque, null);
        builder.addDoubleProperty("yawMaxTorque", () -> yawMaxTorque, null);

        builder.addDoubleProperty("pitchRot", () -> pitchMotor.getPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("pitch", () -> getPitch(), null);
        builder.addDoubleProperty("pitchMinTorque", () -> pitchMinTorque, null);
        builder.addDoubleProperty("pitchMaxTorque", () -> pitchMaxTorque, null);

        builder.addDoubleProperty("Launch Speed", () -> launchMotor.getVelocity().getValueAsDouble(), null);
        builder.addDoubleProperty("Simple Launch RPM Target", () -> simpleLaunchRpm, null);
    }

    double yawSetpoint = Double.NaN;
    double pitchSetpoint = pitchBounds.getFirst();
    private void initTrimmer() {
        Trimmer trimmer = Trimmer.getInstance();
        trimmer.add(
            "Launcher",
            "Yaw +- 1 deg",
            () -> yawSetpoint,
            (up) -> {
                if (Double.isNaN(yawSetpoint)) {
                    yawSetpoint = -yawMotor.getPosition().getValueAsDouble() * 360. / yawGearRatio;
                }
                yawSetpoint += (up ? 1. : -1.);
                setYaw(yawSetpoint);
            }
        );
        trimmer.add(
            "Launcher",
            "Yaw +- 10 deg",
            () -> yawSetpoint,
            (up) -> {
                if (Double.isNaN(yawSetpoint)) {
                    yawSetpoint = -yawMotor.getPosition().getValueAsDouble() * 360. / yawGearRatio;
                }
                yawSetpoint += (up ? 10. : -10.);
                setYaw(yawSetpoint);
            }
        );
        trimmer.add(
            "Launcher",
            "Pitch +- 1 deg",
            () -> pitchSetpoint,
            (up) -> {
                pitchSetpoint += (up ? 1. : -1.);
                setPitch(pitchSetpoint);
            }
        );
        trimmer.add(
            "Launcher",
            "Simple Launch RPM +- 100",
            () -> simpleLaunchRpm,
            (up) -> {
                double delta = 100.0;
                simpleLaunchRpm = Math.max(0.0, simpleLaunchRpm + (up ? delta : -delta));
                System.out.println("[Launcher] Updated simple launch RPM target: " + simpleLaunchRpm);
            }
        );
    }
}
