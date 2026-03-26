package frc.robot.subsystems.launcher;

// import frc.robot.subsystems.launcher.LauncherConstants;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.math.MathUtil;
import frc.robot.Robot;
import frc.robot.Robot.Ports;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Trimmer;
import frc.robot.subsystems.Drive.Drivetrain;

import static frc.robot.Robot.kCANBusJustice;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Amps;

public final class Launcher extends SubsystemBase {
    public static enum Mode {OFF, STANDBY, FIRE}
    private Mode mode = Mode.OFF;

    private final ShooterKinematics kinematics = new ShooterKinematics();

    // Diagnostics & Telemetry
    private double targetDist, targetRadialVelo, targetRPM, adjustedRPM, pitchTarget, yawTarget;
    private boolean blueAlliance, hurling, shooterSpeedReady, nearTrench;
    private double yawMinTorque=0, yawMaxTorque=0, pitchMinTorque=0, pitchMaxTorque=0;

    private Translation3d shootTarget = null; 
    private Translation2d hurlTargetXZ = null; 
    private static Launcher instance = null;

    private final StaticBrake brake = new StaticBrake();
    private final DutyCycleOut feederSpinnerMotionRequest = new DutyCycleOut(LauncherConstants.kFeederSpinnerMotionDuty);
    private final DutyCycleOut feederSpinnerWithIntakeRequest = new DutyCycleOut(LauncherConstants.kFeederSpinnerWithIntakeRequestDuty);
    private final DutyCycleOut feederRollerMotionRequest = new DutyCycleOut(LauncherConstants.kFeederRollerMotionDuty);

    private double launchP = LauncherConstants.kDefaultLaunchP;
    private double launchD = LauncherConstants.kDefaultLaunchD;
    private double launchI = LauncherConstants.kDefaultLaunchI;

    private double pitchP = 0.1, pitchI = 0.0, pitchD = 0.0;

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
    private double simpleLaunchRpm = 1800.0;
    private double simpleLaunchPitch = 25.0;
    private double simpleLaunchYaw = 0.0;
    private boolean toggledOn = false;
    private boolean dynamicYaw;
    private boolean forcingDown = false;
    private double pitchHomingStartTime = 0;
    /**
     * When true: hood always tracks {@link #pitchTarget} from distance/radial-velocity polynomial,
     * ignoring trench proximity and flywheel speed. For testing / tuning only.
     */
    private boolean hoodPolynomialTestingOnly = true;
    /**
     * When true (with launcher {@link Mode#OFF}): turret follows {@link #yawTarget} from pose/motion
     * so you can tune tracking while driving without STANDBY/FIRE. For testing only.
     */
    private boolean turretTrackingTestingOnly = true;

    public static Launcher getInstance() {
        if (instance == null) instance = new Launcher();
        return instance;
    }

    private Launcher() {
        feeder_spinner.getConfigurator().apply(feederMotorConfig);
        feeder_roller.getConfigurator().apply(feederMotorConfig);

        // Don't configure can coder offsets here; enter offsets into c11 and c12 offset constants above
        yaw12cancoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(0).withAbsoluteSensorDiscontinuityPoint(1));
        yaw11cancoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(0).withAbsoluteSensorDiscontinuityPoint(1));

        DriverStation.getAlliance().ifPresentOrElse((color) -> {
            shootTarget = new Translation3d(color == Alliance.Blue ? 4.625594 : 11.915394, 4.034536, 1.8288);
            hurlTargetXZ = new Translation2d(color == Alliance.Blue ? 3.048 : 13.492988, 1.016); //TODO: Set this
            blueAlliance = color == Alliance.Blue;
        }, () -> {
            throw new IllegalArgumentException("Is this code happening too early and the alliance color isn't available yet?");
        });

        

        if (yawMotor.getConfigurator().apply(
            new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(0.1).withKI(0.2))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(60)
                .withMotionMagicAcceleration(100)
                .withMotionMagicJerk(1000))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(Amps.of(40)).withPeakReverseTorqueCurrent(Amps.of(-40)))
        ).isOK()) {
            yawMotor.setPosition(-calcYawDegrees() * LauncherConstants.yawGearRatio / 360.);
        }
        else DriverStation.reportError("Yaw motor config apply failed so the motor couldn't be zeroed", true);

        forcePitchDown();
        setYaw(0);
        setPitch(LauncherConstants.minPitch); // zero at min pitch
        setLaunchMotorConfig(); // separate function to allow for smart dashboard pid tuning
        setPitchMotorConfig();
        initTrimmer();
    }

    private void setLaunchMotorConfig() {
        launchMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(launchP).withKI(launchI).withKD(launchD))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(60)
                .withMotionMagicAcceleration(100)
                .withMotionMagicJerk(1000))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8)))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(Amps.of(30)).withPeakReverseTorqueCurrent(Amps.of(-30))));
    }

    private void setPitchMotorConfig() {
        pitchMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(pitchP).withKI(pitchI).withKD(pitchD))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(60)
                .withMotionMagicAcceleration(200)
                .withMotionMagicJerk(1000))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(Amps.of(20)).withPeakReverseTorqueCurrent(Amps.of(-20)))
        );
    }

    /**
     * calculates the yaw degrees from the cancodeers
     * @return the yaw degrees of the turret
     */
    private double calcYawDegrees() {
        double t12 = ccDegrees(yaw12cancoder, LauncherConstants.c12Offset);
        double t11 = ccDegrees(yaw11cancoder, LauncherConstants.c11Offset);
        int N = (int) Math.round((12 * MathUtil.inputModulus(t11 - t12, -180, 180) - t11) / 360.);
        return (t11 + 360 * N) * 11. / 70.;
    }

    /**
     * CanCoder degrees helper
     */
    private double ccDegrees(CANcoder cc, double ccOffset) {
        return 360 * MathUtil.inputModulus(cc.getAbsolutePosition().getValue().in(Units.Rotations) - ccOffset, 0, 1);
    }

    /**
     * Hood exit angle above horizontal (deg), consistent with {@link #setPitch}:
     * {@code pitch = motorRotations * pitchGearRatio + minPitch}.
     */
    private double getPitch() {
        return pitchMotor.getPosition().getValueAsDouble() * LauncherConstants.pitchGearRatio
            + LauncherConstants.pitchBounds.getFirst();
    }

    private double lastPitchTarget = Double.NaN;

    private void setPitch(double degrees) {
        if (Double.isFinite(lastPitchTarget) && Math.abs(degrees - lastPitchTarget) < 0.1) {
            return;
        }
        lastPitchTarget = degrees;
        double minP = LauncherConstants.pitchBounds.getFirst();
        double maxP = LauncherConstants.pitchBounds.getSecond();
        if (degrees < minP || degrees > maxP) {
            System.out.println("Invalid pitch target: " + degrees);
            return;
        }
        double rotations = (degrees - minP) / LauncherConstants.pitchGearRatio;
        pitchMotor.setControl(new MotionMagicDutyCycle(rotations));
    }

    /** Operator presets: always send Motion Magic (ignores lastPitchTarget debounce). */
    public void commandHoodAngleDegrees(double degrees) {
        lastPitchTarget = Double.NaN;
        double minP = LauncherConstants.pitchBounds.getFirst();
        double maxP = LauncherConstants.pitchBounds.getSecond();
        setPitch(MathUtil.clamp(degrees, minP, maxP));
    }

    /** Hood to homed minimum pitch (same reference as after forcePitchDown / encoder zero). */
    public void commandHoodToHomedZero() {
        commandHoodAngleDegrees(LauncherConstants.pitchBounds.getFirst());
    }
    /**
     * 
     * @param degrees desired yaw in degrees
     */
    private void setYaw(double degrees) {
        yawMotor.setControl(new MotionMagicDutyCycle(-MathUtil.inputModulus(degrees, LauncherConstants.yawBounds.getFirst(), LauncherConstants.yawBounds.getSecond()) * LauncherConstants.yawGearRatio / 360.));
    }

    /**
     * Updates distance, radial velocity, hub/hurl selection, and polynomial targets. No motor outputs.
     */
    private void computeShootingTargets() {
        SwerveDriveState state = Drive.Drivetrain.getInstance().getState();
        Pose2d currentPose = state.Pose;

        double rvx = state.Speeds.vxMetersPerSecond;
        double rvy = state.Speeds.vyMetersPerSecond;
        double robotYaw = currentPose.getRotation().getRadians();
        double fieldVx = rvx * Math.cos(robotYaw) - rvy * Math.sin(robotYaw);
        double fieldVy = rvx * Math.sin(robotYaw) + rvy * Math.cos(robotYaw);

        Pose2d projectedPose = new Pose2d(
            currentPose.getX() + fieldVx * LauncherConstants.projectionTime,
            currentPose.getY() + fieldVy * LauncherConstants.projectionTime,
            new Rotation2d(robotYaw + state.Speeds.omegaRadiansPerSecond * LauncherConstants.projectionTime)
        );

        hurling = blueAlliance ?
            (projectedPose.getX() > 4.625594 && !MathUtil.isNear(4.034536, projectedPose.getY(), 1.2192)) :
            (projectedPose.getX() < 11.915394 && !MathUtil.isNear(4.034536, projectedPose.getY(), 1.2192));

        Translation2d activeTarget = hurling ?
            new Translation2d(hurlTargetXZ.getX(), projectedPose.getY() > 4.034536 ? 6.0519945 : 2.017268) :
            new Translation2d(shootTarget.getX(), shootTarget.getY());

        Translation2d robotToTarget = activeTarget.minus(projectedPose.getTranslation());
        targetDist = robotToTarget.getNorm();
        Rotation2d angleToTarget = robotToTarget.getAngle();

        targetRadialVelo = (fieldVx * angleToTarget.getCos()) + (fieldVy * angleToTarget.getSin());

        targetRPM = kinematics.getTargetFlywheelRPM(targetDist, targetRadialVelo);
        pitchTarget = kinematics.getTargetHoodAngle(targetDist, targetRadialVelo);
        yawTarget = angleToTarget.getDegrees() - projectedPose.getRotation().getDegrees();

        nearTrench = MathUtil.isNear(4.625594, currentPose.getX(), 0.61)
            || MathUtil.isNear(11.915394, currentPose.getX(), 0.61);
    }

    /**
     * Calculates targeting variables using WPILib Geometry and queries the Polynomial Surface.
     */
    private void prepToShoot() {
        computeShootingTargets();

        adjustedRPM = kinematics.getAdjustedFlywheelRPM(targetRPM);

        System.out.println("AdjustedRPM" + adjustedRPM);

        launchMotor.setControl(new MotionMagicVelocityDutyCycle(adjustedRPM / 60.0));
        setYaw(yawTarget);

        double currentRPM = launchMotor.getVelocity().getValueAsDouble() * 60.0;
        shooterSpeedReady = targetRPM > 0 && Math.abs(currentRPM - targetRPM) < (targetRPM * LauncherConstants.kRPMTolerance);

        if (hoodPolynomialTestingOnly) {
            setPitch(pitchTarget);
        } else if (nearTrench || !shooterSpeedReady) {
            setPitch(LauncherConstants.pitchBounds.getFirst());
        } else {
            setPitch(pitchTarget);
        }
    }

    private void setFeeder(boolean on) {
        feeder_spinner.setControl(on ? feederSpinnerMotionRequest : brake);
        feeder_roller.setControl(on ? feederRollerMotionRequest : brake);
    }

    /** Simple shooting mode toggle, using the internal simpleLaunchRpm and a fixed pitch. */
    public void simpleToggle() {
        simpleToggle(simpleLaunchRpm, simpleLaunchPitch, simpleLaunchYaw);
    }

    public void simpleToggle(double launchSpeedRpm, double pitchDegrees) {
        simpleToggle(launchSpeedRpm, pitchDegrees, 0.0);
    }


    /* A yaw of NaN indicates that actually we will use dynamic yaw for hurling */
    public void simpleToggle(double launchSpeedRpm, double pitchDegrees, double yawDegrees) {
        toggledOn = !toggledOn;
        // System.out.println("launcher simpleToggle, toggledOn: " + toggledOn + ", target RPM=" + launchSpeedRpm);
        if (toggledOn) {
            // Closed-loop velocity control: interpret launchSpeedRpm as motor RPM
            double targetRps = launchSpeedRpm / 60.0; // rotations per second
            launchMotor.setControl(new MotionMagicVelocityDutyCycle(targetRps));
            dynamicYaw = Double.isNaN(yawDegrees);
            if (!dynamicYaw) setYaw(yawDegrees);
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
                }).withTimeout(3.0)
                .andThen(new WaitCommand(0.5))
                .andThen(new InstantCommand(() -> {if (toggledOn) setFeeder(true);}, this))
            );
        } else {
            setFeeder(false);
            CommandScheduler.getInstance().schedule(
                new WaitCommand(1.5).andThen(new InstantCommand(() -> {
                    if (!toggledOn) {
                        launchMotor.setControl(brake);
                        forcePitchDown();
                        setYaw(0);
                    }
                }, this))
            );
        }
    }

    public void setMode (Mode newMode) {
        if (mode == newMode) return;

        if (newMode == Mode.OFF) {
            launchMotor.setControl(brake);
            setYaw(0);
            setPitch(LauncherConstants.pitchBounds.getFirst());
            setFeeder(false); // Make sure feeder is off
        }
        else {
            if (mode == Mode.OFF){
                prepToShoot();
            }
            // setFeeder(newMode == Mode.FIRE);
        }
        mode = newMode;
    }

    public boolean isAimingOrShooting() {
        return mode == Mode.STANDBY || mode == Mode.FIRE;
    }

    public void forcePitchDown() {
        forcingDown = true;
        lastPitchTarget = Double.NaN;
        pitchHomingStartTime = Timer.getFPGATimestamp();
        pitchMotor.setControl(new DutyCycleOut(-0.1));
    }

    @Override
    public void periodic() {
        if (mode != Mode.OFF) {
            prepToShoot();
        } else if (!forcingDown && (hoodPolynomialTestingOnly || turretTrackingTestingOnly)) {
            computeShootingTargets();
            if (hoodPolynomialTestingOnly) {
                setPitch(pitchTarget);
            }
            if (turretTrackingTestingOnly) {
                setYaw(yawTarget);
            }
        }

        // Run feeder spinner at slow rate when intake is spinning (unless we're firing)

        if (mode == Mode.FIRE && !toggledOn) {
            // Only push the ball in if the flywheel is at speed and we are out of trench
            setFeeder(shooterSpeedReady && !nearTrench); 
        } else if (mode != Mode.FIRE && !toggledOn) {
            // Standard intake pass-through logic for STANDBY/OFF
            boolean intakeSpinning = Intake.getInstance().isSpinning();
            feeder_spinner.setControl(intakeSpinning ? feederSpinnerWithIntakeRequest : brake);
            feeder_roller.setControl(brake); // Keep roller off unless firing
        }


        double pitchTorque = pitchMotor.getTorqueCurrent().getValueAsDouble();
        if (pitchTorque < pitchMinTorque) pitchMinTorque = pitchTorque;
        if (pitchTorque > pitchMaxTorque) pitchMaxTorque = pitchTorque;

        double yawTorque = yawMotor.getTorqueCurrent().getValueAsDouble();
        if (yawTorque < yawMinTorque) yawMinTorque = yawTorque;
        if (yawTorque > yawMaxTorque) yawMaxTorque = yawTorque;

        if (forcingDown) {
            double vel = pitchMotor.getVelocity().getValueAsDouble();
            boolean stalled = Math.abs(vel) < LauncherConstants.pitchHomingStallVelocityEpsilon
                && Math.abs(pitchTorque) > LauncherConstants.pitchHomingStallTorqueAmps;
            boolean timedOut = Timer.getFPGATimestamp() - pitchHomingStartTime
                >= LauncherConstants.pitchHomingTimeoutSeconds;
            if (stalled || timedOut) {
                if (timedOut && !stalled) {
                    DriverStation.reportWarning(
                        "[Launcher] Pitch homing finished by timeout — check stall torque/velocity thresholds.", false);
                }
                pitchMotor.setPosition(LauncherConstants.pitchLimitRotations);
                forcingDown = false;
                lastPitchTarget = Double.NaN;
                setPitch(LauncherConstants.pitchBounds.getFirst());
            }
        }

        if (toggledOn && dynamicYaw && !turretTrackingTestingOnly) {
            setYaw(-Drive.Drivetrain.getInstance().getState().Pose.getRotation().getDegrees() + (blueAlliance ? 0 : 180));
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Launcher");
        builder.setActuator(true);

        builder.addStringProperty("Mode", () -> mode.toString(), null);
        builder.addBooleanProperty(
            "Hood poly testing (ignore trench/RPM gate)",
            () -> hoodPolynomialTestingOnly,
            v -> hoodPolynomialTestingOnly = v);
        builder.addBooleanProperty(
            "Turret tracking testing (OFF: aim while driving)",
            () -> turretTrackingTestingOnly,
            v -> turretTrackingTestingOnly = v);
        builder.addDoubleProperty("Seconds left until hopper color switches:", () -> Robot.getCountDown(), null);
        builder.addBooleanProperty("hurling", () -> hurling, null);
        builder.addBooleanProperty("shooterSpeedReady", () -> shooterSpeedReady, null);
        builder.addDoubleProperty("shootTargetX", () -> shootTarget.getX(), null);
        builder.addDoubleProperty("shootTargetY", () -> shootTarget.getY(), null);
        builder.addDoubleProperty("hurlTargetX", () -> hurlTargetXZ.getX(), null);

        // New Kinematics Telemetry
        builder.addDoubleProperty("Target Distance (m)", () -> targetDist, null);
        builder.addDoubleProperty("Target Radial Velo (m/s)", () -> targetRadialVelo, null);
        builder.addDoubleProperty("Polynomial Target RPM", () -> targetRPM, null);
        builder.addDoubleProperty("Polynomial Target Pitch", () -> pitchTarget, null);
        builder.addDoubleProperty("Yaw Target", () -> yawTarget, null);

        builder.addDoubleProperty("Launch Speed (RPM)", () -> launchMotor.getVelocity().getValueAsDouble() * 60.0, null);

        builder.addDoubleProperty("cancoder12Raw", () -> yaw12cancoder.getAbsolutePosition().getValue().in(Units.Rotations), null);
        builder.addDoubleProperty("cancoder11Raw", () -> yaw11cancoder.getAbsolutePosition().getValue().in(Units.Rotations), null);
        builder.addDoubleProperty("calcYawDegrees", () -> calcYawDegrees(), null);
        builder.addDoubleProperty("yawMotorRotations", () -> yawMotor.getPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("yawFromMotor", () -> -yawMotor.getPosition().getValueAsDouble() * 360. / LauncherConstants.yawGearRatio, null);
        builder.addDoubleProperty("yawMinTorque", () -> yawMinTorque, null);
        builder.addDoubleProperty("yawMaxTorque", () -> yawMaxTorque, null);

        builder.addDoubleProperty("pitchRot", () -> pitchMotor.getPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("pitch", () -> getPitch(), null);
        builder.addDoubleProperty("pitchMinTorque", () -> pitchMinTorque, null);
        builder.addDoubleProperty("pitchMaxTorque", () -> pitchMaxTorque, null);

        builder.addDoubleProperty("Launch Speed", () -> launchMotor.getVelocity().getValueAsDouble(), null);
        builder.addDoubleProperty("Simple Launch RPM Target", () -> simpleLaunchRpm, null);
        builder.addDoubleProperty("Simple Launch Pitch", () -> simpleLaunchPitch, null);
        builder.addDoubleProperty("Simple Launch Yaw", () -> simpleLaunchYaw, null);
        builder.addDoubleProperty("feeder spinner speed", () -> feeder_spinner.getVelocity().getValueAsDouble(), null);
        builder.addDoubleProperty("feeder roller speed", () -> feeder_roller.getVelocity().getValueAsDouble(), null);
    }

    double yawSetpoint = Double.NaN;
    double pitchSetpoint = LauncherConstants.pitchBounds.getFirst();
    private void initTrimmer() {
        Trimmer trimmer = Trimmer.getInstance();
        trimmer.add(
            "Launcher",
            "Yaw +- 1 deg",
            () -> yawSetpoint,
            (up) -> {
                if (Double.isNaN(yawSetpoint)) {
                    yawSetpoint = -yawMotor.getPosition().getValueAsDouble() * 360. / LauncherConstants.yawGearRatio;
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
                    yawSetpoint = -yawMotor.getPosition().getValueAsDouble() * 360. / LauncherConstants.yawGearRatio;
                }
                yawSetpoint += (up ? 10. : -10.);
                setYaw(yawSetpoint);
            }
        );
        /*trimmer.add(
            "Launcher",
            "Pitch +- 1 deg",
            () -> pitchSetpoint,
            (up) -> {
                pitchSetpoint += (up ? 1. : -1.);
                setPitch(pitchSetpoint);
            }
        );*/
        trimmer.add(
            "Launcher",
            "Simple Launch RPM +- 25",
            () -> simpleLaunchRpm,
            (up) -> {
                double delta = 25.0;
                simpleLaunchRpm = Math.max(0.0, simpleLaunchRpm + (up ? delta : -delta));
                System.out.println("[Launcher] Updated simple launch RPM target: " + simpleLaunchRpm);
            }
        );
        trimmer.add(
            "Launcher",
            "Simple Launch Pitch +- 1",
            () -> simpleLaunchPitch,
            (up) -> {
                double delta = 1.0;
                simpleLaunchPitch = MathUtil.clamp(simpleLaunchPitch + (up ? delta : -delta), 15, 50);
                System.out.println("[Launcher] Updated simple launch Pitch target: " + simpleLaunchPitch);
            }
        );
        trimmer.add(
            "Launcher",
            "Simple Launch Yaw +- 5",
            () -> simpleLaunchYaw,
            (up) -> {
                double delta = 5.0;
                simpleLaunchYaw = MathUtil.clamp(simpleLaunchYaw + (up ? delta : -delta), -100, 260);
                System.out.println("[Launcher] Updated simple launch Yaw target: " + simpleLaunchYaw);
            }
        );
        trimmer.add(
            "Launcher PID",
            "Pitch P",
            () -> pitchP,
            (up) -> {pitchP = Trimmer.increment(pitchP, 0.001, 0.2, up); setPitchMotorConfig();}
        );
        trimmer.add(
            "Launcher PID",
            "Pitch I",
            () -> pitchI,
            (up) -> {pitchI = Trimmer.increment(pitchI, 0.001, 0.2, up); setPitchMotorConfig();}
        );
        trimmer.add(
            "Launcher PID",
            "Pitch D",
            () -> pitchD,
            (up) -> {pitchD = Trimmer.increment(pitchD, 0.001, 0.2, up); setPitchMotorConfig();}
        );
    }
}
