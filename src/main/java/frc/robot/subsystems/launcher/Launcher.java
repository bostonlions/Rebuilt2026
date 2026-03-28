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
import frc.robot.Robot;
import frc.robot.Robot.Ports;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Trimmer;

import static frc.robot.Robot.kCANBusJustice;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Amps;

public final class Launcher extends SubsystemBase {
    public static enum Mode {OFF, STANDBY, FIRE}
    private Mode mode = Mode.OFF;

    private final ShooterKinematics kinematics = new ShooterKinematics();

    // Diagnostics & Telemetry
    private double targetDist, targetRadialVelo, targetTangentialVelo, targetRPM, adjustedRPM, pitchTarget, yawTarget;
    private boolean blueAlliance, hurling, shooterSpeedReady, nearTrench;
    private double yawMinTorque=0, yawMaxTorque=0, pitchMinTorque=0, pitchMaxTorque=0;

    private Translation3d shootTarget = null; 
    private Translation2d hurlTargetXZ = null; 
    private static Launcher instance = null;
    // private Translation3d shotVector = null;

    private final StaticBrake brake = new StaticBrake();
    private final DutyCycleOut feederSpinnerMotionRequest = new DutyCycleOut(LauncherConstants.kFeederSpinnerMotionDuty);
    private final DutyCycleOut feederSpinnerWithIntakeRequest = new DutyCycleOut(LauncherConstants.kFeederSpinnerWithIntakeRequestDuty);
    private final DutyCycleOut feederRollerMotionRequest = new DutyCycleOut(LauncherConstants.kFeederRollerMotionDuty);

    private double launchP = LauncherConstants.kDefaultLaunchP;
    // private double launchD = LauncherConstants.kDefaultLaunchD;
    private double launchI = LauncherConstants.kDefaultLaunchI;

    private double yawP = LauncherConstants.kDefaultYawP;
    private double yawI = LauncherConstants.kDefaultYawI;
    private double yawD = LauncherConstants.kDefaultYawD;

    private double pitchP = 0.1, pitchI = 0.0, pitchD = 0.0;

    private double pitchTorque; // member variable for use in smart dashboard

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
    /** Last Motion Magic scale applied to yaw (1.0 = normal, {@link LauncherConstants#kYawLongPathMotionMagicScale} = wrap path). */
    private double m_yawMotionMagicScale = Double.NaN;
    /**
     * After commanding a wrap (raw angle outside soft limits), keep scaled Motion Magic until the turret reaches
     * the wrapped goal so {@code yawTarget} re-entering [min,max] mid-move does not snap back to full cruise/accel.
     */
    private boolean m_yawLongPathProfileLatched = false;

    public static Launcher getInstance() {
        if (instance == null) instance = new Launcher();
        return instance;
    }

    private Launcher() {
        feeder_spinner.getConfigurator().apply(feederMotorConfig);
        feeder_roller.getConfigurator().apply(feederMotorConfig);

        // Don't configure can coder offsets here; enter offsets into c11 and c12 offset constants
        yaw12cancoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(0).withAbsoluteSensorDiscontinuityPoint(1));
        yaw11cancoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(0).withAbsoluteSensorDiscontinuityPoint(1));
        yaw11cancoder.getAbsolutePosition().waitForUpdate(0.25);
        yaw12cancoder.getAbsolutePosition().waitForUpdate(0.25);
        yawMotor.setPosition(-calcYawDegrees() * LauncherConstants.yawGearRatio / 360.);

        setAlliance();
        forcePitchDown();
        setPitch(LauncherConstants.pitchBounds.getSecond()); // zero at min pitch
        setLaunchMotorConfig(); // separate function to allow for smart dashboard pid tuning
        setPitchMotorConfig();
        setYawMotorConfig();
        setYaw(0);
        initTrimmer();
    }

    private void setAlliance() {
        DriverStation.getAlliance().ifPresentOrElse((color) -> {
            shootTarget = new Translation3d(color == Alliance.Blue ? 4.625594 : 11.915394, 4.034536, 1.8288);
            hurlTargetXZ = new Translation2d(color == Alliance.Blue ? 3.048 : 13.492988, 1.016);
            blueAlliance = color == Alliance.Blue;
        }, () -> {
            throw new IllegalArgumentException("Is this code happening too early and the alliance color isn't available yet?");
        });
    }

    private void setLaunchMotorConfig() {
        launchMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(launchP).withKI(launchI).withKD(0).withKV(0.01035))
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
                .withMotionMagicCruiseVelocity(6000)
                .withMotionMagicAcceleration(1000)
                .withMotionMagicJerk(16000))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(Amps.of(15)).withPeakReverseTorqueCurrent(Amps.of(-15)))
        );
    }

    private void setYawMotorConfig() {
        m_yawMotionMagicScale = Double.NaN;
        m_yawLongPathProfileLatched = false;
        ensureYawMotionMagicScale(1.0);
    }

    /** Applies yaw PID slot + Motion Magic limits scaled by {@code scale} (1.0 normal, {@link LauncherConstants#kYawLongPathMotionMagicScale} for wrap). */
    private void ensureYawMotionMagicScale(double scale) {
        if (Double.isFinite(m_yawMotionMagicScale) && Math.abs(scale - m_yawMotionMagicScale) < 1e-6) {
            return;
        }
        m_yawMotionMagicScale = scale;
        yawMotor.getConfigurator().apply(
            new TalonFXConfiguration()
                .withSlot0(new Slot0Configs().withKP(yawP).withKI(yawI).withKD(yawD))
                .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(LauncherConstants.kYawMotionMagicCruiseVelocity * scale)
                    .withMotionMagicAcceleration(LauncherConstants.kYawMotionMagicAcceleration * scale)
                    .withMotionMagicJerk(LauncherConstants.kYawMotionMagicJerk * scale))
                .withTorqueCurrent(new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(Amps.of(40)).withPeakReverseTorqueCurrent(Amps.of(-40)))
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
     * 
     * @return pitch in degrees
     */
    private double getPitch() {
        return pitchMotor.getPosition().getValueAsDouble() * LauncherConstants.pitchGearRatio + LauncherConstants.pitchBounds.getFirst();
    }

    private void setPitch(double degrees) {
        System.out.println("SetPitch: " + degrees);
        if (degrees < LauncherConstants.pitchBounds.getFirst() || degrees > LauncherConstants.pitchBounds.getSecond()) {
            System.out.println("Invalid pitch target: " + degrees);
            return;
        }
        
        //pitchMotor.setControl(new MotionMagicDutyCycle((degrees - LauncherConstants.pitchBounds.getFirst()) / LauncherConstants.pitchGearRatio));
        pitchMotor.setControl(new MotionMagicDutyCycle((LauncherConstants.pitchBounds.getSecond() - degrees) / LauncherConstants.pitchGearRatio));
    }
    /**
     * @param degrees desired turret yaw in degrees (robot-relative), same convention as {@link MathUtil#inputModulus}.
     */
    private void setYaw(double rawDegrees) {
    final double min = LauncherConstants.yawBounds.getFirst(); // e.g., -90
    final double max = LauncherConstants.yawBounds.getSecond(); // e.g., 230
    final double pastTol = LauncherConstants.kTurretLimitPastHoldDeg;
    final double current = calcYawDegrees();

    // 1. Calculate the shortest physical path safely
    double error = MathUtil.inputModulus(rawDegrees - current, -180.0, 180.0);
    double target = current + error;

    // 2. Wrap-around logic & Latching (Replaces your `if (degrees > max...)`)
    // If the shortest path hits a limit, swing the long way around and latch the profile
    if (target > max) {
        if (target - 360.0 >= min) {
            target -= 360.0;
            m_yawLongPathProfileLatched = true;
        }
    } else if (target < min) {
        if (target + 360.0 <= max) {
            target += 360.0;
            m_yawLongPathProfileLatched = true;
        }
    }

    // 3. Limit Holding Logic (Directly from your snippet, but using 'target')
    final boolean atHighLimit = current >= max - pastTol;
    final boolean atLowLimit = current <= min + pastTol;
    
    final boolean holdHigh = atHighLimit && target > max && target <= max + pastTol;
    final boolean holdLow = atLowLimit && target < min && target >= min - pastTol;

    if (holdHigh || holdLow) {
        m_yawLongPathProfileLatched = false;
        ensureYawMotionMagicScale(1.0);
        
        // CRITICAL FIX: We clamp, but DO NOT use inputModulus here!
        double holdDeg = MathUtil.clamp(current, min, max);
        yawMotor.setControl(new MotionMagicDutyCycle(-holdDeg * LauncherConstants.yawGearRatio / 360.0));
        return;
    }

    // 4. Final safety clamp in case the target is mathematically impossible
    target = MathUtil.clamp(target, min, max);

    // 5. Arrival check to unlatch the profile
    // CRITICAL FIX: Use linear distance (target - current), not circular modulus!
    // If you use modulus here, it unlatches halfway through a 360-degree swing.
    double errDeg = target - current; 
    
    if (m_yawLongPathProfileLatched && Math.abs(errDeg) < LauncherConstants.kYawLongPathArriveEpsilonDeg) {
        m_yawLongPathProfileLatched = false;
    }

    // 6. Apply your Motion Magic Scale
    ensureYawMotionMagicScale(
        m_yawLongPathProfileLatched ? LauncherConstants.kYawLongPathMotionMagicScale : 1.0
    );

    // 7. Command the motor
    yawMotor.setControl(new MotionMagicDutyCycle(-target * LauncherConstants.yawGearRatio / 360.0));
}

    /**
     * Calculates targeting variables using WPILib Geometry and queries the Polynomial Surface.
     */
    private void prepToShoot() {
        SwerveDriveState state = Drive.Drivetrain.getInstance().getState();
        Pose2d currentPose = state.Pose;
        
        // 1. Convert Robot-Centric Speeds to Field-Centric Speeds
        double rvx = state.Speeds.vxMetersPerSecond;
        double rvy = state.Speeds.vyMetersPerSecond;
        double robotYaw = currentPose.getRotation().getRadians();
        double fieldVx = rvx * Math.cos(robotYaw) - rvy * Math.sin(robotYaw);
        double fieldVy = rvx * Math.sin(robotYaw) + rvy * Math.cos(robotYaw);
        double finalRobotYaw = robotYaw + state.Speeds.omegaRadiansPerSecond * LauncherConstants.projectionTime;
        double tx = LauncherConstants.turretPosRobotRel.getX();
        double ty = LauncherConstants.turretPosRobotRel.getY();

        // 2. Project pose slightly into the future based on velocity
        Pose2d projectedPose = new Pose2d(
            currentPose.getX() + fieldVx * LauncherConstants.projectionTime + tx * Math.cos(finalRobotYaw) - ty * Math.sin(finalRobotYaw),
            currentPose.getY() + fieldVy * LauncherConstants.projectionTime + tx * Math.sin(finalRobotYaw) + ty * Math.cos(finalRobotYaw),
            new Rotation2d(finalRobotYaw)
        );

        // 3. Determine Active Target (Hub vs Hurling)
        hurling = blueAlliance ? 
            (projectedPose.getX() > 4.625594 && !MathUtil.isNear(4.034536, projectedPose.getY(), 1.2192)) : //TODO: CHECK
            (projectedPose.getX() < 11.915394 && !MathUtil.isNear(4.034536, projectedPose.getY(), 1.2192));
            
        Translation2d activeTarget = hurling ? 
            new Translation2d(hurlTargetXZ.getX(), projectedPose.getY() > 4.034536 ? 6.0519945 : 2.017268) : 
            new Translation2d(shootTarget.getX(), shootTarget.getY());

        //System.out.println("Active target: (" + activeTarget.getX() + ", " + activeTarget.getY() + " )" );

        // 4. Calculate Distance and Radial Velocity
        Translation2d robotToTarget = activeTarget.minus(projectedPose.getTranslation());
        targetDist = robotToTarget.getNorm();
        Rotation2d angleToTarget = robotToTarget.getAngle();
        //System.out.println("AngleToTarget: " + angleToTarget);

        // Dot product of field velocity and the unit vector pointing at the target
        targetRadialVelo = (fieldVx * angleToTarget.getCos()) + (fieldVy * angleToTarget.getSin());
        targetTangentialVelo = (fieldVy * angleToTarget.getCos()) - (fieldVx * angleToTarget.getSin());

        // 5. Query the Polynomial Fit
        double exitVeloMetersPerSec = kinematics.getExitVeloMetersPerSec(targetDist, targetRadialVelo);
        targetRPM = kinematics.getTargetFlywheelRPM(exitVeloMetersPerSec);
        pitchTarget = kinematics.getTargetHoodAngle(targetDist, targetRadialVelo);

        // 6. Calculate Yaw Target (Angle to target minus projected robot heading)
        yawTarget = angleToTarget.getDegrees() - projectedPose.getRotation().getDegrees();

        // 6b. Adjust Yaw Target for tangential velocity
        double yawAdjustment = MathUtil.clamp(-targetTangentialVelo / exitVeloMetersPerSec / Math.cos(pitchTarget * Math.PI / 180) * 180 / Math.PI, -LauncherConstants.maxYawAdjustment, LauncherConstants.maxYawAdjustment);
        yawTarget += yawAdjustment;

        // 7. Apply Safeties and Send to Motors
        nearTrench = MathUtil.isNear(4.625594, currentPose.getX(), 0.61) || MathUtil.isNear(11.915394, currentPose.getX(), 0.61); //TODO: CHECK
        
        // 8. Compensate for RPM drop when the ball is launched (this is not used)
        adjustedRPM = kinematics.getAdjustedFlywheelRPM(targetRPM);


        launchMotor.setControl(new MotionMagicVelocityDutyCycle(adjustedRPM / 60.0));
        setYaw(yawTarget);
        
        double currentRPM = launchMotor.getVelocity().getValueAsDouble() * 60.0;
        shooterSpeedReady = targetRPM > 0 && Math.abs(currentRPM - targetRPM) < (targetRPM * LauncherConstants.kRPMTolerance);

        if (nearTrench) {
            setPitch(LauncherConstants.pitchBounds.getSecond());
            // if (mode == Mode.FIRE) setMode(Mode.STANDBY); 
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
            if (!dynamicYaw) {
                System.out.println("from simpletoggle");
                setYaw(yawDegrees);
            }
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
            setPitch(LauncherConstants.pitchBounds.getSecond());
            setFeeder(false); // Make sure feeder is off
            shooterSpeedReady = false;
        }
        else {
            if (mode == Mode.OFF){
                prepToShoot();
            }
            else {
                shooterSpeedReady = false;
            }
            // setFeeder(newMode == Mode.FIRE);
        }
        mode = newMode;
    }

    public boolean isAimingOrShooting() {
        return mode == Mode.STANDBY || mode == Mode.FIRE;
    }

    public void forcePitchDown() {
        if (forcingDown) return;
        forcingDown = true;
        pitchMotor.setControl(new DutyCycleOut(-0.1));
    }

    @Override
    public void periodic() {
        setAlliance();

        // Run feeder spinner at slow rate when intake is spinning (unless we're firing)
        if (mode != Mode.OFF) { 
            prepToShoot();
        }

        if (mode == Mode.FIRE && !toggledOn) {
            // Only push the ball in if the flywheel is at speed and we are out of trench
            setFeeder(shooterSpeedReady && !nearTrench); 
        } else if (mode != Mode.FIRE && !toggledOn) {
            // Standard intake pass-through logic for STANDBY/OFF
            boolean intakeSpinning = Intake.getInstance().isSpinning();
            feeder_spinner.setControl(intakeSpinning ? feederSpinnerWithIntakeRequest : brake);
            feeder_roller.setControl(brake); // Keep roller off unless firing
        }

        pitchTorque = pitchMotor.getTorqueCurrent().getValueAsDouble();
        if (pitchTorque < pitchMinTorque) pitchMinTorque = pitchTorque;
        if (pitchTorque > pitchMaxTorque) pitchMaxTorque = pitchTorque;

        double yawTorque = yawMotor.getTorqueCurrent().getValueAsDouble();
        if (yawTorque < yawMinTorque) yawMinTorque = yawTorque;
        if (yawTorque > yawMaxTorque) yawMaxTorque = yawTorque;

        if (
            forcingDown &&
            (pitchTorque < LauncherConstants.pitchForceTorque) &&
            (pitchMotor.getVelocity().getValueAsDouble() > LauncherConstants.pitchForceVelocityLimit)
        ) {
            System.out.println("Pitch bottom limit hit, marking min pitch");
            pitchMotor.setPosition(LauncherConstants.pitchLimitRotations);
            forcingDown = false;
            System.out.println("Setting from force down");
            setPitch(LauncherConstants.pitchBounds.getSecond());
        }

        if (toggledOn && dynamicYaw)
            setYaw(-Drive.Drivetrain.getInstance().getState().Pose.getRotation().getDegrees() + (blueAlliance ? 0 : 180));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Launcher");
        builder.setActuator(true);

        builder.addStringProperty("Mode", () -> mode.toString(), null);
        builder.addDoubleProperty("Seconds left until hopper color switches:", () -> Robot.getCountDown(), null);
        builder.addBooleanProperty("hurling", () -> hurling, null);
        builder.addBooleanProperty("Forcing Down", () -> forcingDown, null);
        builder.addBooleanProperty("shooterSpeedReady", () -> shooterSpeedReady, null);
        builder.addDoubleProperty("shootTargetX", () -> shootTarget.getX(), null);
        builder.addDoubleProperty("shootTargetY", () -> shootTarget.getY(), null);
        builder.addDoubleProperty("hurlTargetX", () -> hurlTargetXZ.getX(), null);

        // New Kinematics Telemetry
        builder.addDoubleProperty("Target Distance (m)", () -> targetDist, null);
        builder.addDoubleProperty("Target Radial Velo (m/s)", () -> targetRadialVelo, null);
        builder.addDoubleProperty("Target Tangential Velo (m/s)", () -> targetTangentialVelo, null);
        builder.addDoubleProperty("Polynomial Target RPM", () -> targetRPM, null);
        builder.addDoubleProperty("Polynomial Target Pitch", () -> pitchTarget, null);
        builder.addDoubleProperty("Yaw Target", () -> yawTarget, null);

        builder.addDoubleProperty("Launch Speed (RPM)", () -> launchMotor.getVelocity().getValueAsDouble() * 60.0, null);
        builder.addDoubleProperty("Shooter Voltage", () -> launchMotor.getDutyCycle().getValueAsDouble(), null);

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
        builder.addDoubleProperty("Pitch Torque", () -> pitchTorque, null);

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
            "Yaw P",
            () -> yawP,
            (up) -> {yawP = Trimmer.increment(yawP, 0.001, 0.2, up); setYawMotorConfig();}
        );
        trimmer.add(
            "Launcher PID",
            "Yaw I",
            () -> yawI,
            (up) -> {yawI = Trimmer.increment(yawI, 0.001, 0.2, up); setYawMotorConfig();}
        );
        trimmer.add(
            "Launcher PID",
            "Yaw D",
            () -> yawD,
            (up) -> {yawD = Trimmer.increment(yawD, 0.001, 0.2, up); setYawMotorConfig();}
        );
    }
}
