package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.Ports;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

import static edu.wpi.first.units.Units.MetersPerSecond;

public final class ControlBoard {
    private static ControlBoard mInstance = null;
    public final CustomXboxController operator;
    private final GenericHID driver;
    private final double speedFactor;
    private final double kSwerveDeadband;

    public static ControlBoard getInstance() {
        if (mInstance == null) mInstance = new ControlBoard();
        return mInstance;
    }

    private ControlBoard() {
        driver = new GenericHID(Ports.DRIVER_CONTROL);
        operator = new CustomXboxController(Ports.OPERATOR_CONTROL);
        speedFactor = ControllerConstants.kInputClipping;
        kSwerveDeadband = ControllerConstants.stickDeadband;
    }

    public Translation2d getSwerveTranslation() {
        double forwardAxis = 0;
        double strafeAxis = 0;
        // if (ControllerConstants.isMambo) {
        //     forwardAxis = driver.getRawAxis(2);
        //     strafeAxis = driver.getRawAxis(1);
        //     double mag = Math.pow(forwardAxis * forwardAxis + strafeAxis * strafeAxis, 0.5);
        //     double curveFactor = Math.pow(mag, 0.25);
        //     forwardAxis = forwardAxis * curveFactor;
        //     strafeAxis = strafeAxis * curveFactor;
        // } else {
        //     forwardAxis = getRightThrottle();
        //     strafeAxis = getRightYaw();
        // }

        // FIXME; TEMPORARY: USING OPERATOR CONTROLS FOR DRIVING and thus commenting out above
        forwardAxis = operator.getAxis(CustomXboxController.Side.RIGHT, CustomXboxController.Axis.Y);
        strafeAxis = operator.getAxis(CustomXboxController.Side.RIGHT, CustomXboxController.Axis.X);



        forwardAxis *= speedFactor;
        strafeAxis *= speedFactor;

        SmartDashboard.putNumber("Raw Y", forwardAxis);
        SmartDashboard.putNumber("Raw X", strafeAxis);

        forwardAxis = ControllerConstants.invertYAxis ? forwardAxis : -forwardAxis;
        strafeAxis = ControllerConstants.invertXAxis ? strafeAxis : -strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.getNorm()) < kSwerveDeadband) return new Translation2d(); else {
            Rotation2d deadband_direction = new Rotation2d(tAxes.getX(), tAxes.getY());
            Translation2d deadband_vector = new Translation2d(kSwerveDeadband, deadband_direction);

            double scaled_x = MathUtil.applyDeadband(forwardAxis, Math.abs(deadband_vector.getX()));
            double scaled_y = MathUtil.applyDeadband(strafeAxis, Math.abs(deadband_vector.getY()));
            return new Translation2d(scaled_x, scaled_y).times(Constants.SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond) / 7);
        }
    }

    public double getSwerveRotation() {
        /* FIXME; TEMPORARY COMMENTING OUT BELOW */
        // double rotAxis = ControllerConstants.isMambo ? driver.getRawAxis(3) : getLeftYaw();
        // rotAxis = ControllerConstants.invertRAxis ? rotAxis : -rotAxis;
        // rotAxis *= speedFactor;

        // if (Math.abs(rotAxis) < kSwerveDeadband) return 0.;
        // return Constants.SwerveConstants.MaxAngularRate *
        //     (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband)) / (1 - kSwerveDeadband);


        return operator.getAxis(CustomXboxController.Side.LEFT, CustomXboxController.Axis.X) *
            Constants.SwerveConstants.MaxAngularRate;
    }

    /** Non-mambo controller. Returns positions from -1 to 1 */
    private double getLeftYaw() {
        double leftYaw = driver.getRawAxis(ControllerConstants.leftXAxis);

        if (leftYaw != 0) leftYaw -= ControllerConstants.LeftYawZero;

        if (leftYaw > kSwerveDeadband) leftYaw /= (ControllerConstants.LeftYawHigh +
            (ControllerConstants.isC1 ? -ControllerConstants.LeftYawZero : ControllerConstants.LeftYawZero));
        else if (leftYaw < -kSwerveDeadband) leftYaw /= (ControllerConstants.LeftYawLow +
            ControllerConstants.LeftYawZero);
        return MathUtil.clamp(leftYaw, -1, 1);
    }

    /** Non-mambo controller. Returns positions from -1 to 1 */
    private double getRightThrottle() {
        double rightThrottle = driver.getRawAxis(ControllerConstants.rightYAxis);

        if (rightThrottle != 0) rightThrottle = rightThrottle - ControllerConstants.RightThrottleZero;

        if (rightThrottle > (ControllerConstants.isC1 ? kSwerveDeadband : 0.102))
            rightThrottle /= (ControllerConstants.RightThrottleHigh + (ControllerConstants.isC1 ?
                -ControllerConstants.RightThrottleZero : ControllerConstants.RightThrottleZero));
        else if (rightThrottle < -kSwerveDeadband) rightThrottle /= (ControllerConstants.RightThrottleLow
            + ControllerConstants.RightThrottleZero);
        return MathUtil.clamp(rightThrottle, -1, 1);
    }

    /** Non-mambo controller. Returns positions from -1 to 1 */
    private double getRightYaw() {
        double rightYaw = driver.getRawAxis(ControllerConstants.rightXAxis);

        if (rightYaw != 0) rightYaw -= ControllerConstants.RightYawZero;

        if (rightYaw > kSwerveDeadband) rightYaw /= (ControllerConstants.RightYawHigh -
            ControllerConstants.RightYawZero);
        else if (rightYaw < -kSwerveDeadband) rightYaw /= (ControllerConstants.RightYawLow +
            ControllerConstants.RightYawZero);
        return MathUtil.clamp(rightYaw, -1, 1);
    }

    public static final class CustomXboxController {
        private final XboxController mController;
        public enum Side { LEFT, RIGHT }
        public enum Axis { X, Y }
        public enum Button {
            A(1), B(2), X(3), Y(4), LB(5), RB(6), BACK(7), START(8), L_JOYSTICK(9), R_JOYSTICK(10);
            public final int id;
            Button(int id) { this.id = id; }
        }

        public CustomXboxController(int port) {
            mController = new XboxController(port);
        }

        public double getAxis(Side side, Axis axis) {
            boolean left = side == Side.LEFT;
            boolean y = axis == Axis.Y;
            return mController.getRawAxis((left ? 0 : 4) + (y ? 1 : 0));
        }

        public boolean getTrigger(Side side) {
            return mController.getRawAxis(side == Side.LEFT ? 2 : 3) > ControllerConstants.kTriggerThreshold;
        }

        public boolean getButton(Button button) {
            return mController.getRawButton(button.id);
        }

        public XboxController getController() {
            return mController;
        }
    }
}
