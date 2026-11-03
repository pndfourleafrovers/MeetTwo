/*
 * PinpointDrive.java
 * - Dedicated driver class for Holonomic Odometry movement.
 * - Used specifically for Autonomous paths.
 */

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PinpointDrive {

    private DcMotorEx FrontLeft, RearLeft, FrontRight, RearRight;

    //PID Variables
    private ElapsedTime pidTimer = new ElapsedTime();
    private double lastTime = 0;
    private double lastErrorX = 0;
    private double lastErrorY = 0;
    private double lastErrorH = 0;
    public static double COSX = 1;
    public static double COSY = -1;
    public static double SINX = 1;
    public static double SINY = 1;

    //PID Tuning
    public static double DRIVE_P = 0.00071; // 0.0015, 0.0017, 0.0020, 0.0015, 0.0017, 0.0018, 0.001755 applies power based on distance - increase until bot moves past target
    public static double DRIVE_D = 0.00008; //was 0.002, 0.0001, 0.0002, 0.00009, 0.00005 applies brakes to slow down approach - helps prevent wobbling - increase if overshoots, decrease if vibrates

    public static double STRAFE_P = 0.0012; //
    public static double STRAFE_D = 0.0001; //

    public static double TURN_P = 0.00054; //0.02 applies power based on distance
    public static double TURN_D = 0.00023; //0.01 applies brakes to slow down approach - helps prevent wobbling



    //Odometry Tuning
    //increases over changes
    //private final double DRIVE_GAIN = 0.02; //may need tuned was 0.03, 0.02 - increases speed over time when going to a point
    //private final double TURN_GAIN  = 0.01; //may need tuned for rotations and strafing was 0.02 - increases speed over time when rotating

    private final double TOLERANCE_MM = 15.0;
    private final double TOLERANCE_DEGREES = 2.0;

    private final double MIN_POWER = 0.15; // keeps bot moving slowly when near target - should allow for more accurate movements. Increase if short, decrease if movement isn't fluid


    public PinpointDrive(HardwareMap hardwareMap) {
        //declares motors
        FrontLeft = hardwareMap.get(DcMotorEx.class, "Left_front");
        RearLeft = hardwareMap.get(DcMotorEx.class, "Left_rear");
        FrontRight = hardwareMap.get(DcMotorEx.class, "Right_front");
        RearRight = hardwareMap.get(DcMotorEx.class, "Right_rear");



        // Directions: Left = Forward, Right = Reverse
        // based on bot 2
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        RearLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        RearRight.setDirection(DcMotor.Direction.REVERSE);

        //ensures motors turn off when powered off
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // *** FIX: Ensure motors are in a RUN mode ***
        DcMotor[] driveMotors = {FrontLeft, RearLeft, FrontRight, RearRight};
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Motors will NOT move without this!
        }
    }

    /**
     * Drives to a specific field position (X, Y, Heading)
     * used for driving motion in auto code
     */
    public boolean runToPosition(double currentX, double currentY, double currentHeadingRad,
                                 double targetX, double targetY, double targetHeadingDeg,
                                 double maxDriveSpeed, double maxTurnSpeed) {

        double xError = targetX - currentX;
        double yError = targetY - currentY;

        double currentHeadingDeg = Math.toDegrees(currentHeadingRad);
        double headingError = targetHeadingDeg - currentHeadingDeg;
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        if (Math.abs(xError) < TOLERANCE_MM &&
                Math.abs(yError) < TOLERANCE_MM &&
                Math.abs(headingError) < TOLERANCE_DEGREES) {
            stop();
            return true;
        }

        // PD timing
        double currentTime = pidTimer.seconds();
        double dt = currentTime - lastTime;
        if (dt < 0.005) dt = 0.005; // Prevent divide by zero
        lastTime = currentTime;

        double sin = Math.sin(currentHeadingRad);
        double cos = Math.cos(currentHeadingRad);

        // Proportional: field-to-robot coordinate transformation
        double pForward = COSX*(xError * cos) + SINY*(yError * sin);
        double pStrafe  = SINX*(xError * sin) + COSY*(yError * cos);
        double pTurn    = headingError;

        // Derivative: rate of change of error
        double dX = (xError - lastErrorX) / dt;
        double dY = (yError - lastErrorY) / dt;
        double dH = (headingError - lastErrorH) / dt;
        // Normalize heading derivative to avoid jumps at ±180
//        while (dH > 180 / dt) dH -= 360 / dt;
//        while (dH < -180 / dt) dH += 360 / dt;

        // Transform derivative to robot-centric
        double dForward = COSX*(dX * cos) + SINY*(dY * sin);
        double dStrafe  = SINX*(dX * sin) + COSY*(dY * cos);
        double dTurn    = dH;

        // Update last errors
        lastErrorX = xError;
        lastErrorY = yError;
        lastErrorH = headingError;

        // PD output
        double forward = (pForward * DRIVE_P) + (dForward * DRIVE_D);
        double strafe  = (pStrafe * STRAFE_P) + (dStrafe * STRAFE_D);
        double turn    = (pTurn * TURN_P) + (dTurn * TURN_D);

        // Apply minimum power (friction feedforward)
        if (Math.abs(forward) < MIN_POWER && Math.abs(forward) > 0.01)
            forward = Math.signum(forward) * MIN_POWER;
        if (Math.abs(strafe) < MIN_POWER && Math.abs(strafe) > 0.01)
            strafe = Math.signum(strafe) * MIN_POWER;
        if (Math.abs(turn) < MIN_POWER && Math.abs(turn) > 0.01)
            turn = Math.signum(turn) * MIN_POWER;

        // Clamp to max speeds
        forward = Math.max(-maxDriveSpeed, Math.min(maxDriveSpeed, forward));
        strafe  = Math.max(-maxDriveSpeed, Math.min(maxDriveSpeed, strafe));
        turn    = Math.max(-maxTurnSpeed,  Math.min(maxTurnSpeed,  turn));

        // Invert forward to match motor configuration (FIX from our debugging)
        //forward = -forward;

        // Mecanum mixing
        double lf = forward + strafe - turn;
        double rf = forward - strafe + turn;
        double lr = forward - strafe - turn;
        double rr = forward + strafe + turn;

        double max = Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lr), Math.abs(rr))));
        if (max > 1.0) {
            lf /= max; rf /= max; lr /= max; rr /= max;
        }

        FrontLeft.setPower(lf);
        RearLeft.setPower(lr);
        FrontRight.setPower(rf);
        RearRight.setPower(rr);

        return false;
    }
    //Get heading to goal
    public double getHeadingToGoal(double currentX, double currentY, double targetX, double targetY) {
        double dy = targetY - currentY; //difference of target and current y
        double dx = targetX - currentX; // difference of target and current x
        double headingToGoal = Math.atan2(dy, dx); // calculates angle into radians
        return Math.toDegrees(headingToGoal); //converts radians to degrees
    }


    public void stop() {
        FrontLeft.setPower(0);
        RearLeft.setPower(0);
        FrontRight.setPower(0);
        RearRight.setPower(0);
    }
}