package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.*;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.LAUNCHER_MIN_VELOCITY;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.launchD;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.launchF;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.launchI;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.launchP;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.liftDownLeft;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.liftDownRight;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.liftUpLeft;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.liftUpRight;
//import static org.firstinspires.ftc.teamcode.subsystems.Constants.liftDown;
//import static org.firstinspires.ftc.teamcode.subsystems.Constants.liftUp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class LauncherSubsystemRookieBot {

    private DcMotorEx launcher;
    private DcMotorEx launchAsst;
    private DcMotorEx intake;
    private Servo FlipperLeft;
    private Servo FlipperRight;

    public LauncherSubsystemRookieBot(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        FlipperLeft = hardwareMap.get(Servo.class, "FlipperLeft");
        FlipperRight = hardwareMap.get(Servo.class, "FlipperRight");
        launcher = hardwareMap.get(DcMotorEx.class, "LaunchLeft");
        launchAsst = hardwareMap.get(DcMotorEx.class, "LaunchRight");

        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launchAsst.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launchAsst.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launcher.setZeroPowerBehavior(BRAKE);
        launchAsst.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launchAsst.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(launchP, launchI, launchD, launchF));
        launchAsst.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(launchP, launchI, launchD, launchF));
    }

    public void setLauncherVelocity(double velocity) {
        launcher.setVelocity(velocity);
        launchAsst.setVelocity(velocity);
    }

    public double getLauncherVelocity() {
        return launcher.getVelocity();
    }
    public boolean isLauncherReady() {
        return launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY;
    }
    public boolean isCloseLauncherReady() {
        return launcher.getVelocity() >= CLOSE_MIN_VELOCITY;
    }
    public void setLiftPosition(double position) {
        FlipperLeft.setPosition(position);
    }
    public void liftUpLeft() {
        FlipperLeft.setPosition(liftUpLeft);
    }
    public void liftDownLeft() {
        FlipperLeft.setPosition(liftDownLeft);
    }

    public void liftUpRight() {
        FlipperRight.setPosition(liftUpRight);
    }
    public void liftDownRight() {
        FlipperRight.setPosition(liftDownRight);
    }

    public void setIntakeVelocity(double velocity) {
        intake.setVelocity(velocity);
    }


    public double getIntakeVelocity() {
        return intake.getVelocity();
    }

    public double getLaunchAsstVelocity() {
        return launchAsst.getVelocity();
    }

    public void stopAll() {
        launcher.setVelocity(0);
        launchAsst.setVelocity(0);
        intake.setVelocity(0);
    }
}
