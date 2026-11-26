package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.*;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.LAUNCHER_MIN_VELOCITY;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.launchD;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.launchF;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.launchI;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.launchP;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class LauncherSubsystemRookieBot {

    private DcMotorEx launcher;
    private DcMotorEx launchAsst;
    private DcMotorEx intake;
    private Servo FlowerLeft;
    private Servo FlowerRight;
    private Servo Hatch;


    public LauncherSubsystemRookieBot(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "Intake_motor");
        launcher = hardwareMap.get(DcMotorEx.class, "Launcher_motor");
        launchAsst = hardwareMap.get(DcMotorEx.class, "Transfer_motor");
        Hatch = hardwareMap.get(Servo.class, "Hatch_servo");
        FlowerLeft = hardwareMap.get(Servo.class, "Left_flower_servo");
        FlowerRight = hardwareMap.get(Servo.class, "Right_flower_servo");

        launcher.setDirection(DcMotorEx.Direction.FORWARD);
        launchAsst.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

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
       // launchAsst.setVelocity(velocity);
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

    public void hatchOpen() {
        Hatch.setPosition(hatchOpen);
        //launchAsst.setVelocity(TRANSFER_VELOCITY);
    }

    public void hatchClosed() {
        Hatch.setPosition(hatchClosed);
        //launchAsst.setVelocity(0);
    }

    public void setIntakeVelocity(double velocity) {
        intake.setVelocity(velocity);
        launchAsst.setVelocity(velocity);
    }

    public void setTransferVelocity(double velocity) {
        launchAsst.setVelocity(velocity);
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
