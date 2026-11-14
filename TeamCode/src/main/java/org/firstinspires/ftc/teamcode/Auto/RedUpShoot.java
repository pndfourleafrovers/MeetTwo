package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.subsystems.Constants.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;
//import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemRookieBot;
import org.firstinspires.ftc.teamcode.subsystems.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;

@Autonomous(name="Up Shoot", group="Red")

public class RedUpShoot extends OpMode {

   // private LauncherSubsystemRookieBot launcherSubsystemRookieBot;


    private PinpointDrive autoDrive = null;
    private GoBildaPinpointDriver pinpoint = null;

    int shotsToFire = 3;
    private ElapsedTime launchTimer = new ElapsedTime();
    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime waitTimer = new ElapsedTime();
    private ElapsedTime stuckTimer = new ElapsedTime();

    public double lastX = 0;
    public double lastY = 0;
    public double lastH = 0;

    private LauncherSubsystemRookieBot launcherSubsystem;

    private enum AutoState {
        POWER_UP,
        SHOOT_ALIGN_UP,
        SHOOT_ALIGN_DOWN,
        SHOOT,
        SHOT_COUNT,
        BACKUP,
        STRAFE,
        GARDEN,
        MIDPOINT,
        PARK,
        WAIT,
    }

    private AutoState autoState;
    private AutoState returnState;

    private AutoState shootState;
    //private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private void checkStuck(AutoState nextState){
        if(stuckTimer.seconds() > 0.5){
            double x = pinpoint.getPosX(DistanceUnit.MM);
            double y = pinpoint.getPosY(DistanceUnit.MM);
            double h = pinpoint.getHeading(AngleUnit.DEGREES);
            if (Math.abs(x - lastX) < 5 && Math.abs(y - lastY) < 5 && Math.abs(h - lastH) < 2){
                autoState = nextState;
            }
            lastX = x;
            lastY = y;
            lastH = h;
            stuckTimer.reset();
        }
    }

    @Override
    public void init(){
        autoDrive = new PinpointDrive(hardwareMap);
        //panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getDEFAULT_FTC());

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
        pinpoint.setOffsets(X_POD_OFFSET,Y_POD_OFFSET,DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.recalibrateIMU();

        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, RED_UP_START_X, RED_UP_START_Y, AngleUnit.DEGREES, RED_UP_START_H));
        pinpoint.update();

        autoState = AutoState.POWER_UP;
        returnState = AutoState.BACKUP;
        shootState = AutoState.BACKUP;

        launcherSubsystem = new LauncherSubsystemRookieBot(hardwareMap);

        launcherSubsystem.liftDownLeft();
        launcherSubsystem.liftDownRight();
    }

    @Override
    public void init_loop(){
        //launcherSubsystem.setLiftPosition(0.5);
        pinpoint.update();

        Pose2D pos = pinpoint.getPosition();
        //  double goalHeading = autoDrive.getHeadingToGoal(pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), RED_GOAL_X, RED_GOAL_Y);
        telemetry.addData("Status", "Initialized - PUSH TO TEST");
        telemetry.addData("X (mm) Front-Back", "%.1f", pos.getX(DistanceUnit.MM));
        telemetry.addData("Y (mm) Left-Right", "%.1f", pos.getY(DistanceUnit.MM));
        telemetry.addData("H (deg)", "%.1f", pos.getHeading(AngleUnit.DEGREES));
        // telemetry.addData("Goal Heading", "%.1f", goalHeading);
        telemetry.update();
    }

    @Override
    public void start(){
        //launcherSubsystem.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
        waitTimer.reset();
        stuckTimer.reset();
    }



    @Override
    public void loop(){
        pinpoint.update();
        double rawX = pinpoint.getPosX(DistanceUnit.MM);
        double rawY = pinpoint.getPosY(DistanceUnit.MM);
        // Scale X and Y to compensate for encoder resolution mismatches
        double curX = pinpoint.getPosX(DistanceUnit.MM);
        double curY = pinpoint.getPosY(DistanceUnit.MM);
        double curH = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);


        //-------------------------------SHOOTING------------------------
        if (autoState == AutoState.SHOOT_ALIGN_UP) {
            // double targetHeading = autoDrive.getHeadingToGoal(curX, curY, RED_GOAL_X, RED_GOAL_Y);
            //telemetry.addData("DEBUG Target Heading", "%.1f", targetHeading);
            //telemetry.addData("DEBUG Current H", "%.1f", curH);
        }

        switch (autoState) {
            case POWER_UP:
                launcherSubsystem.setLauncherVelocity(LAUNCHER_TARGET_VELOCITY);
               shotsToFire = 2;
               launchTimer.reset();
              autoState = AutoState.WAIT;
                break;

            case SHOOT_ALIGN_UP:
                if (autoDrive.runToPosition(curX,curY,curH,RED_UP_SHOOT_X,RED_UP_SHOOT_Y,RED_UP_SHOOT_H, driveSpeedSlow, rotateSpeed)){
                   // launcherSubsystem.setLauncherVelocity(LAUNCHER_TARGET_VELOCITY);
                    //launcherSubsystem.setIntakeVelocity(INTAKE_TARGET_VELOCITY);\
                    returnState = AutoState.BACKUP;
                    autoState = AutoState.SHOOT;
                }
                checkStuck(AutoState.SHOOT);
                break;

            case SHOOT_ALIGN_DOWN:
                if (autoDrive.runToPosition(curX,curY,curH,RED_DOWN_SHOOT_X,RED_DOWN_SHOOT_Y,RED_DOWN_SHOOT_H, driveSpeedSlow, rotateSpeed)){
                    // launcherSubsystem.setLauncherVelocity(LAUNCHER_TARGET_VELOCITY);
                    //launcherSubsystem.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
                    returnState = AutoState.PARK;
                    autoState = AutoState.SHOOT;
                }
                checkStuck(AutoState.SHOOT);
                break;

            case SHOOT:
                if(launcherSubsystem.isLauncherReady() && launchTimer.seconds() > TIME_BETWEEN_SHOTS){
                    feederTimer.reset();
                    launcherSubsystem.liftUpLeft();
                    launcherSubsystem.liftUpRight();
                    autoState = AutoState.SHOT_COUNT;
                }
                break;

            case SHOT_COUNT:
                if(feederTimer.seconds() > LIFT_TIME){
                    launcherSubsystem.liftDownLeft();
                    launcherSubsystem.liftDownRight();
                    shotsToFire--;

                    if (shotsToFire > 0) {
                        launchTimer.reset();
                        autoState = AutoState.SHOOT;
                    } else {
                        autoDrive.stop();
                        launcherSubsystem.setIntakeVelocity(0);
                        autoState = returnState;
                    }
                }
                break;

            //----------------------------BACKUP------------------
            case BACKUP:
                if (autoDrive.runToPosition(curX, curY, curH, RED_UP_BACKUP_X, RED_UP_BACKUP_Y, RED_UP_BACKUP_H, driveSpeedSlow, rotateSpeed)) {
                    autoState = AutoState.STRAFE;
                }
                break;

            //----------------------------STRAFE------------------
            case STRAFE:
                if (autoDrive.runToPosition(curX, curY, curH, RED_UP_STRAFE_X, RED_UP_STRAFE_Y, RED_UP_STRAFE_H, driveSpeedSlow, rotateSpeed)) {
                    launcherSubsystem.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
                    autoState = AutoState.GARDEN;
                }
                break;

            //----------------------------GARDEN------------------
            case GARDEN:
                if (autoDrive.runToPosition(curX, curY, curH, RED_UP_GARDEN_X, RED_UP_GARDEN_Y, RED_UP_GARDEN_H, driveSpeedSlow, rotateSpeed)) {
                    autoState = AutoState.MIDPOINT;
                }
                break;

            //----------------------------MIDPOINT------------------

            case MIDPOINT:
                if (autoDrive.runToPosition(curX, curY, curH, RED_UP_MIDPOINT_X, RED_UP_MIDPOINT_Y, RED_UP_MIDPOINT_H, driveSpeedSlow, rotateSpeed)) {
                    autoState = AutoState.SHOOT_ALIGN_DOWN;
                }
                break;

            //---------------------------------PAUSE---------------------
            case WAIT:
                if(waitTimer.seconds()> waitTime){
                    autoState = returnState;
                }
                break;

            //------------------------------END-------------------------
            case PARK:
                if (autoDrive.runToPosition(curX, curY, curH, RED_UP_PARK_X, RED_UP_PARK_Y, RED_UP_PARK_H, driveSpeedSlow, rotateSpeed)) {
                    autoDrive.stop();
                    PoseStorage.currentX = curX;
                    PoseStorage.currentY = curY;
                    PoseStorage.currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
                }
                break;
        }

        // Debug: show errors and commands
        double xErr = RED_UP_MIDPOINT_X - curX;
        double yErr = RED_UP_MIDPOINT_Y - curY;
        double hErr = RED_UP_MIDPOINT_H - Math.toDegrees(curH);
        while (hErr > 180) hErr -= 360;
        while (hErr <= -180) hErr += 360;

        Pose2D pos = pinpoint.getPosition();
        telemetry.addData("AutoState", autoState);
        telemetry.addData("X", "%.1f → %.1f (err: %.1f)", curX, RED_UP_MIDPOINT_X, xErr);
        telemetry.addData("Y (scaled)", "%.1f → %.1f (err: %.1f)", curY, RED_UP_MIDPOINT_Y, yErr);
        telemetry.addData("X (scaled)", "%.1f → %.1f (err: %.1f)", curX, RED_UP_MIDPOINT_X, xErr);

        telemetry.addData("Y (raw)", "%.1f", rawY);
        telemetry.addData("H", "%.1f → %.1f (err: %.1f)", Math.toDegrees(curH), RED_UP_MIDPOINT_H, hErr);
        telemetry.update();
    }


    @Override
    public void stop(){
    }
}

