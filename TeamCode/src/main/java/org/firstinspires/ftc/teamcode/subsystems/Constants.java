package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {

    public static double aimKP = 0.03;
    public static double aimDeadband = 1;
    public static double aimMinPower = 0.12;

    public static double X_POD_OFFSET = 1.25; //inches
    public static double Y_POD_OFFSET = -7; //inches

    public static double waitTime = 1.0;

 /*   public double kP = 0.0001;
    public double kD = 0.0;
    public double goalX = 0;
    public double lastError = 0;
    public double angleTolerance = 0.2;
    public final double MAX_POWER = 0.8;
    public double power = 0;
    public final ElapsedTime timer = new ElapsedTime();
    */
public static double turretAimKP = 0.00065;
    public static final double TurretMax = 1.0;
    public static final double TurretMid = 0.5;
    public static final double TurretMin = 0.0;

    public static double LIFT_TIME = 0.13;
    public static double TIME_BETWEEN_SHOTS = 0.5; // was 0.2, 0.25, 0.3

    public static double LAUNCHER_TARGET_VELOCITY = 1700;
    public static double LAUNCHER_MIN_VELOCITY = 1680;

    public static double FAR_LAUNCH_POWER = 1800;
    public static double FAR_MIN_VELOCITY = 1789; //MAKE THIS 20rpm LESS THAN ABOVE
    public static double CLOSE_LAUNCH_POWER = 1370;
    public static double CLOSE_MIN_VELOCITY = 1350;

    public static double MID_LAUNCH_POWER = 1750;
    public static double MID_MIN_VELOCITY = 1730; //MAKE THIS 20rpm LESS THAN ABOVE

    public static double TELE_CLOSE_LAUNCH = 1700;
    public static double TELE_CLOSE_MIN_LAUNCH = 1680;// MAKE THIS 20rpm LESS THAN ABOVE

    public static double MAX_LAUNCH_POWER = 2000;

    public static double autoLaunchMinDiff = 20;
    public static double nearestBase = 3.0;
    public static double farthestBase = 9.75;

    public static double LaunchPower = 1640;
    public static double TICKS_PER_REV = 28;

    public static double INTAKE_TARGET_VELOCITY = 2000;

    public static double OUTTAKE_TARGET_VELOCITY = -2000;
    public static double INTAKE_BURST_SHOT = 2800;

    public static double driveSpeed = 0.9;
    public static double driveSpeedSlow = 0.34;
    public static double rotateSpeed = 0.4;
    public static double odoSlowSpeedMultiple = 0.47;

    public static double Kp = 0.03;
    public static double TOLERANCE_MM = 15.0;
    public static double MIN_POWER = 0.12;
    public static double TOLERANCE_DEGREES = 2.0;

    public static double DRIVE_P = 0.00071; //  applies power based on distance - increase until bot moves past target
    public static double DRIVE_D = 0.00008; // applies brakes to slow down approach - helps prevent wobbling - increase if overshoots, decrease if vibrates

    public static double STRAFE_P = 0.0015; //
    public static double STRAFE_D = 0.0001; //

    public static double TURN_P = 0.00054; //applies power based on distance
    public static double TURN_D = 0.00023;

    public static double redBearingOffset = 0.0;
    public static double blueBearingOffset = -2.5;

    public static double launchP = 140;
    public static double launchI = 0;
    public static double launchD = 0;
    public static double launchF = 15;

    public static double liftUpRight = 0.17;
    public static double liftDownRight = 0.0;

    public static double liftUpLeft = 0.0;
    public static double liftDownLeft = 0.15;

    //Blue Location
    public static double BLUE_GOAL_X = 4646;
    public static double BLUE_GOAL_Y = 1223;

    //Far
    public static double BLUE_FAR_START_X = 1624;
    public static double BLUE_FAR_START_Y = 439;
    public static double BLUE_FAR_START_H = 0.0;

    public static double BLUE_UP_SHOOT_X = 1763;
    public static double BLUE_UP_SHOOT_Y = 367;
    public static double BLUE_UP_SHOOT_H = 22;

    public static double BLUE_FAR_FINISH_X = 1998;
    public static double BLUE_FAR_FINISH_Y = 709;
    public static double BLUE_FAR_FINISH_H = 0;

    //Near
    public static double BLUE_NEAR_START_X = 4807;
    public static double BLUE_NEAR_START_Y = 1026;
    public static double BLUE_NEAR_START_H = 5.0;

    public static double BLUE_NEAR_SHOOT_X = 3720;
    public static double BLUE_NEAR_SHOOT_Y = 590;
    public static double BLUE_NEAR_SHOOT_H = 45.0;

    public static double BLUE_NEAR_FINISH_X = 4297;
    public static double BLUE_NEAR_FINISH_Y = 463;
    public static double BLUE_NEAR_FINISH_H = 50;


    //Red Locations

    // UP
    public static double RED_UP_START_X = 0;
    public static double RED_UP_START_Y = 400;
    public static double RED_UP_START_H = 180;

    public static double RED_UP_SHOOT_X = 0;
    public static double RED_UP_SHOOT_Y = -403;
    public static double RED_UP_SHOOT_H = 180;

    public static double RED_UP_MIDPOINT_X = 1543;
    public static double RED_UP_MIDPOINT_Y = 1180;
    public static double RED_UP_MIDPOINT_H = 180;

    public static double RED_UP_BACKUP_X = 567;
    public static double RED_UP_BACKUP_Y = 388;
    public static double RED_UP_BACKUP_H = 180;

    public static double RED_UP_STRAFE_X = 557;
    public static double RED_UP_STRAFE_Y = 1577;
    public static double RED_UP_STRAFE_H = 180;

    public static double RED_UP_GARDEN_X = 13;
    public static double RED_UP_GARDEN_Y = 1588;
    public static double RED_UP_GARDEN_H = 180;

    public static double RED_UP_PARK_X = 2146;
    public static double RED_UP_PARK_Y = 1443;
    public static double RED_UP_PARK_H = 180;

    //DOWN
    public static double RED_DOWN_START_X = 3197;
    public static double RED_DOWN_START_Y = 1791;
    public static double RED_DOWN_START_H = 0;

    public static double RED_DOWN_SHOOT_X = 3071;
    public static double RED_DOWN_SHOOT_Y = 10;
    public static double RED_DOWN_SHOOT_H = 0;

    public static double RED_DOWN_PARK_X = 2871;
    public static double RED_DOWN_PARK_Y = 2300;
    public static double RED_DOWN_PARK_H = 0;

}
