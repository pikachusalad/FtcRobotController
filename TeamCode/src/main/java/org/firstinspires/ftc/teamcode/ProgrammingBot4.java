package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

// This is not an OpMode.  It is a class that holds all the boring stuff

public class ProgrammingBot4 {

    // imu
    private BNO055IMU imu;

    // Motors
    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx BL;
    private DcMotorEx BR;

    private List<DcMotorEx> LeftDriveMotors;
    private List<DcMotorEx> RightDriveMotors;
    private List<DcMotorEx> AllDriveMotors;

    // Encoder CONSTANTS
    private static final double mmPerInch               = 25.4f;    // this is jus math tho
    private static final double maxRPM                  = 340;      // Neverest orbital 20
    private static final double countsPerRevolution     = 537.6f;   // Neverest orbital 20
    private static final double wheelDiameterMM         = 96;       // For figuring circumference
    private static final double WheelDiameterIn         = wheelDiameterMM / mmPerInch;
    private static final double wheelCircumferenceIn    = WheelDiameterIn * Math.PI;
    private static final double countsPerInch         = (countsPerRevolution / wheelCircumferenceIn);
    // 45.276398210782385119532053004212 ticks per inch

    // PID
    private PIDCoefficients driveCoefficients = new PIDCoefficients(0.003,0,0);

    // you will need a reference to your OpMode
    private LinearOpMode OpModeReference;
    public ProgrammingBot4(LinearOpMode opMode) {
        OpModeReference = opMode;
    }

    public void initialize() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//		parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//		parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode

        imu = OpModeReference.hardwareMap.get(BNO055IMU.class, "IMU");

        // wheels
        FL = OpModeReference.hardwareMap.get(DcMotorEx.class, "FL");
        FR = OpModeReference.hardwareMap.get(DcMotorEx.class, "FR");
        BL = OpModeReference.hardwareMap.get(DcMotorEx.class, "BL");
        BR = OpModeReference.hardwareMap.get(DcMotorEx.class, "BR");

        // motor arrays - quick instantiation as immutable lists
        // you only have to add your left and right motors here
        LeftDriveMotors = Arrays.asList(new DcMotorEx[]{FL,BL});
        RightDriveMotors = Arrays.asList(new DcMotorEx[]{FR,BR});

        // motors are added to the "all" list automatically
        List<DcMotorEx> tempAll = new ArrayList<DcMotorEx>();
        tempAll.addAll(LeftDriveMotors);
        tempAll.addAll(RightDriveMotors);
        AllDriveMotors = Collections.unmodifiableList(tempAll);

        // set up our motor defaults
        for (DcMotorEx l : LeftDriveMotors)
            l.setDirection(DcMotorEx.Direction.FORWARD);
        for (DcMotorEx r : RightDriveMotors)
            r.setDirection(DcMotorEx.Direction.REVERSE);

        for (DcMotorEx m : AllDriveMotors){
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        // initialize the IMU
        imu.initialize(parameters);
    }

    private void SetRightMotorsPower(double power) {
        for (DcMotorEx m : RightDriveMotors)
            m.setPower(power);
    }

    private void SetLeftMotorsPower(double power) {
        for (DcMotorEx m : LeftDriveMotors)
            m.setPower(power);
    }

    private void SetAllMotorsPower(double power) {
        for (DcMotorEx m : AllDriveMotors)
            m.setPower(power);
    }

    // This method makes the robot turn.
    // DO NOT try to turn more than 180 degrees in either direction
    // targetAngleDifference is the number of degrees you want to turn
    // should be positive if turning left, negative if turning right
    private void turn(double targetAngleDifference, double power) {
        if (targetAngleDifference == 0)
            return;

        double rightPower = power;
        double leftPower = power;

        if (targetAngleDifference < 0) // target is NEGATIVE, turning RIGHT
            rightPower = -power;
        else if (targetAngleDifference > 0) // target is POSITIVE, turning LEFT
            leftPower = -power;

        // before starting the turn, take note of current angle as startAngle
        double startAngle = GetCurrentZAngle();

        SetRightMotorsPower(rightPower);
        SetLeftMotorsPower(leftPower);

        // for some reason (IMU?), this is necessary
        OpModeReference.sleep(100);

        // calculate essentially percent complete - starts at 0, goes to 1
        double turnCompletion = GetAngleDifference(startAngle) / targetAngleDifference;

        while (OpModeReference.opModeIsActive() && turnCompletion < 1) {

            turnCompletion = GetAngleDifference(startAngle) / targetAngleDifference;
            SetRightMotorsPower(rightPower * getTurnStepDownMultiplier(turnCompletion));
            SetLeftMotorsPower(leftPower * getTurnStepDownMultiplier(turnCompletion));

            OpModeReference.telemetry.addData("target", targetAngleDifference);
            OpModeReference.telemetry.addData("current", GetAngleDifference(startAngle));
            OpModeReference.telemetry.addData("LeftMotorPower", (FL.getPower() + BL.getPower())/2);
            OpModeReference.telemetry.addData("RightMotorPower", (FR.getPower() + BR.getPower())/2);
            OpModeReference.telemetry.update();
        }

        // turn all motors off
        stopDriving();
    }

    // turn completion goes from 0 to 1 - keep the highest at the top
    private double getTurnStepDownMultiplier(double turnCompletion) {

        double multiplier = 1;

        if (turnCompletion > 0.9)
            multiplier = 0.10;
        else if (turnCompletion > 0.75)
            multiplier = 0.25;
        else if (turnCompletion > 0.5)
            multiplier = 0.5;

        return multiplier;
    }

    // this is a method to get the current heading/z angle from the IMU
    // WE WANT THE Z ANGLE :)
    // AxesOrder.XYZ means we want thirdAngle
    // AxesOrder.ZYX would mean we want firstAngle
    private double GetCurrentZAngle() {
        Orientation currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return currentAngles.thirdAngle;
    }

    // This method calculates the difference of the current angle from the start angle
    // If you're left of your original angle, the value will be POSITIVE
    // If you're right of your original angle, the value will be NEGATIVE
    private double GetAngleDifference(double startAngle) {
        double angleDifference = GetCurrentZAngle() - startAngle;

        // handle going past the 0 or 180 barriers
        // where we switch from positive to negative or vice versa
        if (angleDifference < -180)
            angleDifference += 360;
        else if (angleDifference > 180)
            angleDifference -=360;

        return angleDifference;
    }

    //Autonomous Methods:

    public void stopDriving (){
        for (DcMotorEx m : AllDriveMotors)
            m.setPower(0);
    }

    public void DumDrive(double speed) {
        for (DcMotorEx m : AllDriveMotors)
            m.setPower(speed);
    }

    // public method to turn left
    // always pass positive value for angle (don't have to remember which is negative)
    public void turnLeft(double angle, double power) {
        turn(angle, power);
    }

    // public method to turn right
    // always pass positive value for angle (don't have to remember which is negative)
    public void turnRight(double angle, double power) {
        turn(-(angle), power);
    }

    public void drive(double inches, double speed) {

        // Ensure that the opmode is still active
        if (OpModeReference.opModeIsActive()) {

            // calculate the number of ticks you want to travel (cast to integer)
            //int targetTicks = (int) (2 * inches * countsPerInch);
            int targetTicks = (int) (inches * countsPerInch);

            // reset ticks to 0 on all motors
            for (DcMotorEx m : AllDriveMotors)
                m.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            // set target position on all motors
            // mode must be changed to RUN_TO_POSITION

            for(DcMotorEx m : AllDriveMotors) {
                m.setTargetPosition(targetTicks);
                m.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            }

            // turn all motors on!
            SetAllMotorsPower(speed);

            // Keep looping until one motor gets there (not busy)
            // stop if driver station stop button pushed
            // ONLY care if back wheels are busy - since they're the grippy ones
            while (OpModeReference.opModeIsActive() && BL.isBusy() && BR.isBusy()) {
                OpModeReference.telemetry.addData("target ticks", targetTicks);
                OpModeReference.telemetry.addData("FR current", FR.getCurrentPosition());
                OpModeReference.telemetry.addData("FL current", FL.getCurrentPosition());
                OpModeReference.telemetry.addData("BL current", BL.getCurrentPosition());
                OpModeReference.telemetry.addData("BR current", BR.getCurrentPosition());

                OpModeReference.telemetry.update();
            }

            // once all motors get to where they need to be, turn them off
            stopDriving();

            // set motors back to RUN_WITHOUT_ENCODERS
            for (DcMotorEx m : AllDriveMotors)
                m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // experimental / PID

    public void PidDrive(double inches) {
        int targetTicks = (int) (inches * countsPerInch);
        ElapsedTime timer = new ElapsedTime();

        // reset ticks to 0 on all motors
        for (DcMotorEx m : AllDriveMotors) {
            m.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        int error = targetTicks;
        int lastError = error;
        double D = 0;
        double I = 0;
        int iterations = 0;

        while (OpModeReference.opModeIsActive() && Math.abs(error) > 3) {
            error = targetTicks - BL.getCurrentPosition();
            D = (error - lastError) / timer.seconds();
            I += error * timer.seconds();
            lastError = error;
            timer.reset();
            Double out = (driveCoefficients.p * error) + (driveCoefficients.i * I) + (driveCoefficients.d * D);
            SetAllMotorsPower(Range.clip(out, -1, 1));

            OpModeReference.telemetry.addData("error", error);
            OpModeReference.telemetry.addData("D", D);
            OpModeReference.telemetry.addData("I", I);
            OpModeReference.telemetry.addData("out", out);
            OpModeReference.telemetry.addData("target position", targetTicks);
            OpModeReference.telemetry.addData("current position", BL.getCurrentPosition());
            OpModeReference.telemetry.update();
        }

    }
}
