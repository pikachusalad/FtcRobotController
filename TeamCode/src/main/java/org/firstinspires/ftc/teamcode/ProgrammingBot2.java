/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// test

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//IMU
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

// This is not an OpMode.  It is a class that holds all the boring stuff

public class ProgrammingBot2 {

    private List<DcMotorEx> LeftDriveMotors;
    private List<DcMotorEx> RightDriveMotors;
    private List<DcMotorEx> AllDriveMotors;
    private List<LynxModule> allHubs;

    BNO055IMU imu;

    // Motors
    private DcMotorEx FL = null;
    private DcMotorEx FR = null;
    private DcMotorEx BL = null;
    private DcMotorEx BR = null;

    // Encoder values
    private static final double mmPerInch               = 25.4f;    // this is jus math tho
    private static final double maxRPM                  = 340;      // Neverest orbital 20
    private static final double countsPerRevolution     = 537.6f;   // Neverest orbital 20
    private static final double wheelDiameterMM         = 96;       // For figuring circumference
    private static final double WheelDiameterIn         = wheelDiameterMM / mmPerInch;
    private static final double wheelCircumferenceIn    = WheelDiameterIn * Math.PI;
    private static final double countsPerInch         = (countsPerRevolution / wheelCircumferenceIn);


    // you will need a reference to your OpMode
    private LinearOpMode OpModeReference;
    public ProgrammingBot2(LinearOpMode opMode) {
        OpModeReference = opMode;
    }

    public void initialize() {

        allHubs = OpModeReference.hardwareMap.getAll(LynxModule.class);

        // we're setting caching mode so we can do a bulk read
        // important: must clear cache manually in any control loop
        for (LynxModule module : allHubs)
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // imu stuff from last year cause i dont want to rebuild this from the ground up
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode

        imu = OpModeReference.hardwareMap.get(BNO055IMU.class, "IMU");

        // wheels
        FL = OpModeReference.hardwareMap.get(DcMotorEx.class, "FL");
        FR = OpModeReference.hardwareMap.get(DcMotorEx.class, "FR");
        BL = OpModeReference.hardwareMap.get(DcMotorEx.class, "BL");
        BR = OpModeReference.hardwareMap.get(DcMotorEx.class, "BR");

        // motor arrays - quick instantiation as immutable lists
        LeftDriveMotors = Arrays.asList(new DcMotorEx[]{FL,BL});
        RightDriveMotors = Arrays.asList(new DcMotorEx[]{FR,BR});

        // all - at least you don't have to worry about adding individual motors
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
            m.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        // initialize the IMU
        imu.initialize(parameters);
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

    // get tps (ticks per second) based on power
    private double tpsFromPower(double power) {
        if (power == 0)
            return 0;

        double targetRPM = power * maxRPM;
        double RPS = targetRPM / 60;
        return RPS * countsPerRevolution;
    }

    // This method makes the robot turn.
    // DO NOT try to turn more than 180 degrees in either direction
    // targetAngleDifference is the number of degrees you want to turn
    // should be positive if turning left, negative if turning right
    private void turn(double targetAngleDifference, double power) {

        // before starting the turn, take note of current angle as startAngle
        double startAngle = GetCurrentZAngle();

        // just some boolean variables to tell if we've stepped motor power down
        // might actually want more than two steps
        boolean firstStepDownComplete = false;
        boolean secondStepDownComplete = false;

        double rightPower;
        double leftPower;

        // if target angle is Negative, we're turning RIGHT
        if (targetAngleDifference < 0) {
            // turning right, so we want all right motors going backwards
            rightPower = -power;
            leftPower = power;

            double currentRightTPS = tpsFromPower(rightPower);
            double currentLeftTPS = tpsFromPower(leftPower);

            for (DcMotorEx m : RightDriveMotors)
                m.setVelocity(currentRightTPS);
            for (DcMotorEx m : LeftDriveMotors)
                m.setVelocity(currentLeftTPS);

            for (DcMotorEx m : RightDriveMotors)
                m.setVelocity(tpsFromPower(rightPower));
            for (DcMotorEx m : LeftDriveMotors)
                m.setVelocity(tpsFromPower(leftPower));
            // sleep a tenth of a second
            // WARNING - not sure why this is needed - but sometimes right turns didn't work without
            OpModeReference.sleep(100);

            // we're turning right, so our target angle difference will be negative (ex: -90)
            // so GetAngleDifference will go from 0 to -90
            // keep turning while difference is greater than target
            while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) > targetAngleDifference) {

                // important: must clear cache manually in any control loop where reading from motors
                for (LynxModule module : allHubs)
                    module.clearBulkCache();

                double turnCompletion = GetAngleDifference(startAngle) / targetAngleDifference;

                double multiplier = 1;
                if (turnCompletion > 0.75)
                    multiplier = 0.25;
                else if (turnCompletion > 0.5)
                    multiplier = 0.5;

                double targetRightTPS = tpsFromPower(rightPower * multiplier);
                double targetLeftTPS = tpsFromPower(leftPower * multiplier);

                if ((currentLeftTPS != targetLeftTPS) || (currentRightTPS != targetRightTPS)) {
                    currentLeftTPS = targetLeftTPS;
                    currentRightTPS = targetRightTPS;
                    for (DcMotorEx m: RightDriveMotors)
                        m.setVelocity(currentRightTPS);
                    for (DcMotorEx m: LeftDriveMotors)
                        m.setVelocity(currentLeftTPS);
                }

//                // THIS CODE IS FOR STEPPING DOWN MOTOR POWER
//                if (!secondStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
//                    for (DcMotorEx m : RightDriveMotors)
//                        m.setVelocity(tpsFromPower(rightPower * 0.25));
//                    for (DcMotorEx m : LeftDriveMotors)
//                        m.setVelocity(tpsFromPower(leftPower * 0.25));
//                    secondStepDownComplete = true;
//                } else if (!firstStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
//                    for (DcMotorEx m : RightDriveMotors)
//                        m.setVelocity(tpsFromPower(rightPower * 0.5));
//                    for (DcMotorEx m : LeftDriveMotors)
//                        m.setVelocity(tpsFromPower(leftPower * 0.5));
//                    firstStepDownComplete = true;
//                }

                OpModeReference.telemetry.addData("target", targetAngleDifference);
                OpModeReference.telemetry.addData("current", GetAngleDifference(startAngle));
                OpModeReference.telemetry.addData("LeftMotorVelocity", "FL: " + FL.getVelocity() + " -- BL: " + BL.getVelocity());
                OpModeReference.telemetry.addData("RightMotorVelocity", "FR: " + FR.getVelocity() + " -- BR: " + BR.getVelocity());
                OpModeReference.telemetry.update();
            }
            // if targetAngleDifference is Positive, we're turning LEFT
        } else if (targetAngleDifference > 0) {

            // turning left so want all left motors going backwards
            rightPower = power;
            leftPower = -power;

            double currentRightTPS = tpsFromPower(rightPower);
            double currentLeftTPS = tpsFromPower(leftPower);

            for (DcMotorEx m : RightDriveMotors)
                m.setVelocity(currentRightTPS);
            for (DcMotorEx m : LeftDriveMotors)
                m.setVelocity(currentLeftTPS);

            // WARNING not sure if this sleep is needed - seemed necessary for right turns
            OpModeReference.sleep (100);

            // we're turning right, so our target angle difference will be positive (ex: 90)
            // so GetAngleDifference will go from 0 to 90
            // keep turning while difference is less than target
            while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) < targetAngleDifference) {

                // important: must clear cache manually in any control loop where reading from motors
                for (LynxModule module : allHubs)
                    module.clearBulkCache();

                double turnCompletion = GetAngleDifference(startAngle) / targetAngleDifference;

                double multiplier = 1;
                if (turnCompletion > 0.75)
                    multiplier = 0.25;
                else if (turnCompletion > 0.5)
                    multiplier = 0.5;

                double targetRightTPS = tpsFromPower(rightPower * multiplier);
                double targetLeftTPS = tpsFromPower(leftPower * multiplier);

                if ((currentLeftTPS != targetLeftTPS) || (currentRightTPS != targetRightTPS)) {
                    currentLeftTPS = targetLeftTPS;
                    currentRightTPS = targetRightTPS;
                    for (DcMotorEx m: RightDriveMotors)
                        m.setVelocity(currentRightTPS);
                    for (DcMotorEx m: LeftDriveMotors)
                        m.setVelocity(currentLeftTPS);
                }


                // THIS CODE IS FOR STEPPING DOWN MOTOR POWER
//                if (!secondStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
//                    for (DcMotorEx m : RightDriveMotors)
//                        m.setVelocity(tpsFromPower(rightPower * 0.25));
//                    for (DcMotorEx m : LeftDriveMotors)
//                        m.setVelocity(tpsFromPower(leftPower * 0.25));
//                    secondStepDownComplete = true;
//                } else if (!firstStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
//                    for (DcMotorEx m : RightDriveMotors)
//                        m.setVelocity(tpsFromPower(rightPower * 0.5));
//                    for (DcMotorEx m : LeftDriveMotors)
//                        m.setVelocity(tpsFromPower(leftPower * 0.5));
//                    firstStepDownComplete = true;
//                }
                OpModeReference.telemetry.addData("target", targetAngleDifference);
                OpModeReference.telemetry.addData("current", GetAngleDifference(startAngle));
                OpModeReference.telemetry.addData("LeftMotorVelocity", "FL: " + FL.getVelocity() + " -- BL: " + BL.getVelocity());
                OpModeReference.telemetry.addData("RightMotorVelocity", "FR: " + FR.getVelocity() + " -- BR: " + BR.getVelocity());
                OpModeReference.telemetry.update();
            }
        } else {
            // is zero - not turning - just return
            return;
        }

        // turn all motors off
        stopDriving();
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
            m.setVelocity(tpsFromPower(0));
    }

    public void dumDrive(double speed) {
        for (DcMotorEx m : AllDriveMotors)
            m.setVelocity(tpsFromPower(speed));
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
            for (DcMotorEx m : AllDriveMotors)
                m.setVelocity(tpsFromPower(speed));

            // just keep looping while both motors are busy
            // stop if driver station stop button pushed
//            while (OpModeReference.opModeIsActive() && ((FL.isBusy() && FR.isBusy()) && (BL.isBusy() && BR.isBusy()))) {
            // ONLY care if back wheels are busy - since they're the grippy ones
            while (OpModeReference.opModeIsActive() && BL.isBusy() && BR.isBusy()) {
                // important: must clear cache manually in any control loop where reading from motors
                for (LynxModule module : allHubs)
                    module.clearBulkCache();

                OpModeReference.telemetry.addData("target ticks", targetTicks);
                OpModeReference.telemetry.addData("FR current", FR.getCurrentPosition());
                OpModeReference.telemetry.addData("FL current", FL.getCurrentPosition());
                OpModeReference.telemetry.addData("BL current", BL.getCurrentPosition());
                OpModeReference.telemetry.addData("BR current", BR.getCurrentPosition());

                OpModeReference.telemetry.update();
            }

            // once all motors get to where they need to be, turn them off
            stopDriving();

            // set motors back to RUN_USING_ENCODERS
            for (DcMotorEx m : AllDriveMotors)
                m.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    public void driverControl () {

        double drive = -OpModeReference.gamepad1.left_stick_y;
        double turn = OpModeReference.gamepad1.right_stick_x;
        double movingSpeed;

        if (OpModeReference.gamepad1.left_bumper)
            movingSpeed = 0.1;
        else if (OpModeReference.gamepad1.right_bumper)
            movingSpeed = 1;
        else
            movingSpeed = 0.5;

        for (DcMotorEx l : LeftDriveMotors)
            l.setVelocity(tpsFromPower(movingSpeed * (drive + turn)));
        for (DcMotorEx r : RightDriveMotors)
            r.setVelocity(tpsFromPower(movingSpeed * (drive - turn)));
//        OpModeReference.telemetry.addData("Central Velocity", speed*movingSpeed);
//        OpModeReference.telemetry.addData("Lateral Velocity", strafe*movingSpeed);
//        OpModeReference.telemetry.addData("Rotation", rotate*movingSpeed);
    }

}
