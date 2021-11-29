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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//vision import
//dont mind me just importing some imu stuff
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//blinkin import


// This is not an OpMode.  It is a class that holds all the boring stuff

public class ProgrammingBot {


    public DcMotorEx[] LeftDriveMotors = new DcMotorEx[2];
    public DcMotorEx[] RightDriveMotors = new DcMotorEx[2];
    public DcMotorEx[] AllDriveMotors = new DcMotorEx[4];


    BNO055IMU imu;

    // Motors
    public DcMotorEx FL = null;
    public DcMotorEx FR = null;
    public DcMotorEx BL = null;
    public DcMotorEx BR = null;


    // just gonna define some variables for encoders real quick dont mind me
    static final double mmPerInch               = 25.4f;    // this is jus math tho
    static final double maxRPM                  = 340;      // Neverest orbital 20
    static final double countsPerRevolution     = 537.6f;   // Neverest orbital 20
    static final double wheelDiameterMM         = 96;       // For figuring circumference
    static final double WheelDiameterIn         = wheelDiameterMM / mmPerInch;
    static final double wheelCircumferenceIn    = WheelDiameterIn * Math.PI;
    static final double countsPerInch         = (countsPerRevolution / wheelCircumferenceIn);


    // you will need a reference to your OpMode
    private LinearOpMode OpModeReference;
    public ProgrammingBot(LinearOpMode opMode) {
        OpModeReference = opMode;
    }

    public void initialize() {

        // imu stuff from last year cause i dont want to rebuild this from the ground up
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode

        imu = OpModeReference.hardwareMap.get(BNO055IMU.class, "IMU");

        // wheels
        FL = OpModeReference.hardwareMap.get(DcMotorEx.class, "FL");
        FR = OpModeReference.hardwareMap.get(DcMotorEx.class, "FR");
        BL = OpModeReference.hardwareMap.get(DcMotorEx.class, "BL");
        BR = OpModeReference.hardwareMap.get(DcMotorEx.class, "BR");

        // motor arrays
        // left
        LeftDriveMotors[0] = FL;
        LeftDriveMotors[1] = BL;
        // right
        RightDriveMotors[0] = FR;
        RightDriveMotors[1] = BR;
        // all
        AllDriveMotors[0] = FL;
        AllDriveMotors[1] = FR;
        AllDriveMotors[2] = BL;
        AllDriveMotors[3] = BR;

        for (DcMotorEx l : LeftDriveMotors)
            l.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx r : RightDriveMotors)
            r.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx m : AllDriveMotors){
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        // initialize the IMU
        imu.initialize(parameters);
    }

    public void turnLeft(double angle, double power) {
        turn(angle, power);
    }

    public void turnRight(double angle, double power) {
        turn(-(angle), power);
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

            for (DcMotorEx m : RightDriveMotors)
                m.setPower(rightPower);
            for (DcMotorEx m : LeftDriveMotors)
                m.setPower(leftPower);
            // sleep a tenth of a second
            // WARNING - not sure why this is needed - but sometimes right turns didn't work without
            OpModeReference.sleep(100);

            // we're turning right, so our target angle difference will be negative (ex: -90)
            // so GetAngleDifference will go from 0 to -90
            // keep turning while difference is greater than target
            while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) > targetAngleDifference) {

                // THIS CODE IS FOR STEPPING DOWN MOTOR POWER
                if (!secondStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
                    for (DcMotorEx m : RightDriveMotors)
                        m.setPower(rightPower * 0.25);
                    for (DcMotorEx m : LeftDriveMotors)
                        m.setPower(leftPower * 0.25);
                    secondStepDownComplete = true;
                } else if (!firstStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
                    for (DcMotorEx m : RightDriveMotors)
                        m.setPower(rightPower * 0.5);
                    for (DcMotorEx m : LeftDriveMotors)
                        m.setPower(leftPower * 0.5);
                    firstStepDownComplete = true;
                }

                OpModeReference.telemetry.addData("target", targetAngleDifference);
                OpModeReference.telemetry.addData("current", GetAngleDifference(startAngle));
                OpModeReference.telemetry.addData("LeftMotorPower", (FL.getPower() + BL.getPower())/2);
                OpModeReference.telemetry.addData("RightMotorPower", (FR.getPower() + BR.getPower())/2);
                OpModeReference.telemetry.update();
            }
            // if targetAngleDifference is Positive, we're turning LEFT
        } else if (targetAngleDifference > 0) {

            // turning left so want all left motors going backwards
            rightPower = power;
            leftPower = -power;


            for (DcMotorEx m : RightDriveMotors)
                m.setPower(rightPower);
            for (DcMotorEx m : LeftDriveMotors)
                m.setPower(leftPower);

            // WARNING not sure if this sleep is needed - seemed necessary for right turns
            OpModeReference.sleep (100);

            // we're turning right, so our target angle difference will be positive (ex: 90)
            // so GetAngleDifference will go from 0 to 90
            // keep turning while difference is less than target
            while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) < targetAngleDifference) {

                // THIS CODE IS FOR STEPPING DOWN MOTOR POWER
                if (!secondStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
                    for (DcMotorEx m : RightDriveMotors)
                        m.setPower(rightPower * 0.25);
                    for (DcMotorEx m : LeftDriveMotors)
                        m.setPower(leftPower * 0.25);
                    secondStepDownComplete = true;
                } else if (!firstStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
                    for (DcMotorEx m : RightDriveMotors)
                        m.setPower(rightPower * 0.5);
                    for (DcMotorEx m : LeftDriveMotors)
                        m.setPower(leftPower * 0.5);
                    firstStepDownComplete = true;
                }
                OpModeReference.telemetry.addData("target", targetAngleDifference);
                OpModeReference.telemetry.addData("current", GetAngleDifference(startAngle));
                OpModeReference.telemetry.addData("LeftMotorPower", (FL.getPower() + BL.getPower())/2);
                OpModeReference.telemetry.addData("RightMotorPower", (FR.getPower() + BR.getPower())/2);
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
    public double GetCurrentZAngle() {
        Orientation currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return currentAngles.thirdAngle;
    }

    // This method calculates the difference of the current angle from the start angle
    // If you're left of your original angle, the value will be POSITIVE
    // If you're right of your original angle, the value will be NEGATIVE
    public double GetAngleDifference(double startAngle) {
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
                m.setPower(speed/2);

            // just keep looping while both motors are busy
            // stop if driver station stop button pushed
            while (OpModeReference.opModeIsActive() && ((FL.isBusy() && FR.isBusy()) && (BL.isBusy() && BR.isBusy()))) {
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
            l.setPower(movingSpeed * (drive + turn));
        for (DcMotorEx r : RightDriveMotors)
            r.setPower(movingSpeed * (drive - turn));
//        OpModeReference.telemetry.addData("Central Velocity", speed*movingSpeed);
//        OpModeReference.telemetry.addData("Lateral Velocity", strafe*movingSpeed);
//        OpModeReference.telemetry.addData("Rotation", rotate*movingSpeed);
    }

}
