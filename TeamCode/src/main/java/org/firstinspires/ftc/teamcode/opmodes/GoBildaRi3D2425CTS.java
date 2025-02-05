/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;

import java.util.Locale;

/*
 * This is (mostly) the OpMode used in the goBILDA Robot in 3 Days for the 24-25 Into The Deep FTC Season.
 * https://youtube.com/playlist?list=PLpytbFEB5mLcWxf6rOHqbmYjDi9BbK00p&si=NyQLwyIkcZvZEirP (playlist of videos)
 * I've gone through and added comments for clarity. But most of the code remains the same.
 * This is very much based on the code for the Starter Kit Robot for the 24-25 season. Those resources can be found here:
 * https://www.gobilda.com/ftc-starter-bot-resource-guide-into-the-deep/
 *
 * There are three main additions to the starter kit bot code, mecanum drive, a linear slide for reaching
 * into the submersible, and a linear slide to hang (which we didn't end up using)
 *
 * the drive system is all 5203-2402-0019 (312 RPM Yellow Jacket Motors) and it is based on a Strafer chassis
 * The arm shoulder takes the design from the starter kit robot. So it uses the same 117rpm motor with an
 * external 5:1 reduction
 *
 * The drivetrain is set up as "field centric" with the internal control hub IMU. This means
 * when you push the stick forward, regardless of robot orientation, the robot drives away from you.
 * We "took inspiration" (copy-pasted) the drive code from this GM0 page
 * (PS GM0 is a world class resource, if you've got 5 mins and nothing to do, read some GM0!)
 * https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric
 *
 */


/** @noinspection ALL*/
@TeleOp(name="goBILDA Rin3D-CTS", group="Robot")
//@Disabled
public class GoBildaRi3D2425CTS extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();

    double extensionPosition = robot.EXTENSION_COLLAPSED;

    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    double elbowLiftComp = 0;




    /* Variables that are used to set the arm to a specific position */
    double elbowPosition = (int)robot.ELBOW_COLLAPSED_INTO_ROBOT;
    double elbowPositionFudgeFactor;


    @Override
    public void runOpMode() {
        /*
        These variables are private to the OpMode, and are used to control the drivetrain.
         */
        double left;
        double right;
        double forward;
        double rotate;
        double max;
        double servoWristPosition = robot.WRIST_FOLDED_OUT;




        robot.init(hardwareMap, true);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

//        robot.extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        /* Wait for the game driver to press play */
        waitForStart();
        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


        // Initializes ElapsedTimes. One for total runtime of the program and the others set up for toggles.
        ElapsedTime totalRuntime = new ElapsedTime();
        ElapsedTime clawRuntime = new ElapsedTime();
        ElapsedTime rotateClawRuntime = new ElapsedTime();
        ElapsedTime armExtensionRuntime = new ElapsedTime();
        ElapsedTime armClimbRuntime = new ElapsedTime();



        totalRuntime.reset();
        clawRuntime.reset();
        rotateClawRuntime.reset();
        // armExtensionRuntime.reset();
       // armClimbRuntime.reset();

        double denominator = 0;
        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;

        // booleans for keeping track of toggles
        boolean clawOpened = false;
        boolean clawRotated = true;
        boolean armRetracted = true;
        //boolean armClimb = false;



        /* Run until the driver presses stop */
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                robot.pinpoint.recalibrateIMU();
                //recalibrates the IMU without resetting position
            }

            robot.pinpoint.update();    //update the IMU value
            Pose2D pos = robot.pinpoint.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.RADIANS));
            telemetry.addData("Position", data);

            //botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            botHeading = pos.getHeading(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeftPower = (rotY + rotX + rx) / denominator;
            backLeftPower = (rotY - rotX + rx) / denominator;
            frontRightPower = (rotY - rotX - rx) / denominator;
            backRightPower = (rotY + rotX - rx) / denominator;

            robot.leftFrontDrive.setPower(frontLeftPower);
            robot.leftBackDrive.setPower(backLeftPower);
            robot.rightFrontDrive.setPower(frontRightPower);
            robot.rightBackDrive.setPower(backRightPower);

            if (gamepad2.y) {
                extensionPosition = robot.EXTENSION_SCORING_IN_HIGH_BASKET;
            }
            else if (gamepad2.a) {
                extensionPosition = 0;
            }

            if (elbowPosition < 45 * robot.ELBOW_TICKS_PER_DEGREE){
                elbowLiftComp = (.25568 * extensionPosition); //0.25568
            }
            else{
                elbowLiftComp = 0;
            }


            if (gamepad2.right_bumper && (extensionPosition + 20) < robot.EXTENSION_SCORING_IN_HIGH_BASKET){
                extensionPosition += 15;
//                liftPosition += 2800 * cycletime;
            }
            else if (gamepad2.left_bumper && (extensionPosition - 20) > 0){
                extensionPosition -= 15;
//                liftPosition -= 2800 * cycletime;
            }

            // Double check.
            // Checks again if liftPosition is beyond its boundries or not.
            // If it is outside the boundries, then it limits it to the boundries between 0 and the high bucket lift position.
            if (extensionPosition < 0) {
                extensionPosition = 0;
            } else if(elbowPosition <= robot.ELBOW_TRAVERSE){
                if(extensionPosition >= robot.EXTENSION_DOWN_MAX){
                    extensionPosition = robot.EXTENSION_DOWN_MAX;
                }
            } else if (extensionPosition > robot.EXTENSION_SCORING_IN_HIGH_BASKET) {
                extensionPosition = robot.EXTENSION_SCORING_IN_HIGH_BASKET;
            }
            robot.extendMotor.setPower(1);

            if (gamepad2.left_trigger > 0.05 && (extensionPosition + (40 * -gamepad2.right_stick_y)) > 0 && (extensionPosition + (40 * -gamepad2.right_stick_y)) < robot.EXTENSION_DOWN_MAX){
                extensionPosition += (40 * -gamepad2.right_stick_y);
            }

            robot.extendMotor.setTargetPosition((int) extensionPosition);

            /* send telemetry to the driver of the arm's current position and target position */
            //telemetry.addData("arm Target Position: ", robot.armMotor.getTargetPosition());
            //telemetry.addData("arm Encoder: ", robot.armMotor.getCurrentPosition());
            telemetry.addData("Extension variable", extensionPosition);
            telemetry.addData("Lift Target Position",robot.extendMotor.getTargetPosition());
            telemetry.addData("lift current position", robot.extendMotor.getCurrentPosition());
            telemetry.addData("liftMotor Current:",((DcMotorEx) robot.extendMotor).getCurrent(CurrentUnit.AMPS));
            telemetry.update();

        }
    }
}