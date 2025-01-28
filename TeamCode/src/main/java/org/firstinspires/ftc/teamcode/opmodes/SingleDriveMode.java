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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    /**

        CONTROLLER MAPPINGS:

        == Joysticks: Driving ==
        Left Stick: Drive/Strafe
        Right Stick: Rotation

        == Bumpers: Claw Manipulation ==
        LB: Toggle Claw Rotation
        RB: Toggle Claw Grabbing

        == Face Buttons: Scoring Presets ==
        Y / Triangle: High Bucket Scoring Position
        X / Square: Low Bucket Scoring Position
        B / Circle: Clear Barrier Position
        A / Cross: Pickup Position

        == DPAD: Hanging Presets & Extension Toggle ==
        DPAD Up: Hang Ready Position
        DPAD Down: Hang Execution Position
        DPAD Left: Toggle Extension

        == Triggers: Manual Manipulation ==
        Left Trigger + Right Stick: Angle Up/Down
        Right Trigger + Right Stick: Extension Forwards/Backwards

        == Miscellaneous =
        OPTIONS: Recalibrate IMU

    **/

/** @noinspection ALL*/
@TeleOp(name="SingleDrive", group="Robot")
//@Disabled
public class SingleDriveMode extends LinearOpMode {

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
        double poleToucherPosition = robot.POLE_DOWN;


        robot.init(hardwareMap, true);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        robot.extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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

        // Additional ElapsedTime objects for delays.
        ElapsedTime clearExtension = new ElapsedTime();


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

        boolean highBasketDelay = false;
        boolean pickupDelay = false;
        //boolean armClimb = false;



        /* Run until the driver presses stop */
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = 0;

            if (gamepad1.right_trigger < 0.05 && gamepad1.left_trigger < 0.05) {
                rx  = gamepad1.right_stick_x;
            }

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                robot.pinpoint.recalibrateIMU();
                //recalibrates the IMU without resetting position
            }

            robot.pinpoint.update();    //update the IMU value
            Pose2D pos = robot.pinpoint.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
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
            robot.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Here we create a "fudge factor" for the arm position.
            This allows you to adjust (or "fudge") the arm position slightly with the gamepad triggers.
            We want the left trigger to move the arm up, and right trigger to move the arm down.
            So we add the right trigger's variable to the inverse of the left trigger. If you pull
            both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
            than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
            The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function. */

            elbowPositionFudgeFactor = robot.FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));

            // == FACE BUTTONS (ABXY) ==
            if (gamepad1.a) {
                /* This is the intaking/collecting arm position */
                clearExtension.reset();
                pickupDelay = true;
                extensionPosition = robot.EXTENSION_COLLAPSED;
            } else if (gamepad1.b) {
                // Raises the arm up to clear the barrier.
                elbowPosition = robot.ELBOW_TRAVERSE;
            } else if (gamepad1.x) {
                /* This is the correct height to score the sample in the LOW BASKET */
                elbowPosition = robot.ELBOW_SCORE_SAMPLE_IN_LOW;
                extensionPosition = robot.EXTENSION_COLLAPSED;
            } else if (gamepad1.y){
                // Sets up the arm for scoring in the high basket.
                clearExtension.reset();
                highBasketDelay = true;
                elbowPosition = robot.ELBOW_HIGH_BASKET;
            }

            // == FACE BUTTON EXTRAS ==
            if (clearExtension.time() > 1.0 && highBasketDelay == true) {
                // If Y is pressed and the arm is finally up, extend out to score.
                extensionPosition = robot.EXTENSION_SCORING_IN_HIGH_BASKET;
                highBasketDelay = false;
            }
            if (clearExtension.time() > 0.5 && pickupDelay == true) {
                elbowPosition = robot.ELBOW_CLEAR_BARRIER;
                pickupDelay = false;
            }


            // == DPAD ==
            if (gamepad1.dpad_left && armExtensionRuntime.time() > 0.25) {
                if (armRetracted) {
                    extensionPosition = robot.EXTENSION_COLLAPSED;
                    armRetracted = true;
                } else if (!armRetracted) {
                    extensionPosition = robot.EXTENSION_SCORING_IN_HIGH_BASKET;
                    armRetracted = false;
                    armExtensionRuntime.reset();
                }
                //boolean toggle for arm climb
            } else if (gamepad2.dpad_up) {
                elbowPosition = robot.ELBOW_HANG_ATTACH;

            } else if (gamepad2.dpad_down) {
                elbowPosition = robot.ELBOW_HANG_CLIMB;

            }


            // ==TRIGGERS==
            if (gamepad1.right_trigger > 0.05 && (elbowPosition + (20 * gamepad1.right_stick_y)) < robot.ELBOW_HIGH_BASKET && (elbowPosition + (20 * gamepad1.right_stick_y)) > robot.ELBOW_RESET){
                elbowPosition += (10 * gamepad1.right_stick_y);
            }
            else if (gamepad1.left_trigger > 0.05 && (extensionPosition + (20 * gamepad1.right_stick_y)) > 0 && (extensionPosition + (20 * gamepad1.right_stick_y)) < robot.EXTENSION_SCORING_IN_HIGH_BASKET){
                extensionPosition += (20 * -gamepad1.right_stick_y);
            }


            // ==BUMPERS==
            if (gamepad1.left_bumper && rotateClawRuntime.time() > 0.1) {
                if (clawRotated) {
                    servoWristPosition = robot.WRIST_FOLDED_OUT;
                    clawRotated = false;
                } else if (!clawRotated) {
                    servoWristPosition = robot.WRIST_FOLDED_IN;
                    clawRotated = true;
                }
            }
            if (gamepad1.right_bumper && clawRuntime.time() > 0.25) {
                if (clawOpened) {
                    robot.servoClaw.setPosition(robot.CLAW_CLOSED);
                    clawOpened = false;
                } else if (!clawOpened) {
                    robot.servoClaw.setPosition(robot.CLAW_OPEN);
                    clawOpened = true;
                }
                clawRuntime.reset();
            }


            /*
            This is probably my favorite piece of code on this robot. It's a clever little software
            solution to a problem the robot has.
            This robot has an extending lift on the end of an arm shoulder. That arm shoulder should
            run to a specific angle, and stop there to collect from the field. And the angle that
            the shoulder should stop at changes based on how long the arm is (how far the lift is extended)
            so here, we add a compensation factor based on how far the lift is extended.
            That comp factor is multiplied by the number of mm the lift is extended, which
            results in the number of degrees we need to fudge our arm up by to keep the end of the arm
            the same distance from the field.
            Now we don't need this to happen when the arm is up and in scoring position. So if the arm
            is above 45°, then we just set armLiftComp to 0. It's only if it's below 45° that we set it
            to a value.
             */

            if (elbowPosition < 45 * robot.ELBOW_TICKS_PER_DEGREE){
                elbowLiftComp = (.25568 * extensionPosition); //0.25568
            }
            else {
                elbowLiftComp = 0;
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

            robot.elbowMotor.setTargetPosition((int) elbowPosition);
            robot.extendMotor.setTargetPosition((int) extensionPosition);

            robot.elbowMotor.setPower(1);
            robot.extendMotor.setPower(1);
            robot.servoWrist.setPosition(servoWristPosition);


            /* Check to see if our arm is over the current limit, and report via telemetry. */

            /* at the very end of the stream, we added a linear actuator kit to try to hang the robot on.
             * it didn't end up working... But here's the code we run it with. It just sets the motor
             * power to match the inverse of the left stick y.
             */

            /* This is how we check our loop time. We create three variables:
            looptime is the current time when we hit this part of the code
            cycletime is the amount of time in seconds our current loop took
            oldtime is the time in seconds that the previous loop started at

            we find cycletime by just subtracting the old time from the current time.
            For example, lets say it is 12:01.1, and then a loop goes by and it's 12:01.2.
            We can take the current time (12:01.2) and subtract the oldtime (12:01.1) and we're left
            with just the difference, 0.1 seconds.

             */
            looptime = getRuntime();
            cycletime = looptime-oldtime;
            oldtime = looptime;

            //Rumble controller for endgame and flash controller light red
           /* if(totalRuntime.time() > 90 && totalRuntime.time()<90.25){
                gamepad1.rumble(50);
                gamepad1.setLedColor(255,0,0,50);
            } else if(totalRuntime.time() > 91 && totalRuntime.time()<91.25){
                gamepad1.rumble(50);
                gamepad1.setLedColor(255,0,0,50);
            } else if(totalRuntime.time() > 92 && totalRuntime.time()<92.25){
                gamepad1.rumble(50);
                gamepad1.setLedColor(255,0,0,50);
            } else if(totalRuntime.time() > 93) {
                gamepad1.setLedColor(255, 0, 0, 30000);
            }
*/
            /* send telemetry to the driver of the arm's current position and target position */
            //telemetry.addData("arm Target Position: ", robot.armMotor.getTargetPosition());
            //telemetry.addData("arm Encoder: ", robot.armMotor.getCurrentPosition());
            telemetry.addData("lift variable", extensionPosition);
            telemetry.addData("Lift Target Position",robot.extendMotor.getTargetPosition());
            telemetry.addData("lift current position", robot.extendMotor.getCurrentPosition());
            telemetry.addData("liftMotor Current:",((DcMotorEx) robot.extendMotor).getCurrent(CurrentUnit.AMPS));
            telemetry.update();

        }
    }
}