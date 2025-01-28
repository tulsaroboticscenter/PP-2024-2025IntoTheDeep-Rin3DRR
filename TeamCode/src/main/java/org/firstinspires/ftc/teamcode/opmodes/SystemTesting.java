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
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;

import java.util.Locale;


@TeleOp(name="System Test", group="Robot")
//@Disabled
public class SystemTesting extends LinearOpMode {

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
        double denominator = 0;
        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;
        double servoWristPosition = robot.WRIST_FOLDED_OUT;

        robot.init(hardwareMap, true);
        telemetry.addData("System Test:", "Initialized");
        telemetry.update();

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        /* Wait for the game driver to press play */
        waitForStart();
        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        /* Run until the driver presses stop */
        while (opModeIsActive())

        {   double y = -gamepad1.left_stick_y;
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
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            //botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            botHeading = pos.getHeading(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
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

            /* Here we handle the three buttons that have direct control of the intake speed.
            These control the continuous rotation servo that pulls elements into the robot,
            If the user presses A, it sets the intake power to the final variable that
            holds the speed we want to collect at.
            If the user presses X, it sets the servo to Off.
            And if the user presses B it reveres the servo to spit out the element.*/

            /* TECH TIP: If Else statement:
            We're using an else if statement on "gamepad1.x" and "gamepad1.b" just in case
            multiple buttons are pressed at the same time. If the driver presses both "a" and "x"
            at the same time. "a" will win over and the intake will turn on. If we just had
            three if statements, then it will set the intake servo's power to multiple speeds in
            one cycle. Which can cause strange behavior. */

            telemetry.addData("Open Claw = ", "GAMEPAD1.Left_Bumper");
            telemetry.addData("Close Claw = ", "GAMEPAD1.Right_Bumper");

            if (gamepad1.left_bumper) {
                robot.servoClaw.setPosition(robot.CLAW_OPEN);
            } else if (gamepad1.right_bumper) {
                robot.servoClaw.setPosition(robot.CLAW_CLOSED);
            }

            /* Here we create a "fudge factor" for the arm position.
            This allows you to adjust (or "fudge") the arm position slightly with the gamepad triggers.
            We want the left trigger to move the arm up, and right trigger to move the arm down.
            So we add the right trigger's variable to the inverse of the left trigger. If you pull
            both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
            than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
            The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function. */

            elbowPositionFudgeFactor = robot.FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));

            /* Here we implement a set of if else statements to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            intake and wrist) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the armPosition to the ARM_COLLECT position,
            it folds out the wrist to make sure it is in the correct orientation to intake, and it
            turns the intake on to the COLLECT mode.*/

            if(gamepad1.a){
                /* This is the intaking/collecting arm position */
                elbowPosition = robot.ELBOW_TRAVERSE;
                extensionPosition = robot.EXTENSION_COLLAPSED;
                servoWristPosition = robot.WRIST_FOLDED_OUT;
            } else if (gamepad1.b){
                    /*This is about 20° up from the collecting position to clear the barrier
                    Note here that we don't set the wrist position or the intake power when we
                    select this "mode", this means that the intake and wrist will continue what
                    they were doing before we clicked left bumper. */
                elbowPosition = robot.ELBOW_CLEAR_BARRIER;
            } else if (gamepad1.x){
                /* This is the correct height to score the sample in the HIGH BASKET */
                elbowPosition = robot.ELBOW_SCORE_SAMPLE_IN_LOW;
                //liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
            } else if (gamepad1.dpad_left) {
                    /* This turns off the intake, folds in the wrist, and moves the arm
                    back to folded inside the robot. This is also the starting configuration */
                elbowPosition = robot.ELBOW_COLLAPSED_INTO_ROBOT;
                //liftPosition = LIFT_COLLAPSED;
                //.intake.setPower(robot.INTAKE_OFF);
                servoWristPosition = robot.WRIST_FOLDED_OUT;
            } else if (gamepad1.dpad_right){
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                elbowPosition = robot.ELBOW_SCORE_SPECIMEN;
                servoWristPosition = robot.WRIST_FOLDED_IN;
            } else if (gamepad2.dpad_up){
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                elbowPosition = robot.ELBOW_HANG_ATTACH;
                //robot.intake.setPower(robot.INTAKE_OFF);
                servoWristPosition = robot.WRIST_FOLDED_IN;
            } else if (gamepad1.y){
                elbowPosition = robot.ELBOW_EXTENSION_ANGLE;
            } else if (gamepad2.dpad_down){
                /* this moves the arm down to lift the robot up once it has been hooked */
                elbowPosition = robot.ELBOW_HANG_CLIMB;
                //robot.intake.setPower(robot.INTAKE_OFF);
                servoWristPosition = robot.WRIST_FOLDED_IN;
            } else if (gamepad2.y) {
                extensionPosition = robot.EXTENSION_SCORING_IN_HIGH_BASKET;
            } else if (gamepad2.a) {
                extensionPosition = 0;
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
                elbowLiftComp = (0.25568 * extensionPosition);
            }
            else{
                elbowLiftComp = 0;
            }

           /* Here we set the target position of our arm to match the variable that was selected
            by the driver. We add the armPosition Variable to our armPositionFudgeFactor, before adding
            our armLiftComp, which adjusts the arm height for different lift extensions.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/

            //robot.armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp));

            //((DcMotorEx) robot.armMotor).setVelocity(2100);
            //robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            /* Here we set the lift position based on the driver input.
            This is a.... weird, way to set the position of a "closed loop" device. The lift is run
            with encoders. So it knows exactly where it is, and there's a limit to how far in and
            out it should run. Normally with mechanisms like this we just tell it to run to an exact
            position. This works a lot like our arm. Where we click a button and it goes to a position, then stops.
            But the drivers wanted more "open loop" controls. So we want the lift to keep extending for
            as long as we hold the bumpers, and when we let go of the bumper, stop where it is at.
            This allows the driver to manually set the position, and not have to have a bunch of different
            options for how far out it goes. But it also lets us enforce the end stops for the slide
            in software. So that the motor can't run past it's endstops and stall.
            We have our liftPosition variable, which we increment or decrement for every cycle (every
            time our main robot code runs) that we're holding the button. Now since every cycle can take
            a different amount of time to complete, and we want the lift to move at a constant speed,
            we measure how long each cycle takes with the cycletime variable. Then multiply the
            speed we want the lift to run at (in mm/sec) by the cycletime variable. There's no way
            that our lift can move 2800mm in one cycle, but since each cycle is only a fraction of a second,
            we are only incrementing it a small amount each cycle.
             */

            telemetry.addData("Extend Arm = ", "gamepad2.Right_Bumper");
            telemetry.addData("Retract Arm = ", "gamepad2.Left_Bumper");
            // If the button is pressed and liftPosition is not surpassing the range it should be in, then liftPosition is changed accordingly.
            if (gamepad2.right_bumper && (extensionPosition + 20) < robot.EXTENSION_SCORING_IN_HIGH_BASKET){
                extensionPosition += 20;
//                liftPosition += 2800 * cycletime;
            }
            else if (gamepad2.left_bumper && (extensionPosition - 20) > 0){
                extensionPosition -= 20;
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




            robot.elbowMotor.setTargetPosition((int) elbowPosition);
            robot.extendMotor.setTargetPosition((int) extensionPosition);

            robot.elbowMotor.setPower(1);
            robot.extendMotor.setPower(1);


            /* Check to see if our arm is over the current limit, and report via telemetry. */
           // if (((DcMotorEx) robot.armMotor).isOverCurrent()){
             //   telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");

            /* at the very end of the stream, we added a linear actuator kit to try to hang the robot on.
             * it didn't end up working... But here's the code we run it with. It just sets the motor
             * power to match the inverse of the left stick y.
             */

            if(gamepad2.left_stick_y > 0){
                elbowPosition =- 1;
            } else if (gamepad2.left_stick_y < 0){
                elbowPosition =+ 1;
            }
//            robot.hangMotor.setPower(-gamepad2.left_stick_y);

            robot.servoWrist.setPosition(servoWristPosition);

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

            /* send telemetry to the driver of the arm's current position and target position */
            //telemetry.addData("arm Target Position: ", robot.armMotor.getTargetPosition());
            //telemetry.addData("arm Encoder: ", robot.armMotor.getCurrentPosition());
            telemetry.addData("Intake Position Down = ", "GAMEPAD1.A");
            telemetry.addData("Intake Position 20 Degrees= ", "GAMEPAD1.B");
            telemetry.addData("Elbow Position low basket = ", "GAMEPAD1.X");
            telemetry.addData("wrist in, retract arm = ", "GAMEPAD1.DPAD_LEFT");
            telemetry.addData("High Chamber Scoring = ", "GAMEPAD1.DPAD_RIGHT");
            telemetry.addData("Hang Up = ", "GAMEPAD1.DPAD_UP");
            telemetry.addData("Hang Down = ", "GAMEPAD1.DPAD_Down");
            telemetry.addData("Lift Scoring Position = ", "GAMEPAD2.Y");
            telemetry.addData("Reset Lift = ", "GAMEPAD2.A");
            telemetry.addData("lift variable", extensionPosition);
            telemetry.addData("Lift Target Position",robot.extendMotor.getTargetPosition());
            telemetry.addData("lift current position", robot.extendMotor.getCurrentPosition());
            telemetry.addData("liftMotor Current:",((DcMotorEx) robot.extendMotor).getCurrent(CurrentUnit.AMPS));
            telemetry.update();

        }
    }
}