package org.firstinspires.ftc.teamcode.libraries;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.HWProfile;

public class RobotLibrary {

    // Hardware Profile Initialization
    private final HWProfile robot;
    public LinearOpMode opMode;

    public RobotLibrary(HWProfile myRobot, LinearOpMode myOpMode) {
        robot = myRobot;
        opMode = myOpMode;
    }



    public void moveDirection(String direction, double power) {
        // if direction is horizontal and power = 1, the robot will strafe right.

        if (direction == "horizontal") {
            robot.leftFrontDrive.setPower(power);
            robot.leftBackDrive.setPower(-power);
            robot.rightFrontDrive.setPower(-power);
            robot.rightBackDrive.setPower(power);
        } else {
            robot.leftFrontDrive.setPower(power);
            robot.leftBackDrive.setPower(power);
            robot.rightFrontDrive.setPower(power);
            robot.rightBackDrive.setPower(power);
        }
    }

    public void halt() {
        robot.leftFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
    }

}
