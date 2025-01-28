package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GoBilda.GoBildaPinpointDriverRR;

public class HWProfile {

    /* Declare OpMode members. */
    public DcMotor  leftFrontDrive   = null; //the left drivetrain motor
    public DcMotor  rightFrontDrive  = null; //the right drivetrain motor
    public DcMotor  leftBackDrive    = null;
    public DcMotor  rightBackDrive   = null;
    public DcMotor  extendMotor      = null; //
    public DcMotor  elbowMotor       = null;
    public Servo    servoWrist       = null; //the wrist servo
    public Servo    intakeRotate1    = null;
    public Servo    intakeRotate2    = null;
    public Servo    servoClaw        = null;
    public IMU      imu              = null;
    public GoBildaPinpointDriver pinpoint = null; // Declare OpMode member for the Odometry Computer
    public Servo    poleToucherServo = null;


    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    public final double ELBOW_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation


    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    public final double ARM_COLLECT                = 0 * ELBOW_TICKS_PER_DEGREE;
    public final double ELBOW_HANG_CLIMB           = -10  * ELBOW_TICKS_PER_DEGREE;
    public final int    ELBOW_RESET                = 10;
    public final double ELBOW_HANG_ATTACH          = 135 * ELBOW_TICKS_PER_DEGREE;
    public final double ELBOW_COLLAPSED_INTO_ROBOT = 100;
    public final double ELBOW_SCORE_SAMPLE_IN_LOW = 105 * ELBOW_TICKS_PER_DEGREE;
    public final int    ELBOW_CLEAR_BARRIER        = 200;
    public final double ELBOW_EXTENSION_ANGLE      = 400;
    public final int    ELBOW_TRAVERSE             = 600;
    public final double    ELBOW_HIGH_BASKET       = 110 * ELBOW_TICKS_PER_DEGREE;
    public final double ELBOW_SPECIMEN_PICKUP      = 18 * ELBOW_TICKS_PER_DEGREE;
    public final double ELBOW_SCORE_SPECIMEN       = 75  * ELBOW_TICKS_PER_DEGREE;
    public final double ELBOW_ANGLE_AUTON          = 450;



    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    public final double WRIST_FOLDED_IN   = 0.1667;
    public final double WRIST_FOLDED_OUT  = 0.5;
    public final double WRIST_FOLDED_PARTIAL = .25;

    /* A number in degrees that the triggers can adjust the arm position by */
    public final double FUDGE_FACTOR = 45 * ELBOW_TICKS_PER_DEGREE;

    public final double CLAW_OPEN = .25;
    public final double CLAW_CLOSED = 1;
    public final double CLAW_PARTIAL_OPEN = .6;


    public final double EXTENSION_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;
    public final double EXTENSION_COLLAPSED = 0 * EXTENSION_TICKS_PER_MM;
    public final double EXTENSION_SCORING_IN_LOW_BASKET = 0 * EXTENSION_TICKS_PER_MM;
    public final double EXTENSION_SCORING_IN_HIGH_BASKET = 490 * EXTENSION_TICKS_PER_MM;
    public final int    EXTENSION_DOWN_MAX         = 1300;


    public final double POLE_DOWN = 0;
    public final double POLE_UP = 1;

    public Boolean opModeTeleop = null;

    /* local OpMode members. */
    HardwareMap hwMap =  null;


    public HWProfile() {

    }

    public void init(HardwareMap ahwMap, boolean teleOp) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        this.opModeTeleop = teleOp;

        if(opModeTeleop){
            /* Define and Initialize Motors */
            leftFrontDrive  = hwMap.dcMotor.get("frontLeftMotor");
            leftBackDrive   = hwMap.dcMotor.get("backLeftMotor");
            rightFrontDrive = hwMap.dcMotor.get("frontRightMotor");
            rightBackDrive  = hwMap.dcMotor.get("backRightMotor");

           /*
           we need to reverse the left side of the drivetrain so it doesn't turn when we ask all the
           drive motors to go forward.
            */

            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
            much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
            stops much quicker. */
            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            // Retrieve the IMU from the hardware map
            imu  = hwMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);
            pinpoint = hwMap.get(GoBildaPinpointDriver.class,"pinpoint");
            pinpoint.resetPosAndIMU();


        }


        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */

        extendMotor = hwMap.dcMotor.get("liftMotor");
        elbowMotor = hwMap.dcMotor.get("armMotor");

        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */

        extendMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extendMotor.setTargetPosition(0);
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setTargetPosition(0);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        /* Define and initialize servos.*/
        servoWrist = hwMap.get(Servo.class, "wrist");

        /* Make sure that the intake is off, and the wrist is folded in. */
        servoClaw = hwMap.get(Servo.class, "claw");

        poleToucherServo = hwMap.get(Servo.class, "poleToucher");


    }
}
