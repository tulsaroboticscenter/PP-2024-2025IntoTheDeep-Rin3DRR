package org.firstinspires.ftc.teamcode.hardware;

/****
 * DEPRECATED CLASS. Do Not Use!
 */




import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class RRHWProfile {

    /* Public OpMode members. */
    public MotorEx leftFrontDrive = null;
    //
    public MotorEx rightFrontDrive = null;
    //
    public MotorEx leftBackDrive = null;
    //1
    public MotorEx rightBackDrive = null;
    //2
    public DcMotor armMotor = null;
    public DcMotor extendMotor = null;
    //3
    public DcMotorEx odoPerp = null;
    public Servo wrist = null;
    public Servo claw = null;
    public RevIMU imu = null;
    public MecanumDrive mecanum = null;
    public Boolean opModeTeleop = null;

    /* local OpMode members. */
    HardwareMap hwMap =  null;

    /* Constructor */
    public RRHWProfile(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean teleOp) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        this.opModeTeleop = teleOp;

        /*
         * Initialize Motors
         */

        if(opModeTeleop){
            //drive motor init
            leftFrontDrive = new MotorEx(ahwMap, "motorLF", Motor.GoBILDA.RPM_1150);
            leftFrontDrive.setRunMode(Motor.RunMode.RawPower);
            leftFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            leftFrontDrive.resetEncoder();

            leftBackDrive = new MotorEx(ahwMap, "motorLR", Motor.GoBILDA.RPM_1150);
            leftBackDrive.setRunMode(Motor.RunMode.RawPower);
            leftBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.resetEncoder();

            rightFrontDrive = new MotorEx(ahwMap, "motorRF", Motor.GoBILDA.RPM_1150);
            rightFrontDrive.setRunMode(Motor.RunMode.RawPower);
            rightFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.resetEncoder();

            rightBackDrive = new MotorEx(ahwMap, "motorRB", Motor.GoBILDA.RPM_1150);
            rightBackDrive.setRunMode(Motor.RunMode.RawPower);
            rightBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.resetEncoder();

            //motorLiftFront = new MotorEx(ahwMap, "motorLiftFront", 385,435);
            //motorLiftBack = new MotorEx(ahwMap, "motorLiftBack", 385,435);
           // motorLiftBack.resetEncoder();

            odoPerp = hwMap.get(DcMotorEx.class, "odoPerp");
            odoPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


           /* lift = new MotorGroup(motorLiftFront, motorLiftBack);
            lift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            lift.set(0);
            lift.resetEncoder();
*/
            //init imu
            imu = new RevIMU(ahwMap);
            imu.init();

            //drivebase init
            mecanum = new com.arcrobotics.ftclib.drivebase.MecanumDrive(leftFrontDrive,leftBackDrive,rightBackDrive,rightFrontDrive);
        } else   {
          armMotor = hwMap.get(DcMotor.class, "armMotor");
          armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          armMotor.setPower(0);

        }

        extendMotor = hwMap.get(DcMotor.class, "extendMotor");
        extendMotor.setDirection(DcMotor.Direction.FORWARD);
        extendMotor.setTargetPosition(0);
        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor.setPower(0);



        wrist = hwMap.servo.get("wrist");
        claw = hwMap.servo.get("claw");


            /*
            auto_MotorLiftFront = hwMap.get(DcMotorEx.class, "motorLiftFront");
            auto_MotorLiftFront.setDirection(DcMotor.Direction.FORWARD);
            auto_MotorLiftFront.setTargetPosition(0);
            auto_MotorLiftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            auto_MotorLiftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            auto_MotorLiftFront.setPower(0);
        }

        motorIntake = hwMap.get(DcMotorEx.class, "motorIntake");
        motorIntake.setDirection(DcMotor.Direction.REVERSE);
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorIntake.setPower(0);

        servoIntakeLeft = hwMap.servo.get("servoIntakeLeft");
        servoIntakeRight = hwMap.servo.get("servoIntakeRight");
        servoArm = hwMap.servo.get("servoArm");
        servoPlungerAngle = hwMap.servo.get("servoPlungerAngle");
        servoPlunger = hwMap.servo.get("servoPlunger");
        servoDrone = hwMap.get(ServoImplEx.class, "servoDrone");

        servoAgitator = hwMap.crservo.get("servoAgitator");

        beamBucketBottom = hwMap.get(DigitalChannel.class, "beamBucketBottom");
        beamBucketTop = hwMap.get(DigitalChannel.class, "beamBucketTop");

        ledLeftFrontGreen = hwMap.get(DigitalChannel.class, "ledLFGreen");
        ledLeftFrontGreen.setMode(DigitalChannel.Mode.OUTPUT);

        ledLeftFrontRed = hwMap.get(DigitalChannel.class, "ledLFRed");
        ledLeftFrontRed.setMode(DigitalChannel.Mode.OUTPUT);

        ledLeftRearGreen = hwMap.get(DigitalChannel.class, "ledLRGreen");
        ledLeftRearGreen.setMode(DigitalChannel.Mode.OUTPUT);

        ledLeftRearRed = hwMap.get(DigitalChannel.class, "ledLRRed");
        ledLeftRearRed.setMode(DigitalChannel.Mode.OUTPUT);

        ledRightFrontGreen = hwMap.get(DigitalChannel.class, "ledRFGreen");
        ledRightFrontGreen.setMode(DigitalChannel.Mode.OUTPUT);

        ledRightFrontRed = hwMap.get(DigitalChannel.class, "ledRFRed");
        ledRightFrontRed.setMode(DigitalChannel.Mode.OUTPUT);

        ledRightRearGreen = hwMap.get(DigitalChannel.class, "ledRRGreen");
        ledRightRearGreen.setMode(DigitalChannel.Mode.OUTPUT);

        ledRightRearRed = hwMap.get(DigitalChannel.class, "ledRRRed");
        ledRightRearRed.setMode(DigitalChannel.Mode.OUTPUT);

        sensorDistanceFL = hwMap.get(AnalogInput.class, "DistanceFL");
        sensorDistanceFR = hwMap.get(AnalogInput.class, "DistanceFR");

   */ }   // end of init() method

}       // end of the HardwareProfile class