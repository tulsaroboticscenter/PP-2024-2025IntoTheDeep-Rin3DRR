package org.firstinspires.ftc.teamcode.opmodes;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tuning.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.CSAutoParams;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;
import org.firstinspires.ftc.teamcode.libraries.RRMechOps;



//@Disabled
@Autonomous(name = "Auto Specimen + 3 Samples", group = "Competition", preselectTeleOp = "GoBildaRi3D2425")
public class RRAutoSpecimen3Sample extends LinearOpMode{
    public static String TEAM_NAME = "Project Peacock";
    public static int TEAM_NUMBER = 10355;

    //Define and declare Robot Starting Locations
    public enum START_POSITION {
        BLUE_SAMPLES,
        BLUE_SPECIMENS,
        RED_SAMPLES,
        RED_SPECIMENS
    }

    public static RRAutoSample.START_POSITION startPosition;

    public final static HWProfile robot = new HWProfile();
    public LinearOpMode opMode = this;
    public CSAutoParams params = new CSAutoParams();
    public RRMechOps mechOps = new RRMechOps(robot, opMode, params);

    @Override
    public void runOpMode() throws InterruptedException {

        //TODO: Initialize hardware
        robot.init(hardwareMap, false);

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();
        mechOps.clawClose();
        mechOps.rotateClaw(robot.WRIST_FOLDED_OUT);

        while (!isStopRequested() && !opModeIsActive()) {
            // Wait for the DS start button to be touched.
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            runAutonoumousMode();
        }
    }

    //end runOpMode();

    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d specimenScoringPosition = new Pose2d(0, 0, 0);
        Pose2d sampleScoringPosition = new Pose2d(0, 0, 0);
        Pose2d coloredSample1Position = new Pose2d(0, 0, 0);
        Pose2d coloredSample2Position = new Pose2d(0, 0, 0);
        Pose2d coloredSample3Position = new Pose2d(0, 0, 0);
        Pose2d grabSpecimenPosition = new Pose2d(0, 0, 0);
        Pose2d yellowSample1Position = new Pose2d(0, 0, 0);
        Pose2d yellowSample2Position = new Pose2d(0, 0, 0);
        Pose2d yellowSample3Position = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0, 0, 0);
        Pose2d midwayPose2 = new Pose2d(0, 0, 0);
        Pose2d midwayPose3 = new Pose2d(0, 0, 0);
        Pose2d midwayPose4 = new Pose2d(0,0,0);
        Pose2d midwayPose5 = new Pose2d(0,0,0);


        Pose2d parkPrepPose = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0, 0, 0);
        double waitSecondsBeforeDrop = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);


        drive = new MecanumDrive(hardwareMap, initPose);
        specimenScoringPosition = new Pose2d(35,-13, Math.toRadians(0));
        sampleScoringPosition = new Pose2d(2, 46, Math.toRadians(135));
        yellowSample1Position = new Pose2d(32, 33, Math.toRadians(0));
        yellowSample2Position = new Pose2d(33, 43, Math.toRadians(-30));
        yellowSample3Position = new Pose2d(42, 27, Math.toRadians(90));
        midwayPose1 = new Pose2d(15,33, Math.toRadians(135)); // This will be the position right before scoring on high basket
        midwayPose2 = new Pose2d(15,34, Math.toRadians(135)); // position we go to after scoring
        midwayPose3 = new Pose2d(27, 17, Math.toRadians(90)); // prep for sample #3
        midwayPose4 = new Pose2d(1,24, Math.toRadians(0)); // position we go right after scoring specimen
        midwayPose5 = new Pose2d(16,32, Math.toRadians(0)); // prep before grabbing sample 2
        parkPose = new Pose2d(2.5, -40, Math.toRadians(0));

        /**
         * For Specimen Preload Scoring and then 3 Samples
         **/
        if (startPosition == RRAutoSample.START_POSITION.BLUE_SAMPLES ||
                startPosition == RRAutoSample.START_POSITION.RED_SAMPLES) {

            // Raise Arm To score Specimen
            if(opModeIsActive()) {
                robot.elbowMotor.setPower(1);
                robot.elbowMotor.setTargetPosition((int) robot.ELBOW_SCORE_SAMPLE_IN_LOW);
                // TODO: Add code to release the sample and lower the arm
            }

            // Drive to scoring position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
                            .build());


            //scoring the specimen
            if(opModeIsActive()) {
                mechOps.specimenScoring();
                safeWaitSeconds(.25);
                mechOps.clawOpen();
            }

            //move to prep pose and then to sample 1
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose4.position, midwayPose4.heading)
                            .strafeToLinearHeading(yellowSample1Position.position, yellowSample1Position.heading)
                            .build());


            //grab Sample1 and move arm up
            if(opModeIsActive()){
                robot.elbowMotor.setPower(1);
                robot.elbowMotor.setTargetPosition(robot.ELBOW_RESET);
                safeWaitSeconds(.75);
                mechOps.clawClose();
                safeWaitSeconds(.2);
                robot.elbowMotor.setPower(1);
                robot.elbowMotor.setTargetPosition((int) robot.ELBOW_SCORE_SAMPLE_IN_LOW);
            }

            //move to midway before extending up
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .build());


            //extend up to score
            if (opModeIsActive()){
                mechOps.scoreHighBasket();
                safeWaitSeconds(.5);
            }


            //move to scoring position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                            .build());

            // Release the sample into the basket
            if(opModeIsActive()) {
                mechOps.clawOpen();
                safeWaitSeconds(.15);
            }

            // Drive to prep position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .build());

            //reset arm
            if(opModeIsActive()) {
                mechOps.extendArm((int) robot.EXTENSION_COLLAPSED);
                robot.elbowMotor.setTargetPosition(robot.ELBOW_TRAVERSE);
                safeWaitSeconds(.2);
            }


            // Drive to pick up Sample2 Position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose5.position, midwayPose5.heading)
                            .strafeToLinearHeading(yellowSample2Position.position, yellowSample2Position.heading)
                            .build());


            // Pick up Sample2
            if(opModeIsActive()) {
                robot.elbowMotor.setPower(1);
                robot.elbowMotor.setTargetPosition((int) robot.ELBOW_RESET);
                safeWaitSeconds(.6);
                mechOps.clawClose();
                safeWaitSeconds(0.3);
                robot.elbowMotor.setPower(1);
                robot.elbowMotor.setTargetPosition((int) robot.ELBOW_SCORE_SAMPLE_IN_LOW);
            }


            // Drive to prep position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .build());


            // Raise arm to high basket scoring position
            if(opModeIsActive()) {
                mechOps.scoreHighBasket();
                safeWaitSeconds(.5);
            }

            // Drive to scoring position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                            .build());

            // Release sample2 into the basket
            if(opModeIsActive()) {
                mechOps.clawOpen();
                safeWaitSeconds(.15);
            }

            // Drive to prep position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .build());

            //reset arm position
            if(opModeIsActive()) {
                mechOps.extendArm((int) robot.EXTENSION_COLLAPSED);
                robot.elbowMotor.setTargetPosition(robot.ELBOW_TRAVERSE);
            }

            //Drive to prep for third sample
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose3.position, midwayPose3.heading)
                            .build());

            //extension for third sample
            if(opModeIsActive()) {
                mechOps.rotateClaw( robot.WRIST_FOLDED_PARTIAL);
                robot.elbowMotor.setPower(1);
                robot.extendMotor.setPower(1);
                robot.elbowMotor.setTargetPosition((int) robot.ELBOW_ANGLE_AUTON);
                robot.extendMotor.setTargetPosition(robot.EXTENSION_DOWN_MAX);
                mechOps.clawOpen();
            }

            //drive to third sample position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(yellowSample3Position.position, yellowSample3Position.heading)
                            .build());


            // Pick up Sample3
            if(opModeIsActive()) {
                robot.elbowMotor.setPower(.25);
                robot.elbowMotor.setTargetPosition((int) robot.ELBOW_CLEAR_BARRIER);
                safeWaitSeconds(.5);
                mechOps.clawClose();
                safeWaitSeconds(.25);
                robot.extendMotor.setPower(1);
                robot.extendMotor.setTargetPosition((int)robot.EXTENSION_COLLAPSED);
                robot.elbowMotor.setTargetPosition(robot.ELBOW_TRAVERSE);
            }


            // Raise Arm to high basket scoring position and wrist folding out
            if(opModeIsActive()) {
                mechOps.rotateClaw(robot.WRIST_FOLDED_OUT);
                robot.elbowMotor.setPower(1);
                robot.elbowMotor.setTargetPosition((int) robot.ELBOW_SCORE_SAMPLE_IN_LOW);
            }

            //move to prep for scoring
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .build());

            //extend up
            if(opModeIsActive()) {
                mechOps.scoreHighBasket();
                safeWaitSeconds(.5);
            }

            //move in to score
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                            .build());

            // Release the sample into the basket
            if(opModeIsActive()) {
                mechOps.clawOpen();
            }

            // Drive to prep position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .build());


            //reset arm
            if(opModeIsActive()) {
                mechOps.extendArm((int) robot.EXTENSION_COLLAPSED);
                mechOps.resetArm();
            }


            //Park
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(parkPose.position, parkPose.heading)
                            .build());

        }

        //end of if (startPosition == BLUE_SAMPLES || RED_SAMPLES)

        /**
         *  For Specimen Scoring onto high bar
         **/
        if (startPosition == RRAutoSample.START_POSITION.BLUE_SPECIMENS ||
                startPosition == RRAutoSample.START_POSITION.RED_SPECIMENS) {

            // Raise Arm to high basket scoring position
            if(opModeIsActive()){
                robot.elbowMotor.setTargetPosition(robot.ELBOW_TRAVERSE);
                // TODO: Add code to raise claw to high basket
            }

            // Drive to scoring position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
                            .build());

            // Release the sample into the basket
            // Lower the arm
            if(opModeIsActive()) {
                // TODO: Add code to release the sample and lower the arm
            }

            // Drive to color sample1 Position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                            .strafeToLinearHeading(coloredSample1Position.position, coloredSample1Position.heading)
                            .build());

            // Push Color Sample1 into the Observation area
            // Drive to color sample1 Position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
                            .build());

            // Grab the specimen
            if(opModeIsActive()) {
                // TODO: Add code to grab the specimen from the observation area (from the floor)
            }

            // Raise Arm to high basket scoring position
            if(opModeIsActive()) {
                // TODO: Add code to raise claw to specimen high bar
            }

            // Drive to specimen scoring position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
                            .build());

            // Score the specimen on the high bar
            // Lower the arm
            if(opModeIsActive()) {
                // TODO: Add code to score the specimen
            }

            // Drive to Color Sample2 Position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                            .strafeToLinearHeading(coloredSample2Position.position, coloredSample2Position.heading)
                            .build());

            // Push Color Sample1 into the Observation area
            // Drive to color sample1 Position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
                            .build());

            // Grab the specimen
            if(opModeIsActive()) {
                // TODO: Add code to grab the specimen from the observation area (from the floor)
            }

            // Raise Arm to high basket scoring position
            if(opModeIsActive()) {
                // TODO: Add code to raise claw to specimen high bar
            }

            // Drive to specimen scoring position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
                            .build());

            // Score the specimen on the high bar
            // Lower the arm
            if(opModeIsActive()) {
                // TODO: Add code to score the specimen
            }

            // Drive to colored Sample3 Position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                            .strafeToLinearHeading(coloredSample3Position.position, coloredSample3Position.heading)
                            .build());

            // Push Color Sample3 into the Observation area
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
                            .build());

            // Grab the specimen
            if(opModeIsActive()) {
                // TODO: Add code to grab the specimen from the observation area (from the floor)
            }

            // Raise Arm to high basket scoring position
            if(opModeIsActive()) {
                // TODO: Add code to raise claw to specimen high bar
            }

            // Drive to specimen scoring position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
                            .build());

            // Score the specimen on the high bar
            // Lower the arm
            if(opModeIsActive()) {
                // TODO: Add code to score the specimen
            }

            // Park
            if(opModeIsActive()) {
                // TODO: Add code to park
                // set claw and motors into correct position
            }

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(parkPose.position, parkPose.heading)
                            .build());

        }

    }

    /**
     *
     */

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        RRAutoSample.State setupConfig = RRAutoSample.State.START_POSITION;
        Boolean menuActive = true;

        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested() && menuActive){
            switch(setupConfig){
                case START_POSITION:
                    telemetry.addData("Initializing Autonomous:",
                            TEAM_NAME, " ", TEAM_NUMBER);
                    telemetry.addData("---------------------------------------","");
                    telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
                    telemetry.addData("    Blue Specimen + 3 Samples   ", "(X / ▢)");
                    telemetry.addData("    Red Specimen + 3 Samples    ", "(B / O)");
                    if(gamepad1.x){
                        startPosition = RRAutoSample.START_POSITION.BLUE_SAMPLES;
                        menuActive = false;
                    }
                    if(gamepad1.b){
                        startPosition = RRAutoSample.START_POSITION.RED_SAMPLES;
                        menuActive = false;
                    }
                    telemetry.update();
                    break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    public enum State {
        START_POSITION,
        PARK_POSITION
    }

}   // end class

