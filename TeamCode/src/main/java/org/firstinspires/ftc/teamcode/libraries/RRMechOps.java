package org.firstinspires.ftc.teamcode.libraries;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.CSAutoParams;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;

public class RRMechOps {

    public HWProfile robot;
    public LinearOpMode opMode;
    public CSAutoParams params;

    /*
     * Constructor
     */
    public RRMechOps(HWProfile myRobot, LinearOpMode myOpMode, CSAutoParams autoParams){
        robot = myRobot;
        opMode = myOpMode;
        params = autoParams;

    }   // close RRMechOps constructor

    /*
     * ###########################################################################################
     * ###########################################################################################
     * #######################      SYSTEM CONTROLS      #########################################
     * ###########################################################################################
     * ###########################################################################################
     */

    public void clawOpen(){

        robot.servoClaw.setPosition(robot.CLAW_OPEN);
    }

    public void poleTouch(){
        robot.poleToucherServo.setPosition(robot.POLE_UP);
    }

    public void rotateClaw(double targetClawPosition){
        robot.servoWrist.setPosition(targetClawPosition);
    }

    public void specimenScoring(){
        robot.elbowMotor.setTargetPosition((int) robot.ELBOW_SCORE_SPECIMEN);
    }

    public void clawClose(){
        robot.servoClaw.setPosition(robot.CLAW_CLOSED);    // TODO: create target position constant
    }

    public void clawPartial(){
        robot.servoClaw.setPosition(robot.CLAW_PARTIAL_OPEN);
    }
    public void scoreHighBasket(){
        // set arm position to up
        robot.elbowMotor.setPower(1);
        robot.elbowMotor.setTargetPosition((int) robot.ELBOW_SCORE_SAMPLE_IN_LOW);
        extendArm((int) robot.EXTENSION_SCORING_IN_HIGH_BASKET);
        //robot.armMotor.setPosition(params);

    }

    public void scoreLowBasket(){
        // set arm position to up
        robot.elbowMotor.setPower(1);

        // extend arm to full reach
        robot.elbowMotor.setPower((int) robot.ELBOW_HIGH_BASKET);
        //robot.armMotor.setPosition(params);
    }

    public void extendArm(int targetPosition){
        robot.extendMotor.setPower(1);
        robot.extendMotor.setTargetPosition(targetPosition);
    }

    public void resetArm(){
        // retract the arm
        robot.extendMotor.setPower(1);
        robot.extendMotor.setTargetPosition((int) robot.EXTENSION_COLLAPSED);

        robot.elbowMotor.setPower(1);
        robot.elbowMotor.setTargetPosition((int) robot.ELBOW_RESET);

        // set claw to reset position
        robot.servoClaw.setPosition(robot.CLAW_OPEN);
        robot.servoWrist.setPosition(robot.WRIST_FOLDED_OUT);
    }

    /***
     * This method provides a delay that will cancel if the program is stopped
     * @param time
     */
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();

        // wait while a stop is not requested
        while (!opMode.isStopRequested() && timer.time() < time) {
        }
    }

}   // close the RRMechOps class