/*
package org.firstinspires.ftc.teamcode.libraries;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.CSAutoParams;
import org.firstinspires.ftc.teamcode.hardware.RRHWProfile;

public class mechanismsControlThread implements Runnable{

    private final double STALL_CURRENT = 10;              // Value to check for stalled motor
    private final double EJECT_TIMER = 0.450;            // time to allow to auto-eject pixels
    private final double AUTO_SHUTOFF_TIMER = 0.75;         // time to wait before shutting down intake motor after determining
    private final double BUCKET_BEAM_TIMER = 0.250;
    private RRHWProfile robot = null;
    private CSAutoParams params = null;
    private boolean threadIsRunning = true;              // controls when thread is killed
    private boolean intakeReverse = false;               //
    private boolean powerOn = false;
    private boolean kickDelaySet = false;
    private boolean timeOutSet = false;
    private boolean isDormant = false;
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime kickTime = new ElapsedTime();
    private ElapsedTime timeoutTimer = new ElapsedTime();
    private double kickDelay = 0.5;
    private boolean bottomBeamFlag = false;
    private boolean topBeamFlag = false;
    private ElapsedTime topBeamTime = new ElapsedTime();
    private boolean bucketIsFull = false;
    private PIDController controller=null;


    public int target=0, offset=0;

    public mechanismsControlThread(RRHWProfile myRobot){
        this.robot = myRobot;
        this.params = new CSAutoParams();

        controller=new PIDController(params.P,params.I,params.D);
    }

    public void setLiftTarget(int ticks, int bottomOffset){
        target=ticks;
        offset=bottomOffset;
    }

    public void setIntakeOn(){
        kickTime.reset();
        powerOn = true;
    }

    public void setIntakeOff(){
        powerOn = false;
    }

    public void setKickDelay(double kickDelay){
        if(powerOn) {
            this.kickDelay = kickDelay;
            this.kickDelaySet = true;
            kickTime.reset();
        }
    }



    public void setDormant (boolean dormancy){
        this.isDormant = isDormant;
    }

    private void controlMechs(){
        if(robot.opModeTeleop) {
//            teleop_runTo(target, offset);
        } else if(!robot.opModeTeleop){
//            auto_runTo(target);
        }

    }

    public void stopThread(){
        threadIsRunning = false;
    }

    @Override
    public void run() {
        while (threadIsRunning){
            if (!isDormant) {
                controlMechs();
            }
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }   // end of try - catch
        }   // end of while(isRunning)
    }
}
*/