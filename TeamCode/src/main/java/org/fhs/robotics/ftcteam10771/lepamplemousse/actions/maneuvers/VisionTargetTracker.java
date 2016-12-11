package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import android.os.Handler;
import android.os.Looper;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * Created by 84ven on 12/8/2016.
 */
@Autonomous (name = "Vision_Target")
public class VisionTargetTracker extends LinearOpMode {

    protected String teamColor = "red";
    private DcMotor motorA;
    private DcMotor motorB;
    private DcMotor motorC;
    private DcMotor motorD;
    CameraVision cameraVision;
    AutoDrive autoDrive;
    private boolean opModeFinished = false;
    public Integer iteration = 0;

    @Override
    public void runOpMode() throws InterruptedException{
        //motorsInit();
        //autoDrive = new AutoDrive(motorA, motorB, motorC, motorD);
        cameraVision = new CameraVision();
        cameraVision.vuforiaInit();
        waitForStart();
        Runnable r = new Runnable() {
            @Override
            public void run() {
                while(cameraVision.vuforiaRunning) {
                    cameraVision.runImageTracking(VisionTargetTracker.this);
                    telemetry.addData("Iterations", iteration);
                    telemetry.update();
                }
            }
        };
        Thread thread = new Thread(r);
        thread.start();
        do {
            if (!opModeFinished){
                //Do things!!!!
                iteration++;
            }
            opModeFinished = true;
        } while (opModeIsActive());
        cameraVision.vuforiaRunning = false;
        thread.interrupt();
    }

    /**
     * Initializes the motors
     */
    private void motorsInit(){
        motorA = hardwareMap.dcMotor.get("motorFrontRight"); //sets variable motorA to front right motor
        motorB = hardwareMap.dcMotor.get("motorFrontLeft"); //sets variable motorB to front left motor
        motorC = hardwareMap.dcMotor.get("motorBackLeft"); //sets variable motorC to back left motor
        motorD = hardwareMap.dcMotor.get("motorBackRight"); //sets variable motorD to back right mot
        motorA.setDirection(DcMotor.Direction.REVERSE);
        motorB.setDirection(DcMotor.Direction.FORWARD);
        motorC.setDirection(DcMotor.Direction.REVERSE);
        motorD.setDirection(DcMotor.Direction.FORWARD);
    }

    private void search(){
        while(cameraVision.countTrackedImages()!=1){
            //Use autodrive to configure some sort of drive pattern
        }
    }
}
