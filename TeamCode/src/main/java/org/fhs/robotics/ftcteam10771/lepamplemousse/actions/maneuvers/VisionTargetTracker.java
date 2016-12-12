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
    private int designatedImage;

    @Override
    public void runOpMode() throws InterruptedException{
        motorsInit();
        autoDrive = new AutoDrive(motorA, motorB, motorC, motorD);
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
        while (opModeIsActive()) {
            if (!opModeFinished){
                //Do things!!!!
                while(cameraVision.countTrackedImages()!=1){
                    search();
                }
                setDesignatedImage();
                //TODO: Find acceptable margins for parameters of each function
                faceImage(0.2);
                center(10);
                approach(650);
                iteration++;
            }
            opModeFinished = true;
        }
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
        //Use autodrive to configure some sort of drive pattern
    }

    /**
     * Move the robot left or right to center the image in the camera view
     */
    private void center(double valueMargin){
        //TODO: Find which direction to drive for each condition
        double margin = Math.abs(valueMargin);
        while (Math.abs(cameraVision.imageData[designatedImage].translation.get(0))>margin && opModeIsActive()){
            if (cameraVision.imageData[designatedImage].translation.get(0)>margin){
                autoDrive.drive(AutoDrive.Direction.LEFT, AutoDrive.Speed.SLOW);
            }
            if (cameraVision.imageData[designatedImage].translation.get(0)<-1 * margin){
                autoDrive.drive(AutoDrive.Direction.RIGHT, AutoDrive.Speed.SLOW);
            }
        }
        stop();
    }

    /**
     * Rotate the robot so that it is perpendicular to the image
     */
    private void faceImage(double valueMargin){
        double margin = Math.abs(valueMargin);
        while (Math.abs(cameraVision.imageData[designatedImage].translation.get(0))>margin && opModeIsActive()){
            if (cameraVision.imageData[designatedImage].degreesToTurn> margin){
                autoDrive.rotate(AutoDrive.Direction.CLOCKWISE, AutoDrive.Speed.SLOW);
            }
            if (cameraVision.imageData[designatedImage].degreesToTurn< margin * -1){
                autoDrive.rotate(AutoDrive.Direction.COUNTERCLOCKWISE, AutoDrive.Speed.SLOW);
            }
        }
        autoDrive.stop();
    }

    /**
     * Drive straight to approach the image
     */
    private void approach(double designatedDistance){
        double distanceToStop = Math.abs(designatedDistance);
        while(Math.abs(cameraVision.imageData[designatedImage].translation.get(2))>distanceToStop && opModeIsActive()) {
            autoDrive.drive(AutoDrive.Direction.FORWARDS, AutoDrive.Speed.SLOW);
        }
        autoDrive.stop();
    }

    private void setDesignatedImage(){
        for (int i=0; i < cameraVision.beacons.size(); i++){
            if (cameraVision.imageData[i].matrix != null){
                designatedImage = i;
            }
        }
    }
}
