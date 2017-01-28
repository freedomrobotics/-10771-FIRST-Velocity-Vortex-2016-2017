package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.CameraVision;

/**
 * Created by joelv on 1/28/2017.
 */
@Autonomous(name="BaseFinder")
public class CameraBaseOpMode extends LinearOpMode {

    CameraVision cameraVision;


    @Override
    public void runOpMode() throws InterruptedException {
        cameraVision = new CameraVision();
        //range = new UltrasonicRange2(hardwareMap.analogInput.get("range"), hardwareMap.digitalChannel.get("switch"));
        Thread opThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!Thread.interrupted()){
                    cameraVision.runImageTracking();
                    //range.streamDistance();
                }
            }
        });
        cameraVision.setUnitToRadians(false);
        waitForStart();

        //demo of feature I added that does what I think you hoped for the stream to do.
        //range.toggleStream(true);
        //range.setAveragingTime(100);

        opThread.start();
        double totalBase = 0.0;
        double totalRatio = 0.0;
        int iterations = 0;
        while (opModeIsActive()){
            if (cameraVision.countTrackedImages()==1){
                cameraVision.setADetectedImageAsTarget();
                iterations++;
            }
            else cameraVision.setTargetImage(null);
            telemetry.addData("Image", cameraVision.getTargetedImage().getName());
            telemetry.addData("Camera Vision Distance", cameraVision.getZ(cameraVision.getTargetedImage()));
            if (cameraVision.imageInSight(cameraVision.getTargetedImage()) && cameraVision.getZ(cameraVision.getTargetedImage())!=0f){
                totalBase  += getBase();
                telemetry.addData("Base", getBase());
                telemetry.addData("Average Base", totalBase / iterations);
            }
            telemetry.addData("Angle To Turn", cameraVision.getAngleToTurn(cameraVision.getTargetedImage()));
            telemetry.update();
        }
        opThread.interrupt();
    }

    public double getBase(){
        return Math.pow(cameraVision.getX(cameraVision.getTargetedImage()), 1.0 / (cameraVision.getZ(cameraVision.getTargetedImage())));
    }
}
