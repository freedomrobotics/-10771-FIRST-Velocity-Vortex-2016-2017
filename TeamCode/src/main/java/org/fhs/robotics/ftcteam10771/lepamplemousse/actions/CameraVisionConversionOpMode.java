package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.UltrasonicRange;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.CameraVision;

/**
 * Created by joelv on 1/20/2017.
 */

public class CameraVisionConversionOpMode extends LinearOpMode {

    UltrasonicRange range;
    CameraVision cameraVision;

    @Override
    public void runOpMode() throws InterruptedException {
        cameraVision = new CameraVision();
        range = new UltrasonicRange(hardwareMap.analogInput.get("range"), hardwareMap.digitalChannel.get("switch"));
        Thread opThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!Thread.interrupted()){
                    cameraVision.runImageTracking();
                    range.streamDistance();
                }
            }
        });
        cameraVision.setUnitToRadians(false);
        waitForStart();
        opThread.start();
        double totalBase = 0.0;
        double totalRatio = 0.0;
        int iterations = 0;
        while (opModeIsActive()){
            iterations++;
            if (cameraVision.countTrackedImages()==1){
                cameraVision.setADetectedImageAsTarget();
            }
            telemetry.addData("Ultrasonic Distance", range.getDistance());
            telemetry.addData("Camera Vision Distance", cameraVision.getZ(cameraVision.getTargetedImage()));
            if (cameraVision.getTargetedImage() != CameraVision.Image.NULL && cameraVision.getZ(cameraVision.getTargetedImage())!=0f){
                totalRatio += getCameraConverted();
                totalBase  += getBase();
                telemetry.addData("U:C Distance Ratio", getCameraConverted());
                telemetry.addData("Average Ratio", totalRatio / iterations);
                telemetry.addData("Base", getBase());
                telemetry.addData("Average Base", totalBase / iterations);
            }
            telemetry.addData("Angle To Turn", cameraVision.getAngleToTurn(cameraVision.getTargetedImage()));
            telemetry.update();
        }
        opThread.interrupt();
    }

    public double getCameraConverted(){
        return range.getDistance() / cameraVision.getZ(cameraVision.getTargetedImage());
    }

    public double getBase(){
        return Math.pow(cameraVision.getX(cameraVision.getTargetedImage()), cameraVision.getZ(cameraVision.getTargetedImage()));
    }
}