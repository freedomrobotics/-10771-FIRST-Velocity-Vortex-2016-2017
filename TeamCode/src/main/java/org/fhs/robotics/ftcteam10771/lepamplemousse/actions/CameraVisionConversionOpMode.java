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
        waitForStart();
        opThread.start();
        while (opModeIsActive()){
            if (cameraVision.countTrackedImages()==1){
                cameraVision.setADetectedImageAsTarget();
            }
            telemetry.addData("Ultrasonic Distance", range.getDistance());
            telemetry.addData("Camera Vision Distance", cameraVision.getZ(cameraVision.getTargetedImage()));
            if (cameraVision.getZ(cameraVision.getTargetedImage())!=0f){
                telemetry.addData("U:C Distance Ratio", range.getDistance() / cameraVision.getZ(cameraVision.getTargetedImage()));
            }
            telemetry.update();
        }
        opThread.interrupt();
    }
}
