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
        cameraVision.setUnitToRadians(false);
        waitForStart();
        cameraVision.cameraThread.start();
        double totalBase = 0.0;
        double totalRatio = 0.0;
        int iterations = 0;
        while (opModeIsActive()){
            if (cameraVision.countTrackedImages()==1){
                cameraVision.setADetectedImageAsTarget();
                iterations++;
            }
            else cameraVision.setTargetImage(null);
            if (cameraVision.imageInSight(cameraVision.getTargetedImage())){
                totalBase  += getBase();
                telemetry.addData("Base", getBase());
                telemetry.addData("Average Base", totalBase / iterations);
                telemetry.addData("Image", cameraVision.getTargetedImage().getName());
                telemetry.addData("Camera Vision Distance", cameraVision.getZ(cameraVision.getTargetedImage()));
            
            }
            else{
                totalBase  += getBase();
                telemetry.addData("Base", "null");
                telemetry.addData("Average Base", "null");
                telemetry.addData("Image", "null");
                telemetry.addData("Camera Vision Distance", "null");
            }
            telemetry.update();
        }
        cameraVision.cameraThread.interrupt();
    }

    public double getBase(){
        return cameraVision.getZ(cameraVision.getTargetedImage())!=0f ? Math.pow(cameraVision.getX(cameraVision.getTargetedImage()), 1.0 / (cameraVision.getZ(cameraVision.getTargetedImage()))) :
                0.0;
    }
}
