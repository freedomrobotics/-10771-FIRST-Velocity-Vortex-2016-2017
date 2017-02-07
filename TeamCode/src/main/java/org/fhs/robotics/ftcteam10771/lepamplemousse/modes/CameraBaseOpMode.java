package org.fhs.robotics.ftcteam10771.lepamplemousse.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.CameraVision;

/**
 * Created by joelv on 1/28/2017.
 */
@Autonomous(name="CamCalc")
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
                cameraVision.setHighestIndexedImageAsTarget();
            }
            else cameraVision.setTargetImage(null);
            if (cameraVision.imageInSight()){
                telemetry.addData("Image", cameraVision.target().getName());
                telemetry.addData("Overall Distance", hypotenuse());
                telemetry.addData("Overall Angle", angle());
            }
            else{
                telemetry.addData("Image", "NULL");
            }
            telemetry.update();
        }
        cameraVision.cameraThread.interrupt();
    }

    public double hypotenuse(){
        double x = cameraVision.getX();
        double z = cameraVision.getZ();
        return Math.sqrt((x*x)+(z*z));
    }

    public double angle(){
        double wallAngle = -cameraVision.getAngleToTurn();
        double x = Math.abs(cameraVision.getZ());
        double y = -cameraVision.getX();
        double imageAngle = Math.toDegrees(Math.atan2(y, x));
        return 90.0 - wallAngle + imageAngle;
    }
}
