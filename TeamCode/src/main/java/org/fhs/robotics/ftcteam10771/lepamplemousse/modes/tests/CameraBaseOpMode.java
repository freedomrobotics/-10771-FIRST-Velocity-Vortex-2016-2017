package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.CameraVision;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Coordinate;

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
            if (cameraVision.imageInSight()){
                telemetry.addData("Image", cameraVision.target().getName());
                telemetry.addData("X", cameraVision.updateCoordinates().getX());
                telemetry.addData("Y", cameraVision.updateCoordinates().getY());
            }
            else{
                telemetry.addData("Image", "NULL");
            }
            telemetry.update();
        }
        cameraVision.cameraThread.interrupt();
    }
}
