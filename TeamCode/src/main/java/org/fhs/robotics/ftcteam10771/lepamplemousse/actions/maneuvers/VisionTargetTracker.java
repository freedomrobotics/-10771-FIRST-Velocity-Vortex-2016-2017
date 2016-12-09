package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * Created by 84ven on 12/8/2016.
 */
@Autonomous (name = "Vision_Target")
public class VisionTargetTracker extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        CameraVision cameraVision = new CameraVision();
        cameraVision.vuforiaInit();
        while(opModeIsActive()){
            cameraVision.runImageTracking(this);
            telemetry.addData("Number of Images", cameraVision.countTrackedImages());
            telemetry.update();
        }
    }


}
