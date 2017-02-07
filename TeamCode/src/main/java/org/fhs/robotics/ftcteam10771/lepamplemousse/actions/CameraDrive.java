package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.CameraVision;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Coordinate;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Class that handles the robot's drive
 * in reference to the image it tracks
 * Created by joelv on 2/3/2017.
 */
public class CameraDrive {

    private CameraVision cameraVision;
    private Drive autoDrive;
    private Config.ParsedData settings;
    private boolean backCamera = true;

    public CameraDrive(CameraVision cameraVision1, Drive autoDrive1, Config.ParsedData settings1){
        cameraVision = cameraVision1;
        autoDrive = autoDrive1;
        settings = settings1;
        backCamera = (cameraVision.getCameraDirection()== VuforiaLocalizer.CameraDirection.BACK);
    }

    public void center() {
        float marginofError = settings.subData("drive").subData("camera_settings").getFloat("centering_margin");
        if (targeted()){
            while(Math.abs(cameraVision.getX())>marginofError) {
                boolean left = backCamera ? (cameraVision.getX()>0.0) : (cameraVision.getX()<0.0);
                float theta = left ? (float)Math.PI : 0.0f;
                float radius = (Coordinate.convertTo((float) Math.abs(cameraVision.getX()), Coordinate.UNIT.MM_TO_UNIT));
                radius = (Coordinate.convertTo(radius, Coordinate.UNIT.UNIT_TO_DM));
                autoDrive.setVelocity(radius, theta);
            }
        }
    }

    public void approach(){
        float distance_to_stop = settings.subData("drive").subData("camera_settings").getFloat("distance_to_stop");
        if (targeted()){
            while(Math.abs(cameraVision.getZ())>distance_to_stop){
                float theta = backCamera ? 3.0f*(float)Math.PI/2.0f : (float)Math.PI/2.0f;
                float radius = (Coordinate.convertTo((float) cameraVision.getZ(), Coordinate.UNIT.MM_TO_UNIT));
                radius = 0.5f * (Coordinate.convertTo(radius, Coordinate.UNIT.UNIT_TO_M));
                autoDrive.setVelocity(radius, theta);
            }
        }
    }

    public void rotate(){
        float rotate_margin = settings.subData("drive").subData("camera_settings").getFloat("angle_margin");
        if (targeted()){
            while(Math.abs(cameraVision.getAngleToTurn())>rotate_margin){
                autoDrive.setRoation((float)cameraVision.getAngleToTurn());
            }
        }
    }

    private boolean targeted(){
        return cameraVision.imageInSight(cameraVision.target());
    }
}
