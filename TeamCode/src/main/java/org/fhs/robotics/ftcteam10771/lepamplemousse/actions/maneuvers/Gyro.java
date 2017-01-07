package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by joelv on 1/7/2017.
 */

public class Gyro {

    GyroSensor gyro = null;

    Gyro(){

    }

    Gyro(GyroSensor gyroSensor){
        gyro = gyroSensor;
    }

    public int raw(String axis){
        if (gyro!=null){
            if (axis=="X") gyro.rawX();
            else if (axis=="Y") gyro.rawY();
            else if (axis=="Z") gyro.rawZ();
            else return 404;
        }
        return 0;
    }
}
