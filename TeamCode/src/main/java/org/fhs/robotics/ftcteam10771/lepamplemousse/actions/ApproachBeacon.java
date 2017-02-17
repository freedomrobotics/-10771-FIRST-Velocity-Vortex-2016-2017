package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;

/**
 * Created by Adam Li on 2/17/2017.
 * Derived from the CameraDriveOp class
 */

public class ApproachBeacon {
    private final Config.ParsedData settings;
    private final Drive drive;
    private VectorR driveVector;

    public ApproachBeacon(Config.ParsedData settings, Drive drive){
        this.drive = drive;
        this.settings = settings;
        this.driveVector = drive.getVectorR();
    }
}
