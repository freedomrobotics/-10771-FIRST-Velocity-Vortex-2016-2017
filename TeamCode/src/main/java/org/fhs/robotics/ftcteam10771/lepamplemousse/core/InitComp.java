package org.fhs.robotics.ftcteam10771.lepamplemousse.core;

import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Components;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.components.Aliases;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.components.Core;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.Camera;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.ColorGrid;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.ReturnValues;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 * Create the actual references to the components based on the configuration
 */
public class InitComp {

    HardwareMap hardwareMap = null;

    Components components = null;

    Telemetry telemetry = null;

    List<ReturnValues> failures = new LinkedList<ReturnValues>();

    //shh
    java.util.regex.Pattern p = java.util.regex.Pattern.compile("[a-zA-Z]");

    /**
     * Constructs the Components Initialization Object
     *
     * @param hardwareMap Reference to the hardwareMap of the OpMode
     * @param telemetry   Reference to the telemetry output for debug
     * @param components  Reference to the components configuration object
     */
    public InitComp(HardwareMap hardwareMap, Telemetry telemetry, Components components) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.components = components;
    }

    /**
     * Initializes all of the components
     *
     * @return The ReturnValue of the function
     */
    public ReturnValues initialize() {
        failures.clear();
        Core.motor = new DcMotor[components.count(mappedType(hardwareMap.dcMotor))];
        if (objectInit(hardwareMap.dcMotor, Core.motor) == ReturnValues.FAIL) {
            failures.add(ReturnValues.MOTOR_NOT_INIT);
        }
        Core.servo = new Servo[components.count(mappedType(hardwareMap.servo))];
        if (objectInit(hardwareMap.servo, Core.servo) == ReturnValues.FAIL) {
            failures.add(ReturnValues.SERVO_NOT_INIT);
        }
        Core.touchSensor = new TouchSensor[components.count(mappedType(hardwareMap.touchSensor))];
        if (objectInit(hardwareMap.touchSensor, Core.touchSensor) == ReturnValues.FAIL) {
            failures.add(ReturnValues.TOUCHSENSOR_NOT_INIT);
        }
        Core.lightSensor = new LightSensor[components.count(mappedType(hardwareMap.lightSensor))];
        if (objectInit(hardwareMap.lightSensor, Core.lightSensor) == ReturnValues.FAIL) {
            failures.add(ReturnValues.LIGHTSENSOR_NOT_INIT);
        }
        Core.colorSensor = new ColorSensor[components.count(mappedType(hardwareMap.colorSensor))];
        if (objectInit(hardwareMap.colorSensor, Core.colorSensor) == ReturnValues.FAIL) {
            failures.add(ReturnValues.COLORSENSOR_NOT_INIT);
        }
        Core.irSeeker = new IrSeekerSensor[components.count(mappedType(hardwareMap.irSeekerSensor))];
        if (objectInit(hardwareMap.irSeekerSensor, Core.irSeeker) == ReturnValues.FAIL) {
            failures.add(ReturnValues.IRSEEKER_NOT_INIT);
        }
        Core.gyrometer = new GyroSensor[components.count(mappedType(hardwareMap.gyroSensor))];
        if (objectInit(hardwareMap.gyroSensor, Core.gyrometer) == ReturnValues.FAIL) {
            failures.add(ReturnValues.GYROMETER_NOT_INIT);
        }
        Core.accelerometer = new AccelerationSensor[components.count(mappedType(hardwareMap.accelerationSensor))];
        if (objectInit(hardwareMap.accelerationSensor, Core.accelerometer) == ReturnValues.FAIL) {
            failures.add(ReturnValues.ACCELEROMETER_NOT_INIT);
        }
        Core.camera = new Camera[components.count("camera", "camera")];
        if (cameraInit(Core.camera) == ReturnValues.FAIL) {
            failures.add(ReturnValues.CAMERA_NOT_INIT);
        }
        if (failures.size() > 1) {
            return ReturnValues.MULTI_NOT_INIT;
        } else if (failures.size() == 1) {
            return failures.get(0);
        }
        return ReturnValues.SUCCESS;
    }

    //TODO: 12/7/2015 make each private variable in Components.java work
    //todo            in a way that makes a unique value for each device type

    /**
     * Initializes each individual object type
     *
     * @param devices The array to assign to
     * @return ReturnValues whether or not the method succeeded
     */
    private ReturnValues objectInit(HardwareMap.DeviceMapping deviceMapping, Object devices[]) {
        String deviceType = mappedType(deviceMapping);
        if (components.deviceExists(deviceType)) {
            for (int i = 0; i < components.count(deviceType); i++) {
                int id = i + 1;
                int max = components.maxSubdevices(deviceType);
                while ((!(components.deviceEnabled(deviceType, id)) && (id <= max))) {
                    id++;
                }
                if (components.deviceEnabled(deviceType, id) && componentExists(components.getMapName(deviceType, id), deviceMapping)) {
                    devices[i] = deviceMapping.get(components.getMapName(deviceType, id));
                    setAlias(deviceType, i, id);
                } else {
                    //Returns FAIL if there are not any existing enabled device keys to initialize the array elements with
                    return ReturnValues.FAIL;

                }
            }
            return ReturnValues.SUCCESS;
        }
        return ReturnValues.DEVICE_DOES_NOT_EXIST;
    }

    /**
     * Initializes camera
     *
     * @param devices The array to assign to
     * @return ReturnValues whether or not the method succeeded
     */
    //I'm cheap, don't bother with it. This entire class needs a rewrite.
    private ReturnValues cameraInit(Object devices[]) {
        String device = "camera";
        if (components.deviceExists(device)) {
            for (int i = 0; i < components.count(device, device); i++) {
                int id = i + 1;
                int max = components.maxSubdevices(device);
                while ((!(components.deviceEnabled(device, device, id)) && (id <= max))) {
                    id++;
                }
                if (components.deviceEnabled(device, device, id)) {
                    Map<String, Object> TcameraObj = components.getSubdevice(device, device, id);
                    String func = TcameraObj.get("function").toString();
                    if (func.equals("color_grid")){
                        if (TcameraObj.get("extra") == null) continue;
                        Map<String, Object> extraParam = (Map)TcameraObj.get("extra");
                        int gridX = 0, gridY = 0;
                        float refresh = 0.0f;
                        if (extraParam.get("grid_x") != null && extraParam.get("grid_y") != null){
                            //grid x + y
                            gridX = returnInt(extraParam.get("grid_x"));
                            gridY = returnInt(extraParam.get("grid_y"));
                        }
                        if (extraParam.get("grid_side") != null){
                            //grid_size
                            gridX = gridY = returnInt(extraParam.get("grid_side"));
                        }
                        if (extraParam.get("refresh_rate") != null){
                            //refresh rate
                            refresh = returnFloat(extraParam.get("refresh_rate"));
                        }
                        if (gridX <= 0 || refresh <= 0.0f){
                            continue;
                        }
                        devices[i] = new ColorGrid(gridX, gridY, refresh, hardwareMap.appContext);
                    }else{
                        continue;
                    }
                    setAlias(device, i, id);
                } else {
                    //Returns FAIL if there are not any existing enabled device keys to initialize the array elements with
                    return ReturnValues.FAIL;

                }
            }
            return ReturnValues.SUCCESS;
        }
        return ReturnValues.DEVICE_DOES_NOT_EXIST;
    }

    public void cleanUp(){
        stopAllCamera();
    }

    private void stopAllCamera(){
        String device = "camera";
        if (components.deviceExists(device)) {
            for (int i = 0; i < components.count(device, device); i++) {
                int id = i + 1;
                int max = components.maxSubdevices(device);
                while ((!(components.deviceEnabled(device, device, id)) && (id <= max))) {
                    id++;
                }
                if (components.deviceEnabled(device, device, id)) {
                    Map<String, Object> TcameraObj = components.getSubdevice(device, device, id);
                    String func = TcameraObj.get("function").toString();
                    if (func.equals("color_grid")){
                        ((ColorGrid) Core.camera[i]).closeGrid();
                        Core.camera[i].stopCamera();
                    }
                }
            }
        }
    }

    private Integer returnInt(Object o){
        java.util.regex.Matcher m = p.matcher(o.toString());
        if (m.find()) {
            return 0;
        }
        if (o.toString().contains(".")) {
            return ((Double) o).intValue();
        }
        return (Integer) o;
    }

    private Float returnFloat(Object o){
        java.util.regex.Matcher m = p.matcher(o.toString());
        if (m.find()) {
            return 0.0f;
        }
        if (!o.toString().contains(".")) {
            return ((Integer) o).floatValue();
        }
        return ((Double) o).floatValue();
    }

    //TODO: 12/14/2015 Change method to case statements or better yet, mappings(Map<Map, String>)(make DYNAMIC return values)
    //TODO: 12/14/2015 Figure out a way to make these comparisons under objectKey() function // FIXME: 12/15/2015 Moved to the components class by adam
    //Method for retrieving string from hardware maps
    private String mappedType(HardwareMap.DeviceMapping deviceMap) {
        if (deviceMap == hardwareMap.dcMotor) return "dc_motors";
        else if (deviceMap == hardwareMap.servo) return "servos";
        else if (deviceMap == hardwareMap.touchSensor) return "touch_sensors";
        else if (deviceMap == hardwareMap.lightSensor) return "light_sensors";
        else if (deviceMap == hardwareMap.colorSensor) return "color_sensors";
        else if (deviceMap == hardwareMap.irSeekerSensor) return "ir_seekers";
        else if (deviceMap == hardwareMap.gyroSensor) return "gyrometers";
        else if (deviceMap == hardwareMap.accelerationSensor) return "accelerometers";
            //(deviceMap for camera does not exist)
        else return null; //if no map matches any above return null as string
    }

    /**
     * A method derived from the HardwareMap.DeviceMapping Class to determine if a named component exists.
     *
     * @param mapName       The name of the mapping to check
     * @param deviceMapping The HardwareMap.DeviceMapping object used in the OpMode
     * @return A boolean stating whether the component exists or not.
     */
    private boolean componentExists(String mapName, HardwareMap.DeviceMapping deviceMapping) {
        // Now this is a little complex.
        // So I decompiled the HardwareMap class to figure this out since there is no built in way to check
        // First I call to get the entry set of the map in the class, which similar to a list of unique entries of map key/values
        // Then I call iterator on it to run through each entry.
        Iterator devices = deviceMapping.entrySet().iterator();
        // From here I say, while there is another entry in the set, do...
        while (devices.hasNext()) {
            // This! I pull out the entry from the iterator and run a check on it
            Map.Entry entry = (Map.Entry) devices.next();
            // If it's a HardwareDevice and the mapName and the key of the entry have the same content
            if (entry.getValue() instanceof HardwareDevice && mapName.equals(entry.getKey())) {
                // Then the component exists on the hardwareMap and we don't have to perform a non-remote robot restart because of annoying exceptions.
                return true;
            }
        }
        // Otherwise the component doesn't exist and we return a failure.
        return false;
    }

    /**
     * A method to store to the alias map. Figures out which aliases to get based on the rule that the device name is either the same or without an extra s
     *
     * @param deviceType The type of the device
     * @param deviceId   The id of the core object to the device
     * @param configId   The id of the device in the config
     */
    //CHEAP I know, but it works (I was being dumb with the old way, so that didn't work :/)
    private void setAlias(String deviceType, Integer deviceId, Integer configId) {
        if (deviceType.equals("dc_motors")) {
            Aliases.put(components.getAlias(deviceType, configId), Core.motor[deviceId]);
        }
        if (deviceType.equals("servos")) {
            Aliases.put(components.getAlias(deviceType, configId), Core.servo[deviceId]);
        }
        if (deviceType.equals("touch_sensors")) {
            Aliases.put(components.getAlias(deviceType, configId), Core.touchSensor[deviceId]);
        }
        if (deviceType.equals("light_sensors")) {
            Aliases.put(components.getAlias(deviceType, configId), Core.lightSensor[deviceId]);
        }
        if (deviceType.equals("color_sensors")) {
            Aliases.put(components.getAlias(deviceType, configId), Core.colorSensor[deviceId]);
        }
        if (deviceType.equals("ir_seekers")) {
            Aliases.put(components.getAlias(deviceType, configId), Core.irSeeker[deviceId]);
        }
        if (deviceType.equals("gyrometers")) {
            Aliases.put(components.getAlias(deviceType, configId), Core.gyrometer[deviceId]);
        }
        if (deviceType.equals("accelerometers")) {
            Aliases.put(components.getAlias(deviceType, configId), Core.accelerometer[deviceId]);
        }
        if (deviceType.equals("camera")) {
            Aliases.put(components.getAlias(deviceType, configId), Core.camera[deviceId]);
        }
    }

    /**
     * @return The list of failures that occured during initialization
     */
    public List<ReturnValues> getFailures() {
        return failures;
    }
}
