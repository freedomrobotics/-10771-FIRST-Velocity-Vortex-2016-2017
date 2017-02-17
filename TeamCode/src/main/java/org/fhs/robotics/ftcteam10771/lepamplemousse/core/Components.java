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

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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
 * <b>Deprecated for now. Use the direct hardware map and define map names in the settings file</b>
 */
@Deprecated
public class Components {

    HardwareMap hardwareMap = null;

    Map<String, Object> components = null;

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
    public Components(HardwareMap hardwareMap, Telemetry telemetry, Config components) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.components = components.getRawMap();
    }

    /**
     * Initializes all of the components
     *
     * @return The ReturnValue of the function
     */
    public ReturnValues initialize() {
        failures.clear();
        Core.motor = new DcMotor[count(mappedType(hardwareMap.dcMotor))];
        if (objectInit(hardwareMap.dcMotor, Core.motor) == ReturnValues.FAIL) {
            failures.add(ReturnValues.MOTOR_NOT_INIT);
        }
        Core.servo = new Servo[count(mappedType(hardwareMap.servo))];
        if (objectInit(hardwareMap.servo, Core.servo) == ReturnValues.FAIL) {
            failures.add(ReturnValues.SERVO_NOT_INIT);
        }
        Core.touchSensor = new TouchSensor[count(mappedType(hardwareMap.touchSensor))];
        if (objectInit(hardwareMap.touchSensor, Core.touchSensor) == ReturnValues.FAIL) {
            failures.add(ReturnValues.TOUCHSENSOR_NOT_INIT);
        }
        Core.lightSensor = new LightSensor[count(mappedType(hardwareMap.lightSensor))];
        if (objectInit(hardwareMap.lightSensor, Core.lightSensor) == ReturnValues.FAIL) {
            failures.add(ReturnValues.LIGHTSENSOR_NOT_INIT);
        }
        Core.colorSensor = new ColorSensor[count(mappedType(hardwareMap.colorSensor))];
        if (objectInit(hardwareMap.colorSensor, Core.colorSensor) == ReturnValues.FAIL) {
            failures.add(ReturnValues.COLORSENSOR_NOT_INIT);
        }
        Core.irSeeker = new IrSeekerSensor[count(mappedType(hardwareMap.irSeekerSensor))];
        if (objectInit(hardwareMap.irSeekerSensor, Core.irSeeker) == ReturnValues.FAIL) {
            failures.add(ReturnValues.IRSEEKER_NOT_INIT);
        }
        Core.gyrometer = new GyroSensor[count(mappedType(hardwareMap.gyroSensor))];
        if (objectInit(hardwareMap.gyroSensor, Core.gyrometer) == ReturnValues.FAIL) {
            failures.add(ReturnValues.GYROMETER_NOT_INIT);
        }
        Core.accelerometer = new AccelerationSensor[count(mappedType(hardwareMap.accelerationSensor))];
        if (objectInit(hardwareMap.accelerationSensor, Core.accelerometer) == ReturnValues.FAIL) {
            failures.add(ReturnValues.ACCELEROMETER_NOT_INIT);
        }
        Core.camera = new Camera[count("camera", "camera")];
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
        if (deviceExists(deviceType)) {
            for (int i = 0; i < count(deviceType); i++) {
                int id = i + 1;
                int max = maxSubdevices(deviceType);
                while ((!(deviceEnabled(deviceType, id)) && (id <= max))) {
                    id++;
                }
                if (deviceEnabled(deviceType, id) && componentExists(getMapName(deviceType, id), deviceMapping)) {
                    devices[i] = deviceMapping.get(getMapName(deviceType, id));
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
        if (deviceExists(device)) {
            for (int i = 0; i < count(device, device); i++) {
                int id = i + 1;
                int max = maxSubdevices(device);
                while ((!(deviceEnabled(device, device, id)) && (id <= max))) {
                    id++;
                }
                if (deviceEnabled(device, device, id)) {
                    Map<String, Object> TcameraObj = getSubdevice(device, device, id);
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
        if (deviceExists(device)) {
            for (int i = 0; i < count(device, device); i++) {
                int id = i + 1;
                int max = maxSubdevices(device);
                while ((!(deviceEnabled(device, device, id)) && (id <= max))) {
                    id++;
                }
                if (deviceEnabled(device, device, id)) {
                    Map<String, Object> TcameraObj = getSubdevice(device, device, id);
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
            Aliases.put(getAlias(deviceType, configId), Core.motor[deviceId]);
        }
        if (deviceType.equals("servos")) {
            Aliases.put(getAlias(deviceType, configId), Core.servo[deviceId]);
        }
        if (deviceType.equals("touch_sensors")) {
            Aliases.put(getAlias(deviceType, configId), Core.touchSensor[deviceId]);
        }
        if (deviceType.equals("light_sensors")) {
            Aliases.put(getAlias(deviceType, configId), Core.lightSensor[deviceId]);
        }
        if (deviceType.equals("color_sensors")) {
            Aliases.put(getAlias(deviceType, configId), Core.colorSensor[deviceId]);
        }
        if (deviceType.equals("ir_seekers")) {
            Aliases.put(getAlias(deviceType, configId), Core.irSeeker[deviceId]);
        }
        if (deviceType.equals("gyrometers")) {
            Aliases.put(getAlias(deviceType, configId), Core.gyrometer[deviceId]);
        }
        if (deviceType.equals("accelerometers")) {
            Aliases.put(getAlias(deviceType, configId), Core.accelerometer[deviceId]);
        }
        if (deviceType.equals("camera")) {
            Aliases.put(getAlias(deviceType, configId), Core.camera[deviceId]);
        }
    }

    /**
     * @return The list of failures that occured during initialization
     */
    public List<ReturnValues> getFailures() {
        return failures;
    }

    //region Retrieval Code

    //region Device Types / Overarching Device Methods

    /**
     * Checks the existance of an overarching device/device category given it's device type
     *
     * @param deviceType The name of hte overarching device
     * @return a boolean of the device's existence
     */
    private boolean deviceExists(String deviceType) {
        return components.containsKey(deviceType);
    }

    /**
     * Gets the map object associated with a device type
     *
     * @param deviceType The name of the device type (e.g. dc_motors)
     * @return A map object associated with the deviceType
     */
    private Map getDevice(String deviceType) {
        if (deviceExists(deviceType)) {
            return (Map) components.get(deviceType);
        }
        return null;
    }
    //endregion

    //region Device Name / Subdevice Methods

    /**
     * Checks whether a subdevice exists or not depending on the given custom name
     *
     * @param deviceType The name of the overarching device
     * @param deviceName The name of the subdevice
     * @param id         The id of the device
     * @return An boolean of the subdevice's existence
     */
    private boolean subDeviceExists(String deviceType, String deviceName, Integer id) {
        Map device;
        if ((device = getDevice(deviceType)) != null) {
            return device.containsKey(deviceName + id);
        }
        return false;
    }

    /**
     * Checks whether a subdevice exists or not depending on the given type and id. Uses the rule that the device name is either the same or without an extra s
     *
     * @param deviceType The name of the overarching device
     * @param id         The id of the device
     * @return An boolean of the subdevice's existence
     */
    private boolean subDeviceExists(String deviceType, Integer id) {
        return subDeviceExists(deviceType, deviceName(deviceType), id);
    }

    /**
     * Retrieves a subdevice based on a custom name
     *
     * @param deviceType The name of the overarching device
     * @param deviceName The name of the subdevice
     * @param id         The id of the device
     * @return A map object associated with the subdevice
     */
    private Map getSubdevice(String deviceType, String deviceName, Integer id) {
        Map device = getDevice(deviceType);
        if (subDeviceExists(deviceType, deviceName, id)) {
            return (Map) device.get(deviceName + id);
        }
        return null;
    }

    /**
     * Retrieves a subdevice based on the rule that the device name is either the same or without an extra s
     *
     * @param deviceType The name of the overarching device
     * @param id         The id of the device
     * @return An boolean of the subdevice's existence
     */
    private Map getSubdevice(String deviceType, Integer id) {
        return getSubdevice(deviceType, deviceName(deviceType), id);
    }

    /**
     * Checks if device is deviceEnabled or not given a custom name
     *
     * @param deviceType The name of the overarching device
     * @param deviceName The name of the subdevice
     * @param id         The id of the device
     * @return An boolean of whether the subdevice is deviceEnabled or not
     */
    private boolean deviceEnabled(String deviceType, String deviceName, Integer id) {
        Map device;
        if ((device = getSubdevice(deviceType, deviceName, id)) != null) {
            return device.get("enabled").equals(true);
        }
        return false;
    }

    /**
     * Checks if device is deviceEnabled or not based on the rule that the device name is either the same or without an extra s
     *
     * @param deviceType The name of the overarching device
     * @param id         The id of the device
     * @return An boolean of whether the subdevice is deviceEnabled or not
     */
    private boolean deviceEnabled(String deviceType, Integer id) {
        return deviceEnabled(deviceType, deviceName(deviceType), id);
    }

    /**
     * Retrieves the hardware_map name of a subdevice given a custom name
     *
     * @param deviceType The name of the overarching device
     * @param deviceName Dhe name of the subdevice
     * @param id         The id of the device
     * @return The name of the hardware map / the map name
     */
    private String getMapName(String deviceType, String deviceName, Integer id) {
        Map device;
        if ((device = getSubdevice(deviceType, deviceName, id)) != null) {
            if (device.get("map_name") != null)
                return device.get("map_name").toString();
        }
        return null;
    }

    /**
     * Retrieves the hardware_map name of a subdevice based on the rule that the device name is either the same or without an extra s
     *
     * @param deviceType The name of the overarching device
     * @param id         The id of the device
     * @return The name of the hardware map / the map name
     */
    private String getMapName(String deviceType, Integer id) {
        return getMapName(deviceType, deviceName(deviceType), id);
    }

    /**
     * Counts the subdevices matching a device name in a device types
     *
     * @param deviceType The name of the overarching device
     * @param deviceName The name of the device
     * @return An integer count of the number of subdevices matching the device name
     */
    private Integer count(String deviceType, String deviceName) {
        int quantity = 0;
        Map device;
        if ((device = getDevice(deviceType)) != null) {
            for (int i = 1; i <= device.size(); i++) {
                if (deviceEnabled(deviceType, deviceName, i)) {
                    quantity++;
                }
            }
        }
        return quantity;
    }

    /**
     * Counts the subdevices in a device type based on the rule that the device name is either the same or without an extra s
     *
     * @param deviceType The name of the overarching device
     * @return An integer count of the number of subdevices matching the device name
     */
    private Integer count(String deviceType) {
        return count(deviceType, deviceName(deviceType));
    }

    /**
     * Returns the total count of Subdevices in a device type
     *
     * @param deviceType The name of the overarchin device
     * @return An integer count of the total number of subdevices in the device type
     */
    private Integer maxSubdevices(String deviceType) {
        Map device;
        if ((device = getDevice(deviceType)) != null) {
            return device.size();
        } else return 0;
    }

    /**
     * Returns a list of the aliases in a subdevice including the map_name given a custom name
     *
     * @param deviceType The name of the overarching device
     * @param deviceName The name of the device
     * @param id         The id of the device
     * @return A list object of the aliases
     */
    private List<String> getAlias(String deviceType, String deviceName, Integer id) {
        Map device;
        List<String> alias = null;
        if ((device = getSubdevice(deviceType, deviceName, id)) != null) {
            if ((device.get("alias")) != null) {
                alias = (List<String>) device.get("alias");
                if (getMapName(deviceType, deviceName, id) != null)
                    alias.add(getMapName(deviceType, deviceName, id));
            }
        }
        return alias;
    }

    /**
     * Returns a list of the aliases in a subdevice including the map_name based on the rule that the device name is either the same or without an extra s
     *
     * @param deviceType The name of the overarching device
     * @param id         The id of the device
     * @return A list object of the aliases
     */
    private List<String> getAlias(String deviceType, Integer id) {
        return getAlias(deviceType, deviceName(deviceType), id);
    }
    //endregion

    //region Misc Methods

    /**
     * Returns the general device/subdevice name
     *
     * @param deviceType The name of the overarching device
     * @return The name of the device
     */
    private String deviceName(String deviceType) {
        String deviceName = deviceType;
        if (deviceType.charAt(deviceType.length() - 1) == 's')
            deviceName = deviceType.substring(0, deviceType.length() - 1);
        return deviceName;
    }

    /**
     * Returns the device/subdevice name and it's id
     *
     * @param deviceType The name of the overarching device
     * @param id         The id of the device
     * @return The name of the device
     */
    private String deviceName(String deviceType, Integer id) {
        return deviceName(deviceType) + id;
    }
    //endregion

    //endregion

    //endregion
}
