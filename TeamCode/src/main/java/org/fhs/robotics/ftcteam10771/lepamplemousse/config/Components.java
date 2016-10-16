package org.fhs.robotics.ftcteam10771.lepamplemousse.config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Dynamic;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.ReturnValues;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;
import org.yaml.snakeyaml.Yaml;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.util.List;
import java.util.Map;

/**
 * Components file accessor for the everything
 * todo simplify and make some more merges with config.java
 * todo improve
 * todo make more standalone for plug and play capability into config app
 */
public class Components extends Config {

    // Filename with the suffix
    private String fileName = Static.configCompFileName + Static.configFileSufffix;
    // The actual file
    private File configFile = new File(configDirectory, fileName);
    // Does the file exist or no.
    private boolean fileExists = true;
    // variable for telemetry
    private Telemetry telemetry = null;
    // The YAML object.
    private Yaml yaml = new Yaml();
    // An object to string map.
    private Map<String, Object> data = null;

    /**
     * Debuggable Constructor
     *
     * @param telemetry Telemetry output for Debug
     */
    public Components(Telemetry telemetry) {
        //Calling the superclass' (Config.class) constructor.
        super(telemetry);
        this.telemetry = telemetry;
        init();
    }

    /**
     * Non-debuggable constructor
     */
    public Components() {
        //Calling the superclass' (Config.class) constructor.
        super();
        init();
    }

    /**
     * Identifies and locates the file and such.
     * Runs within the constructor
     */
    @Override
    protected void init() {
        // saves code by not typing true a billion times.
        fileExists = true;
        // if the configuration directory even exists, then check of the config exists
        if (Dynamic.configDirExists) {
            //region Debug block
            // If debug is deviceEnabled, verbose output
            if (Static.Debug && telemetry != null) {
                if (configFile.exists()) {
                    telemetry.addData("CompConfFile", "exists");
                } else {
                    telemetry.addData("CompConfFile", "does not exist... creating with defaults...");
                    if (createDefaults(fileName) && writable) {
                        telemetry.addData("CompConfFile", "created successfully");
                    } else {
                        telemetry.addData("CompConfFile", "failed to create... using defaults");
                        fileExists = false;
                    }
                }
            }
            //endregion
            else {
                // shortened logic
                if (!configFile.exists() && writable) {
                    if (!createDefaults(fileName)) {
                        fileExists = false;
                    }
                } else if (!configFile.exists()) {
                    fileExists = false;
                }
            }
        }
        // If config doesn't exists, obviously the file doesn't and the system can't be
        // writable, so just say it doesn't exist
        else {
            if (Static.Debug && telemetry != null)
                telemetry.addData("CompConfFile", "using defaults");
            fileExists = false;
        }
    }

    /**
     * Read the values from the configuration file.
     * Runs and passes the value from verify and load.
     *
     * @return True if file loaded; false if defaults loaded.
     */
    @Override
    public boolean read() {
        if (load()) {
            if (!verify()) {
                load(true);
                return false;
            }
            return true;
        }
        load(true);
        return false;
    }

    /**
     * Creates the configuration using the default or stored values.
     * If the configuration file exists, it will be replaced.
     *
     * @param useDefaults Whether or not to load the defaults
     * @param loadAfter   Whether or not to verify and load the file after replacing
     * @return Success Value based on the ReturnValues enumeration
     */
    @Override
    public ReturnValues create(boolean useDefaults, boolean loadAfter) {
        boolean result;
        if (useDefaults) {
            result = createDefaults(fileName);
            if (Static.Debug && telemetry != null) {
                if (result) {
                    telemetry.addData("CreatedCompConfFile", "default created");
                    fileExists = true;
                } else {
                    telemetry.addData("CreatedCompConfFile", "failed to create default");
                }
            }
        } else {
            try {
                FileWriter configWrite = new FileWriter(configFile);
                configWrite.write(yaml.dump(data));
                configWrite.flush();
                configWrite.close();
                if (Static.Debug && telemetry != null)
                    telemetry.addData("CreatedCompConfFile", "created successfully");
                result = fileExists = true;
            } catch (IOException e) {
                e.printStackTrace();
                if (Static.Debug && telemetry != null)
                    telemetry.addData("CreatedCompConfFile", "failed to create");
                result = false;
            }
        }
        if (!loadAfter) {
            return (result) ? ReturnValues.SUCCESS : ReturnValues.FAIL;     // Look up Ternary operator if you don't understand
        } else if (!result) {
            return ReturnValues.FAIL;
        } else {
            if (load()) {
                if (verify()) {
                    if (Static.Debug && telemetry != null)
                        telemetry.addData("LoadedCompConFile", "default loaded");
                    return ReturnValues.SUCCESS;
                } else {
                    if (Static.Debug && telemetry != null)
                        telemetry.addData("LoadedCompConFile", "failed to verify");
                    return ReturnValues.UNABLE_TO_VERIFY;
                }
            } else {
                if (Static.Debug && telemetry != null)
                    telemetry.addData("LoadedCompConFile", "failed to load");
                return ReturnValues.UNABLE_TO_LOAD;
            }
        }
    }

    /**
     * Verify the stored values. True if verified, false if not.
     * Will replace bad values with defaults
     *
     * @return Verify state
     */
    @Override
    public boolean verify() {
        // TODO: 11/28/2015 Add a range checker
        return true;
    }

    /**
     * Loads values.
     *
     * @param loadDefault Whether to load from defaults or file.
     * @return Loaded successfully or not
     */
    @Override
    public boolean load(boolean loadDefault) {
        InputStream config;
        data = null;
        if (loadDefault) {
            try {
                config = Dynamic.globalAssets.open(fileName);
                if (Static.Debug && telemetry != null)
                    telemetry.addData("LoadCompConfFile", "default selected");
            } catch (IOException e) {
                if (Static.Debug && telemetry != null)
                    telemetry.addData("LoadCompConfFile", "failed to load default");
                e.printStackTrace();
                return false;
            }
        } else if (fileExists) {
            try {
                config = new FileInputStream(configFile);
            } catch (FileNotFoundException e) {
                e.printStackTrace();
                if (Static.Debug && telemetry != null)
                    telemetry.addData("LoadCompConfFile", "file failed to load");
                return false;
            }
            if (Static.Debug && telemetry != null)
                telemetry.addData("LoadCompConfFile", "file selected");
        } else {
            return false;
        }
        if (config != null) {
            data = (Map<String, Object>) yaml.load(config);
            if (Static.Debug && telemetry != null)
                telemetry.addData("LoadCompConfFile", "failed to load");
            if (data != null) {
                if (Static.Debug && telemetry != null)
                    telemetry.addData("LoadCompConfFile", "loaded");
                return true;
            }
        }
        return false;
    }

    //region Getters and Setters

    /**
     * Retrieves the object associated with the string
     *
     * @param key The string to search and retrieve the object from
     * @return The object mapped to the string
     */
    @Override
    public Object retrieve(String key) {
        if (data != null) {
            return data.get(key);
        }
        return null;
    }

    /**
     * Stores or replaces the object associated with the string
     *
     * @param key    The string to search and store the object to
     * @param object The Object to store
     */
    @Override
    public boolean store(String key, Object object) {
        if (data != null) {
            data.put(key, object);
        }
        return false;
    }

    //region Retrieval Code

    //region Device Types / Overarching Device Methods

    /**
     * Checks the existance of an overarching device/device category given it's device type
     *
     * @param deviceType The name of hte overarching device
     * @return a boolean of the device's existence
     */
    public boolean deviceExists(String deviceType) {
        return data.containsKey(deviceType);
    }

    /**
     * Gets the map object associated with a device type
     *
     * @param deviceType The name of the device type (e.g. dc_motors)
     * @return A map object associated with the deviceType
     */
    public Map getDevice(String deviceType) {
        if (deviceExists(deviceType)) {
            return (Map) data.get(deviceType);
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
    public boolean subDeviceExists(String deviceType, String deviceName, Integer id) {
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
    public boolean subDeviceExists(String deviceType, Integer id) {
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
    public Map getSubdevice(String deviceType, String deviceName, Integer id) {
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
    public Map getSubdevice(String deviceType, Integer id) {
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
    public boolean deviceEnabled(String deviceType, String deviceName, Integer id) {
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
    public boolean deviceEnabled(String deviceType, Integer id) {
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
    public String getMapName(String deviceType, String deviceName, Integer id) {
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
    public String getMapName(String deviceType, Integer id) {
        return getMapName(deviceType, deviceName(deviceType), id);
    }

    /**
     * Counts the subdevices matching a device name in a device types
     *
     * @param deviceType The name of the overarching device
     * @param deviceName The name of the device
     * @return An integer count of the number of subdevices matching the device name
     */
    public Integer count(String deviceType, String deviceName) {
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
    public Integer count(String deviceType) {
        return count(deviceType, deviceName(deviceType));
    }

    /**
     * Returns the total count of Subdevices in a device type
     *
     * @param deviceType The name of the overarchin device
     * @return An integer count of the total number of subdevices in the device type
     */
    public Integer maxSubdevices(String deviceType) {
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
    public List<String> getAlias(String deviceType, String deviceName, Integer id) {
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
    public List<String> getAlias(String deviceType, Integer id) {
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
