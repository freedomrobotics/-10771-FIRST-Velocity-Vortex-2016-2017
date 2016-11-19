package org.fhs.robotics.ftcteam10771.lepamplemousse.config;

import android.os.Environment;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.yaml.snakeyaml.Yaml;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Created by Adam Li on 11/17/2016.
 * rewrite of configuration files and management.
 *
 * todo: Add ability to set values in the ParsedData object and the convert a whole ParsedData object tree back into a Map.
 */
public class Config {

    /** The location of the "root" directory */
    protected String dataPath = Environment.getExternalStorageDirectory().toString() + "/Grapefruit";
    /** The parent directory as a File object */
    private File parentDirectory = null;
    /** The config file as a File Object */
    private File configFile = null;
    /** Whether or not sdcard is writable */
    static protected boolean fsWrite = false;


    /** YAML file parser */
    private Yaml yaml = new Yaml();
    /** Data Map. Key is string id, Value is the object held */
    private Map<String, Object> data = null;
    /** ParsedData object containing processed data */
    private ParsedData parsedData = null;

    /** To debug or not to debug */
    private boolean debug = true;
    /** NOT USED NOR IMPLEMENTED. Flag to debug to log file */
    private boolean logOutput = false;

    private String parentPath;
    private String filename;
    private final Telemetry telemetry;
    private final String id;

    /**
     * Enumeration of various specific return values.
     */
    public enum State {
        FAILED, FILE_EXISTS, DEFAULT_EXISTS, MISSING_FILEPATH, SUCCESS
    }

    /**
     * Class which contains a HashMap and defines various functions to quickly retrieve values
     */
    public class ParsedData {
        Map<String, Object> data = new HashMap<String, Object>();

        /**
         * Default constructor for the object which contains a HashMap and various functions to quickly retrieve values
         */
        public ParsedData() {

        }

        /**
         * Cosntructor that copies an existing map to the ParsedData Object
         *
         * @param map The map to copy
         */
        public ParsedData(Map<String, Object> map) {
            if (map != null) {
                data.putAll(map);
            }
        }

        /**
         * Adds a value to the ParsedData object
         *
         * @param key    The key associated with the object
         * @param object The value to store
         */
        public void addToMap(String key, Object object) {
            data.remove(key);
            if (key != null) {
                data.put(key, object);
            }
        }

        /**
         * Copies an existing map to the ParsedData Object
         *
         * @param map The map to copy
         */
        public void addToMap(Map<String, Object> map) {
            if (map != null) {
                data.putAll(map);
            }
        }

        /**
         * Retrieves the object associated with the key
         *
         * @param key The key to retrieve the object from
         * @return The object associated with the key
         */
        public Object get(String key) {
            if (valueExists(key)) {
                return data.get(key);
            }
            return null;
        }

        /**
         * Removes the object associated with the key
         *
         * @param key The key to remove the object association
         */
        public void remove(String key) {
            data.remove(key);
        }

        /**
         * Checks to see if a key exists
         *
         * @param key The key to check for
         * @return A boolean stating whether the key exists or not
         */
        public boolean valueExists(String key) {
            return data.containsKey(key);
        }

        /**
         * Retrieves the object associated with the key
         *
         * @param key The key to retrieve the object from
         * @return The object associated with the key
         */
        public Object getObject(String key) {
            return get(key);
        }

        /**
         * Retrieves the boolean associated with the key
         *
         * @param key The key to retrieve the boolean from
         * @return The boolean associated with the key
         */
        public boolean getBool(String key) {
            return get(key).toString().equals("true");
        }

        /**
         * Retrieves the string associated with the key
         *
         * @param key The key to retrieve the string from
         * @return The string associated with the key
         */
        public String getString(String key) {
            return get(key).toString();
        }

        /**
         * Retrieves the integer associated with the key
         *
         * @param key The key to retrieve the integer from
         * @return The integer associated with the key
         */
        public int getInt(String key) {
            if (valueExists(key)) {
                Pattern p = Pattern.compile("[a-zA-Z]");
                Matcher m = p.matcher(data.get(key).toString());
                if (m.find()) {
                    return 0;
                }
                if (data.get(key).toString().contains(".")) {
                    return ((Double) data.get(key)).intValue();
                }
                return (Integer) data.get(key);
            }
            return 0;
        }

        /**
         * Retrieves the float associated with the key
         *
         * @param key The key to retrieve the float from
         * @return The float associated with the key
         */
        public float getFloat(String key) {
            if (valueExists(key)) {
                Pattern p = Pattern.compile("[a-zA-Z]");
                Matcher m = p.matcher(data.get(key).toString());
                if (m.find()) {
                    return 0;
                }
                if (!data.get(key).toString().contains(".")) {
                    return ((Integer) data.get(key)).floatValue();
                }
                return ((Double) data.get(key)).floatValue();
            }
            return 0;
        }

        /**
         * Retrieves the ParsedData object associated with the key
         *
         * @param key The key to retrieve the ParsedData object from
         * @return The ParsedData object associated with the key
         */
        public ParsedData subData(String key) {
            if (valueExists(key)) {
                Pattern p = Pattern.compile("[\\{\\}]");
                Matcher m = p.matcher(data.get(key).toString());
                if (m.find()) {
                    return (ParsedData) data.get(key);
                }
            }
            return null;
        }

        @Override
        public String toString() {
            return data.toString();
        }
    }

    /**
     * Non-verbose constructor to create an empty config object.
     * Outputs to given telemetry
     */
    public Config() {
        this(null, null, null, null);
    }

    /**
     * Verbose constructor to create an empty config object.
     * Outputs to given telemetry
     *
     * @param telemetry Telemetry output to driver station
     * @param id String to use to identify instance output
     */
    public Config(Telemetry telemetry, String id) {
        this(null, null, telemetry, id);
    }

    /**
     * Non-verbose constructor to create config object pointing
     * to a file relative to a sdcard root.
     *
     * @param filename Complete filename
     */
    public Config(String filename) {
        this(null, filename, null, null);
    }

    /**
     * Non-verbose constructor to create config object pointing
     * to a file relative to a parent directory, which is relative
     * to sdcard root.
     *
     * @param parentPath Path to parent directory from "root" directory
     * @param filename Complete filename
     */
    public Config(String parentPath, String filename) {
        this(parentPath, filename, null, null);
    }

    /**
     * Verbose constructor to create config object pointing
     * to a file relative to a sdcard root.
     *
     * @param filename Complete filename
     * @param telemetry Telemetry output to driver station
     * @param id String to use to identify instance output
     */
    public Config(String filename, Telemetry telemetry, String id) {
        this(null, filename, telemetry, id);
    }

    /**
     * Non-verbose constructor to create config object pointing
     * to a file relative to a parent directory, which is relative
     * to sdcard root.
     *
     * @param parentPath Path to parent directory from "root" directory
     * @param filename Complete filename
     * @param telemetry Telemetry output to driver station
     * @param id String to use to identify instance output
     */
    public Config(String parentPath, String filename, Telemetry telemetry, String id) {
        this.telemetry = telemetry;
        this.id = id;
        this.parentPath = parentPath;
        this.filename = filename;

        //Disable debug flag regardless of original setting if incomplete information is provided.
        if (telemetry == null || id == null)
            debug = false;

        //Check for write permissions
        if (!fsWrite) {
            fsWrite = Environment.getExternalStorageDirectory().canWrite();
            if (debug) telemetry.addData("FS-Write", fsWrite);
        }

        //Check and prepare parent dirctory
        if (this.parentPath != null && !this.parentPath.equals("")){
            parentDirectory = new File(dataPath + this.parentPath);
            if (parentDirectory.exists()) {
                if (debug) telemetry.addData(this.parentPath, "exists");
            } else {
                if (debug) telemetry.addData(this.parentPath, "does not exist... creating...");

                if (parentDirectory.mkdirs()) {
                    if (debug) telemetry.addData(this.parentPath, "created successfully");
                } else {
                    if (debug) telemetry.addData(this.parentPath, "failed to create");
                }
            }
        }

        // Set parent directory to root if undefined.
        if (parentDirectory == null)
            parentDirectory = new File(dataPath);

        //Check and prepare file
        //If file does not exist, attempt to create it with defaults.
        if (this.filename != null && !this.filename.equals("")){
            configFile = new File(parentDirectory, this.filename);
            if (configFile.exists()) {
                telemetry.addData("File-" + this.id, "exists");
            } else {
                telemetry.addData("File-" + this.id, "does not exist... creating with defaults...");
                if (create(true) == State.SUCCESS && fsWrite) {
                    telemetry.addData("File-"+id, "created successfully");
                } else {
                    telemetry.addData("File-"+id, "failed to create...");
                }
            }
        }
    }

    /**
     * Reads the file if it exists
     *
     * @return A {@link State} value regarding different things
     * <li><b>State.SUCCESS:</b> The file was read successfully.
     * <li><b>State.DEFAULT_EXISTS:</b> The file was not read. The default file
     * exists, but was not read.
     * <li><b>State.FAILED:</b> Neither the actual nor the default file exist
     * and nothing was read.
     */
    public State read() {
        return read(false);
    }

    /**
     * Reads the file if it exists. If true is given, the default file is read instead.
     *
     * @param useDefault Whether or not to read the default file.
     *
     * @return A {@link State} value regarding different things
     * <li><b>State.SUCCESS:</b> The file chosen was read successfully.
     * <li><b>State.DEFAULT_EXISTS:</b> The file was not read for some reason. The
     * default file exists, but was not read.
     * <li><b>State.FILE_EXISTS:</b> The default file was not read for some reason.
     * The actual file exists, but was not read.
     * <li><b>State.FAILED:</b> Either neither the actual nor the default file exist
     * or nothing was read for whatever reason.
     */
    public State read(boolean useDefault) {
        InputStream config;
        data = null;


        if (useDefault) {
            try {
                config = FtcRobotControllerActivity.getGlobalAssets().open(filename);
                if (debug)
                    telemetry.addData("LoadFile-" + id, "default selected");
            } catch (IOException e) {
                if (debug)
                    telemetry.addData("LoadFile-" + id, "failed to read default");
                e.printStackTrace();
                if (configFile.exists())
                    return State.FILE_EXISTS;
                return State.FAILED;
            }
        } else {
            try {
                config = new FileInputStream(configFile);
            } catch (FileNotFoundException e) {
                e.printStackTrace();
                if (debug)
                    telemetry.addData("LoadFile-" + id, "failed to read file");
                try {
                    FtcRobotControllerActivity.getGlobalAssets().open(filename);
                    return State.DEFAULT_EXISTS;
                } catch (IOException a) {
                    return State.FAILED;
                }
            }
            telemetry.addData("LoadFile-" + id, "file selected");
        }


        if (config != null) {
            data = (Map<String,Object>) yaml.load(config);
            if (data != null) {
                if (debug)
                    telemetry.addData("LoadFile-" + id, (useDefault ? "default" : "file") + "loaded");
                return State.SUCCESS;
            } else {
                if (debug)
                    telemetry.addData("LoadFile-" + id, "failed to load");
                return State.FAILED;
            }
        }
        return State.FAILED;
    }

    /**
     * Parses the raw data obtained from reading the config file.
     *
     * @return ParsedData object
     */
    public ParsedData parse() {
        parsedData = parse(data);
        return parsedData;
    }

    /**
     * Processes a Map to a ParsedData object and returns it.
     *
     * @param dataMap Data to process
     * @return The processed data as a ParsedData object
     */
    public ParsedData parse(Map<String, Object> dataMap) {
        //Processed data
        ParsedData processed = new ParsedData();
        //Ternary operation to make sure entrySet does not throw NullPointerException.
        Set<Map.Entry<String, Object>> dataSet = (dataMap != null ? dataMap.entrySet() : new HashMap<String,Object>().entrySet());

        //Retrieve the iterator and the entry associated with the iteration
        for (Object o : dataSet) {
            Map.Entry settingObject = (Map.Entry) o;
            //Send it to the recursive function to process it all.
            processed.addToMap(settingObject.getKey().toString(), iterateOverValues(settingObject.getValue()));
        }
        return processed;
    }

    /**
     * Used to recursively process the whole map from the parse function.
     *
     * @param object The object to check as a Map object to convert to settings
     * @return An object of either the ParsedData object of the map or the original object
     * passed if not a map.
     */
    private Object iterateOverValues(Object object) {
        //Begin assembling the ParsedData
        ParsedData parsedData = new ParsedData();
        //Regular Expressions to determine if the Object is a Map or not
        Pattern p = Pattern.compile("[\\{\\}]");
        Matcher m = p.matcher(object.toString());
        if (m.find()) {
            //Iterate over the Map if it is
            for (Object o : ((Map) object).entrySet()) {
                Map.Entry valuesObject = (Map.Entry) o;
                //And continue the iteration
                parsedData.addToMap(valuesObject.getKey().toString(), iterateOverValues(valuesObject.getValue()));
            }
            return parsedData;
        }
        //Otherwise return the object back
        return object;
    }

    /**
     * Creates the config file. This will overwrite the existing config file if it exists.
     * If true is given, the default file is created.
     *
     * @return A {@link State} value regarding different things
     * <li><b>State.SUCCESS:</b> The file was created successfully.
     * <li><b>State.MISSING_FILEPATH:</b> There is no appropriate filepath.
     * <li><b>State.FAILED:</b> Something went wrong
     */
    public State create() {
        return create(false);
    }

    /**
     * Creates the config file. This will overwrite the existing config file if it exists.
     * If true is given, the default file is created.
     *
     * @param createDefault Whether or not to create the default file.
     *
     * @return A {@link State} value regarding different things
     * <li><b>State.SUCCESS:</b> The file was created successfully.
     * <li><b>State.MISSING_FILEPATH:</b> There is no appropriate filepath.
     * <li><b>State.FAILED:</b> Something went wrong
     */
    public State create(boolean createDefault) {
        if (configFile != null)
            return State.MISSING_FILEPATH;
        if (createDefault) {
            try {
                if (!configFile.isFile() && !configFile.createNewFile()) {
                    if (debug)
                        telemetry.addData("CreatedFile-" + id, "failed to create default");
                    return State.FAILED;
                }
                InputStream in = FtcRobotControllerActivity.getGlobalAssets().open(filename);
                OutputStream out = new FileOutputStream(configFile, false);

                byte[] buffer = new byte[1024];
                int readlen;
                while ((readlen = in.read(buffer)) != -1) {
                    out.write(buffer, 0, readlen);
                }
                in.close();
                out.flush();
                out.close();

                if (debug)
                    telemetry.addData("CreateFile-" + id, "default created");

                return State.SUCCESS;
            } catch (Exception e) {
                e.printStackTrace();
                if (debug)
                    telemetry.addData("CreatedFile-" + id, "failed to create default");
                return State.FAILED;
            }
        }
        try {
            FileWriter configWrite = new FileWriter(configFile);
            configWrite.write(yaml.dump(data));
            configWrite.flush();
            configWrite.close();
            if (debug)
                telemetry.addData("CreateFile-" + id, "created successfully");
            return State.SUCCESS;
        } catch (IOException e) {
            e.printStackTrace();
            if (debug)
                telemetry.addData("CreatedFile-" + id, "failed to create");
            return State.FAILED;
        }
    }

    // TODO: 11/18/2016 Actually make
    public State setDir(String parentPath) {
        this.parentPath = parentPath;
        parentDirectory = new File(dataPath + this.parentPath);
        if (!parentDirectory.exists()){
            parentDirectory.mkdirs();
        }
        configFile = new File(parentDirectory, filename);
        if (!configFile.exists())
            create(true);
        return State.SUCCESS;
    }

    // TODO: 11/18/2016 Actually make
    public State setFilename(String filename) {
        this.filename = filename;
        configFile = new File(parentDirectory, this.filename);
        if (!configFile.exists())
            create(true);
        return State.SUCCESS;
    }


    /**
     * Returns a the raw map of values read from the configuration file
     *
     * @return A Hashmap with key as String and values as Object
     */
    public Map<String, Object> getRawMap() {
        if (data != null) {
            return data;
        }
        return new HashMap<>();
    }

    /**
     * Returns the parsedData object. Contains the processed values
     *
     * @return A {@link ParsedData} object with data processed.
     */
    public ParsedData getParsedData() {
        if (parsedData != null) {
            return parsedData;
        }
        return new ParsedData();
    }

    //todo implement
    private void enableLogOutput(boolean logOutput) {
        this.logOutput = logOutput;
    }
}
