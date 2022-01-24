package utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Properties;
import java.util.Set;

public class StormProp {
    //TODO put config file in /home/lvuser
    // Maybe rename this file to something better
    private static final String path = "/home/lvuser/deploy";
    private static final String name = "config.properties";
    private static final String backUP = "config_backup.properties";
    private static final File configFile = new File(path, name);
    private static final HashMap<String, Double> m_number_map = new HashMap<>();
    private static final HashMap<String, Integer> m_int_map = new HashMap<>();
    private static final HashMap<String, Boolean> m_bool_map = new HashMap<>();
    private static final HashMap<String, String> m_string_map = new HashMap<>();
    private static String overrideName = null;
    private static File overrideConfigFile;
    private static Properties properties;
    private static Properties overrideProperties;
    private static boolean initialized = false;
    private static boolean overrideInit = false;

    public static void init() {
        properties = new Properties();
        FileInputStream inputStream = null;
        try {
            inputStream = new FileInputStream(configFile);
            properties.load(inputStream);
            System.out.println("LOADEDLOADEDLOADEDLOADEDLOADEDLOADEDLOADEDLOADEDLOADEDLOADEDLOADEDLOADEDLOADEDLOADEDLOADEDLOADEDLOADEDLOADEDLOADEDLOADED");
            System.out.println(properties.getProperty("override"));
        } catch (IOException e) {
            System.out.println("Using backup config file");
            try {
                inputStream = new FileInputStream(new File("/home/lvuser/deploy", backUP));
                properties.load(inputStream);
            } catch (IOException w) {
                System.out.println("Failed to find back up file");
            }

        } finally {
            if (inputStream != null) {
                try {
                    inputStream.close();
                } catch (IOException e) {
                    System.out.println("Error coming from config. This should not run");
                }
            }
        }
        initialized = true;
        if (!overrideInit) {
            overrideInit();
        }
    }

    public static void overrideInit() {
        overrideName = properties.getProperty("override");
        overrideConfigFile = new File(path, overrideName);
        overrideProperties = new Properties();
        FileInputStream OverrideInputStream = null;
        try {
            OverrideInputStream = new FileInputStream(overrideConfigFile);
            overrideProperties.load(OverrideInputStream);
        } catch (IOException e) {
            System.out.println("No override file detected");
        }
        if (OverrideInputStream != null) {
            try {
                OverrideInputStream.close();
            } catch (IOException e) {
                System.out.println("Error coming from override config. This should not run");
            }
        }
        overrideInit = true;

    }

    private static String getPropString(String key) {
        if (!initialized) {
            init();
        }
        if (!overrideInit) {
            overrideInit();
        }
        if (!overrideProperties.containsKey(key) && !properties.containsKey(key)) {
            return null;
        } else if (overrideProperties.containsKey(key)) {
            return overrideProperties.getProperty(key);
        } else {
            return properties.getProperty(key);
        }
    }

    public static String getString(String key, String defaultVal) {
        try {
            String ret_val;
            if (m_string_map.containsKey(key)) return (m_string_map.get(key));
            else if (getPropString(key) != null) {
                m_string_map.put(key, getPropString(key));
                return (m_string_map.get(key));
            } else {
                System.out.println("WARNING: default used for key " + key);
                return (defaultVal);
            }
        } catch (Exception e) {
            System.out.println("WARNING: default used for key " + key);
            return (defaultVal);
        }
    }

    public static double getNumber(String key, Double defaultVal) {
        try {
            if (m_number_map.containsKey(key)) return (m_number_map.get(key));
            else if (getPropString(key) != null) {
                m_number_map.put(key, Double.parseDouble(getPropString(key)));
                return (m_number_map.get(key));
            } else {
                System.out.println("WARNING: default used for key " + key);
                return (defaultVal);
            }
        } catch (Exception e) {
            System.out.println("WARNING: default used for key " + key);
            return (defaultVal);
        }
    }

    public static int getInt(String key, int defaultVal) {
        try {
            if (m_int_map.containsKey(key)) return (m_int_map.get(key));
            else if (getPropString(key) != null) {
                m_int_map.put(key, Integer.parseInt(getPropString(key)));
                return (m_int_map.get(key));
            } else {
                System.out.println("WARNING: default used for key " + key);
                return (defaultVal);
            }
        } catch (Exception e) {
            System.out.println("WARNING: default used for key " + key);
            return (defaultVal);
        }
    }

    public static boolean getBoolean(String key, Boolean defaultVal) {
        try {
            if (m_bool_map.containsKey(key)) return (m_bool_map.get(key));
            else if (getPropString(key) != null) {
                m_bool_map.put(key, getPropString(key).equalsIgnoreCase("true"));
                return (m_bool_map.get(key));
            } else {
                System.out.println("WARNING: default used for key " + key);
                return (defaultVal);
            }
        } catch (Exception e) {
            System.out.println("WARNING: default used for key " + key);
            return (defaultVal);
        }
    }

    public static void toSmartDashBoard() {
        if (!initialized) {
            init();
        }
        if (!overrideInit) {
            overrideInit();
        }
        String[] Blacklist = {"robotName", "hasNavX", "rearRightTalonId", "rearLeftTalonId", "frontRightTalonId", "frontLeftTalonId", "wheelRadius"};
        Set<String> keys = properties.stringPropertyNames();
        for (String key : keys) {
            if (!Arrays.asList(Blacklist).contains(key)) {
                SmartDashboard.putString(key, getPropString(key));
            }
        }
    }

    //updates properties object using values from SmartDashboard
    public static void updateProperties() {
        if (!initialized) {
            init();
        }
        Set<String> keys = properties.stringPropertyNames();
        for (String key : keys) {
            String str = SmartDashboard.getString(key, "MISSING");
            if (!str.equals("MISSING")) {
                overrideProperties.setProperty(key, str);
                if (m_int_map.containsKey(key)) m_int_map.put(key, Integer.parseInt(str));
                else if (m_int_map.containsKey(key)) m_number_map.put(key, Double.parseDouble(str));
                else if (m_bool_map.containsKey(key)) m_bool_map.put(key, str.equalsIgnoreCase("true"));
                else if (m_string_map.containsKey(key)) m_string_map.put(key, str);
            }
        }
    }
}