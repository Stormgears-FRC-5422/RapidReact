package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.utils.configfile.StormProp;
import frc.utils.data.StormStruct;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashMap;
import java.util.Vector;

public class Vision extends SubsystemBase {
    private NetworkTable m_visionDataTable;
    private NetworkTableEntry m_visionTargetEntry;
    private NetworkTableEntry m_tgt_img_size_entry;
    private final String m_baseTableName = "vision_data";
    private final String m_structName = "opencv_tgt";
    private final String m_targetDataEntryName = "target_data";
    private final byte[] m_defaultVisionData = new byte[0];
    private final Number[] m_img_size;
    private NetworkTableInstance m_nt_instance;
    private StormStruct m_stormStruct;    

    public Vision() {
        m_nt_instance = NetworkTableInstance.getDefault();

        m_visionDataTable = m_nt_instance.getTable(m_baseTableName + "/binary_data/" + m_structName);
        m_visionTargetEntry = m_visionDataTable.getEntry(m_targetDataEntryName);
        m_tgt_img_size_entry = m_nt_instance.getEntry(m_baseTableName + "target_frame_size");
 
        Number default_data[] = {320.0, 240.0};
        m_img_size = m_tgt_img_size_entry.getNumberArray(default_data);

        m_stormStruct = new StormStruct(m_nt_instance,m_baseTableName,m_structName);

    }

    public void periodic() {
        if (m_stormStruct.get_size() == 0) {
            // Try and initialize again
            m_stormStruct = new StormStruct(m_nt_instance,m_baseTableName,m_structName);
        }
    }


    public double get_target_offset_degrees() {
        SmartDashboard.putNumber("Turret Offset (degrees)",get_target_offset()*53);

        return(get_target_offset()*53); // 53 degrees field of view in HD3000 camera

    }

    public double get_target_offset() {
        byte[] rawData = m_visionTargetEntry.getRaw(m_defaultVisionData);

        Vector<HashMap<String,Integer>> list = m_stormStruct.unpack(rawData);

        if (m_stormStruct.get_size() == 0) {
            System.out.println("StormStruct is not defined!");
            return(0);
        }

        if (!list.isEmpty()) {
            // Assume first object is the target
            HashMap<String,Integer> target = list.get(0);
            if (target.containsKey("x")) {
                int x = target.get("x");
                //System.out.println("Vision: x=" + x);
                double offset = (x - m_img_size[0].intValue() / 2.0) / m_img_size[0].intValue(); // Range -.5 -> .5
                return (offset);
            }
            else {
                System.out.println("Target has no key x. Dump raw data: length=" + rawData.length);
                for (int i=0; i<rawData.length; ++i)
                    System.out.println(rawData[i]);

                return 0.0;
            }
        }
        return(0.0);
    }   

    public double get_target_distance() {
        byte[] rawData = m_visionTargetEntry.getRaw(m_defaultVisionData);
        Vector<HashMap<String,Integer>> list = m_stormStruct.unpack(rawData);
        if (!list.isEmpty()) {
            if (list.get(0).containsKey("distance")) {
                return(list.get(0).get("distance")/list.get(0).get("float_scaling")); // Distance is scaled up on the python side since only ints are passed in binary struct
            }
        }

        return(-1.0);
    }
        
    public int get_target_heading() {
        byte[] rawData = m_visionTargetEntry.getRaw(m_defaultVisionData);
        Vector<HashMap<String,Integer>> list = m_stormStruct.unpack(rawData);
        if (!list.isEmpty()) {
            if (list.get(0).containsKey("heading")) {
                return(list.get(0).get("heading")); // Distance is scaled up by 10 on the python side since it is an int
            }
        }

        return(0);
    }
}