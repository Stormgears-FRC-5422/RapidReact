package frc.utils.data;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.sql.Struct;
import java.util.HashMap;
import java.util.Vector;

public class StormStruct {
    // Describes a data structure that has been published and described in network tables
    // Can unpack binary data once intialized
    private final String[] m_fieldNames;
    private final HashMap<String, Integer> m_fields; // key is field name, data is position index in binary data
    private final HashMap<String, Integer> m_sizes; // key is field name, data is byte size in binary data
    private int m_struct_size;
    private final int m_typeid;
    private final NetworkTableInstance m_ntinst;

    /**  
    * Populate HashMap fields and size by reading definition of struct_name from network tables (populated by struct provider)
    * 
    * @param nt_inst Network Tables instance
    * @param base_table table name where struct data is stored
    * @param struct_name name of struct being used
    */
    public StormStruct(final NetworkTableInstance nt_inst,final String base_table,final String struct_name) {
        this.m_ntinst = nt_inst;
        m_fields = new HashMap<String,Integer>();
        m_sizes = new HashMap<String,Integer>();
        m_struct_size = 0;

        NetworkTableEntry names_entry = nt_inst.getTable(base_table + "/structs/" + struct_name).getEntry("name");
        NetworkTableEntry sizes_entry = nt_inst.getTable(base_table + "/structs/" + struct_name).getEntry("size");
        NetworkTableEntry type_entry = nt_inst.getTable(base_table + "/structs/" + struct_name).getEntry("type");
        m_typeid = type_entry.getNumber(-1).intValue();
        
        String[] names = names_entry.getStringArray(new String[0]);
        Number[] sizes = sizes_entry.getNumberArray(new Number[0]);

        m_fieldNames = new String[names.length];

        for (int i=0;i<names.length; i++) {
            m_fields.put(names[i],i);
            m_sizes.put(names[i],sizes[i].intValue());
            m_struct_size += Math.abs(sizes[i].intValue());
            m_fieldNames[i] = names[i];
        }
    }

    /**
     * Get the data structure size
     * @return Size of data structure
     */
    public int get_size() {
        return(m_struct_size);
    }

        /**
         * Get the field name given the index
         * 
         * @param index of data item
         * @return Name of field
         */
    public String get_name_by_index(final int index) {
        for (final String field : m_fields.keySet()) {
            if (m_fields.get(field) == index) {
                return(field);
            }
        }
        return("");
    }

    /**
         * Unpack a binary byte stream using this structure definition
         * 
         * @param data_stream The binary data
         * @return Returns a HashMap of field,value pairs
         * 
    */
    public HashMap<String, Integer> decode_struct(final byte[] data_stream) {

        return(decode_struct(data_stream,0));
    }

    /**
     * Unpack an entire binary stream into a list of HashMaps
     * @param data_stream
     * @return List (vector) of hashmaps.  each hashmap represents a data structure being transferred
     */
    public Vector<HashMap<String, Integer>> unpack(final byte[] data_stream) {
        Vector<HashMap<String, Integer>> list;
        if (data_stream.length < 3) {
            //System.out.println("data received from Raspberry Pi is too short. Expect 3 or longer, but received " + data_stream.length + " bytes");
            return(new Vector<HashMap<String, Integer>>());
        }
        else {
            list = new Vector<HashMap<String, Integer>>();
            byte id = data_stream[0];
            int count = (data_stream[1] << 8) + data_stream[2];

            int offset = 3;
            for (int i=0; i< count; i++) {
                list.add(decode_struct(data_stream,offset));
                offset += get_size();  // Move pointer to next Struct
            }
            return(list);
        }
    }

    /**
         * Unpack a binary byte stream using this structure definition starting at a given offset
         * 
         * @param data_stream The binary data
         * @param _offset The offset in the binary data stream to extract from
         * @return Returns a HashMap of field,value pairs
         * 
    */
    private HashMap<String, Integer> decode_struct(final byte[] data_stream,final int _offset) {
        HashMap<String,Integer> ret_map = new HashMap<String,Integer>();
        // Data is big endian
        int offset = _offset;
        for (String field : m_fieldNames) {
            if (field.startsWith("_")) {
                //System.out.println("Unpack - skipping " + field);
                continue;
            }
            //System.out.println("unpack " + field);
            int data = 0;
            int size = Math.abs(m_sizes.get(field));
            boolean signed = (m_sizes.get(field) < 0);
            //System.out.println("unpack " + field + "; size=" + size);
            for (int i = 0; i < size; i++) {
                data = data << 8;           
                data |= data_stream[offset + i] & 0xFF;
            }

            // Extend sign on signed int. ints are 4 bytes in java
            if (signed && size < 4) {
                data = data << (4 - size);
                data = data >> (4 - size);
            }
            offset += size;
            ret_map.put(field,data);
        }
        return(ret_map);
    }

}
