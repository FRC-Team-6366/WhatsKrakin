package frc.entech.util;

import java.util.Map;
import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import java.io.Reader;
import java.lang.reflect.Type;

public class MapPresetReader {
    
    private MapPresetReader(){

    }
    public static Map<String,Double> readPresets(Reader input ){

        Gson gson = new Gson();
        // Define the type for Map<String, Double>
        Type type = new TypeToken<Map<String, Double>>() {}.getType();
        
        return gson.fromJson(input, type);    
    }
}
