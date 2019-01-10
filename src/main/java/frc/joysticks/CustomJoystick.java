package frc.joysticks;

import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
/*
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class CustomJoystick {
    HashMap<String, JoystickAxis> axisMap = new HashMap<String, JoystickAxis>();
    HashMap<String, Integer> buttonsMap = new HashMap<String, Integer>();

    String fileName;
    public CustomJoystick(String fileName) {
        this.fileName = fileName;
        Object obj;
        try {
            obj = new JSONParser().parse(new FileReader("JSONExample.json"));
            parseButtonJSON((JSONObject) obj);
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
    }

    // store json data into 
    private void parseJSON() {
        
    }

    private void parseAxisJSON() {

    }
    private void parseButtonJSON(JSONObject obj) {
        JSONObject jsonObj = (JSONObject) obj;
        JSONObject axis = (JSONObject) jsonObj.get("axis");
        Iterator<String> iter = axis.entrySet().iterator();

        while (iter.hasNext()) {
            String key = iter.next();
            System.out.println(key);
        }
        
        
        
    }
}

class JoystickAxis {
    int portNumber;
    boolean isReversed;

    public JoystickAxis(int portNumber, boolean isReversed) {
        this.portNumber = portNumber;
        this.isReversed = isReversed;
    }
}
*/
class CustomJoystick {};