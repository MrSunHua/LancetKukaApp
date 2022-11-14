package parameters;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import properties.Point3DParameterProperty;

import units.AbstractCommandParameter;
import units.AbstractCommandParameterProperty;

public class MovePCommandParamerer extends BasicRobotCommandParameter {
	
    @Override
    public AbstractCommandParameter CreateCommandParameter(String parameter,
    		Map<String, Object> mapRuntimeProperties) {

        MovePCommandParamerer tempParameter = new MovePCommandParamerer();
        
        // Parse input parameters && Fill Parameters.
        List<AbstractCommandParameterProperty> listParameter = this.CreateProperty(parameter);
        if(null != listParameter) {
        	for(int index = 0; index < listParameter.size(); ++index) {
        		if(null != listParameter.get(index)) {
        			tempParameter.SetProperty(listParameter.get(index).GetKeyString(), listParameter.get(index));
        		}
        	}
        }
        
        // Configuring the runtime environment for command execution.
        tempParameter.UpdatesRoboticsProperties(mapRuntimeProperties);
        
        return tempParameter;
    }

    @Override
    public String GetErrorString() {
        return "Non";
    }

    @Override
    public List<AbstractCommandParameterProperty> CreateProperty(String parameter) {
        
    	List<AbstractCommandParameterProperty> listResult = new ArrayList<AbstractCommandParameterProperty>();
    	
    	String[] parameArray = parameter.split(",");
    	if(7 <= parameArray.length) {
    		// originPoint, Start point of point-to-point movement.
    		Point3DParameterProperty originPoint = new Point3DParameterProperty("originPoint");
    		originPoint.GetParameterPoint3DObject().SetPoint(
    				Double.parseDouble(parameArray[1]), // X
    				Double.parseDouble(parameArray[2]), // Y 
    				Double.parseDouble(parameArray[3]));// Z
    		
    		// targetPoint, End point of point-to-point movement.
    		Point3DParameterProperty targetPoint = new Point3DParameterProperty("targetPoint");
    		targetPoint.GetParameterPoint3DObject().SetPoint(
    				Double.parseDouble(parameArray[4]), // X
    				Double.parseDouble(parameArray[5]), // Y 
    				Double.parseDouble(parameArray[6]));// Z
    		
    		listResult.add(originPoint);
    		listResult.add(targetPoint);
    	}else {
    		System.out.println("[ERROR] Input MoveP parameter error!");
    	}
    	
    	
        return listResult;
    }

	@Override
	public boolean IsSecurity() {

		boolean bsecurity = true;
		
		// <3 Detect whether the command parameters meet.
		bsecurity &= super.IsSecurity();
		bsecurity &= this.IsVaild();
		bsecurity &= null != this.GetProperty("originPoint");
		bsecurity &= null != this.GetProperty("targetPoint");
		
		return bsecurity;
	}
}
