package parameters;

import java.util.List;
import java.util.Map;

import com.kuka.roboticsAPI.deviceModel.LBR;

import units.AbstractCommandParameter;
import units.AbstractCommandParameterEx;
import units.AbstractCommandParameterProperty;

public class BasicRobotCommandParameter extends AbstractCommandParameterEx {

	protected LBR lbr;
	
	@Override
	public boolean IsSecurity() {
		boolean bsecurity = true;
		
		// <1 Detection parameter validity flag.
		
		// <2 Whether the detection environment parameters meet the requirements.
		bsecurity &= this.GetRoboticsObject() != null;
		if(null != this.GetRoboticsObject()) {
			String lbrName = this.GetRoboticsObject().getClass().getSimpleName();
			bsecurity &= lbrName.toLowerCase().contains("lbrmedkg14");
		}else {
			// Parameter error of mechanical arm instance object.
			bsecurity = false;
		}
		
		// <3 Detect whether the command parameters meet.
		bsecurity &= false == this.GetInputString().isEmpty();
		
		return bsecurity;
	}

	@Override
	public AbstractCommandParameter CreateCommandParameter(String parame,
			Map<String, Object> mapRuntimeProperties) {

		BasicRobotCommandParameter tempParameter = new BasicRobotCommandParameter();
		tempParameter.UpdatesRoboticsProperties(mapRuntimeProperties);
        
		return tempParameter;
	}

	@Override
	public String GetErrorString() {
		return null;
	}

	@Override
	public List<AbstractCommandParameterProperty> CreateProperty(
			String parameter) {
		return null;
	}
	
    public LBR GetRoboticsObject() {
    	return this.lbr;
    }
    
    public void SetRoboticsObject(LBR plbr) {
    	this.lbr = plbr;
    }
    
    protected boolean UpdatesRoboticsProperties(Map<String, Object> mapRuntimeProperties) {

        // Configuring the runtime environment for command execution.
        Object o_lbr = mapRuntimeProperties.get("lbrmedkg14");
        if(null != o_lbr && null != o_lbr.getClass()) {
        	this.SetRoboticsObject((LBR)o_lbr);
        	return true;
        }
    	return false;
    }
}