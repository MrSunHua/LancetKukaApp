package commands;

import parameters.BasicRobotCommandParameter;
import units.AbstractCommand;
import units.AbstractCommandEx;
import units.AbstractCommandParameter;
import units.AbstractCommandRet;
import static com.kuka.roboticsAPI.motionModel.HRCMotions.handGuiding;

public class FreeHandler extends AbstractCommandEx {

	@Override
	public String GetNameString() {
		
		return "FreeHandler";
	}

	@Override
	public AbstractCommand CreateCommand() {
		
		return new FreeHandler();
	}

	@Override
	public AbstractCommandRet Execute(AbstractCommandParameter parameter) {
		
        if(null == parameter || null == parameter.getClass() 
        || false == parameter.getClass().getSimpleName().contains("BasicRobotCommandParameter")) {
            System.out.println("[WARNING] Input parameter is null!");
            return null;
        }
        
        BasicRobotCommandParameter basicRobotCommandParameter = (BasicRobotCommandParameter)parameter;
        if(false == basicRobotCommandParameter.IsSecurity()) {
        	System.out.println("[WARNING] Input parameter is null!");
        	return null;
        }
        
        basicRobotCommandParameter.GetRoboticsObject().
            move(handGuiding().setJointVelocityLimit(1.0).setCartVelocityLimit(2500).setJointLimitViolationFreezesAll(false));
        
        System.out.println("[INFO] Execute FreeHandler");
		return null;
	}
}
