package commands;

import parameters.MovePCommandParamerer;
import properties.Point3DParameterProperty;
import units.AbstractCommand;
import units.AbstractCommandEx;
import units.AbstractCommandParameter;
import units.AbstractCommandRet;

public class MoveP extends AbstractCommandEx {

    @Override
    public AbstractCommand CreateCommand() {
        return new MoveP();
    }

    @Override
    public AbstractCommandRet Execute(AbstractCommandParameter parameter) {

    	// Parameter check.
        if(null == parameter || null == parameter.getClass() 
        || false == parameter.getClass().getSimpleName().contains("MovePCommandParamerer")) {
            System.out.println("[WARNING] Input parameter is null!");
            return null;
        }
        
        MovePCommandParamerer basicCommandParameter = (MovePCommandParamerer)parameter;
        if(false == basicCommandParameter.IsSecurity()) {
        	System.out.println("[WARNING] The parameter command is not safe, " +
        			"ignore this execution and report to the relevant department.");
        	return null;
        }

        Point3DParameterProperty originPoint = 
        		(Point3DParameterProperty)basicCommandParameter.GetProperty("originPoint");
        Point3DParameterProperty targetPoint =
        		(Point3DParameterProperty)basicCommandParameter.GetProperty("targetPoint");
        
        System.out.println("[INFO] MoveP.originPoint " + originPoint.GetParameterPoint3DObject());
        System.out.println("[INFO] MoveP.targetPoint " + targetPoint.GetParameterPoint3DObject());
        System.out.println("[INFO] Execute MoveP");

        return null;
    }

    @Override
    public String GetNameString() {
        return "MoveP";
    }
    
}
