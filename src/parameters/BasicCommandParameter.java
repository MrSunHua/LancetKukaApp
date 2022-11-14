package parameters;

import java.util.List;
import java.util.Map;

import units.AbstractCommandParameter;
import units.AbstractCommandParameterEx;
import units.AbstractCommandParameterProperty;

public class BasicCommandParameter extends AbstractCommandParameterEx {

    public BasicCommandParameter() { }

    public BasicCommandParameter(String parame) {
        this.SetInputString(parame);
    }

    @Override
    public AbstractCommandParameter CreateCommandParameter(String parame, Map<String, Object> mapRuntimeProperties) {

        return new BasicCommandParameter(parame);
    }

    @Override
    public String GetErrorString() {
        return "Non";
    }

    @Override
    public List<AbstractCommandParameterProperty> CreateProperty(String parameter) {
        return null;
    }

	@Override
	public boolean IsSecurity() {
		return false;
	}    
}
