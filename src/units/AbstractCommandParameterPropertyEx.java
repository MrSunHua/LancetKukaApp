package units;

/**
 * Command abstract parameter attribute extension class.
 * 
 * <p>
 * See the detailed description of the parent class.<br>
 * 
 * To handle the interface functions that are not necessarily handled by derived 
 * classes, because Java interface classes cannot declare interfaces other than 
 * public attributes, an abstract extension class was born.<br>
 * 
 * The user does not have to worry about the relationship between this class and 
 * the parent class of the abstract interface, or what unknown impact the parameter 
 * data class derived from this class will have. It clearly tells you that it will 
 * not have any impact on the extensibility, flexibility, etc. of the entity class.
 * </p>
 * 
 * @author Sun
 * @see units.AbstractCommandParameterProperty
 */
public abstract class AbstractCommandParameterPropertyEx implements AbstractCommandParameterProperty {

	protected Object property;
	protected String keyString;
	
	protected AbstractCommandParameterPropertyEx(String key) {
		this.SetKeyString(key);
	}
	
	protected void SetKeyString(String key) {
		this.keyString = key;
	}
	
    @Override
    public String GetKeyString() {
        return this.keyString;
    }

	@Override
	public Object GetParameterTypeObject() {

		return property;
	}

	@Override
	public void SetParameterTypeObject(Object o) {

		this.property = o;
	}
}