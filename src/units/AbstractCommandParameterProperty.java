package units;

/**
 * Command Abstract Parameter Attribute Class.
 * 
 * <p>
 * The parameter attribute class is one of the key points of the intervention 
 * model. All parameter objects or data are transferred to the model on the 
 * basis of it, and distributed to the corresponding places by the model.
 * <p>
 * 
 * @author Sun
 * @see units.AbstractCommandParameterPropertyEx
 */
public interface AbstractCommandParameterProperty {
 
	/**
	 * Get the Key value string.
	 * 
	 * <p>
	 * The Key value is used for the model management parameter data object. The 
	 * value is defined externally and can be used to operate the parameter data 
	 * object externally.<br>
	 * <b>Return Non by default.</b>
	 * </p>
	 * 
	 */
    public abstract String GetKeyString();

	/**
	 * Get the type name of the managed parameter data object.
	 * 
	 * <p>
	 * This interface is very useful and is often used for type safety check to 
	 * avoid causing Java memory exceptions.
	 * </p>
	 * 
	 * <p><b>Warning:</b>
	 * If the managed parameter data object is null or other exceptions occur, 
	 * the default value of the interface range is NON.
	 * </p>
	 * 
	 * <blockquote><pre>
     * AbstractCommandParameterProperty parameter = new MyCommandParameterProperty("my property");
     * parameter.SetParameterTypeObject(new Integer(999));
     * // usage method.
     * // if(parameter.GetParameterTypeName() == "Integer")
     * if(parameter.GetParameterTypeName().toLowerCase() == "integer") {
     *   System.out.println("Read parameter property of integer " + (Integer)parameter.GetParameterTypeObject());
     * }
     * </pre></blockquote>
	 */
    public abstract String GetParameterTypeName();

    public abstract Object GetParameterTypeObject();
    public abstract void SetParameterTypeObject(Object o);
}
