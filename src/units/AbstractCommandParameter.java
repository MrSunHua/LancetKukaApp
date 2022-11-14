package units;

import java.util.List;
import java.util.Map;

/**
 * Abstract command parameter class.
 * 
 * <p><br>
 * As the auxiliary class of all command execution classes, the abstract command 
 * parameter class should have all the parameters or running environment of the 
 * entity command class execution function.
 * <p>
 * 
 * <p><b>Command parameters:</b><br>
 * Command parameter refers to the command parameter object obtained by parsing 
 * the parameter string transmitted from the remote host.
 * <p>
 * 
 * <p><b>Operating environment:</b><br>
 * The running environment refers to the resources required for the execution of 
 * a command, such as mechanical arm objects, application objects, or other 
 * objects or data.
 * <p>
 * 
 * @since 1.0
 * 
 * @see java.util.Map
 * @see java.lang.String
 * @see units.AbstractCommandParameterEx
 * @see units.AbstractCommandParameterProperty
 */
public interface AbstractCommandParameter {

	/**
	 * Get/Set input string.
	 * 
	 * <p>
	 * This string should store the original format of the command.<br>
	 * foramt eg. [command: parameters].
	 * </p>
	 * 
	 * @author Sun
	 */
    public abstract String GetInputString();
    public abstract void SetInputString(String dataStream);
    
    /**
     * Whether the parameter class is safe internally
     * 
     * <p>
     * Different commands will monitor the security of different parameters. When 
     * the command needs to be executed, the interface will be called to determine 
     * whether the parameters are complete.
     * <p>
     * 
     * @return True security, otherwise false.
     */
    public abstract boolean IsSecurity();
    
    /**
     * Create command parameter class interface.
     * 
     * <p>
     * Create the target command parameter class object. The derived class should 
     * override this interface, which will directly affect the correspondence 
     * between the derived command parameter class and the entity command class.
     * </p>
     * 
     * <p><b>explain:</b><br>
     * The target user of this interface is the autonomous running model object. In 
     * order to facilitate the creation of the target command parameter object by 
     * using dynamic, encapsulation and inheritance methods for the autonomous 
     * running model object, and push the target command parameter object to the 
     * processing plant.
     * </p>
     * 
     * @param parame Target command in string representation.
     * @param mapRuntimeProperties All running environment instance objects.
     * 
     * <blockquote><pre>
     * AbstractCommandParameter parameter = new MyCommandParameter();
     * parameter.CreateCommandParameter("MoveP,1.0,2.0,3.0,4.0,5.0,6.0", new Map<String, Object>());
     * </pre></blockquote>
     * 
     * @return Parameter class entity object of the target command.
     */
    public abstract AbstractCommandParameter CreateCommandParameter(String parame,
    		Map<String, Object> mapRuntimeProperties);

    /**
     * Returns the validity property of the current command parameter class.
     * 
     * <p>
     * When the validity attribute is <b>true</b>, it means that the internal 
     * operation of the parameter class is normal, and the external can be used 
     * with confidence.<br><br>
     * When the validity attribute is <b>false</b>, it indicates that there is 
     * an error in the internal operation of the parameter class. For details, 
     * refer to the error interface.
     * </p>
     * 
     * <blockquote><pre>
     * AbstractCommandParameter parameter = new MyCommandParameter();
     * if(false == parameter.IsVaild()) {
     *     System.out.printIn("error string: " + param.GetErrorString());
     * }
     * </pre></blockquote>
     * 
     * @see units.AbstractCommandParameter#GetErrorString()
     */
    public abstract boolean IsVaild();

    /**
     * Get the internal detailed error information string.
     * 
     * @return java.lang.String
     * 
     * @see units.AbstractCommandParameter#IsVaild()
     */
    public abstract String GetErrorString();

    /**
     * Get the number of abstract parameter object elements.
     * 
     * @return A value of 0 indicates that there is no abstract parameter object 
     *         element.
     *         
     * @see units.AbstractCommandParameter#GetProperty()
     * @see units.AbstractCommandParameter#SetProperty()
     */
    public abstract int GetPropertySize();

    /**
     * Get target parameter object.
     * 
     * <p>
     * Query the target parameter object from the current parameter list and return 
     * the abstract parameter object.
     * </p>
     * 
     * <p><b>explain:</b><br>
     * The target user of this interface is the autonomous running model object. In 
     * order to facilitate the creation of the target command parameter object by 
     * using dynamic, encapsulation and inheritance methods for the autonomous 
     * running model object, and push the target command parameter object to the 
     * processing plant.
     * </p>
     * 
     * @param key The type name of the target parameter object should be entered.
     * 
     * <blockquote><pre>
     * AbstractCommandParameter parameter = new MyCommandParameter();
     * AbstractCommandParameter my_parameter_copy = parameter.GetProperty("MyCommandParameter");
     * </pre></blockquote>
     * 
     * @see units.AbstractCommandParameterProperty
     * @see units.AbstractCommandParameter#GetPropertySize()
     * @see units.AbstractCommandParameter#SetProperty(String, AbstractCommandParameterProperty)
     */
    public abstract AbstractCommandParameterProperty GetProperty(String key);

    /**
     * Set target parameter object.
     * 
     * <p>
     * If the element with the target key value already exists in the parameter 
     * list, the element corresponding to the target key value will be overwritten; 
     * otherwise, the object corresponding to the key value will be created.
     * </p>
     * 
     * <p><b>explain:</b><br>
     * The target user of this interface is the autonomous running model object. In 
     * order to facilitate the creation of the target command parameter object by 
     * using dynamic, encapsulation and inheritance methods for the autonomous 
     * running model object, and push the target command parameter object to the 
     * processing plant.
     * </p>
     * 
     * <p><b>warning:</b><br>
     * The memory is managed to the Java mechanism, and this is a shallow copy. The 
     * external should not perform any improper operations on the memory.
     * </p>
     * 
     * @param key The key value of the Map container. The property of this value is 
     *        unique.
     * @param property Abstract Parameter Property Object.
     * 
     * @see units.AbstractCommandParameterProperty
     * @see units.AbstractCommandParameter#GetPropertySize()
     * @see units.AbstractCommandParameter#GetProperty(String)
     */
    public abstract void SetProperty(String key, AbstractCommandParameterProperty property);

    /**
     * Remove the Key element from the parameter list.
     * 
     * @return If the deletion is successful, return the true value; otherwise, fail.
     */
    public abstract boolean RemoveProperty(String key);

    /**
     * Create Entity Parameter Attribute Class.
     * 
     * <p>
     * This method is determined by the derived parameter property class, but it should 
     * be concrete. eg. MovePParameterProperty: poX, poY, poZ, ptX, ptY, ptZ;
     * </p>
     * 
     * <p><b>explain:</b><br>
     * The target user of this interface is the autonomous running model object. In 
     * order to facilitate the creation of the target command parameter object by 
     * using dynamic, encapsulation and inheritance methods for the autonomous 
     * running model object, and push the target command parameter object to the 
     * processing plant.
     * </p>
     * 
     * @param parameter Parameter class original string. eg. MoveP, p1, p2...
     * 
     * @see units.AbstractCommandParameterProperty
     */
    public abstract List<AbstractCommandParameterProperty> CreateProperty(String parameter);
}