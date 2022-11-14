package units;

import java.util.HashMap;
import java.util.Map;

/**
 * Abstract command parameter class extension object.
 * 
 * <p>
 * The command parameter class may have some common basic methods, which are 
 * implemented in the extension class to avoid excessive redundant code generated 
 * by the entity command parameter class.
 * </p>
 * 
 * Example of usage: Show you how to use derived classes.
 * <blockquote><pre>
 * MyClass my_class = new MyClass();
 * AbstractCommandParameterEx parameter = new MyCommandParameter();
 * 
 * Usage 1: "myclass": my_class
 * parameter.SetProperty(my_class.getClass().getSimpleName(), my_class);
 * 
 * Usage 2: "myclass": my_class
 * parameter.SetProperty("MyClass", my_class);
 * 
 * Usage 3: "myclassfield": my_class
 * parameter.SetProperty("MyClassField", my_class);
 * 
 * </pre></blockquote>
 * 
 * Extension Example: Demonstrates how to extend a custom derived class.
 * <pre><blockquote>
 * public class MyCommandParameter extends AbstractCommandParameterEx {
 * 
 *     private String errorString;
 *     @Override
 *     public AbstractCommandParameterProperty CreateProperty(String parameter) {
 *         TODO: Process parameter strings into memory objects, and store the 
 *               processed memory objects in the parameter list to provide parameter 
 *               services for commands.
 *         return MyCommandParameterProperty;
 *     }
 *     
 *     @Override
 *     public AbstractCommandParameter CreateCommandParameter(String parameter,
 *          Map<String, Object> mapRuntimeProperties) {
 *          TODO: Override this method to provide a method for the model to copy 
 *                parameter objects.
 *          return new MyCommandParameter(parameter);
 *     }
 *     
 *     @Override
 *     public String GetErrorString() {
 *         return this.errorString;
 *     }
 * }
 * </pre></blockquote><br>
 * 
 * 
 * @author Sun
 * @since 1.0
 * 
 * @see units.AbstractCommandParameter
 */
public abstract class AbstractCommandParameterEx implements AbstractCommandParameter {

	/**
	 * A variable that stores the original string of a command.
	 * 
	 * @see units.AbstractCommandParameter#GetInputString()
	 * @see units.AbstractCommandParameter#SetInputString(String)
	 */
    protected String inputString = "";

    /**
     * The parameter list container uses the Map property to manage the parameter 
     * attribute element table.
     * 
     * <p><b>Warning:</b><br>
     * String is internally processed into lowercase format. This operation can be 
     * ignored externally, but it is necessary to know the internal lowercase format 
     * processing of the Key value.
     * </p>
     * 
     * @param String Type name string.
     * @param AbstractCommandParameterProperty Abstract Command Parameter Attribute 
     *        Object.
     *        
     * @see units.AbstractCommandParameterProperty
     */
    protected Map<String, AbstractCommandParameterProperty> properties = 
    		new HashMap<String, AbstractCommandParameterProperty>();

    @Override
    public String GetInputString() {
        return this.inputString;
    }

    @Override
    public AbstractCommandParameterProperty GetProperty(String key) {
        return this.properties.get(key.toLowerCase());
    }

    @Override
    public int GetPropertySize() {
        return this.properties.size();
    }

    @Override
    public boolean IsVaild() {
        return false == this.properties.isEmpty();
    }

    @Override
    public boolean RemoveProperty(String key) {

        if(this.properties.containsKey(key.toLowerCase())) {

            return null != this.properties.remove(key.toLowerCase());
        }
        return false;
    }

    @Override
    public void SetInputString(String dataStream) {
        this.inputString = dataStream;
    }

    @Override
    public void SetProperty(String key, AbstractCommandParameterProperty property) {
        
        this.properties.put(key.toLowerCase(), property);
    }
    
}
