package factorys;

import java.util.HashMap;
import java.util.Map;

import units.AbstractCommandParameter;

public class CommandParameterFactory {
    
    /**
     * Resource Object Container Required for Command Running.
     * 
     * This container will register runtime resources of all commands. When the model 
     * runs automatically, it will tell the corresponding resources to the command 
     * function execution object in the form of command parameter object.
     */
    private Map<String, Object> mapRuntimeProperties = new HashMap<String, Object>();

    /**
     * Command parameter class work instruction container.
     * 
     * A work instruction containing all the required command parameter classes, 
     * according to which the target product can be perfectly copied.
     * 
     * @see factorys.CommandParameterFactory#MakeParameter(String, String)
     * @see factorys.CommandParameterFactory#RegisterParameter(AbstractCommandParameter)
     * @see factorys.CommandParameterFactory#UnRegisterParameter(AbstractCommandParameter)
     * 
     * @see factorys.CommandParameterFactory#commandParameters
     */
    private Map<String, AbstractCommandParameter> commandParameters = new HashMap<String, AbstractCommandParameter>();

    /**
     * Register the target parameter to the factory production line, or it can be 
     * understood as the operation instructions for delivering the factory target 
     * products, to ensure that the factory has the ability to produce the target 
     * products.
     * 
     * @param parameter Object instance of registration target.
     * @return If the registration is successful, return true; otherwise, release.
     * 
     * @see units.AbstractCommandParameter
     * @see factorys.CommandParameterFactory#commandParameters
     */
    public boolean RegisterParameter(String product, AbstractCommandParameter parameter) {

        if(null == parameter || null == product) {
            return false;
        }
        if(false == this.commandParameters.containsKey(product.toLowerCase())) {
            this.commandParameters.put(product.toLowerCase(), parameter);
        }
        return true;
    }
    
    /**
     * Unload the target product to the factory production line, or understand the 
     * factory's ability to stop producing the target product.
     * 
     * @param parameter Object instance of registration target.
     * @return If the registration is successful, return true; otherwise, release.
     * 
     * @see units.AbstractCommandParameter
     * @see factorys.CommandParameterFactory#commandParameters
     */
    public boolean UnRegisterParameter(AbstractCommandParameter parameter) {

        if(null == parameter) {
            return false;
        }
        if(false == this.commandParameters.containsKey(parameter.getClass().getSimpleName().toLowerCase())) {
            this.commandParameters.remove(parameter.getClass().getSimpleName().toLowerCase());
        }
        return true;
    }

    /**
     * To produce the target product instance, the factory will look for the production 
     * mode in all the operation instructions, and the production will be smooth. The 
     * factory will deliver an external AbstractCommand object instance, otherwise it 
     * will deliver  a null object.
     * 
     * 
     * @param product Characteristics of the target product, such as brand.
     * @param datas Parameter string object passed by remote PC.
     * @return AbstractCommandParameter object.
     * 
     * @see units.AbstractCommandParameter
     * @see factorys.CommandParameterFactory#commandParameters
     */
    public AbstractCommandParameter MakeParameter(String product, String datas) {

        if(null != product) {

            if(true == this.commandParameters.containsKey(product.toLowerCase())) {
                return this.commandParameters.get(product.toLowerCase()).CreateCommandParameter(datas, mapRuntimeProperties);
            }
        }

        return null;
    }

    
    /**
     * Register the target parameter to the factory production line, or it can be 
     * understood as the operation instructions for delivering the factory target 
     * products, to ensure that the factory has the ability to produce the target 
     * products.
     * 
     * @param parameter Object instance of registration target.
     * @return If the registration is successful, return true; otherwise, release.
     * 
     * @see factorys.CommandParameterFactory#mapRuntimeProperties
     */
    public boolean RegisterRunTimeProperty(Object o) {

        if(null == o) {
            return false;
        }
        this.mapRuntimeProperties.put(o.getClass().getSimpleName().toLowerCase(), o);
        return true;
    }
    
    /**
     * Unload the target product to the factory production line, or understand the 
     * factory's ability to stop producing the target product.
     * 
     * @param parameter Object instance of registration target.
     * @return If the registration is successful, return true; otherwise, release.
     * 
     * @see factorys.CommandParameterFactory#mapRuntimeProperties
     */
    public boolean UnRegisterRunTimeProperty(Object o) {

        if(null == o) {
            return false;
        }
        if(false == this.mapRuntimeProperties.containsKey(o.getClass().getSimpleName().toLowerCase())) {
            this.mapRuntimeProperties.remove(o.getClass().getSimpleName().toLowerCase());
        }
        return true;
    }
}
