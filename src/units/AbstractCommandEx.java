package units;

/**
 * Abstract Command Class Extension Object.
 * 
 * <p>
 * The command class may have some common basic methods, which are implemented in 
 * the extension class to avoid excessive redundant code generated by the entity 
 * command class.
 * </p>
 * 
 * @since 1.0
 * 
 * @author Sun
 * 
 * @see units.AbstractCommand
 */
public abstract class AbstractCommandEx implements AbstractCommand {

	/**
	 * Abstract command parameter class object.
	 * 
	 * <p>
	 * which should be pointed to concrete or entity command parameter class object.
	 * </p>
	 * 
	 * @author Sun
	 * 
	 * @see units.AbstractCommandParameter
	 */
    protected AbstractCommandParameter abstractCommandParameter = null;

    @Override
    public AbstractCommandParameter GetParameterObject() { 
        return this.abstractCommandParameter;
    }

    @Override
    public void SetParameterObject(AbstractCommandParameter parameter) {
        this.abstractCommandParameter = parameter;
    }
}
