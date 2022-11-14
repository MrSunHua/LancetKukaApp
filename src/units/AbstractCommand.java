package units;

/**
 * Command interface class.
 * 
 * The extended command function should be developed on the basis of this interface 
 * class, which is also one of the important components of the manipulator runtime 
 * model, because the model can help developers configure the execution environment 
 * of target commands and parameter conversion.
 * 
 * @since 1.0
 * 
 * <blockquote><pre>
 *     // Example of use:
 *     AbstractCommand command = new MyCommand();
 *     AbstractCommandParameter = new MyCommandParameter();
 *     AbstractCommandRet command_ret = command.Execute();
 *     
 *     if(command_ret.GetCode() != 0) {
 *          // error
 *     }
 * 
 *     // Extension Example
 *     public class MyCommand implements AbstractCommand {
 * 
 *          @Override
 *          public String GetNameString() {
 *              return "MyCommand";
 *          }
 *          @Override
 *          public AbstractCommand CreateCommand() {
 *              return new MyCommand();
 *          }
 *          @Override
 *          public AbstractCommandRet Execute(AbstractCommandParameter parameter) {
 *              AbstractCommandRet ExecuteRet = new MyCommandRet;
 *              // Do Something...
 *              return ExecuteRet;
 *          }
 *     }
 * </pre></blockquote>
 * 
 * @see units.AbstractCommandRet
 * @see units.AbstractCommandParameter
 */
public interface AbstractCommand {
    
    /**
     * @return Command name string.
     */
    public abstract String GetNameString();


    /**
     * @return Returns an entity derived class command object.
     */
    public abstract AbstractCommand CreateCommand();

    /**
     * Execute the command business function. 
     * The successor should implement the business function within the interface.
     * 
     * @param parameter Parameter objects required for command execution.
     * @return Implementation feedback.
     * 
     * @see units.AbstractCommandRet
     * @see units.AbstractCommandParameter
     */
    public abstract AbstractCommandRet Execute(AbstractCommandParameter parameter);

    /**
     * Get command parameter object.
     * 
     * @return AbstractCommandParameter object.
     * 
     * @see units.AbstractCommandParameter
     */
    public abstract AbstractCommandParameter GetParameterObject();

    /**
     * Set command parameter object.
     * 
     * @param parameter AbstractCommandParameter object.
     * 
     * @see units.AbstractCommandParameter
     */
    public abstract void SetParameterObject(AbstractCommandParameter parameter);

}
