package factorys;

import java.util.HashMap;
import java.util.Map;

import units.AbstractCommand;

public class CommandFactory {
    /**
     * Order the work instruction template container of the factory product, and the 
     * production line will manufacture the target product from the existing work 
     * instruction.
     * 
     * <blockquote><pre>
     *     CommandFactory factory = new CommandFactory();
     *     factory.RegisterProduct(new MoveP());
     *     
     *     MoveP commandMovep = factory.MakeProduct("MoveP");
     * </pre></blockquote>
     */
    private Map<String, AbstractCommand> commands = new HashMap<String, AbstractCommand>();

    /**
     * Register the target products to the factory production line, or it can be 
     * understood as the operation instructions for delivering the factory target 
     * products, to ensure that the factory has the ability to produce the target 
     * products.
     * 
     * @param command Object instance of registration target.
     * @return If the registration is successful, return true; otherwise, release.
     * 
     * @see units.AbstractCommand
     */
    public boolean RegisterProduct(AbstractCommand command) {

        if(null == command) {
            return false;
        }
        if(false == this.commands.containsKey(command.GetNameString().toLowerCase())) {
            this.commands.put(command.GetNameString().toLowerCase(), command);
        }
        return true;
    }
    
    /**
     * Unload the target product to the factory production line, or understand the 
     * factory's ability to stop producing the target product.
     * 
     * @param command Object instance of registration target.
     * @return If the registration is successful, return true; otherwise, release.
     * 
     * @see units.AbstractCommand
     */
    public boolean UnRegisterProduct(AbstractCommand command) {

        if(null == command) {
            return false;
        }
        if(false == this.commands.containsKey(command.GetNameString().toLowerCase())) {
            this.commands.put(command.GetNameString().toLowerCase(), command);
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
     * @return AbstractCommand object.
     * 
     * @see units.AbstractCommand
     */
    public AbstractCommand MakeProduct(String product) {

        if(null != product) {

            if(true == this.commands.containsKey(product.toLowerCase())) {
                return this.commands.get(product.toLowerCase()).CreateCommand();
            }
        }

        return null;
    }
}
