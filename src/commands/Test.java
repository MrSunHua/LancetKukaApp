package commands;

import units.AbstractCommand;
import units.AbstractCommandEx;
import units.AbstractCommandParameter;
import units.AbstractCommandRet;

public class Test extends AbstractCommandEx {

    @Override
    public AbstractCommand CreateCommand() {
        return new Test();
    }

    @Override
    public AbstractCommandRet Execute(AbstractCommandParameter parameter) {

        System.out.println("Execute command: " + this.GetNameString() + "; parameter.value " + parameter.GetInputString());
        return null;
    }

    @Override
    public String GetNameString() {
        return "Test";
    }
    
}
