package properties;

import units.AbstractCommandParameterProperty;

public class IntTestParameterProperty implements AbstractCommandParameterProperty {

    protected Object property = null;

    public IntTestParameterProperty() { }

    public IntTestParameterProperty(Object o) {
        this.SetParameterTypeObject(o);
    }

    @Override
    public String GetParameterTypeName() {
        if(null != property && null != property.getClass()) {
            return property.getClass().getSimpleName();
        }
        return "Non";
    }

    @Override
    public Object GetParameterTypeObject() {
        return property;
    }

    @Override
    public void SetParameterTypeObject(Object o) {
        this.property = o;
    }

    @Override
    public String GetKeyString() {
        return "IntTestParameterProperty";
    }

    public Integer GetTargetTypeObject() {

        if(null != this.property && this.property.getClass().getSimpleName().contains("Integer")) {
            return ((Integer)this.property);
        }
        return null;
    }
}
