package properties;

import types.Point3D;
import units.AbstractCommandParameterPropertyEx;

public class Point3DParameterProperty extends AbstractCommandParameterPropertyEx {

	public Point3DParameterProperty() {
		super("Non");
		this.SetParameterTypeObject(new Point3D());
	}
	
	public Point3DParameterProperty(String key) {
		super(key);
		this.SetParameterTypeObject(new Point3D());
	}
	
	@Override
	public String GetParameterTypeName() {
		if(null != this.GetParameterTypeObject() && null != this.GetParameterTypeObject().getClass()) {
            return this.GetParameterTypeObject().getClass().getSimpleName();
        }
		return "Non";
	}

	
	public Point3D GetParameterPoint3DObject() {
		if(null != this.GetParameterTypeObject() && 
				this.GetParameterTypeName().toLowerCase().contains("point3d")) {

			return (Point3D)this.GetParameterTypeObject();
		}
		return null;
	}
	
	@Override
	public String toString() {
		if(null == this.GetParameterTypeName()) {
			return "Point object is null.";
		}
		return "Point3D " + this.GetParameterPoint3DObject().toString();
	}
}