package types;

public class Point3D 
{
	protected double x, y, z;
	
	public Point3D(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	public Point3D() {
		this.SetPoint(0.0, 0.0, 0.0);
	}
	
	public Point3D(Point3D pt) {
		this.SetPoint(pt);
	}
	
	public void SetPoint(Point3D pt) {
		this.x = pt.GetX();
		this.y = pt.GetY();
		this.z = pt.GetZ();
	}

	public void SetPoint(double x, double y, double z) {
		this.SetPoint(new Point3D(x, y, z));
	}
	
	public double GetX() { return this.x; }
	public double GetY() { return this.y; }
	public double GetZ() { return this.z; }
	
	public double DistanceTo(Point3D pt) {
		if(null == pt) {
			return -1.79E+308;
		}
		
		// Calculate the difference between X, Y and Z axes.
		double tx = this.GetX() - pt.GetX();
		double ty = this.GetY() - pt.GetY();
		double tz = this.GetZ() - pt.GetZ();
		
		return Math.sqrt(tx * tx + ty * ty + tz * tz);
	}
	
	@Override
	public String toString() {
		return String.format("[X:%.4f, Y:%.4f, Z:%.4f]", this.GetX(), this.GetY(), this.GetZ());
	}
}