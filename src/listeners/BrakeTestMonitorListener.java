package listeners;

import com.kuka.med.cyclicBrakeTest.BrakeTestMonitorEvent;
import com.kuka.med.cyclicBrakeTest.IBrakeTestMonitorListener;

class BrakeTestMonitorListener implements IBrakeTestMonitorListener {

	@Override
	public void onMonitoringStateChanged(BrakeTestMonitorEvent event) {
		if(null == event) {
			System.out.println("listeners.BrakeTestMonitorListener.onMonitoringStateChanged: input event object is null.");
		}
		System.out.println("listeners.BrakeTestMonitorListener.onMonitoringStateChanged: log." + event.toString());
	}
	
}