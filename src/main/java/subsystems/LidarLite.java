package subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


// NOTE: the lidar sensor measurement is from the back of lidar sensor

public class LidarLite extends SubsystemBase {

    private final Counter m_counter;
	private int printedWarningCount = 5;

    public LidarLite(DigitalSource source) {
        m_counter = new Counter(source);
        m_counter.setMaxPeriod(1.0);
        // Configure for measuring rising to falling pulses
        m_counter.setSemiPeriodMode(true);
        m_counter.reset();
    }

    /**
     * Take a measurement and return the distance in cm
     * @param int offset in cm
     * @return Distance in cm
     */
    public double getDistance(int offset) {
        double cm;
		/* If we haven't seen the first rising to falling pulse, then we have no measurement.
	 	* This happens when there is no LIDAR-Lite plugged in, btw.
	 	*/
		if (m_counter.get() < 1) {
			if (printedWarningCount-- > 0) {
				System.out.println("LidarLite: waiting for distance measurement");
			}
		return 0;
		}
        /*
         * getPeriod returns time in seconds. The hardware resolution is microseconds.
         * The LIDAR-Lite unit sends a high signal for 10 microseconds per cm of
         * distance.
         */
        cm = (m_counter.getPeriod() * 1000000.0 / 10.0) - offset;
        return cm;
    }
}