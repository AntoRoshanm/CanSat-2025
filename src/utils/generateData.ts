import { SensorData, HistoricalData } from '../types';
import { format } from 'date-fns';

const MAX_ALTITUDE = 1000;

export function generateDummyData(packetCount: number): SensorData {
  const altitude = Math.max(MAX_ALTITUDE - packetCount * 0.5, 0);

  let currentState = "Ascent";
  if (altitude === 0) {
    currentState = "CanDown";
  } else if (altitude < 300) {
    currentState = "Parachute Deployment";
  } else if (altitude < 10) {
    currentState = "CanDown";
  }

  // Simulate three battery voltages, starting at 4.2V, slowly dropping
  const battery1 = Number((4.2 - packetCount * 0.005).toFixed(2));
  const battery2 = Number((4.2 - packetCount * 0.0045).toFixed(2));
  const battery3 = Number((4.2 - packetCount * 0.0055).toFixed(2));

  const gyroX = Number((Math.random() * 20 - 10).toFixed(2));
  const gyroY = Number((Math.random() * 20 - 10).toFixed(2));
  const gyroZ = Number((Math.random() * 20 - 10).toFixed(2));
  const angleChange = Number(
    Math.sqrt(gyroX ** 2 + gyroY ** 2 + gyroZ ** 2).toFixed(2)
  );

  const particleUnit = Math.random() > 0.5 ? "ppm" : "ppb";
  const rssi = Number((-120 + Math.random() * 80).toFixed(2)); // RSSI in dBm
  const gnssSignal = Number((20 + Math.random() * 40).toFixed(2)); // GNSS signal strength in dB-Hz

  return {
    altitude,
    temperature: Number((20 + 5 * (Math.random() * 2 - 1)).toFixed(2)),
    pressure: Number((1013 - altitude * 0.05).toFixed(2)),
    battery1,
    battery2,
    battery3,
    gyroX,
    gyroY,
    gyroZ,
    angleChange,
    particleCount: packetCount % 100,
    particleUnit,
    rssi,
    rssiUnit: "dBm",
    gnssSignal,
    gnssSignalUnit: "dB-Hz",
    dataPacketCount: packetCount,
    navSatellites: Math.floor(Math.random() * 2) + 1, // 1 or 2 only
    missionTime: format(new Date(), 'HH:mm:ss'),
    state: currentState,
    parachuteDeployed: altitude < 300,
    sensorsFunctional: true,
  };
}

export function initializeHistoricalData(): HistoricalData {
  const now = new Date();

  const particleUnit = Math.random() > 0.5 ? "ppm" : "ppb";

  return {
    missionTime: Array(10).fill(0).map((_, i) =>
      format(new Date(now.getTime() - (9 - i) * 1000), 'HH:mm:ss')
    ),
    altitude: Array(10).fill(0).map(() => Math.floor(Math.random() * 600)),
    temperature: Array(10).fill(0).map(() => Number((Math.random() * 45 - 10).toFixed(2))),
    pressure: Array(10).fill(0).map(() => Number((Math.random() * 200 + 900).toFixed(2))),
    battery1: Array(10).fill(0).map((_, i) => Number((12 - i * 0.005).toFixed(2))),
    battery2: Array(10).fill(0).map((_, i) => Number((12 - i * 0.0045).toFixed(2))),
    battery3: Array(10).fill(0).map((_, i) => Number((12 - i * 0.0055).toFixed(2))),
    gyroX: Array(10).fill(0).map(() => Number((Math.random() * 20 - 10).toFixed(2))),
    gyroY: Array(10).fill(0).map(() => Number((Math.random() * 20 - 10).toFixed(2))),
    gyroZ: Array(10).fill(0).map(() => Number((Math.random() * 20 - 10).toFixed(2))),
    angleChange: Array(10).fill(0).map(() => Number((Math.random() * 30).toFixed(2))),
    particleCount: Array(10).fill(0).map(() => Math.floor(Math.random() * 100)),
    particleUnit,
    navSatellites: Array(10).fill(0).map(() => Math.floor(Math.random() * 2) + 1), // 1 or 2 only

    rssi: Array(10).fill(0).map(() => Number((-120 + Math.random() * 80).toFixed(2))),
    rssiUnit: "dBm",
    gnssSignal: Array(10).fill(0).map(() => Number((20 + Math.random() * 40).toFixed(2))),
    gnssSignalUnit: "dB-Hz",
    parachuteDeployed: Array(10).fill(0).map(() => Math.random() > 0.7),
    sensorsFunctional: Array(10).fill(true),
  };
}
