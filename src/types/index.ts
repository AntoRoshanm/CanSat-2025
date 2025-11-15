export interface SensorData {
  altitude: number;
  temperature: number;
  pressure: number;
  battery1: number;
  battery2: number;
  battery3: number;
  gyroX: number;
  gyroY: number;
  gyroZ: number;
  angleChange: number;
  particleCount: number;
  particleUnit: string;
  rssi: number;
  rssiUnit: string;
  gnssSignal: number;
  gnssSignalUnit: string;
  dataPacketCount: number;
  navSatellites: number;
  missionTime: string;
  state: string;
  parachuteDeployed: boolean;
  sensorsFunctional: boolean;
}

export interface HistoricalData {
  missionTime: string[];
  altitude: number[];
  temperature: number[];
  pressure: number[];
  battery1: number[];
  battery2: number[];
  battery3: number[];
  gyroX: number[];
  gyroY: number[];
  gyroZ: number[];
  angleChange: number[];
  particleCount: number[];
  particleUnit: string;
  navSatellites: number[];
  rssi: number[];
  rssiUnit: string;
  gnssSignal: number[];
  gnssSignalUnit: string;
  parachuteDeployed: boolean[];
  sensorsFunctional: boolean[];
}
