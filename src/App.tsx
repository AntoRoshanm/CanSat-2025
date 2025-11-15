import React, { useState, useEffect } from 'react';
import Navbar from './components/Navbar';
import GaugeCard from './components/GaugeCard';
import LineChart from './components/LineChart';
import ParachuteState from './components/ParachuteState';
import DataTable from './components/DataTable';
import Map from './components/Map';
import BatteryStatus from './components/BatteryStatus';
import { generateDummyData, initializeHistoricalData } from './utils/generateData';
import { SensorData, HistoricalData } from './types';

function App() {
  const [packetCount, setPacketCount] = useState(0);
  const [currentData, setCurrentData] = useState<SensorData>(generateDummyData(0));
  const [historicalData, setHistoricalData] = useState<HistoricalData>(initializeHistoricalData());
  const [tableData, setTableData] = useState<SensorData[]>([]);

  useEffect(() => {
    const interval = setInterval(() => {
      setPacketCount((prevCount) => {
        const newPacketCount = prevCount + 1;

        const newData = generateDummyData(newPacketCount);
        setCurrentData(newData);

        setHistoricalData(prev => {
          const updated = { ...prev };

          updated.missionTime = [...updated.missionTime, newData.missionTime].slice(-100);
          updated.altitude = [...updated.altitude, newData.altitude].slice(-100);
          updated.temperature = [...updated.temperature, newData.temperature].slice(-100);
          updated.pressure = [...updated.pressure, newData.pressure].slice(-100);
          updated.battery1 = [...updated.battery1, newData.battery1].slice(-100);
          updated.battery2 = [...updated.battery2, newData.battery2].slice(-100);
          updated.battery3 = [...updated.battery3, newData.battery3].slice(-100);
          updated.particleCount = [...updated.particleCount, newData.particleCount].slice(-100);
          updated.gyroX = [...updated.gyroX, newData.gyroX].slice(-100);
          updated.gyroY = [...updated.gyroY, newData.gyroY].slice(-100);
          updated.gyroZ = [...updated.gyroZ, newData.gyroZ].slice(-100);
          updated.rssi = [...updated.rssi, newData.rssi].slice(-100);

          return updated;
        });

        setTableData(prev => [newData, ...prev].slice(0, 10));

        return newPacketCount;
      });
    }, 1000);

    return () => clearInterval(interval);
  }, []); // Empty deps to run once on mount

  return (
    <div className="min-h-screen bg-slate-950 text-white p-4">
      <div className="container mx-auto">
        {/* Navbar */}
        <Navbar packetCount={currentData.dataPacketCount} />

        {/* Gauge Cards */}
        <div className="grid grid-cols-4 md:grid-cols-5 lg:grid-cols-5 gap-4 mb-6">
          <GaugeCard title="Altitude" value={currentData.altitude} min={0} max={1000} unit="m" />
          <GaugeCard title="Temperature" value={currentData.temperature} min={-10} max={50} unit="°C" />
          <GaugeCard title="Pressure" value={currentData.pressure} min={900} max={1100} unit="Pa" />
          <div className="col-span-1">
            <BatteryStatus
              battery1={currentData.battery1}
              battery2={currentData.battery2}
              battery3={currentData.battery3}
            />
          </div>
          <GaugeCard title="RSSI" value={currentData.rssi} min={-120} max={-40} unit="dBm" />
          <GaugeCard title="Gyroscope X" value={currentData.gyroX} min={-20} max={20} unit="°/s" />
          <GaugeCard title="Gyroscope Y" value={currentData.gyroY} min={-20} max={20} unit="°/s" />
          <GaugeCard title="Gyroscope Z" value={currentData.gyroZ} min={-20} max={20} unit="°/s" />
          <GaugeCard
            title="Particle Count"
            value={currentData.particleCount}
            min={0}
            max={100}
            unit={currentData.particleUnit}
          />
          <GaugeCard title="Sat Count" value={currentData.navSatellites} min={0} max={2} unit="Sats" />
        </div>

        {/* Parachute State */}
        <div className="mb-6">
          <ParachuteState state={currentData.state} />
        </div>

        {/* Charts - First Row */}
        <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mb-6">
          <LineChart
            title="Altitude"
            labels={historicalData.missionTime}
            data={historicalData.altitude}
            borderColor="rgb(59, 130, 246)"
            backgroundColor="rgba(59, 130, 246, 0.1)"
            yAxisLabel="Altitude (m)"
            currentValue={currentData.altitude}
            unit="m"
          />
          <LineChart
            title="Temperature"
            labels={historicalData.missionTime}
            data={historicalData.temperature}
            borderColor="rgb(239, 68, 68)"
            backgroundColor="rgba(239, 68, 68, 0.1)"
            yAxisLabel="Temperature (°C)"
            currentValue={currentData.temperature}
            unit="°C"
          />
          <LineChart
            title="Pressure"
            labels={historicalData.missionTime}
            data={historicalData.pressure}
            borderColor="rgb(16, 185, 129)"
            backgroundColor="rgba(16, 185, 129, 0.1)"
            yAxisLabel="Pressure (Pa)"
            currentValue={currentData.pressure}
            unit="Pa"
          />
        </div>

        {/* Charts - Second Row */}
        <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mb-6">
          <LineChart
            title="Voltage Battery 1"
            labels={historicalData.missionTime}
            data={historicalData.battery1}
            borderColor="rgb(245, 158, 11)"
            backgroundColor="rgba(245, 158, 11, 0.1)"
            yAxisLabel="Voltage (V)"
            currentValue={currentData.battery1}
            unit="V"
          />
          <LineChart
            title="Voltage Battery 2"
            labels={historicalData.missionTime}
            data={historicalData.battery2}
            borderColor="rgb(236, 72, 153)"
            backgroundColor="rgba(236, 72, 153, 0.1)"
            yAxisLabel="Voltage (V)"
            currentValue={currentData.battery2}
            unit="V"
          />
          <LineChart
            title="Voltage Battery 3"
            labels={historicalData.missionTime}
            data={historicalData.battery3}
            borderColor="rgb(96, 165, 250)"
            backgroundColor="rgba(96, 165, 250, 0.1)"
            yAxisLabel="Voltage (V)"
            currentValue={currentData.battery3}
            unit="V"
          />
        </div>

        {/* Charts - Third Row */}
        <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mb-6">
          <LineChart
            title="Particle Count"
            labels={historicalData.missionTime}
            data={historicalData.particleCount}
            borderColor="rgb(236, 72, 153)"
            backgroundColor="rgba(236, 72, 153, 0.1)"
            yAxisLabel="Particle Count (mg/m³)"
            currentValue={currentData.particleCount}
            unit="mg/m³"
          />
          <LineChart
            title="Gyroscope X"
            labels={historicalData.missionTime}
            data={historicalData.gyroX}
            borderColor="rgb(96, 165, 250)"
            backgroundColor="rgba(96, 165, 250, 0.1)"
            yAxisLabel="Gyro X (°/s)"
            currentValue={currentData.gyroX}
            unit="°/s"
          />
          <LineChart
            title="Gyroscope Y"
            labels={historicalData.missionTime}
            data={historicalData.gyroY}
            borderColor="rgb(132, 204, 22)"
            backgroundColor="rgba(132, 204, 22, 0.1)"
            yAxisLabel="Gyro Y (°/s)"
            currentValue={currentData.gyroY}
            unit="°/s"
          />
        </div>

        {/* Charts - Fourth Row */}
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4 mb-6">
          <LineChart
            title="Gyroscope Z"
            labels={historicalData.missionTime}
            data={historicalData.gyroZ}
            borderColor="rgb(244, 114, 182)"
            backgroundColor="rgba(244, 114, 182, 0.1)"
            yAxisLabel="Gyro Z (°/s)"
            currentValue={currentData.gyroZ}
            unit="°/s"
          />
          <LineChart
            title="RSSI"
            labels={historicalData.missionTime}
            data={historicalData.rssi}
            borderColor="rgb(120, 120, 120)"
            backgroundColor="rgba(120, 120, 120, 0.1)"
            yAxisLabel="RSSI (dBm)"
            currentValue={currentData.rssi}
            unit="dBm"
          />
        </div>

        {/* Map */}
        <div className="mb-6">
          <Map />
        </div>

        {/* Data Table */}
        <div className="mb-6">
          <DataTable data={tableData} />
        </div>
      </div>
    </div>
  );
}

export default App;
