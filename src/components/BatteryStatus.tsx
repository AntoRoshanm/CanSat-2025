import React from 'react';

interface BatteryStatusProps {
  battery1: number;
  battery2: number;
  battery3: number;
  maxVoltage?: number;
}

const BatteryStatus: React.FC<BatteryStatusProps> = ({
  battery1,
  battery2,
  battery3,
  maxVoltage = 4.2, // max per battery, adjust as needed
}) => {
  const batteries = [battery1, battery2, battery3];

  const getFillPercent = (voltage: number) =>
    Math.min(Math.max((voltage / maxVoltage) * 100, 0), 100);

  const totalVoltage = battery1 + battery2 + battery3;
  const maxTotalVoltage = maxVoltage * 3;
  const totalPercent = Math.min(Math.max((totalVoltage / maxTotalVoltage) * 100, 0), 100);

  return (
    <div className="battery-status flex flex-col items-center space-y-4">
      <div className="flex space-x-4">
        {batteries.map((voltage, i) => {
          const fill = getFillPercent(voltage);
          return (
            <div key={i} className="battery flex flex-col items-center">
              <div className="battery-label mb-1 text-sm">Battery {i + 1}</div>
              <div
                className="battery-bar relative w-8 h-24 border border-white rounded-md bg-gray-700"
                title={`${voltage.toFixed(2)} V`}
              >
                <div
                  className="battery-fill absolute bottom-0 left-0 right-0 bg-green-400 rounded-b-md transition-all duration-500"
                  style={{ height: `${fill}%` }}
                />
              </div>
              <div className="voltage-label mt-1 text-xs">{voltage.toFixed(2)} V</div>
            </div>
          );
        })}
      </div>

      <div className="total-battery w-full max-w-xs mt-2">
        <div className="battery-label mb-1 text-sm">Total Voltage</div>
        <div className="battery-bar relative w-full h-6 border border-white rounded-md bg-gray-700">
          <div
            className="battery-fill absolute top-0 left-0 bottom-0 bg-yellow-400 rounded-l-md transition-all duration-500"
            style={{ width: `${totalPercent}%` }}
            title={`${totalVoltage.toFixed(2)} V`}
          />
        </div>
        <div className="voltage-label mt-1 text-xs text-center">{totalVoltage.toFixed(2)} V</div>
      </div>
    </div>
  );
};

export default BatteryStatus;
