import React from 'react';
import { SensorData } from '../types';
import { Download } from 'lucide-react';

interface DataTableProps {
  data: SensorData[];
}

const DataTable: React.FC<DataTableProps> = ({ data }) => {
  const downloadCSV = () => {
    if (data.length === 0) return;

    const headers = [
      'Altitude (m)',
      'Temperature (°C)',
      'Pressure (Pa)',
      'Battery1 (V)',
      'Battery2 (V)',
      'Battery3 (V)',
      'Gyro X (°/s)',
      'Gyro Y (°/s)',
      'Gyro Z (°/s)',
      'Particle Count',
      'Packet Count',
      'Mission Time',
      'State',
    ];

    const csvContent = [
      headers.join(','),
      ...data.map(row =>
        [
          typeof row.altitude === 'number' ? row.altitude.toFixed(2) : '-',
          typeof row.temperature === 'number' ? row.temperature.toFixed(2) : '-',
          typeof row.pressure === 'number' ? row.pressure.toFixed(2) : '-',
          typeof row.battery1 === 'number' ? row.battery1.toFixed(2) : '-',
          typeof row.battery2 === 'number' ? row.battery2.toFixed(2) : '-',
          typeof row.battery3 === 'number' ? row.battery3.toFixed(2) : '-',
          typeof row.gyroX === 'number' ? row.gyroX.toFixed(2) : '-',
          typeof row.gyroY === 'number' ? row.gyroY.toFixed(2) : '-',
          typeof row.gyroZ === 'number' ? row.gyroZ.toFixed(2) : '-',
          row.particleCount ?? '-',
          row.dataPacketCount ?? '-',
          row.missionTime ?? '-',
          row.state ?? '-',
        ].join(',')
      ),
    ].join('\n');

    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.setAttribute('href', url);
    link.setAttribute('download', `cansat_data_${new Date().toISOString()}.csv`);
    link.style.visibility = 'hidden';
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
  };

  // Helper: average battery voltage
  const getAvgBattery = (row: SensorData) => {
    const voltages = [row.battery1, row.battery2, row.battery3].filter(v => typeof v === 'number') as number[];
    if (voltages.length === 0) return '-';
    const avg = voltages.reduce((a, b) => a + b, 0) / voltages.length;
    return avg.toFixed(2);
  };

  return (
    <div className="bg-gradient-to-br from-slate-900 to-slate-800 rounded-xl shadow-xl p-4 text-white overflow-x-auto border border-slate-700">
      <div className="flex justify-between items-center mb-4">
        <h3 className="text-xl font-semibold text-slate-200">Mission Data Log</h3>
        <button
          onClick={downloadCSV}
          className="flex items-center bg-blue-600 hover:bg-blue-700 text-white px-3 py-2 rounded-lg text-sm font-medium transition-colors duration-200 shadow-lg shadow-blue-600/20"
          aria-label="Export data as CSV"
        >
          <Download size={16} className="mr-2" />
          Export CSV
        </button>
      </div>

      <div className="overflow-x-auto">
        <table className="min-w-full divide-y divide-slate-700/50">
          <thead>
            <tr className="bg-slate-800/70">
              <th className="px-6 py-3 text-left text-xs font-medium text-slate-300 uppercase tracking-wider">Altitude (m)</th>
              <th className="px-6 py-3 text-left text-xs font-medium text-slate-300 uppercase tracking-wider">Temperature (°C)</th>
              <th className="px-6 py-3 text-left text-xs font-medium text-slate-300 uppercase tracking-wider">Pressure (Pa)</th>
              <th className="px-6 py-3 text-left text-xs font-medium text-slate-300 uppercase tracking-wider">Voltage (V)</th>
              <th className="px-6 py-3 text-left text-xs font-medium text-slate-300 uppercase tracking-wider">Gyro X (°/s)</th>
              <th className="px-6 py-3 text-left text-xs font-medium text-slate-300 uppercase tracking-wider">Gyro Y (°/s)</th>
              <th className="px-6 py-3 text-left text-xs font-medium text-slate-300 uppercase tracking-wider">Gyro Z (°/s)</th>
              <th className="px-6 py-3 text-left text-xs font-medium text-slate-300 uppercase tracking-wider">Particle Count</th>
              <th className="px-6 py-3 text-left text-xs font-medium text-slate-300 uppercase tracking-wider">Packet Count</th>
              <th className="px-6 py-3 text-left text-xs font-medium text-slate-300 uppercase tracking-wider">Mission Time</th>
              <th className="px-6 py-3 text-left text-xs font-medium text-slate-300 uppercase tracking-wider">State</th>
            </tr>
          </thead>
          <tbody className="divide-y divide-slate-700/30">
            {data.map((row, index) => {
              let rowClass = index % 2 === 0 ? 'bg-slate-800/20' : 'bg-slate-800/10';
              if (row.state === 'Parachute Deployment') {
                rowClass += ' bg-green-900/10';
              } else if (row.state === 'CanDown') {
                rowClass += ' bg-rose-900/10';
              }

              return (
                <tr key={index} className={rowClass}>
                  <td className="px-6 py-3 text-sm text-slate-300 whitespace-nowrap">{typeof row.altitude === 'number' ? row.altitude.toFixed(2) : '-'}</td>
                  <td className="px-6 py-3 text-sm text-slate-300 whitespace-nowrap">{typeof row.temperature === 'number' ? row.temperature.toFixed(2) : '-'}</td>
                  <td className="px-6 py-3 text-sm text-slate-300 whitespace-nowrap">{typeof row.pressure === 'number' ? row.pressure.toFixed(2) : '-'}</td>
                  <td
                    className="px-6 py-3 text-sm text-slate-300 whitespace-nowrap cursor-default"
                    title={`Battery1: ${row.battery1?.toFixed(2) ?? '-'} V, Battery2: ${row.battery2?.toFixed(2) ?? '-'} V, Battery3: ${row.battery3?.toFixed(2) ?? '-'} V`}
                  >
                    {getAvgBattery(row)}
                  </td>
                  <td className="px-6 py-3 text-sm text-slate-300 whitespace-nowrap">{typeof row.gyroX === 'number' ? row.gyroX.toFixed(2) : '-'}</td>
                  <td className="px-6 py-3 text-sm text-slate-300 whitespace-nowrap">{typeof row.gyroY === 'number' ? row.gyroY.toFixed(2) : '-'}</td>
                  <td className="px-6 py-3 text-sm text-slate-300 whitespace-nowrap">{typeof row.gyroZ === 'number' ? row.gyroZ.toFixed(2) : '-'}</td>
                  <td className="px-6 py-3 text-sm text-slate-300 whitespace-nowrap">{row.particleCount ?? '-'}</td>
                  <td className="px-6 py-3 text-sm text-slate-300 whitespace-nowrap">{row.dataPacketCount ?? '-'}</td>
                  <td className="px-6 py-3 text-sm text-slate-300 whitespace-nowrap">{row.missionTime ?? '-'}</td>
                  <td className="px-6 py-3 text-sm">
                    <span
                      className={`px-2 py-1 rounded-full text-xs font-medium ${
                        row.state === 'Ascent'
                          ? 'bg-blue-900/30 text-blue-400'
                          : row.state === 'Parachute Deployment'
                          ? 'bg-green-900/30 text-green-400'
                          : 'bg-rose-900/30 text-rose-400'
                      }`}
                    >
                      {row.state ?? '-'}
                    </span>
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
    </div>
  );
};

export default DataTable;
