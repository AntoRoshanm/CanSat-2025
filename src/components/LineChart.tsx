import React from 'react';
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
  Filler,
} from 'chart.js';
import { Line } from 'react-chartjs-2';

ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
  Filler
);

interface LineChartProps {
  title: string;
  labels: string[];
  data: number[];
  borderColor?: string;
  backgroundColor?: string;
  yAxisLabel: string;
  currentValue: number;
  unit: string;
}

const LineChart: React.FC<LineChartProps> = ({
  title,
  labels,
  data,
  borderColor = 'rgb(75, 192, 192)',
  backgroundColor = 'rgba(75, 192, 192, 0.2)',
  yAxisLabel,
  currentValue,
  unit
}) => {
  const options = {
    responsive: true,
    maintainAspectRatio: false,
    animations: {
      tension: {
        duration: 1000,
        easing: 'linear',
        from: 0.8,
        to: 0.2,
        loop: false
      }
    },
    plugins: {
      legend: {
        display: false,
      },
      title: {
        display: true,
        text: title,
        color: 'white',
        font: {
          size: 16,
          weight: 'bold',
          family: "'Inter', sans-serif",
        },
        padding: {
          top: 10,
          bottom: 20
        }
      },
      tooltip: {
        backgroundColor: 'rgba(15, 23, 42, 0.8)',
        titleFont: {
          size: 14,
          family: "'Inter', sans-serif",
        },
        bodyFont: {
          size: 13,
          family: "'Inter', sans-serif",
        },
        padding: 12,
        borderColor: 'rgba(148, 163, 184, 0.2)',
        borderWidth: 1,
        displayColors: false,
        callbacks: {
          label: function(context: any) {
            return `${context.parsed.y} ${unit}`;
          }
        }
      }
    },
    scales: {
      y: {
        title: {
          display: true,
          text: yAxisLabel,
          color: 'rgba(255, 255, 255, 0.7)',
          font: {
            size: 12,
            family: "'Inter', sans-serif",
          }
        },
        ticks: {
          color: 'rgba(255, 255, 255, 0.7)',
          font: {
            size: 11,
            family: "'Inter', sans-serif",
          },
          padding: 8
        },
        grid: {
          color: 'rgba(255, 255, 255, 0.05)',
          drawBorder: false,
        },
        border: {
          display: false
        }
      },
      x: {
        ticks: {
          color: 'rgba(255, 255, 255, 0.7)',
          maxRotation: 45,
          minRotation: 45,
          font: {
            size: 10,
            family: "'Inter', sans-serif",
          },
          padding: 8
        },
        grid: {
          color: 'rgba(255, 255, 255, 0.05)',
          drawBorder: false,
          display: false
        },
        border: {
          display: false
        }
      },
    },
    elements: {
      line: {
        tension: 0.4,
        borderWidth: 2,
      },
      point: {
        radius: 0,
        hitRadius: 10,
        hoverRadius: 4,
      }
    }
  };

  const chartData = {
    labels,
    datasets: [
      {
        label: title,
        data,
        borderColor,
        backgroundColor,
        fill: true,
        pointBackgroundColor: borderColor,
      },
    ],
  };

  // Determine color based on the title for the value display
  let valueColor = 'text-blue-400';
  if (title === 'Temperature') valueColor = 'text-rose-400';
  if (title === 'Pressure') valueColor = 'text-emerald-400';
  if (title === 'Voltage') valueColor = 'text-amber-400';
  if (title === 'Airspeed') valueColor = 'text-violet-400';
  if (title === 'Particle Count') valueColor = 'text-pink-400';

  return (
    <div className="bg-gradient-to-br from-slate-900 to-slate-800 rounded-xl shadow-xl p-4 text-white h-full border border-slate-700 relative overflow-hidden">
      {/* Background glow effect */}
      <div className="absolute -bottom-10 -right-10 w-40 h-40 rounded-full blur-3xl opacity-10" style={{ backgroundColor: borderColor }}></div>
      
      <div className="h-64 relative z-10">
        <Line options={options as any} data={chartData} />
      </div>
      
      <div className="mt-4 text-center relative z-10">
        <div className="flex items-center justify-center">
          <div className="w-3 h-3 rounded-full mr-2 animate-pulse" style={{ backgroundColor: borderColor }}></div>
          <h5 className="text-lg font-semibold text-slate-200">{title}</h5>
        </div>
        <p className={`text-3xl font-bold mt-1 ${valueColor}`}>
          {currentValue.toFixed(2)} <span className="text-sm text-slate-400">{unit}</span>
        </p>
      </div>
    </div>
  );
};

export default LineChart;