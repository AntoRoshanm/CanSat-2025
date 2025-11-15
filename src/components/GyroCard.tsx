import React from 'react';

interface GyroCardProps {
  x: number;
  y: number;
  z: number;
}

const GyroCard: React.FC<GyroCardProps> = ({ x, y, z }) => {
  const format = (val: number) => val.toFixed(2).padStart(6, ' ');

  return (
    <div className="bg-gradient-to-br from-slate-900 to-slate-800 rounded-xl shadow-xl p-4 text-white border border-slate-700">
      <h3 className="text-sm font-semibold mb-2 text-center tracking-wider uppercase text-slate-300">
        Gyroscope (°/s)
      </h3>
      <div className="grid grid-cols-3 gap-2 text-center text-lg font-mono">
        <div>
          <div className="text-slate-400 text-sm mb-1">X</div>
          <div className="text-blue-400">{format(x)}</div>
        </div>
        <div>
          <div className="text-slate-400 text-sm mb-1">Y</div>
          <div className="text-green-400">{format(y)}</div>
        </div>
        <div>
          <div className="text-slate-400 text-sm mb-1">Z</div>
          <div className="text-rose-400">{format(z)}</div>
        </div>
      </div>
    </div>
  );
};

export default GyroCard;
