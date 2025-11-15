import React from 'react';

interface GaugeCardProps {
  title: string;
  value: number;
  min: number;
  max: number;
  unit: string;
}

const GaugeCard: React.FC<GaugeCardProps> = ({ title, value, min, max, unit }) => {
  const percentage = Math.min(100, Math.max(0, ((value - min) / (max - min)) * 100));

  let color = 'bg-emerald-500';
  let textColor = 'text-emerald-400';
  let strokeColor = 'stroke-emerald-400';
  let glowColor = 'shadow-emerald-500/50';

  if (percentage > 80) {
    color = 'bg-rose-500';
    textColor = 'text-rose-400';
    strokeColor = 'stroke-rose-400';
    glowColor = 'shadow-rose-500/50';
  } else if (percentage > 60) {
    color = 'bg-amber-500';
    textColor = 'text-amber-400';
    strokeColor = 'stroke-amber-400';
    glowColor = 'shadow-amber-500/50';
  }

  const radius = 40;
  const circumference = 2 * Math.PI * radius;
  const strokeDashoffset = circumference - (percentage / 100) * circumference;

  const isAngle = unit === '°' || unit === 'deg';
  const angleRotation = (value % 360) - 90; // rotate from top (0°)

  return (
    <div className="bg-gradient-to-br from-slate-900 to-slate-800 rounded-xl shadow-xl p-4 text-white border border-slate-700 relative overflow-hidden">
      <div className={`absolute -bottom-2 -right-2 w-16 h-16 rounded-full blur-xl opacity-20 ${color}`} />

      <h3 className="text-sm font-semibold mb-2 text-center tracking-wider uppercase text-slate-300">
        {title}
      </h3>

      {/* Circular Gauge or Angle Indicator */}
      <div className="flex justify-center mb-3">
        <div className="relative w-28 h-28">
          <svg className="w-full h-full -rotate-90" viewBox="0 0 100 100">
            {/* Base */}
            <circle cx="50" cy="50" r={radius} fill="none" stroke="#1e293b" strokeWidth="10" />
            <circle cx="50" cy="50" r={radius} fill="none" stroke="#334155" strokeWidth="10"
              strokeDasharray={circumference}
              strokeDashoffset={0}
              strokeLinecap="round"
            />
            {!isAngle && (
              <circle
                cx="50"
                cy="50"
                r={radius}
                fill="none"
                className={`${strokeColor} transition-all duration-500 ease-out`}
                strokeWidth="10"
                strokeDasharray={circumference}
                strokeDashoffset={strokeDashoffset}
                strokeLinecap="round"
              />
            )}

            {/* Rotating Needle for Angle */}
            {isAngle && (
              <line
                x1="50"
                y1="50"
                x2="50"
                y2="15"
                stroke="white"
                strokeWidth="3"
                strokeLinecap="round"
                transform={`rotate(${angleRotation}, 50, 50)`}
              />
            )}
          </svg>

          {/* Value */}
          <div className="absolute inset-0 flex flex-col items-center justify-center">
            <span className={`text-2xl font-bold ${textColor}`}>
              {value.toFixed(1)}
            </span>
            <span className="text-xs text-slate-400">{unit}</span>
          </div>
        </div>
      </div>

      {/* Min / Max */}
      <div className="flex justify-between text-xs text-slate-400 px-1 mb-1">
        <span>{min}</span>
        <span>{max}</span>
      </div>

      {/* Linear Bar */}
      <div className="w-full bg-slate-700/50 rounded-full h-1.5 mb-3">
        <div
          className={`h-1.5 rounded-full ${color} transition-all duration-500 ease-out ${glowColor}`}
          style={{ width: `${percentage}%` }}
        />
      </div>

      {/* Status */}
      <div className="flex items-center justify-center">
        <div className={`w-2 h-2 rounded-full ${color} mr-1.5 animate-pulse ${glowColor}`} />
        <span className={`text-xs font-medium ${textColor}`}>
          {percentage < 30 ? 'Low' : percentage > 70 ? 'High' : 'Normal'}
        </span>
      </div>
    </div>
  );
};

export default GaugeCard;
