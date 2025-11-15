import React from 'react';
import { PackageOpen, Rocket, PackageCheck } from 'lucide-react';

interface ParachuteStateProps {
  state: string;
}

const ParachuteState: React.FC<ParachuteStateProps> = ({ state }) => {
  // Determine color and icon based on state
  let color = 'text-blue-500 border-blue-500';
  let bgColor = 'from-blue-900/20 to-blue-800/10';
  let shadowColor = 'shadow-blue-500/20';
  let Icon = Rocket;
  let stateDescription = "Satellite is ascending";
  
  if (state === 'Parachute Deployment') {
    color = 'text-green-500 border-green-500';
    bgColor = 'from-green-900/20 to-green-800/10';
    shadowColor = 'shadow-green-500/20';
    Icon = PackageOpen;
    stateDescription = "Parachute has been deployed";
  } else if (state === 'CanDown') {
    color = 'text-rose-500 border-rose-500';
    bgColor = 'from-rose-900/20 to-rose-800/10';
    shadowColor = 'shadow-rose-500/20';
    Icon = PackageCheck;
    stateDescription = "Satellite has landed";
  }
  
  return (
    <div className={`bg-gradient-to-br ${bgColor} border-2 ${color} rounded-xl shadow-lg ${shadowColor} p-6 flex flex-col md:flex-row items-center justify-center md:justify-between`}>
      <div className="flex items-center mb-4 md:mb-0">
        <div className={`p-3 rounded-full bg-slate-900/50 ${color} mr-4`}>
          <Icon className={color} size={32} />
        </div>
        <div>
          <h3 className={`text-2xl font-bold ${color}`}>
            {state}
          </h3>
          <p className="text-slate-300 text-sm">
            {stateDescription}
          </p>
        </div>
      </div>
      
      {/* Status Timeline */}
      <div className="flex items-center space-x-2">
        <div className={`w-8 h-8 rounded-full flex items-center justify-center border-2 ${state === 'Ascent' ? color : 'border-slate-600 text-slate-600'} ${state === 'Ascent' ? 'bg-slate-900' : 'bg-transparent'}`}>
          <span className="text-xs font-bold">1</span>
        </div>
        <div className={`w-12 h-0.5 ${state === 'Parachute Deployment' || state === 'CanDown' ? color : 'bg-slate-600'}`}></div>
        <div className={`w-8 h-8 rounded-full flex items-center justify-center border-2 ${state === 'Parachute Deployment' ? color : 'border-slate-600 text-slate-600'} ${state === 'Parachute Deployment' ? 'bg-slate-900' : 'bg-transparent'}`}>
          <span className="text-xs font-bold">2</span>
        </div>
        <div className={`w-12 h-0.5 ${state === 'CanDown' ? color : 'bg-slate-600'}`}></div>
        <div className={`w-8 h-8 rounded-full flex items-center justify-center border-2 ${state === 'CanDown' ? color : 'border-slate-600 text-slate-600'} ${state === 'CanDown' ? 'bg-slate-900' : 'bg-transparent'}`}>
          <span className="text-xs font-bold">3</span>
        </div>
      </div>
    </div>
  );
};

export default ParachuteState;