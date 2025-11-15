import React from 'react';
import { Satellite, Wifi, Clock } from 'lucide-react';

interface NavbarProps {
  packetCount: number;
}

const Navbar: React.FC<NavbarProps> = ({ packetCount }) => {
  // Get current time
  const [currentTime, setCurrentTime] = React.useState(new Date());
  
  React.useEffect(() => {
    const timer = setInterval(() => {
      setCurrentTime(new Date());
    }, 1000);
    
    return () => clearInterval(timer);
  }, []);
  
  const formattedTime = currentTime.toLocaleTimeString();
  const formattedDate = currentTime.toLocaleDateString(undefined, {
    weekday: 'short',
    year: 'numeric',
    month: 'short',
    day: 'numeric'
  });
  
  return (
    <nav className="bg-gradient-to-r from-slate-900 to-slate-800 rounded-2xl shadow-xl mb-6 p-3 border border-slate-700">
      <div className="container mx-auto flex flex-col md:flex-row justify-between items-center">
        <div className="flex items-center space-x-4">
          <div className="bg-blue-600 p-2 rounded-xl shadow-lg shadow-blue-600/20">
            <Satellite size={36} className="text-white" />
          </div>
          <div>
            <h1 className="text-xl md:text-2xl font-bold text-white">
              TEAM ID: 2024-ASI-CANSAT-062
            </h1>
            <p className="text-sm md:text-base text-slate-300">
              TEAM NAME: Dr.MGR-ACS SAT
            </p>
          </div>
        </div>
        
        <div className="flex items-center space-x-6 mt-4 md:mt-0">
          {/* Signal Strength Indicator */}
          <div className="flex items-center">
            <Wifi size={20} className="text-green-400 mr-2" />
            <span className="text-slate-300 text-sm">Signal: Strong</span>
          </div>
          
          {/* Time Display */}
          <div className="flex items-center">
            <Clock size={20} className="text-slate-300 mr-2" />
            <div className="flex flex-col">
              <span className="text-white font-medium">{formattedTime}</span>
              <span className="text-xs text-slate-400">{formattedDate}</span>
            </div>
          </div>
          
          {/* Packet Counter */}
          <div className="bg-gradient-to-r from-red-600 to-red-500 px-4 py-2 rounded-lg shadow-lg shadow-red-600/20">
            <div className="text-sm text-red-100">Packet Count</div>
            <div className="text-xl md:text-2xl font-bold text-white">{packetCount}</div>
          </div>
        </div>
      </div>
    </nav>
  );
};

export default Navbar;