import React, { useEffect, useRef } from 'react';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import { MapPin, Navigation } from 'lucide-react';

interface MapProps {
  className?: string;
}

const Map: React.FC<MapProps> = ({ className }) => {
  const mapRef = useRef<HTMLDivElement>(null);
  const mapInstanceRef = useRef<L.Map | null>(null);

  useEffect(() => {
    if (mapRef.current && !mapInstanceRef.current) {
      // Initialize map
      mapInstanceRef.current = L.map(mapRef.current).setView([13.08303, 80.271605], 18);
      
      // Add dark theme tile layer
      L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors &copy; <a href="https://carto.com/attributions">CARTO</a>',
        subdomains: 'abcd',
        maxZoom: 20
      }).addTo(mapInstanceRef.current);
      
      // Add trail coordinates
      const trailCoordinates = [
        [13.08303, 80.271605],
        [13.083689, 80.270456],
        [13.081773, 80.270079],
        [13.08227, 80.270709],
        [13.082363, 80.270162],
        [13.083155, 80.270358],
        [13.081788, 80.271165],
        [13.082125, 80.270099],
        [13.081837, 80.270416],
        [13.081741, 80.271272],
        [13.083512, 80.271194],
      ];
      
      // Create a custom icon for the start point
      const startIcon = L.divIcon({
        html: `<div class="flex items-center justify-center w-6 h-6 bg-blue-600 rounded-full shadow-lg shadow-blue-600/50">
                <div class="w-2 h-2 bg-white rounded-full"></div>
              </div>`,
        className: '',
        iconSize: [24, 24],
        iconAnchor: [12, 12]
      });
      
      // Create a custom icon for the current position
      const currentIcon = L.divIcon({
        html: `<div class="flex items-center justify-center w-8 h-8 bg-red-600 rounded-full shadow-lg shadow-red-600/50 animate-pulse">
                <div class="w-3 h-3 bg-white rounded-full"></div>
              </div>`,
        className: '',
        iconSize: [32, 32],
        iconAnchor: [16, 16]
      });
      
      // Add start marker
      L.marker([trailCoordinates[0][0], trailCoordinates[0][1]], { icon: startIcon })
        .addTo(mapInstanceRef.current)
        .bindPopup('Launch Point')
        .openPopup();
      
      // Add polyline with gradient effect
      const polyline = L.polyline(trailCoordinates, { 
        color: '#3b82f6', 
        weight: 4,
        opacity: 0.8,
        lineCap: 'round',
        lineJoin: 'round',
        dashArray: '0, 10',
        dashOffset: '0'
      }).addTo(mapInstanceRef.current);
      
      // Animate the polyline
      let dashOffset = 0;
      const animatePolyline = () => {
        dashOffset -= 0.5;
        polyline.setStyle({ dashOffset: String(dashOffset) });
        requestAnimationFrame(animatePolyline);
      };
      animatePolyline();
      
      // Add current position marker
      L.marker([trailCoordinates[trailCoordinates.length - 1][0], trailCoordinates[trailCoordinates.length - 1][1]], 
        { icon: currentIcon })
        .addTo(mapInstanceRef.current)
        .bindPopup('Current Position');
      
      // Add distance scale
      L.control.scale({
        imperial: false,
        position: 'bottomright'
      }).addTo(mapInstanceRef.current);
    }
    
    // Cleanup function
    return () => {
      if (mapInstanceRef.current) {
        mapInstanceRef.current.remove();
        mapInstanceRef.current = null;
      }
    };
  }, []);

  return (
    <div className="bg-gradient-to-br from-slate-900 to-slate-800 rounded-xl shadow-xl p-4 text-white border border-slate-700">
      <div className="flex justify-between items-center mb-4">
        <h3 className="text-xl font-semibold text-slate-200">Mission Map</h3>
        <div className="flex space-x-4">
          <div className="flex items-center">
            <div className="w-3 h-3 rounded-full bg-blue-600 mr-2"></div>
            <span className="text-xs text-slate-300">Launch Point</span>
          </div>
          <div className="flex items-center">
            <div className="w-3 h-3 rounded-full bg-red-600 mr-2 animate-pulse"></div>
            <span className="text-xs text-slate-300">Current Position</span>
          </div>
        </div>
      </div>
      <div 
        ref={mapRef} 
        className={`h-[600px] rounded-lg border border-slate-700 shadow-inner ${className}`}
      ></div>
      
      <div className="mt-4 grid grid-cols-2 gap-4">
        <div className="bg-slate-800/50 rounded-lg p-3 border border-slate-700">
          <div className="text-xs text-slate-400 mb-1">Current Coordinates</div>
          <div className="flex items-center">
            <MapPin size={16} className="text-red-400 mr-2" />
            <span className="text-sm font-mono text-slate-200">13.083512, 80.271194</span>
          </div>
        </div>
        <div className="bg-slate-800/50 rounded-lg p-3 border border-slate-700">
          <div className="text-xs text-slate-400 mb-1">Heading</div>
          <div className="flex items-center">
            <Navigation size={16} className="text-blue-400 mr-2" />
            <span className="text-sm font-mono text-slate-200">NE (45°)</span>
          </div>
        </div>
      </div>
    </div>
  );
};

export default Map;