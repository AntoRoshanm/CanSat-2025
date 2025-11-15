import React from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import * as THREE from 'three';

interface Gyro3DVisualizerProps {
  gyroX: number; // degrees per second
  gyroY: number;
  gyroZ: number;
}

const RotatingCylinder: React.FC<Gyro3DVisualizerProps> = ({ gyroX, gyroY, gyroZ }) => {
  const ref = React.useRef<THREE.Mesh>(null!);

  useFrame(() => {
    if (ref.current) {
      const factor = (Math.PI / 180) / 60; // degrees to radians / 60 fps
      ref.current.rotation.x += gyroX * factor;
      ref.current.rotation.y += gyroY * factor;
      ref.current.rotation.z += gyroZ * factor;
    }
  });

  return (
    <group>
      {/* Main cylinder body */}
      <mesh ref={ref}>
        <cylinderGeometry args={[1.2, 1.2, 4, 32]} />
        <meshStandardMaterial 
          color="#2563eb" 
          metalness={0.7}
          roughness={0.3}
        />
      </mesh>
      
      {/* Top cap */}
      <mesh ref={ref} position={[0, 2, 0]}>
        <cylinderGeometry args={[1.2, 1.2, 0.2, 32]} />
        <meshStandardMaterial 
          color="#1d4ed8" 
          metalness={0.8}
          roughness={0.2}
        />
      </mesh>
      
      {/* Bottom cap */}
      <mesh ref={ref} position={[0, -2, 0]}>
        <cylinderGeometry args={[1.2, 1.2, 0.2, 32]} />
        <meshStandardMaterial 
          color="#1d4ed8" 
          metalness={0.8}
          roughness={0.2}
        />
      </mesh>
      
      {/* Center band detail */}
      <mesh ref={ref}>
        <cylinderGeometry args={[1.25, 1.25, 0.3, 32]} />
        <meshStandardMaterial 
          color="#1e40af" 
          metalness={0.9}
          roughness={0.1}
        />
      </mesh>
    </group>
  );
};

const Gyro3DVisualizer: React.FC<Gyro3DVisualizerProps> = (props) => {
  return (
    <div className="w-full h-96 bg-slate-900 rounded-xl shadow-xl border border-slate-700 p-4">
      <div className="mb-4">
        <h3 className="text-white text-lg font-semibold mb-2">CanSat Gyroscope Visualization</h3>
        <div className="grid grid-cols-3 gap-4 text-sm">
          <div className="text-center">
            <div className="text-red-400 font-medium">X-Axis</div>
            <div className="text-white">{props.gyroX.toFixed(1)}°/s</div>
          </div>
          <div className="text-center">
            <div className="text-green-400 font-medium">Y-Axis</div>
            <div className="text-white">{props.gyroY.toFixed(1)}°/s</div>
          </div>
          <div className="text-center">
            <div className="text-blue-400 font-medium">Z-Axis</div>
            <div className="text-white">{props.gyroZ.toFixed(1)}°/s</div>
          </div>
        </div>
      </div>
      <Canvas camera={{ position: [6, 4, 6], fov: 50 }}>
        <ambientLight intensity={0.4} />
        <directionalLight position={[10, 10, 5]} intensity={1.2} />
        <pointLight position={[-10, -10, -5]} intensity={0.5} color="#3b82f6" />
        <RotatingCylinder {...props} />
        <gridHelper args={[10, 10]} />
        <axesHelper args={[3]} />
      </Canvas>
    </div>
  );
};

export default Gyro3DVisualizer;