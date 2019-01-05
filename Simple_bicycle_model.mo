package Bicycle_model
model Bicycle_equ
  
  parameter Real wb = 2.807; //wheel base of the car
  parameter Real b = 1.4; // CG to rear axle.
  parameter Real a = wb - b; // CG to front axle.
  parameter Real u = 100*(5/18); // longitudinal velocity
  parameter Real is = 16; // steering ratio 
  parameter Real cyf = 105500; // cornering stiffness front
  parameter Real cyr = 316000; // cornering stiffness rear
  parameter Real ma = 1870; // Mass of the car
  parameter Real Izz = 2500; // Moment of inertia about z axis.
  parameter Real g = 9.81;
  Real delta;
  Real delta_swa;
  Real vy (start = 0); // lateral verlocity 
  Real vydot (start = 0); // derivative lateral velocity
  Real r (start = 0); // yaw rate
  Real rdot (start = 0); // yaw acceleration
  Real ay ;// lateral acceleration 
  constant Real pi=Modelica.Constants.pi;
equation

  vydot = der(vy);
  rdot = der(r);
  
  delta_swa = if time < 20 then 0 else 75;
  
  delta = delta_swa*(pi/180)/(is);
  
  
  
  ma*(vydot + u*r) = cyf*(delta - ((vy + a*r)/(u))) + cyr*(-(vy - b*r)/(u));
  
  Izz*rdot = a*cyf*(delta - ((vy + a*r)/(u))) - b*cyr*(-(vy - b*r)/(u));
  
  ay = vydot + u*r;
  
    annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=50));
end Bicycle_equ;














end Bicycle_model;
