within ;
package Simple_car_dymola "Simple model car can be run on simulink and dymola"

  package Experiment

    model Experiment_1
      //XC90 simple model
      Simple_car_dymola.Vehicles.vehicle vehicle;
      constant Real pi=Modelica.Constants.pi;
      Real swdamp=(110*pi)/180 "amplitude of the swd manouvre";
      //  output Real vxkph;
    equation

      //vehicle.Aped =if time<5 then 0 elseif time< 20 then 100 elseif time < 35 then 100 elseif time < 60 then 20 else 0;

      //vehicle.Bped = 0;//if time < 7 then 0 else if time < 17 then 100 elseif time < 20 then 0 elseif time < 25 then 0 else 0;

      //vehicle.trans.r_gear = if time<5 then 0 elseif time< 20 then 0 else 0;

      vehicle.chassis.theta = 0;

      vehicle.swa = if time < 25 then 0 else 1.5;
      //if time < 10 then 0 elseif time < 10 + (1 / 0.7 * 3) / 4 then swdamp * sin(2 * pi * 0.7 * (time - 10)) elseif time < 10 + (1 / 0.7 * 3) / 4 + 0.5 then -swdamp elseif time < 10 + (1 / 0.7 * 4) / 4 + 0.5 then swdamp * sin(2 * pi * 0.7 * (time - 10 - 0.5)) else 0;

      //  vxkph = (vehicle.vx)*18/5;

      //if time < 10 then 0 elseif time < 10 + (1 / 0.7 * 3) / 4 then swdamp * sin(2 * pi * 0.7 * (time - 10)) elseif time < 10 + (1 / 0.7 * 3) / 4 + 0.5 then -swdamp elseif time < 10 + (1 / 0.7 * 4) / 4 + 0.5 then swdamp * sin(2 * pi * 0.7 * (time - 10 - 0.5)) else 0;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=50));
    end Experiment_1;


  end Experiment;

  package Vehicles

    model vehicle
      //car simple model
      //Simple_car_dymola.Components.Vehicle_control_and_actuators.Driveline.transmission_modular
    //    trans(m=chassis.m, st_agear=st_agear);
      Simple_car_dymola.Components.Vehicle_motion.chassis chassis;

      // for quick simulation uncomment the two lines for the wheel model and
      // comment the two brush_tire_model_2 lines

        Simple_car_dymola.Components.Wheels.wheel wheel1(m=chassis.m_f, vx0=
            chassis.vx0);
        Simple_car_dymola.Components.Wheels.wheel wheel2(m=chassis.m_r, vx0=
            chassis.vx0);

    //    Simple_car_dymola.Components.Wheels.brush_tire_model_2 wheel1(vx0=chassis.vx0);
    //    Simple_car_dymola.Components.Wheels.brush_tire_model_2 wheel2(vx0=chassis.vx0);
    //  Simple_car_dymola.Components.Vehicle_control_and_actuators.brakes brake;
    //  input Real Aped; //cnst vx
    //  input Real Bped; //cnst vs
      input Real swa;
      input Real theta;
    //  input Real r_gear; // cnst vx
    //  parameter Integer st_agear= 1; //cnst vx
      output Real ax_sim;
      output Real ax;
      output Real decl;
      output Real vx;
      output Real yaw_angle;
      output Real x;
      output Real y;
      output Real ay;
      output Real delta;
    //  output Integer agear; //cnst vx
      output Real vy;
      output Real yaw_velocity;
    //  output Real w_eng; //cnst cx
    //  output Real Tbw; //cnst vx
    //  output Real Tw;
    //  output Real Tbmax; //cnst vx
      output Real yaw_acc;
    equation
      //Forces
      wheel1.F_x = chassis.Fxf;

      wheel2.F_x = chassis.Fxr;

      wheel1.F_y = chassis.Fyf;

      wheel2.F_y = chassis.Fyr;

      wheel1.Fz = chassis.Fzf;

      wheel2.Fz = chassis.Fzr;

      //Velocity

      wheel1.Vx = chassis.vx;

      wheel2.Vx = chassis.vx;

      wheel1.Vy = chassis.vy + chassis.yaw_velocity*chassis.a;

      wheel2.Vy = chassis.vy - chassis.yaw_velocity*chassis.b;

      wheel1.DeltaWheel = chassis.delta;

      wheel2.DeltaWheel = 0;

    //  wheel1.Brake_torque = brake.Tbw;

    //  wheel2.Brake_torque = brake.Tbw;

      chassis.womega = (wheel1.omega + wheel2.omega)/2;

      // powertrain
    //  r_gear = trans.r_gear;

    //  wheel1.Drive_torque = trans.M_wheel_1;

    //  wheel2.Drive_torque = if r_gear == 1 then trans.rev_torque else trans.M_wheel_2;

    //  chassis.vx = trans.Vx;

    //  wheel1.omega = trans.w_wheel_1;

    //  wheel2.omega = trans.w_wheel_2;

    //  trans.Fx = chassis.Fx;

    //   chassis.vx = 10;
    //
    //   wheel1.omega = 32;
    //
    //   wheel2.omega = 32;

      //
    //  Bped = brake.brakeped;

    //  trans.Aped = Aped;

      swa= chassis.swa;

      theta = chassis.theta;

      yaw_angle = chassis.yaw_angle;

      ay = chassis.ay;

      delta = chassis.delta;

      decl = chassis.decl;

      x = chassis.x;

      y = chassis.y;

      vx = chassis.vx;

      ax_sim =  chassis.axp;

    //  agear = trans.agear;

      vy = chassis.vy;

      yaw_velocity = chassis.yaw_velocity;

    //  w_eng = trans.w_engine;

    //  Tbw = brake.Tbw;

    //  Tw = trans.Tw;

    //  Tbmax = brake.Tbmax;

      ax = chassis.ax;

      yaw_acc = chassis.yaw_acc;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end vehicle;

  end Vehicles;

  package Components

    package Vehicle_control_and_actuators

      package Driveline

        model engine_modular
        //   import SI = Modelica.SIunits;
        //   constant Real pi = Modelica.Constants.pi;
        //   parameter Real m;
        //   //Engine coefficients
        //   parameter SI.Torque Tsplit = 300;
        //   parameter SI.AngularVelocity w_eng_max = 6800*pi/30
        //     "maximum engine rotation";
        //   parameter SI.AngularVelocity w_eng_idl = 700*pi/30
        //                                              " engine idle speed";
        //   parameter SI.Acceleration axh = 1.9 "Upper acceleration limit";
        //   parameter SI.Acceleration axl = 1.6 "Lower acceleration limit";
        //   parameter Real E_factor = 0.95;
        //   parameter Real k = 1 "Boost pressure coefficient";
        //   Real perc_throttle;
        //   Real Aped "Accelerator pedal position [%]";
        //   SI.Velocity Vx "velocity of the vehicle m/s";
        //   SI.Torque torque(start = 0) "engine torque";
        //   SI.Torque T_e;
        //   SI.Torque Te;
        //   SI.Torque Tbase;
        //   SI.Torque Tdynreq;
        //   SI.Torque Ttop;
        //   SI.Torque max_torque;
        //
        //   SI.Force Fx "Force to compute ax";
        //   SI.AngularVelocity w_engine(start = w_eng_idl) "engine speed in rad/s";
        //   Real rpm_engine "engine speed in rpm";
        //      Modelica.Blocks.Tables.CombiTable1Ds tab_torque(table = [0,0;
        //                                                                30.00,140;
        //                                                                68.067,225;
        //                                                                83.77,250;
        //                                                                104.719,290;
        //                                                                125.66,330;
        //                                                                146.6,355;
        //                                                                157.07,365;
        //                                                                178.023,380;
        //                                                                188.5,387;
        //                                                                209.4,400;
        //                                                                261.8,400;
        //                                                                314.16,400;
        //                                                                366.52,400;
        //                                                                418.8,400;
        //                                                                471.23,400;
        //                                                                523.6,395;
        //                                                                549.77,385;
        //                                                                555.01,380;
        //                                                                596.9,378;
        //                                                                628.3,360;
        //                                                                659.73,333;
        //                                                                680.67,317;
        //                                                                712.094,293;
        //                                                                733.03,0],smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments)
        //                                                                                                                                     "Engine speed in rpm and torque in Nm";
        // equation
        //
        //   rpm_engine=  w_engine* 30 / pi;
        //
        //   tab_torque.u = w_engine;
        //
        //   tab_torque.y[1] = max_torque;
        //
        //   perc_throttle = max(min(Aped/100, 1), 0.0);
        //
        //   torque =  perc_throttle*max_torque;//if perc_throttle < 0.8 then (perc_throttle *  ((max_torque)))*(1/0.8) else perc_throttle*max_torque;
        //
        //   Tbase = min(torque,Tsplit);
        //
        //   Tdynreq = torque - Tbase;
        //
        //   der(Ttop) = k*(Tdynreq - Ttop);
        //
        //   T_e = Tbase + Ttop;
        //
        //   Te = noEvent(if Vx < 0 then max(0, T_e) else T_e);
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        end engine_modular;

        model transmission_modular
        //   //car simple model
        //   extends
        //     Simple_car_dymola.Components.Vehicle_control_and_actuators.Driveline.engine_modular;
        //   import SI = Modelica.SIunits;
        //   constant Real pi = Modelica.Constants.pi;
        //    // transmission parameters
        //   parameter Real i_tm[8] = {5.2,3.029,1.96,1.469,1.231,1,0.809,0.673}
        //     "gear ratios from 1st up to 12th gear";
        //   parameter Real i_final = 3.329 "gear ratio final gear";
        //   parameter Real eff_tr = 0.90 "transmission efficiency";
        //   parameter Real upshift[8] = {1.2,3.4,5.3,7.53,10.6,14.1,20,1000000}*1;
        //   parameter Real downshift[8] = {-1000000,0.7,2.2,4.4,6.1,8.6,11,17}*1;
        //   parameter Integer st_agear;
        //   parameter Real Je=0.5  "Engine inertia";
        //
        //   Integer agear(start = st_agear) " the automatic gear";
        //   Real i_T(start = 0) "total transmission ratio";
        //   Real r_gear;
        //   SI.Torque Tc;
        //   SI.Torque Tt;
        //   SI.Torque Tp;
        //   SI.Torque Tf;
        //   SI.Torque Td;
        //   SI.Torque Tw;
        //   SI.AngularVelocity w_c;
        //   SI.AngularVelocity w_t;
        //   SI.AngularVelocity w_p;
        //   SI.AngularVelocity w_f;
        //   SI.AngularVelocity w_w;
        //   SI.Torque M_wheel_1;
        //   SI.Torque M_wheel_2;
        //   SI.AngularVelocity w_wheel_1;
        //   SI.AngularVelocity w_wheel_2;
        //   SI.AngularVelocity w_mean_wheels;
        //   SI.Torque rev_torque;
        //
        //   output Real r_c;
        // equation
        //
        //   i_T = i_final * i_tm[agear];
        //    when Vx > upshift[pre(agear)] then
        //
        //      agear = pre(agear)+1;
        //
        //    elsewhen Vx < downshift[pre(agear)] then
        //
        //      agear = pre(agear)-1;
        //
        //    end when;
        //
        //   Je*der(w_engine) = Te- Tc; // Torque balance according to Newtons law
        //
        //   w_engine = min(w_eng_max,max(w_c,w_eng_idl));
        //
        //   Tc = Tt;
        //
        //   w_c = w_t*i_tm[agear];
        //
        //   Tp = Tt*i_tm[agear];
        //
        //   w_t = w_p;
        //
        //   Tp = Tf;
        //
        //   w_p = w_f*i_final;
        //
        //   eff_tr*Tf*i_final = Td;
        //
        //   Td = Tw;
        //
        //   w_f = w_w;
        //
        //   w_w = w_mean_wheels;
        //
        //   w_mean_wheels = (w_wheel_1+w_wheel_2)/2;
        //
        //   rev_torque = -1*r_c*max(Tt,0)*i_tm[6]*i_final;
        //
        //   r_c = if r_gear > 0 then 1 else 0;
        //
        //   M_wheel_2 = max(Tw*0.5,0);// torque equally distribued to rear wheel
        //
        //   M_wheel_1 = max(Tw*0.5,0);
        //                              // torque equally distribued to front wheel

            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        end transmission_modular;
      end Driveline;

      model brakes
      //   //car simple model
      //   //brakes model
      //   Real brakeped;
      //   Real Tbw;
      //   Real Tbwact;
      //   parameter Real b_delay = 3.5;
      //   parameter Real Tbmax=1800;
      // equation
      //
      //   Tbwact = brakeped*Tbmax*0.01;
      //
      //   der(Tbw) = b_delay*(Tbwact - Tbw);
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end brakes;
    end Vehicle_control_and_actuators;

    package Vehicle_motion

      model chassis
        //car simple model
        //Equation for forces and moments resolved about the CoG
        constant Real g=Modelica.Constants.g_n;
        parameter Real m=2194;
        //mass of the vehicle[kg]
        parameter Real a=1.41;
        //distance from CG to front axel [m]
        parameter Real b=1.576;
        //distance from CG to rear axel [m]
        parameter Real Izz=4770;
        // Mass moment of inertia [kgm^2]
        parameter Real i_s=16;
        // steering ratio
        parameter Real Iw=1;
        // Wheel Inertia[kgm^2]
        parameter Real rw=0.3125;
        //Wheel radius [m]
        //Braking torque[Nm]
        parameter Real m_f=m*(b/(a + b));
        parameter Real m_r=m*(a/(a + b));
        parameter Real Fzf=m*g*(b/(a + b));
        // Front vertical load
        parameter Real Fzr=m*g*(a/(a + b));
        //Rear vertical load
        parameter Real vx0 = 10;
        //initial longitudinal velocity [m/s]
        parameter Real rho=1.225 "air density";
        parameter Real Cd=0.3 "Coefficient of drag";
        parameter Real ac=2.78 "Frontal area";
        parameter Real mu= 0.9 "Road fricition";

        Real vy(start=0.0) "Vehicle Lateral Velocity [m/sec]";
        Real yaw_angle(start=0);
        Real yaw_velocity(start=0);
        Real yaw_acc;
        Real ay;
        Real Fx;
        Real Fxf;
        Real Fyr;
        Real Fxr;
        Real Fd(start=0.0);
        Real Fg; //force due to gradient
        Real delta;
        Real womega;
        Real swa "Steering angle [rad]";
        Real theta;//road gradient
        Real beta(start = 0);
        output Real vx(start=vx0) "Vehicle longitudinal velocity";
        output Real x(start=0);
        output Real y(start=0);
        output Real ax;
        output Real Fyf;
        output Real decl(start=0);
        output Real axp;

      equation
        // Longitudinal Dynamics ---------------

        Fx = Fxf + Fxr + Fd + Fg*tanh(10 * abs(womega)) * tanh(0.5 * abs(vx));

        (m)*ax = (Fx);

        Fd = -sign(vx)*0.5*rho*Cd*ac*vx^2;

        Fg = -m*g*sin(theta);

        // Lateral Dynamics -------------------

        Izz*yaw_acc = (a*Fyf - b*Fyr);

        m*(ay) = (Fyf + Fyr);

        der(yaw_angle) = yaw_velocity;

        der(yaw_velocity) = yaw_acc;

        ay = der(vy) + vx*yaw_velocity;

        ax = der(vx) - vy*yaw_velocity;

        der(x) = vx*cos(yaw_angle) - vy*sin(yaw_angle);

        der(y) = vy*cos(yaw_angle) + vx*sin(yaw_angle);

        decl = if ax > 0 then 0 else ax;

        delta = swa/i_s;

        axp = max(ax,0);

        beta = atan(vy/max(vx,0.01));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end chassis;
    end Vehicle_motion;

    package Wheels

      model brush_tire_model_2

       constant Real g=Modelica.Constants.g_n;
        import SI = Modelica.SIunits;
        constant Real pi = Modelica.Constants.pi;
        parameter Real rw = 0.3125 "Radius of wheel";
        parameter Real mu=0.9;
        parameter Real vx0;
        parameter Real I_tyre = 1;
        parameter Real f_r=0.00075 "Rolling resistance coefficient";
        parameter Real a=0.11 "contact patch length";
        parameter Real K_muv = 1 "fraction stactic vs sliding friction <=1";
        parameter Real mu_i= max(mu, 0.05);
        parameter Real cpx = 270000;
        parameter Real cpy = 145000;
        parameter Real muk = 0.9;
        parameter Real kz = 1e6 "Tyre vertical flexibility";
        parameter Real cz = 1e3 "Tyre vertical damping";
        SI.Force Fz;
        parameter Real t = 0.04 "Pneumatic trail";
        SI.Velocity Vx(start = vx0) "Chassis long velocity";
        SI.Velocity Vy(start = 0.0) "Chassis lat velocity";
        SI.Velocity vx "wheel lon velocity";
        SI.Velocity vy "wheel lat velcoity";
        SI.AngularVelocity omega(start = vx0/rw) "wheel angular velocity";
        SI.Angle DeltaWheel;
        SI.Force Fxw;
        SI.Force Fyw;
        SI.Force F_x;
        SI.Force F_y;
        SI.Force Fr(start=0.0);
        SI.Torque Mz "Aligning toruqe";
        SI.Torque Drive_torque;
        SI.Torque Brake_torque;
        Real sx( start = 0.0001)
                                "instantaneous longitudinal slip";
        Real sy( start = 0.0001)
                                "instantaneous lateral slip ";
        SI.Force F_xi;
        SI.Force F_yi;
        Real s;
        Real F_i(start = 0.0);
        Real s_x_0 "limit longitudinal slip ";
        Real s_y_0 "limit lateral slip ";
        Real psi;

      equation

        vx = Vx * cos(DeltaWheel) + Vy * sin(DeltaWheel);

        vy = (-Vx * sin(DeltaWheel)) + Vy * cos(DeltaWheel);

        sx = -(vx - omega * rw) / max(abs(omega * rw), 0.1);

        sy = -vy / max(abs(omega * rw),0.01);

        s = max(sqrt(sx^2+sy^2),0.01);

        s_x_0 = (3*Fz*mu)/(cpx);

        s_y_0 = (3*Fz*mu)/(cpy);

        psi = sqrt((sx/s_x_0)^2+(sy/s_y_0)^2);

        F_xi = if psi < 1 then -cpx*sx*(1-psi)^2 - Fz*(sx/max(s,0.001))*muk*psi^2*(3-2*psi) else muk*Fz*(sx/max(s,0.001));

        F_yi = if psi < 1 then -cpy*sy*(1-psi)^2 - Fz*(sy/max(s,0.001))*muk*psi^2*(3-2*psi) else muk*Fz*(sy/max(s,0.001));

        F_i = sqrt(F_xi^2+F_yi^2);

        Fxw = F_i*(sx/max(s,0.001));

        Fyw = F_i*(sy/max(s,0.001));

        Mz = if psi < 1 then a*cpy*sy*((3*Fz*muk - cpy*sy)^3/(162*Fz^3*muk^3)) else 0;//if noEvent(s < s_x_0) then (2/3)*sy*C_i*etac^3/a + muk*Fz*(0.5*a - 2*etac*(etac/a)^2 + 1.5*etac*(etac/a)^3) else 0;

        I_tyre * der(omega) = Drive_torque - tanh(10 * omega) * tanh(0.5 * abs(vx)) * Brake_torque - Fxw * rw - Fr * rw;

        Fr = f_r*Fz*min(1, vx);

        F_x = cos(DeltaWheel)*Fxw - sin(DeltaWheel)*Fyw;

        F_y = sin(DeltaWheel)*Fxw + cos(DeltaWheel)*Fyw;

          annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end brush_tire_model_2;

      model wheel
        //car simple model
        //wheel and tire model

        constant Real g=Modelica.Constants.g_n;

        import SI = Modelica.SIunits;

        constant Real pi = Modelica.Constants.pi;
        parameter Real rw = 0.3125 "Radius of wheel";
        parameter Real c=18;
        parameter Real mu=0.9;
        parameter Real vx0;
        parameter Real I_tyre = 1;
        parameter Real f_r=0.00075 "Rolling resistance coefficient";
        parameter Real m;
        SI.Velocity Vx(start = vx0) "Chassis long velocity";
        SI.Velocity Vy(start = 0.0) "Chassis lat velocity";
        SI.Velocity vx "wheel lon velocity";
        SI.Velocity vy "wheel lat velcoity";
        SI.AngularVelocity omega(start = vx0/rw) "wheel angular velocity";
        SI.Force Fxy "Tire Force combined";
        SI.Angle DeltaWheel;
        SI.Force Fz;
        SI.Force Fx;
        SI.Force Fy;
        SI.Force Fr;
        //SI.Torque Drive_torque; //cnst vx
        //SI.Torque Brake_torque; //cnst vx
        Real sxy "combined slip";
        Real fsxy "tire coefficient";
        Real sx "longitudinal slip";
        Real sy "Lateral slip";
        SI.Force F_x;
        SI.Force F_y;
      equation

        vx = Vx * cos(DeltaWheel) + Vy * sin(DeltaWheel);

        vy = (-Vx * sin(DeltaWheel)) + Vy * cos(DeltaWheel);

        sx = -(vx - omega * rw) / max(abs(omega * rw), 0.01);

        sy = vy / max(abs(omega * rw),0.01);

        sxy = sqrt((sx^2)+(sy^2));

        fsxy = 2 / pi * atan(c * (2 / pi) * sxy);

        Fxy =  mu * Fz * fsxy;

        Fx = (rw*omega - vx)*Fxy/max((sqrt((rw*omega - vx)^2 + vy^2)),0.01);

        Fy = -vy*Fxy/max((sqrt((rw*omega - vx)^2 + vy^2)),0.01);

        //(I_tyre) * der(omega) = Drive_torque - tanh(10 * omega) * tanh(0.5 * abs(vx)) * Brake_torque - Fx * rw - Fr*rw; //wheel inertia not added for cnst vx

        omega = 32; //set as 33.3

        Fr = f_r*m*g*min(1, vx);

        F_x = cos(DeltaWheel)*Fx - sin(DeltaWheel)*Fy;

        F_y = sin(DeltaWheel)*Fx + cos(DeltaWheel)*Fy;

        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end wheel;

    end Wheels;
  end Components;

  package SimulationExperiment "file for FMU"
    model simulink_new
      //car simple model
      //   Simple_car_dymola.Components.Vehicle_control_and_actuators.Driveline.transmission_modular
      //     trans(m=chassis.m, st_agear=st_agear);
      Simple_car_dymola.Components.Vehicle_motion.chassis chassis;

      // for quick simulation uncomment the two lines for the wheel model and
      // comment the two brush_tire_model_2 lines

      Simple_car_dymola.Components.Wheels.wheel wheel1(m=chassis.m_f, vx0=chassis.vx0);
      Simple_car_dymola.Components.Wheels.wheel wheel2(m=chassis.m_r, vx0=chassis.vx0);

      //    Simple_car_dymola.Components.Wheels.brush_tire_model_2 wheel1(vx0=chassis.vx0);
      //    Simple_car_dymola.Components.Wheels.brush_tire_model_2 wheel2(vx0=chassis.vx0);


      //   Simple_car_dymola.Components.Vehicle_control_and_actuators.brakes brake;
      // input Real Aped;// commenting for constant velocity model
      // input Real Bped; // commenting for contant velocity model
      input Real swa;
      input Real theta;
      //   input Real r_gear; // commenting reverse gear for constant velocity model
      //   parameter Integer st_agear= 1; // commenting start gear for constant velocity

      output Real ax_sim;
      output Real ax;
      output Real decl;
      output Real vx;
      output Real yaw_angle;
      output Real x;
      output Real y;
      output Real ay;
      output Real delta;
      // output Integer agear; //commenting agear for constant velocity
      output Real vy;
      output Real yaw_velocity;
      // output Real w_eng; // commenting for constant velocity
      //output Real Tbw;    // commenting for constant velocity
      // output Real Tw;   // commenting for constant velocity
      //output Real Tbmax;   // commenting for constant velocity
      output Real yaw_acc;
    equation
      //Forces
      wheel1.F_x = chassis.Fxf;

      wheel2.F_x = chassis.Fxr;

      wheel1.F_y = chassis.Fyf;

      wheel2.F_y = chassis.Fyr;

      wheel1.Fz = chassis.Fzf;

      wheel2.Fz = chassis.Fzr;

      //Velocity

      wheel1.Vx = chassis.vx;

      wheel2.Vx = chassis.vx;

      wheel1.Vy = chassis.vy + chassis.yaw_velocity*chassis.a;

      wheel2.Vy = chassis.vy - chassis.yaw_velocity*chassis.b;

      wheel1.DeltaWheel = chassis.delta;

      wheel2.DeltaWheel = 0;

      //wheel1.Brake_torque = brake.Tbw; //cnst vx

      //wheel2.Brake_torque = brake.Tbw; //cnst vx

      chassis.womega = (wheel1.omega + wheel2.omega)/2;

      // powertrain
      //   r_gear = trans.r_gear; //cnst vx

      //   wheel1.Drive_torque = trans.M_wheel_1; //csnt vx

      //   wheel2.Drive_torque = if r_gear == 1 then trans.rev_torque else trans.M_wheel_2; // cnst vx

      // chassis.vx = trans.Vx; // cnst vx

      chassis.vx = 10;
      // setting to 10m/sec

      //wheel1.omega = trans.w_wheel_1; // need to be constant this is atleast an assumption

      //wheel2.omega = trans.w_wheel_2; // need to be constant this is atleast an assumption

      //   trans.Fx = chassis.Fx;
      // check the link with chassis.Fx?

      //   trans.Aped = Aped;

      wheel1.omega = 32;
      //rad/sec assuming rw = 0.3125m

      wheel2.omega = 32;
      // rad/sec assumning rw = 0.3125m

      //Bped = brake.brakeped;

      swa = chassis.swa;

      theta = chassis.theta;

      yaw_angle = chassis.yaw_angle;

      ay = chassis.ay;

      delta = chassis.delta;

      decl = chassis.decl;

      x = chassis.x;

      y = chassis.y;

      vx = chassis.vx;

      ax_sim = chassis.axp;

      //   agear = trans.agear;

      vy = chassis.vy;

      yaw_velocity = chassis.yaw_velocity;

      //   w_eng = trans.w_engine;

      //Tbw = brake.Tbw;

      //   Tw = trans.Tw;

      //Tbmax = brake.Tbmax;

      ax = chassis.ax;

      yaw_acc = chassis.yaw_acc;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end simulink_new;





  end SimulationExperiment;
  annotation (uses(Modelica(version="3.2.2")));
end Simple_car_dymola;
