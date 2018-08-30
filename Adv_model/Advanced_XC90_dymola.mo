within ;
package Advanced_XC90_dymola
  "Advanced model for XC90 can be simulated on Dymola and Simulink"
  // XC90 Advanced Model
  // some parts of the code are from vehilib2 written  by Fredrik Bruzelius and Jorge Gomez Fernandez
  // Changes made to the Powertain and transmission block.

  package Vehicle
    // XC90 Advanced Model
    model two_track
      import SI = Modelica.SIunits;
      //parameters for the two track and assembly of all components
      parameter SI.Mass m = 2194 "mass of the full vehicle";
      parameter SI.Mass m_usp = 260.516 "total unsprung mass";
      parameter SI.Mass mu_f = m_usp * L2 / (L1 + L2) / 2
        " 48.509 unsprung mass front per wheel";
      parameter SI.Mass mu_r = m_usp * L1 / (L1 + L2) / 2
        "46.093 unsprung mass rear per wheel";
      parameter SI.Mass m_s = m - 2 * mu_f - 2 * mu_r "sprung mass";
      parameter SI.Length R_nom =  0.347 "Nominal road wheel radius";
      parameter SI.Length h_CG = 0.68 "Centre of Gravity height,[m]";
      parameter SI.Length L1 = 1.41
        "Distance between COG and front axle,[m]";
      parameter SI.Length L2 = 1.576
        "Distance between COG and rear axle,[m]";
      parameter SI.Length TW_f = 1.722 "front track width,[m]";
      parameter SI.Length TW_r = 1.730 "rear track width,[m]";
      parameter SI.Inertia I_z = 4770
        "Yaw moment of inertia (about z-axis at CG)";
      parameter SI.Inertia I_y = 4446.3
        "Pitch moment of inertia (about y-axis at CG)";
      parameter SI.Inertia I_x = 894.402
        "Roll moment of inertia (about x-axis at CG)";
      parameter SI.Distance h_frc = 0.115
        "0.25 Front suspension roll centre heigth (m)";
      parameter SI.Distance h_rrc = 0.12
        "0.3 Rear suspension roll centre heigth (m)";
      parameter SI.Distance h_pc = 0.29;
      constant Real pi = Modelica.Constants.pi;
      parameter Real K0_f = 32000/2 "Spring suspension stiffness [N/m] front";
      parameter Real K0_r = 27000/2 "Spring stiffness [N/m] rear";
      parameter Real Dfb_f = 2750 "damper fast bounce front";
      parameter Real Dfr_f = 900;
      parameter Real Dsb_f = 10000.0;
      parameter Real Dsr_f = 5500 "damper slow rebounce front";
      parameter Real v_rebound_f = -0.132 "breakpoint fast/slow front";
      parameter Real v_bump_f = 0.131;
      parameter Real Dfb_r = 3137 "damper fast rebounce front";
      parameter Real Dfr_r = 3137;
      parameter Real Dsb_r = 3137;
      parameter Real Dsr_r = 3137 "damper slow rebounce front";
      parameter Real v_rebound_r = -0.132 "breakpoint fast/slow front";
      parameter Real v_bump_r = 0.131;
      parameter Real k_antiroll_r= 367 * 180 / pi*TW_r/2
        "anti-rollbar stifness rear";
      parameter Real k_antiroll_f = 652 * 180 / pi*TW_f/2 " --";
      // Tire model parameters
      parameter Real cx = 25000 "Tire longitudinal stiffness";
      parameter Real cy = 18000 "Tire vertical stiffness";
      parameter Real Kt= 250e3 "Carcass vertical stiffness";
      parameter Real Ct=10e3 "Carcass vertical damping";
      parameter Integer st_agear = 1;
      parameter Real vx0 = 0;
      Components.Wheels_and_suspension.brush_tire_model_2 wheel1(
        rw=0.3125,
        vx0=vx0,
        Fz0=suspension.F_z_f0,
        cpx=cx,
        cpy=cy);
      Components.Wheels_and_suspension.brush_tire_model_2 wheel2(
        rw=0.3125,
        vx0=vx0,
        Fz0=suspension.F_z_f0,
        cpx=cx,
        cpy=cy);
      Components.Wheels_and_suspension.brush_tire_model_2 wheel3(
        rw=0.3125,
        vx0=vx0,
        Fz0=suspension.F_z_r0,
        cpx=cx,
        cpy=cy);
      Components.Wheels_and_suspension.brush_tire_model_2 wheel4(
        rw=0.3125,
        vx0=vx0,
        Fz0=suspension.F_z_r0,
        cpx=25000,
        cpy=18000);
      Components.Vehicle_motion.chassis chassis(m=m, vxstart=vx0);
      Advanced_XC90_dymola.Components.Wheels_and_suspension.Suspension.suspension
        suspension(
        L1=L1,
        L2=L2,
        TW_f=TW_f,
        TW_r=TW_r,
        h_CG=h_CG,
        m=m,
        mu_f=mu_f,
        mu_r=mu_r,
        R_nom=wheel1.rw,
        K0_f=K0_f,
        Dfb_f=Dfb_f,
        Dfr_f=Dfr_f,
        Dsb_f=Dsb_f,
        Dsr_f=Dsr_f,
        v_rebound_f=v_rebound_f,
        v_bump_f=v_bump_f,
        k_antiroll_f=k_antiroll_f,
        K0_r=K0_r,
        Dfb_r=Dfb_r,
        Dfr_r=Dfr_r,
        Dsb_r=Dsb_r,
        Dsr_r=Dsr_r,
        v_rebound_r=v_rebound_r,
        v_bump_r=v_bump_r,
        k_antiroll_r=k_antiroll_r,
        h_frc=h_frc,
        h_rrc=h_rrc,
        h_pc=h_pc,
        Kt=Kt,
        Ct=Ct);
      Advanced_XC90_dymola.Components.Vehicle_control_and_actuators.Steering.steering_new
        steering(L=(L1 + L2), TW=TW_f);
    //     Modular_5.Steering.try_steering steering;

      Advanced_XC90_dymola.Components.Vehicle_control_and_actuators.Powertrain.transmission
        powertrain(st_agear=st_agear);
      Components.Vehicle_control_and_actuators.brakes brake;
      SI.Position z_mean "mean value of height of wheel positions";
      SI.Position zr[4] "position of the road under all 4 wheels with zero mean";
      parameter Real zwheel[4] = {0,0,0,0};
      input Real Aped;
      input Real Bped;
      input Real swa;
      input Real r_gear;
      output Real ax_sim;
      output Real ax;
      output Real decl;
      output Real vx;
      output Real yaw_angle;
      output Real x;
      output Real y;
      output Real ay;
      output Real delta;
      output Integer agear;
      output Real vy;
      output Real yaw_velocity;
      output Real w_eng;
      output Real Tbw;
      output Real Tw;
      output Real Tbmax;
      output Real yaw_acc;
    equation
        //road
        z_mean = (zwheel[1] + zwheel[2] + zwheel[3] + zwheel[4]) / 4;
        zr[1] = zwheel[1] - z_mean;
        zr[2] = zwheel[2] - z_mean;
        zr[3] = zwheel[3] - z_mean;
        zr[4] = zwheel[4] - z_mean;
        chassis.road_slope = atan((zr[1] + zr[2] - zr[3] - zr[4]) / 2 / (L1 + L2));
        chassis.road_bank = atan((zr[1] + zr[3] - zr[2] - zr[4]) / 2 / (TW_f + TW_r) / 2);
      //Forces
      chassis.Fxf1 = wheel1.F_x;
      chassis.Fxf2 = wheel2.F_x;
      chassis.Fxr3 = wheel3.F_x;
      chassis.Fxr4 = wheel4.F_x;
      chassis.Fyf1 = wheel1.F_y;
      chassis.Fyf2 = wheel2.F_y;
      chassis.Fyr3 = wheel3.F_y;
      chassis.Fyr4 = wheel4.F_y;

      chassis.ext_Fx = 0;
      chassis.ext_Fy = 0;
      chassis.ext_Fz = 0;
      chassis.ext_Mx = 0;
      chassis.ext_My = 0;
      chassis.ext_Mz = 0;
      //Velocity of vehicle and wheel

         chassis.vxw1 = wheel1.Vx;
         chassis.vxw2 = wheel2.Vx;
         chassis.vxw3 = wheel3.Vx;
         chassis.vxw4 = wheel4.Vx;

         chassis.vyw1 = wheel1.Vy;
         chassis.vyw2 = wheel2.Vy;
         chassis.vyw3 = wheel3.Vy;
         chassis.vyw4 = wheel4.Vy;

         //angle
         chassis.del_w1 = wheel1.DeltaWheel;
         chassis.del_w2 = wheel2.DeltaWheel;
         chassis.del_w3 = wheel3.DeltaWheel;
         chassis.del_w4 = wheel4.DeltaWheel;

         steering.DeltaWheel_1 = wheel1.DeltaWheel;
         steering.DeltaWheel_2 = wheel2.DeltaWheel;
         steering.DeltaWheel_3 = wheel3.DeltaWheel;
         steering.DeltaWheel_4 = wheel4.DeltaWheel;
          steering.Mz = (wheel1.Mz + wheel2.Mz)/2;
         steering.Delta = swa;
         // suspension
         suspension.pitch_angle = chassis.phi;
         suspension.roll_angle = chassis.theta;
         //chassis and suspension forces
         suspension.FC1 = chassis.FC1;
         suspension.FC2 = chassis.FC2;
         suspension.FC3 = chassis.FC3;
         suspension.FC4 = chassis.FC4;

         // the road profile at each wheel
         suspension.zr1 = zr[1];
         suspension.zr2 = zr[2];
         suspension.zr3 = zr[3];
         suspension.zr4 = zr[4];
        // forces to the suspension from the wheels
         wheel1.F_y = suspension.F_y_1;
         wheel2.F_y = suspension.F_y_2;
         wheel3.F_y = suspension.F_y_3;
         wheel4.F_y = suspension.F_y_4;
         wheel1.F_x = suspension.F_x_1;
         wheel2.F_x = suspension.F_x_2;
         wheel3.F_x = suspension.F_x_3;
         wheel4.F_x = suspension.F_x_4;
         wheel1.Fz = suspension.F_z_1;
         wheel2.Fz = suspension.F_z_2;
         wheel3.Fz = suspension.F_z_3;
         wheel4.Fz = suspension.F_z_4;
         // vertical motion of the COG
         suspension.d_roll = chassis.d_roll;
         suspension.d_pitch = chassis.d_pitch;
         suspension.z = chassis.z;

         //powertrain
         powertrain.Aped = Aped;
         powertrain.w_wheel_1 = wheel1.omega;
         powertrain.w_wheel_2 = wheel2.omega;
         powertrain.w_wheel_3 = wheel3.omega;
         powertrain.w_wheel_4 = wheel4.omega;
         powertrain.Vx = chassis.vx;
         powertrain.r_gear = r_gear;
         vx = chassis.vx;
         wheel1.Drive_torque = if r_gear == 1 then powertrain.rev_torque else powertrain.M_wheel_1;
         wheel2.Drive_torque = if r_gear == 1 then powertrain.rev_torque else powertrain.M_wheel_2;
         wheel3.Drive_torque = if r_gear == 1 then powertrain.rev_torque else powertrain.M_wheel_3;
         wheel4.Drive_torque = if r_gear == 1 then powertrain.rev_torque else powertrain.M_wheel_4;

         //brakes
         brake.brakeped = Bped;
         wheel1.Brake_torque = brake.Tbw;
         wheel2.Brake_torque = brake.Tbw;
         wheel3.Brake_torque = brake.Tbw;
         wheel4.Brake_torque = brake.Tbw;
         ax_sim = chassis.axp;
         decl = chassis.decl;
         ax = chassis.ax;
         ay = chassis.ay;
         delta = chassis.del_w1;
         agear = powertrain.agear;
         yaw_velocity = chassis.psidot;
         Tbw = brake.Tbw;
         w_eng = powertrain.w_engine;
         Tbmax = brake.Tbmax;
         Tw = powertrain.Tw;
         yaw_angle = chassis.psi;
         yaw_acc = chassis.psiddot;
         x = chassis.x;
         y = chassis.y;
         vy = chassis.vy;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end two_track;
  end Vehicle;

  package Experiment

    model Experiment_1
      // XC90 Advanced Model
      // Modelica simulation file
      // vary the Aped,bped and SWA
      Advanced_XC90_dymola.Vehicle.two_track vehicle;
        constant Real pi = Modelica.Constants.pi;
        Real swa;
          Real swdamp = (110 * pi) / 180 "amplitude of the swd manouvre";

    equation
      vehicle.r_gear = 0;
      vehicle.Bped = if time < 25 then 0 else if time < 30 then 10 else 0 "in percent";
      vehicle.Aped = if time < 5 then 0 else if time < 25 then 25 else 0 "in percent";
      vehicle.swa = if time < 5 then 0 else if time < 15 then 12 else if time < 17 then 0 else if time < 20 then -1.5 else 0 "in radians";

      //if time < 10 then 0 elseif time < 10 + (1 / 0.7 * 3) / 4 then swdamp * sin(2 * pi * 0.7 * (time - 10)) elseif time < 10 + (1 / 0.7 * 3) / 4 + 0.5 then -swdamp elseif time < 10 + (1 / 0.7 * 4) / 4 + 0.5 then swdamp * sin(2 * pi * 0.7 * (time - 10 - 0.5)) else 0;
      swa = der(if time < 5 then 0 else if time < 11.28 then sin(time - 5) else 0);
      annotation (experiment(
          StopTime=50,
          __Dymola_fixedstepsize=0.0005,
          __Dymola_Algorithm="Dassl"),
                             Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Experiment_1;
  end Experiment;

  package Components

    package Vehicle_motion

      model chassis
        // XC90 Advanced Model
        // This file is used from vehlib2.
        import SI = Modelica.SIunits;
        constant SI.Acceleration g = Modelica.Constants.g_n
          "gravity acceleration [m/s^2]";
        constant SI.Density air_dens = 1.225 "air density [kg/m^3]";
        constant Real pi = Modelica.Constants.pi;
        // Chassis parameters
        parameter Real Cd = 0.3 "vehicle drag coefficient";
        parameter Real Af = 2.78 "Vechicle frontal area, m^2";
        parameter Modelica.SIunits.Mass m "vehicle curb mass+driver(75 kg) [kg]";
        parameter Modelica.SIunits.Mass m_uns = 260.516 "vehicle unsprung mass";
        parameter Modelica.SIunits.Mass m_s = m - m_uns "vehicle sprung mass";
        parameter Modelica.SIunits.Length h_CG = 0.683 "Centre of Gravity height,[m]";
        parameter Modelica.SIunits.Distance h_frc = 0.045
          "Front suspension roll centre heigth (m)";
        parameter Modelica.SIunits.Distance h_rrc = 0.101
          "rear suspension roll centre heigth (m)";
       parameter Modelica.SIunits.Distance h_pc = 0.29
          "Suspension pitch center height (m)";
         parameter Modelica.SIunits.Length L1 = 1.41
          "Distance between COG and front axle,[m]";
        parameter Modelica.SIunits.Length L2 = 1.576
          "Distance between COG and rear axle,[m]";
        parameter Modelica.SIunits.Length TW_f = 1.722 "front track width,[m]";
        parameter Modelica.SIunits.Length TW_r = 1.730 "rear track width,[m]";
        parameter Modelica.SIunits.Inertia I_x = 668
          "moment of inertia, x axis [kg*m^2]";
        parameter Modelica.SIunits.Inertia I_y = 3848
          "moment of inertia, y axis,[kg*m^2]";
        parameter Modelica.SIunits.Inertia I_z = 3992
          "moment of inertia, vertical axis[kg*m^2]";
        parameter Modelica.SIunits.Inertia I_xy = 0
          "product of inertia, vertical axis[kg*m^2]";
        parameter Modelica.SIunits.Inertia I_xz = 0
          "product of inertia, vertical axis[kg*m^2]";
        parameter Real f_r = 0.00164 "Rolling resistance coefficient";
        //parameter Real dc = 0.01 "caster distance";
        //parameter Real d_pitch = 0.25
        //  "distance between COG and pitch axis. Pitch axis below COG, m";
        parameter Real vxstart = 0 "initial vehicle speed";
        // suspension parameters used by the chassis model
        SI.Position d_roll "distance between CG and RC m";
        SI.Position d_pitch "distancse between CG and PC m";
        SI.Force Fyf1 "Front wheel 1 or inner lateral Force N";
        SI.Force Fyf2 "Front wheel 2 or outer lateral Force N";
        SI.Force Fyr3 "Rear wheel 3 or inner lateral Force N";
        SI.Force Fyr4 "Rear wheel 4 or outer lateral force N";
        SI.Force Fxf1(start = 0) "Front wheel 1 or inner longitudinal Force N";
        SI.Force Fxf2(start = 0) "Front wheel 2 or outer longitudinal Force N";
        SI.Force Fxr3(start = 0) "Rear wheel 3 or inner longitudinal Force N";
        SI.Force Fxr4(start = 0) "Rear wheel 4 or outer longitudinal force N";
        //SI.Torque Mz1 "aligning torque wheel1";
        //SI.Torque Mz2 "aligning torque wheel2";
        //SI.Torque Mz3 "aligning torque wheel3";
        //SI.Torque Mz4 "aligning torque wheel4";
        SI.Force ext_Fx(start = 0) "external longitudinal force COG";
        SI.Force ext_Fy(start = 0) "external lateral force COG";
        SI.Force ext_Fz(start = 0) "external vertical force COG";
        SI.Torque ext_Mx(start = 0) "external moment cog about 'x' roll Nm";
        SI.Torque ext_My(start = 0) "external moment cog about 'y' pitch Nm";
        SI.Torque ext_Mz(start = 0) "external moment cog about 'z' yaw Nm";
        SI.Angle del_w1(start = 0) "steering angle wheel 1 rad";
        SI.Angle del_w2(start = 0) "steering angle wheel 2 rad";
        SI.Angle del_w3(start = 0) "steering angle wheel 3 rad";
        SI.Angle del_w4(start = 0) "steering angle wheel 4 rad";
        SI.Angle phi(start = 0) "pitch angle rad y ";
        SI.Angle theta(start = 0) "roll angle rad x";
        SI.Angle psi(start = 0) "yaw angle rad z";
        SI.AngularVelocity phidot "pitch rate rad/s y";
        SI.AngularVelocity thetadot(start = 0) "roll rate rad x";
        SI.AngularVelocity psidot(start = 0) "yaw rate rad z";
        SI.AngularAcceleration phiddot "pitch acc rad/s^2 y";
        SI.AngularAcceleration thetaddot(start = 0) "roll acc rad/s^2 x";
        SI.AngularAcceleration psiddot(start = 0) "yaw acc rad/s^2 z";
        SI.Velocity vx(start = vxstart)  "vehicle velocity longitudinal m/s";
        SI.Velocity vy(start = 0)  "vehicle velocity lateral m/s";
        SI.Velocity vz(start = 0)  "vehicle velocity vertical m/s";
        SI.Acceleration ax(start = 0) "longitudinal vehicle acc m/s^2";
        SI.Acceleration ay(start = 0) "lateral vehicle acc m/s^2";
        SI.Acceleration az(start = 0) "Vertical vehicle acc m/s^2";
        SI.Position x(start = 0) "x for trajectory m";
        SI.Position y(start = 0) "y for trajectory m";
        SI.Position z(start = h_CG) "z for trajectory m";
        SI.Angle road_slope(start = 0) "road slope";
        SI.Angle road_bank(start = 0) "road banking";
        SI.Velocity vxw1(start = vxstart) "wheel hub 1 speed in long dir";
        SI.Velocity vxw2(start = vxstart) "wheel hub 2 speed in long dir";
        SI.Velocity vxw3(start = vxstart) "wheel hub 3 speed in long dir";
        SI.Velocity vxw4(start = vxstart) "wheel hub 4 speed in long dir";
        SI.Velocity vyw1(start = 0) "wheel hub 1 speed in lat dir";
        SI.Velocity vyw2(start = 0) "wheel hub 2 speed in lat dir";
        SI.Velocity vyw3(start = 0) "wheel hub 3 speed in lat dir";
        SI.Velocity vyw4(start = 0) "wheel hub 4 speed in lat dir";
        SI.Force F_rolling(start = 0) "rolling resistance force";
        SI.Force FC1(start = 0) "suspension force at wheel1";
        SI.Force FC2(start = 0) "suspension force at wheel2";
        SI.Force FC3(start = 0) "suspension force at wheel3";
        SI.Force FC4(start = 0) "suspension force at wheel4";
        SI.Force F_gs(start = m_s * g)
          "gravitational force on sprung mass";
        SI.Angle body_slip_angle(start = 0) "the body slip angle";
        SI.Force Fd(start = 0) "drag force";
        SI.Acceleration decl;
        SI.Acceleration axp;

      equation
        F_gs = m_s * g * cos(road_slope) * cos(road_bank);
        m*ax = Fxf1*cos(del_w1) - Fyf1*sin(del_w1) + Fxf2*cos(del_w2) - Fyf2*sin(del_w2) + Fxr3*cos(del_w3) - Fyr3*sin(del_w3) + Fxr4*cos(del_w4) - Fyr4*sin(del_w4)  + Fd - m*g*sin(road_slope) + ext_Fx;
        ax = der(vx) - vy*psidot + vz*phidot;
        F_rolling = -f_r * m * g * min(1, vx);
        Fd = -sign(vx)*0.5*air_dens*Cd*Af*vx^2;
        m*ay = Fyf1*cos(del_w1) + Fxf1*sin(del_w1) + Fyf2*cos(del_w2) + Fxf2*sin(del_w2) + Fyr3*cos(del_w3) + Fxr3*sin(del_w3) + Fyr4*cos(del_w4) + Fxr4*sin(del_w4) + m * g * sin(-road_bank) * cos(road_slope) + ext_Fy;
        ay = der(vy) + vx*psidot - vz*thetadot;
        FC1 + FC2 + FC3 + FC4 - F_gs + ext_Fz = m_s * az;
        der(z) = vz;
        az = der(vz) - vx*psidot + vy*thetadot;
        //Yaw moment balance
        ext_Mz + (Fyf1 * cos(del_w1) - Fxf1 * sin(del_w1)) * L1 + (-Fxf2 * sin(del_w2) + Fyf2 * cos(del_w2)) * L1 - (-Fxr3 * sin(del_w3) + Fyr3 * cos(del_w3)) * L2 - (-Fxr4 * sin(del_w4) + Fyr4 * cos(del_w4)) * L2 + (Fxf1 * cos(del_w1) + Fyf1 * sin(del_w1)) * TW_f / 2 - (Fxf2 * cos(del_w2) + Fyf2 * sin(del_w2)) * TW_f / 2 + (Fxr3 * cos(del_w3) + Fyr3 * sin(del_w3)) * TW_r / 2 - (Fxr4 * cos(del_w4) + Fyr4 * sin(del_w4)) * TW_r / 2 = I_z *psiddot - I_xy * thetaddot + (I_y - I_x) * thetadot * phidot + I_xz * phidot * psidot;
        /*Fx down*/
                                                                                                                                                                                                                /*- (M_z_1 + M_z_2 + M_z_3 + M_z_4) - (Fyf1 + Fyf2 + Fyr3 + Fyr4) * dc*/
        //ext_Mz + (Fyf1*cos(del_w1)+Fxf1*sin(del_w1))*L1 + (Fyf2*cos(del_w2) + Fxf2*sin(del_w2))*L1 - (Fyr3*cos(del_w3) + Fxr3*sin(del_w3))*L2 -(Fyr4*cos(del_w4)+Fxr4*sin(del_w4))*L2 -(Fxf1*cos(del_w1)-Fyf1*sin(del_w1))*TW_f/2 + (Fxf1*cos(del_w2)-Fyf2*sin(del_w2))*TW_f/2 -(Fxr3*cos(del_w3)-Fyr3*sin(del_w3))*TW_r/2 + (Fxr4*cos(del_w4)-Fyr4*sin(del_w4))*TW_r/2  = I_z *psiddot - I_xy * thetaddot + (I_y - I_x) * thetadot * phidot + I_xz * phidot * psidot;
        /*Fx up */
                                                                                                                                                                                  /*- (M_z_1 + M_z_2 + M_z_3 + M_z_4)*/
                                                                                                                                                                                                                /* - (Fyf1 + Fyf2 + Fyr3 + Fyr4) * dc*/
        // Roll moment balance
        //(LHS CCW +ve )
        (I_x+ m_s * d_roll ^ 2)  * thetaddot - I_xz * psiddot + (I_z - I_y) * phidot * psidot + I_z * phidot * psidot - I_xz * thetadot * phidot = (FC1 - FC2) * TW_f / 2 + (FC3 - FC4) * TW_r / 2 - (Fyf1 * cos(del_w1) + Fyf2 * cos(del_w2)) * h_frc - (Fyr3 * cos(del_w3) + Fyr4 * cos(del_w4)) * h_rrc + F_gs * d_roll * theta + m_s*ay*d_roll+ ext_Mx;
        //Pitch Moment balance
        (I_y + m_s * d_pitch ^ 2) * phiddot + (I_x - I_z) * thetadot * psidot + I_xy * thetadot ^ 2 - I_xz * psidot ^ 2 = (-(FC1 + FC2) * L1) + (FC3 + FC4) * L2 - (Fxf1 * cos(del_w1) + Fxf2 * cos(del_w2) + Fxr3 * cos(del_w3) + Fxr4 * cos(del_w4)) * h_pc + F_gs * d_pitch * phi + m_s * d_pitch * ax +ext_My;

        psiddot = der(psidot);
        psidot = der(psi);
        phiddot = der(phidot);
        phidot = der(phi);
        thetaddot = der(thetadot);
        thetadot = der(theta);
        //tyre longitudinal velocities at the wheel hubs
        vxw1 = vx - psidot * TW_f / 2;
        vxw2 = vx + psidot * TW_f / 2;
        vxw3 = vx - psidot * TW_r / 2;
        vxw4 = vx + psidot * TW_r / 2;
        //tyre lateral velocities at the wheel hubs
        vyw1 = vy + psidot * L1;
        vyw2 = vy + psidot * L1;
        vyw3 = vy - psidot * L2;
        vyw4 = vy - psidot * L2;
        body_slip_angle = atan2(vy,vx);
        der(x) = vx*cos(psi) - vy*sin(psi);
        der(y) = vy*cos(psi) + vx*sin(psi);
        decl = if ax > 0 then 0 else ax;
        axp = max(ax,0);

        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
                    Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
                    Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end chassis;

      annotation ();
    end Vehicle_motion;

    package Wheels_and_suspension

      package Suspension

        model suspension
          /* 
                                    This file is used from vehlib2.
                                                                                                                                                                                                                            
                                */
            // XC90 Advanced Model
          constant Modelica.SIunits.Acceleration g = Modelica.Constants.g_n
            "gravity acceleration [m/s^2]";
          constant Real pi = Modelica.Constants.pi;
          // Suspension parameters
          parameter Modelica.SIunits.Distance h_frc = 0.045
            "Front suspension roll centre heigth (m)";
          parameter Modelica.SIunits.Distance h_rrc = 0.101
            "rear suspension roll centre heigth (m)";
          parameter Modelica.SIunits.Distance h_pc = 0.29
            "Suspension pitch center height (m)";
            parameter Real Kt "Tire carcass vertical stiffness";
            parameter Real Ct "Tire carcass vertical damping";
          // anti-roll bar parameters
          parameter Real G = 84000000000.0
            "Steel transverse displacement module G=2/5*E [N/m^2]";
          parameter Modelica.SIunits.Distance d_f = 0.016
            "front antiroll bar diameter, m";
          parameter Modelica.SIunits.Distance d_r = 0.014
            "rear antiroll bar diameter, m";
          parameter Modelica.SIunits.Distance L_f = 0.9 "front antiroll bar length, m";
          parameter Modelica.SIunits.Distance L_r = 0.8 "rear antiroll bar length, m";
          parameter Modelica.SIunits.Distance L_lever_f = 0.25
            "front antiroll bar lever, m";
          parameter Modelica.SIunits.Distance L_lever_r = 0.3
            "rear antiroll bar lever, m";
          // visible external constants
          parameter Modelica.SIunits.Distance L1
            "Distance between COG and front axle,[m]";
          parameter Modelica.SIunits.Distance L2
            "Distance between COG and rear axle,[m]";
          parameter Modelica.SIunits.Distance TW_f "Track width front";
          parameter Modelica.SIunits.Distance TW_r "Track width rear";
          parameter Modelica.SIunits.Length h_CG = 0.543 "Centre of Gravity height,[m]";
          parameter Modelica.SIunits.Mass m "Vehicle mass";
          parameter Modelica.SIunits.Distance R_nom "wheel radius [m]";
          // masses
          parameter Modelica.SIunits.Mass m_uns = 260.56 "vehicle unsprung mass";
          parameter Modelica.SIunits.Mass mu_f = m_uns * L2 / (L1 + L2) / 2
            "unsprung mass wheel 1 and 2";
          parameter Modelica.SIunits.Mass mu_r = m_uns * L1 / (L1 + L2) / 2
            "unsprung mass wheel 3 and 4";
          parameter Modelica.SIunits.Mass ms_f = m * L2 / (L1 + L2) / 2 - mu_f
            "sprung mass over wheels 1 and 2";
          parameter Modelica.SIunits.Mass ms_r = m * L1 / (L1 + L2) / 2 - mu_r
            "sprung mass over wheels 3 and 4";
          //dampers
          parameter Real Dfr_f = 400 "front fast rebound damping coefficient";
          parameter Real Dsr_f = 5000 "front slow rebound damping coefficient";
          parameter Real Dsb_f = 8000 "front slow bump damping coefficient";
          parameter Real Dfb_f = 1000 "front fast bump damping coefficient";
          parameter Real v_bump_f = 0.125
            "front damper velocity of transition between slow and fast bump";
          parameter Real v_rebound_f = -0.125
            "front damper velocity of transition between slow and fast rebound";
          parameter Real Dfr_r = 4000 "rear fast rebound damping coefficient";
          parameter Real Dsr_r = 4000 "rear slow rebound damping coefficient";
          parameter Real Dsb_r = 4000 "rear slow bump damping coefficient";
          parameter Real Dfb_r = 4000 "rear fast bump damping coefficient";
          parameter Real v_bump_r = 0.135
            "rear damper velocity of transition between fast and slow bump";
          parameter Real v_rebound_r = -0.25
            "rear damper velocity of transition between fast and slow rebound";
          //nonlinear spring parameters
          parameter Modelica.SIunits.Mass m_GV = m + 75 * 4 + 30 * 5
            "total gross vehicle mass";
          parameter Modelica.SIunits.Force F_GVM_f = m_GV * L2 / (2 * (L1 + L2)) * g
            "Force on the suspensions 1 and 2 in full weight condition";
          parameter Modelica.SIunits.Force F_GVM_r = m_GV * L1 / (2 * (L1 + L2)) * g
            "Force on the suspensions 3 and 4 in curb weight condition";
          parameter Modelica.SIunits.Force F_curb_f = ms_f * g
            "Force on the suspensions 1 and 2 in curb weight condition";
          parameter Modelica.SIunits.Force F_curb_r = ms_r * g
            "Force on the suspensions 3 and 4 in curb weight condition";
          parameter Modelica.SIunits.Force F_bump_1 = 3 * F_GVM_f
            "Force on the suspensions 1 and 2 in full bump condition";
          parameter Modelica.SIunits.Force F_bump_3 = 3 * F_GVM_r
            "Force on the suspensions 3 and 4 in full bump condition";
          // nonlinear front springs
          parameter Real Ff = 1.27 "Front sprung mass natural frequency";
          parameter Real K0_f = ms_f * (2 * pi * Ff) ^ 2
            "Stiffness of the front suspensions in the normal operative region";
          parameter Real zGVM_f = (F_GVM_f - F_curb_f) / K0_f
            "Maximum front spring displacement before bump stopper intervention (rel. to curb condition)";
          parameter Real z_bump_f = 0.14
            "Maximum front spring displacement during full bump (relative to curb condition)";
          parameter Real K_bump_f = (F_bump_1 - F_GVM_f) / (z_bump_f - zGVM_f)
            "Front bump stopper stiffness";
          //nonlinear rear springs
          parameter Real Fr = 1.5 "Rear sprung mass natural frequency";
          parameter Real K0_r = ms_r * (2 * pi * Fr) ^ 2
            "Stiffness of the rear suspensions in the normal operative region";
          parameter Real zGVM_r = (F_GVM_r - F_curb_r) / K0_r
            "Maximum rear spring displacement before bump stopper intervention";
          parameter Real z_bump_r = 0.14
            "Maximum front spring displacement during full bump (relative to curb condition)";
          parameter Real K_bump_r = (F_bump_3 - F_GVM_r) / (z_bump_r - zGVM_r)
            "Rear bump stopper stiffness";
          parameter Real k_antiroll_f = G * I_antiroll_f * L_f / L_lever_f ^ 2;
          parameter Real k_antiroll_r = G * I_antiroll_r * L_r / L_lever_r ^ 2;
          parameter Modelica.SIunits.Force F_z_f0 = m * g * L2 / (L1 + L2) / 2
            "front nominal load";
          parameter Modelica.SIunits.Force F_z_r0 = m * g * L1 / (L1 + L2) / 2
            "rear nominal load";
          Modelica.SIunits.Position d_roll
            "distance between COG and pitch axis. Pitch axis below COG, m";
          Modelica.SIunits.Position d_pitch
            "distance between COG and pitch axis. Pitch axis below COG, m";
          Modelica.SIunits.Position z(start = h_CG) "vehicle vertical position";
          Modelica.SIunits.Force F_z_1(start = F_z_f0)
            "the vertical force at front left wheel";
          Modelica.SIunits.Force F_z_2(start = F_z_f0);
          Modelica.SIunits.Force F_z_3(start = F_z_r0);
          Modelica.SIunits.Force F_z_4(start = F_z_r0);
          Modelica.SIunits.Force F_x_1 "the long force at front left wheel";
          Modelica.SIunits.Force F_x_2 "the long force at front right wheel";
          Modelica.SIunits.Force F_x_3 "the long force at rear left wheel";
          Modelica.SIunits.Force F_x_4 "the long force at rear right wheel";
          Modelica.SIunits.Force F_y_1 "the lat force at front left wheel";
          Modelica.SIunits.Force F_y_2 "the lat force at front right wheel";
          Modelica.SIunits.Force F_y_3 "the lat force at rear left wheel";
          Modelica.SIunits.Force F_y_4 "the lat force at rear right wheel";
          Modelica.SIunits.Position zr1 "road profile under tire 1";
          Modelica.SIunits.Position zr2 "road profile under tire 2";
          Modelica.SIunits.Position zr3 "road profile under tire 3";
          Modelica.SIunits.Position zr4 "road profile under tire 4";
          Modelica.SIunits.Force FC1
            "Force transmitted by the suspension of wheel 1 to the sprung mass";
          Modelica.SIunits.Force FC2
            "Force transmitted by the suspension of wheel 2 to the sprung mass";
          Modelica.SIunits.Force FC3
            "Force transmitted by the suspension of wheel 3 to the sprung mass";
          Modelica.SIunits.Force FC4
            "Force transmitted by the suspension of wheel 4 to the sprung mass";
          Modelica.SIunits.Angle pitch_angle;
          Modelica.SIunits.Angle roll_angle;
        protected
          parameter Real I_antiroll_f = pi * d_f ^ 4 / 32;
          parameter Real I_antiroll_r = pi * d_r ^ 4 / 32;
          //Real Delta_Fz_f;
          //Real Delta_Fz_r;
          //Real DeltaFz_pitch(start = 0);
          Advanced_XC90_dymola.Components.Wheels_and_suspension.Suspension.springdamp
            spring1(
            R_nom=R_nom,
            Dfr=Dfr_f,
            Dsr=Dsr_f,
            Dsb=Dsb_f,
            Dfb=Dfb_f,
            v_bump=v_bump_f,
            v_rebound=v_rebound_f,
            mu=mu_f,
            ms=ms_f,
            K0=K0_f,
            zGVM=zGVM_f,
            K_bump=K_bump_f,
            h_CG=h_CG,
            Kt=Kt,
            Ct=Ct);
          Advanced_XC90_dymola.Components.Wheels_and_suspension.Suspension.springdamp
            spring2(
            R_nom=R_nom,
            Dfr=Dfr_f,
            Dsr=Dsr_f,
            Dsb=Dsb_f,
            Dfb=Dfb_f,
            v_bump=v_bump_f,
            v_rebound=v_rebound_f,
            mu=mu_f,
            ms=ms_f,
            K0=K0_f,
            zGVM=zGVM_f,
            K_bump=K_bump_f,
            h_CG=h_CG,
            Kt=Kt,
            Ct=Ct);
          Advanced_XC90_dymola.Components.Wheels_and_suspension.Suspension.springdamp
            spring3(
            R_nom=R_nom,
            Dfr=Dfr_r,
            Dsr=Dsr_r,
            Dsb=Dsb_r,
            Dfb=Dfb_r,
            v_bump=v_bump_r,
            v_rebound=v_rebound_r,
            mu=mu_r,
            ms=ms_r,
            K0=K0_r,
            zGVM=zGVM_f,
            K_bump=K_bump_r,
            h_CG=h_CG,
            Kt=Kt,
            Ct=Ct);
          Advanced_XC90_dymola.Components.Wheels_and_suspension.Suspension.springdamp
            spring4(
            R_nom=R_nom,
            Dfr=Dfr_r,
            Dsr=Dsr_r,
            Dsb=Dsb_r,
            Dfb=Dfb_r,
            v_bump=v_bump_r,
            v_rebound=v_rebound_r,
            mu=mu_r,
            ms=ms_r,
            K0=K0_r,
            zGVM=zGVM_f,
            K_bump=K_bump_r,
            h_CG=h_CG,
            Kt=Kt,
            Ct=Ct);
        equation
          //------------------------- SUSPENSION  ----------------
          /*
                                                                                                                                                                                                                                                                                                       In the suspension, the roll and pitch stiffness and damping are calculated from the parameters of the suspension.
                                                                                                                                                                                                                                                                                                       Then the vertical load at each tire, used as an input for the tire model is calculated considering the static wieght 
                                                                                                                                                                                                                                                                                                       distribution, the influence of road slope and banking in the static weight distribution and also the dynamics load transfer
                                                                                                                                                                                                                                                                                                       generated for longitudinal and lateral accelerations
                                                                                                                                                                                                                                                                                                    */
          // the antiroll-bar forces
          spring1.Far = k_antiroll_f * (spring1.zs - spring2.zs);//+200*der(spring1.zs - spring2.zs);
          spring2.Far = -spring1.Far;
          spring3.Far = k_antiroll_r * (spring3.zs - spring4.zs);//+200*der(spring1.zs - spring2.zs);
          spring4.Far = -spring3.Far;
          // displacement of the sprung part of the springdamper system
          spring1.zs = z - L1 * pitch_angle + TW_f * roll_angle / 2;
          spring2.zs = z - L1 * pitch_angle - TW_f * roll_angle / 2;
          spring3.zs = z + L2 * pitch_angle + TW_r * roll_angle / 2;
          spring4.zs = z + L2 * pitch_angle - TW_r * roll_angle / 2;
          spring1.FC = FC1;
          spring2.FC = FC2;
          spring3.FC = FC3;
          spring4.FC = FC4;
          spring1.zr = zr1;
          spring2.zr = zr2;
          spring3.zr = zr3;
          spring4.zr = zr4;
          spring1.F_z = F_z_1;
          spring2.F_z = F_z_2;
          spring3.F_z = F_z_3;
          spring4.F_z = F_z_4;
          d_roll = z - (L2 * h_frc + L1 * h_rrc) / (L1 + L2);
          d_pitch = z - h_pc;

        end suspension;

        model springdamp
        /* 
This file is used from vehlib2.
*/

          // tire related
          parameter Real R_nom "tyre nominal radius [m], for 205/55 R16 tires";
          parameter Real Kt "tire vertical stiffness N/m";
          parameter Real Ct "tire vertical damping";
          // damper related
          parameter Real Dfr "fast rebound damping coefficient";
          parameter Real Dsr "slow rebound damping coefficient";
          parameter Real Dsb "slow bump damping coefficient";
          parameter Real Dfb "fast bump damping coefficient";
          parameter Real v_bump
            "damper velocity of transition between slow and fast bump";
          parameter Real v_rebound
            "damper velocity of transition between slow and fast rebound";
          // spring related
          parameter Real K0
            "Stiffness of the suspension in the normal operative region";
          parameter Real zGVM
            "Maximum spring displacement before bump stopper intervention (relative to curb condition)";
          parameter Real K_bump "Front bump stopper stiffness";
          // mass related
          parameter Modelica.SIunits.Mass mu "unsprung mass";
          parameter Modelica.SIunits.Mass ms "sprung mass";
          parameter Modelica.SIunits.Length h_CG;
          constant Modelica.SIunits.Acceleration g = Modelica.Constants.g_n
            "gravity acceleration [m/s^2]";

          //Modelica.SIunits.Velocity V_zs(start = 0) "velocity of the sprung mass";
          Modelica.SIunits.Position z "spring displacement";
          Modelica.SIunits.Velocity v "schock absorber velocity";
          Modelica.SIunits.Position zu(start = R_nom)
            "displacement of the unsprung mass (wheel hub)";
          Modelica.SIunits.Velocity V_zu(start = 0) "velocity of the wheel hub";
          Modelica.SIunits.Acceleration A_zu(start = 0) "acceleration of the wheel hub";
          Modelica.SIunits.Force Fs "spring force of suspension";
          Modelica.SIunits.Force Fd "damper force of suspension";
          parameter Modelica.SIunits.Time T_dzr = 0.05
            "Filter constant of derivative of the zr";
          Real zr_int "Internal filer state of the zr filer";
          Real zr_der "the filtered version of der(zr)";
          parameter Real k = 100 "transision gain between straight lines using tanh";

          Modelica.SIunits.Position zs "displacement of the sprung mass";
          Modelica.SIunits.Force Far "anit-rollbar force of suspension";
          Modelica.SIunits.Position zr "Road profile under";
          Modelica.SIunits.Force FC
            "Force transmitted by the suspension to the sprung mass";
          Modelica.SIunits.Force F_z "Vertial tire force";
        equation
          //a fix to force relative degree down, a filtered version of the zr
          // implements a version of: 0.05*Delta_der+Delta_der = der(Delta) without derivating Delta
          T_dzr * der(zr_int) + zr_int = zr / T_dzr;
          zr_der = zr / T_dzr - zr_int;
          mu * A_zu - F_z + FC + mu * g = 0;
          A_zu = der(V_zu);
          V_zu = der(zu);
          //V_zs = der(zs);
          //FC = max(-Fs - Fd - Far,-mu*g);// + ms * g;
          FC =-(Fs + Fd) - Far;// + ms * g;
          //F_z = max(Kt * (zr - zu + R_nom + (ms + mu) * g / Kt) + Ct * (-der(zu)), 0);
          F_z = max(Kt * (zr - zu + R_nom + (ms + mu) * g / Kt) + Ct * (zr_der - der(zu)), 0);
          //F_z = max(Kt * (zr - zu+R_nom)  + Ct * (zr_der - der(zu)), 0);
          //F_z= Kt * (zr - zu + R_nom+(ms+mu)*g/Kt) + Ct * (zr_der - der(zu));
           z = -(zu - R_nom  - zs + h_CG + ms*g/K0);
          Fs = K0 * z;// + (1 + tanh(k * (z + zGVM))) / 2 * (K_bump - K0) * (z + zGVM);
          //Fs = K_bump * z + (1 + tanh(k * (z - zGVM))) / 2 * (K0-K_bump)  * (z - zGVM);
          //Fs=K0*z;
          v = der(z);
          //V_zu - V_zs;
          Fd = Dfr * (v - v_rebound) + Dsr * v_rebound + (1 + tanh(k * (v - v_rebound))) / 2 * (Dsr - Dfr) * (v - v_rebound) + (1 + tanh(k * v)) / 2 * (Dsb - Dsr) * v + (1 + tanh(k * (v - v_bump))) * (Dfb - Dsr) * (v - v_bump);
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        end springdamp;
      end Suspension;

      model brush_tire_model_2

       constant Real g=Modelica.Constants.g_n;
        import SI = Modelica.SIunits;
        constant Real pi = Modelica.Constants.pi;
        parameter Real rw  "Radius of wheel";
        parameter Real mu=0.9;
        parameter Real vx0;
        parameter Real I_tyre = 1;
        parameter Real f_r=0.00075 "Rolling resistance coefficient";
        parameter Real a=0.11 "contact patch length";
        parameter Real K_muv = 1 "fraction stactic vs sliding friction <=1";
        parameter Real mu_i= max(mu, 0.05);
        parameter Real cpx;
        parameter Real cpy;
        parameter Real muk = 0.9;
        parameter Real kz = 1e6 "Tyre vertical flexibility";
        parameter Real cz = 1e3 "Tyre vertical damping";
        parameter Real t = 0.04 "Pneumatic trail";
        parameter Real Fz0;
        SI.Force Fz( start = Fz0);
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
        SI.Torque T_roll(start=0.0);
        SI.Torque Mz "Aligning toruqe";
        SI.Torque Drive_torque;
        SI.Torque Brake_torque;
        Real sx( start = 0.0)   "instantaneous longitudinal slip";
        Real sy( start = 0.0)   "instantaneous lateral slip ";
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
        sx = -(vx - omega * rw) / max(abs(omega * rw), 0.03);
        sy = -vy / max(abs(omega * rw),0.03);
        s = max(sqrt(sx^2+sy^2),0.03);
        s_x_0 = (3*Fz*mu)/(cpx);
        s_y_0 = (3*Fz*mu)/(cpy);
        psi = sqrt((sx/max(s_x_0,0.03))^2+(sy/max(s_y_0,0.03))^2);
        F_xi = if psi < 1 then -cpx*sx*(1-psi)^2 - Fz*(sx/max(s,0.03))*muk*psi^2*(3-2*psi) else muk*Fz*(sx/max(s,0.03));
        F_yi = if psi < 1 then -cpy*sy*(1-psi)^2 - Fz*(sy/max(s,0.03))*muk*psi^2*(3-2*psi) else muk*Fz*(sy/max(s,0.03));
        F_i = sqrt(F_xi^2+F_yi^2);
        Fxw = F_i*(sx/max(s,0.03));
        Fyw = F_i*(sy/max(s,0.03));
        Mz = Fyw*0.002;//if psi < 1 then a*cpy*sy*((3*Fz*muk - cpy*sy)^3/(162*Fz^3*muk^3)) else 0;//if noEvent(s < s_x_0) then (2/3)*sy*C_i*etac^3/a + muk*Fz*(0.5*a - 2*etac*(etac/a)^2 + 1.5*etac*(etac/a)^3) else 0;
        I_tyre * der(omega) = Drive_torque - tanh(10 * omega) * tanh(0.5 * abs(vx)) * Brake_torque - Fxw * rw - T_roll;
        T_roll = f_r*Fz*min(1, vx)*rw;
        F_x = cos(DeltaWheel)*Fxw - sin(DeltaWheel)*Fyw;
        F_y = sin(DeltaWheel)*Fxw + cos(DeltaWheel)*Fyw;

          annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end brush_tire_model_2;
    end Wheels_and_suspension;

    package Vehicle_control_and_actuators

      package Powertrain

        model transmission
          // XC90 advanced model
          extends
            Advanced_XC90_dymola.Components.Vehicle_control_and_actuators.Powertrain.engine_modular;
        //   extends Modular_3.Powertrain.Torque_converter;
          import SI = Modelica.SIunits;
          constant Real pi = Modelica.Constants.pi;
           // transmission parameters
          parameter Real i_tm[8] = {5.2,2.972,1.96,1.469,1.231,1,0.824,0.685}
            "gear ratios from 1st up to 12th gear";
          parameter Real i_final = 3.329 "gear ratio final gear";
          parameter Real eff_tr = 0.90 "transmission efficiency";
          parameter Real upshift[8] = {1.2,3.4,5.3,7.53,10.6,14.1,20,1000000}*1;
          parameter Real downshift[8] = {-1000000,0.7,2.2,4.4,6.1,8.6,11,17}*1;
          parameter SI.Torque max_drive_torque = 3.45e5;
          parameter Integer st_agear;
          parameter Real Je = 0.5;
          Integer agear(start = st_agear) " the automatic gear";
          Real i_T(start = 0) "total transmission ratio";
          Real r_gear;
          SI.Torque Tt;
          SI.Torque Tp;
          SI.Torque Tf;
          SI.Torque Td;
          SI.Torque Tw;
          SI.Torque Tc;
          SI.AngularVelocity w_t;
          SI.AngularVelocity w_p;
          SI.AngularVelocity w_f;
          SI.AngularVelocity w_w;
          SI.AngularVelocity w_c;
          SI.Torque M_wheel_1;
          SI.Torque M_wheel_2;
          SI.Torque M_wheel_3;
          SI.Torque M_wheel_4;
          SI.AngularVelocity w_wheel_1;
          SI.AngularVelocity w_wheel_2;
          SI.AngularVelocity w_wheel_3;
          SI.AngularVelocity w_wheel_4;
          SI.AngularVelocity w_mean_wheels;
          SI.Torque rev_torque;
        //   Real startup_facotr( start = 0);
          output Real r_c;
        equation

          i_T = i_final * i_tm[agear];
           when Vx > upshift[pre(agear)] then
             agear = pre(agear)+1;
           elsewhen Vx < downshift[pre(agear)] then
             agear = pre(agear)-1;
           end when;

          // Je*der(w_engine) = Te - T_impeller;
            Je*der(w_engine) = Te - Tc;

        //   w_engine = w_impeller;

        //   startup_facotr = if Aped > 0 then 1 else 0;
        //   T_turbine*startup_facotr = Tt;

        //   w_turbine = w_t*i_tm[agear];
        w_engine = min(w_eng_max,max(w_c,w_eng_idl));

          Tc = Tt;

          w_c = w_t*i_tm[agear];
          Tp = Tt*i_tm[agear];
          w_t = w_p;

          Tp = Tf;

          w_p = w_f*i_final;
          eff_tr*Tf*i_final = Td;

          Td = Tw;

          w_f = w_w;
          w_w = w_mean_wheels;
          w_mean_wheels = (w_wheel_1+w_wheel_2 +w_wheel_3+w_wheel_4) /4;
          rev_torque = -1*r_c*max(Tt,0)*i_tm[6]*i_final;
          r_c = if r_gear > 0 then 1 else 0;

          M_wheel_2 = max(Tw*0.25,0);
          M_wheel_1 = max(Tw*0.25,0);
          M_wheel_3 = max(Tw*0.25,0);
          M_wheel_4 = max(Tw*0.25,0);
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        end transmission;

        model Torque_converter
        import SI = Modelica.SIunits;
          constant Real pi = Modelica.Constants.pi;
          SI.AngularVelocity w_turbine(start = 0);
          SI.AngularVelocity w_impeller(start = w_eng_idl);
          SI.Torque T_impeller;
          SI.Torque T_turbine;
          Real Speed_ratio(start = 0);
          Real Torque_ratio;
          Real Kfactor;
          parameter SI.AngularVelocity w_eng_max = 6800*pi/30
            "maximum engine rotation";
          parameter SI.AngularVelocity w_eng_idl = 700*pi/30
                                                     " engine idle speed";
          Modelica.Blocks.Tables.CombiTable1D kfact_spdratio(tableOnFile=true,
          tableName="kfact_spdratio",
          fileName="loadfile_trqconv.mat",
          smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments) "u1:Speed_ratio, y:Kfactor";

          Modelica.Blocks.Tables.CombiTable1D trq_spdratio(tableOnFile=true,
          tableName="trq_spdratio",
          fileName="loadfile_trqconv.mat",
          smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments) "u1:Speed_ratio, y:Torque_ratio";

        equation

          w_turbine = Speed_ratio*min(w_eng_max,max(w_impeller,w_eng_idl));
          kfact_spdratio.u[1] = Speed_ratio;
          kfact_spdratio.y[1] = Kfactor;

          T_impeller = (w_impeller)^2*Kfactor;

          trq_spdratio.u[1] = Speed_ratio;
          trq_spdratio.y[1] = Torque_ratio;

          T_turbine = T_impeller*Torque_ratio*1.25;

        end Torque_converter;

        model engine_modular
          import SI = Modelica.SIunits;
          constant Real pi = Modelica.Constants.pi;
          //Engine coefficients
          parameter SI.Torque Tsplit = 300;
          parameter SI.AngularVelocity w_eng_max = 6800*pi/30
            "maximum engine rotation";
          parameter SI.AngularVelocity w_eng_idl = 700*pi/30
                                                     " engine idle speed";
          parameter SI.Acceleration axh = 1.9 "Upper acceleration limit";
          parameter SI.Acceleration axl = 1.6 "Lower acceleration limit";
          parameter Real k = 1 "Boost pressure coefficient";
          Real perc_throttle;
          Real Aped "Accelerator pedal position [%]";
          SI.Velocity Vx "velocity of the vehicle m/s";
          SI.Torque torque(start = 0) "engine torque";
          SI.Torque T_e;
          SI.Torque Te;
          SI.Torque Tbase;
          SI.Torque Tdynreq;
          SI.Torque Ttop;
          SI.Torque max_torque;
          SI.AngularVelocity w_engine(start = w_eng_idl) "engine speed in rad/s";
          Real rpm_engine "engine speed in rpm";
             Modelica.Blocks.Tables.CombiTable1Ds tab_torque(table = [0,0;
                                                                       1.04,140;
                                                                       68.067,225;
                                                                       83.77,250;
                                                                       104.719,290;
                                                                       125.66,330;
                                                                       146.6,355;
                                                                       157.07,365;
                                                                       178.023,380;
                                                                       188.5,387;
                                                                       209.4,400;
                                                                       261.8,400;
                                                                       314.16,400;
                                                                       366.52,400;
                                                                       418.8,400;
                                                                       471.23,400;
                                                                       523.6,395;
                                                                       549.77,385;
                                                                       555.01,380;
                                                                       596.9,378;
                                                                       628.3,360;
                                                                       659.73,333;
                                                                       680.67,317;
                                                                       712.094,293;
                                                                       733.03,0],smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments)
                                                                                                                                            "Engine speed in rpm and torque in Nm";
        equation
          rpm_engine=  w_engine* 30 / pi;
          tab_torque.u = w_engine;
          tab_torque.y[1] = max_torque;
          perc_throttle = max(min(Aped/100, 1), 0.0);
          torque = (perc_throttle *  ((max_torque))); // if perc_throttle < 0.8 then (perc_throttle *  ((max_torque)))*(1/0.8) else (perc_throttle *  ((max_torque)));
          Tbase = min(torque,Tsplit);
          Tdynreq = torque - Tbase;
          der(Ttop) = k*(Tdynreq - Ttop);
          T_e = Tbase + Ttop;
          Te = noEvent(if Vx < 0 then max(0, T_e) else T_e);
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        end engine_modular;

        model transmission_modular
          //XC90 simple model
          extends
            Advanced_XC90_dymola.Components.Vehicle_control_and_actuators.Powertrain.engine_modular;
          extends
            Advanced_XC90_dymola.Components.Vehicle_control_and_actuators.Powertrain.Torque_converter;
          import SI = Modelica.SIunits;
          constant Real pi = Modelica.Constants.pi;
           // transmission parameters
          parameter Real i_tm[8] = {5.2,3.029,1.96,1.469,1.231,1,0.809,0.673}
            "gear ratios from 1st up to 8th gear";
          parameter Real i_final = 3.329 "gear ratio final gear";
          parameter Real eff_tr = 0.90 "transmission efficiency";
          parameter Real upshift[8] = {1.2,3.4,5.3,7.53,10.6,14.1,20,1000000}*1;
          parameter Real downshift[8] = {-1000000,0.7,2.2,4.4,6.1,8.6,11,17}*1;
          parameter Integer st_agear;
          parameter Real Je=0.5  "Engine inertia";
          Integer agear(start = st_agear) " the automatic gear";
          Real i_T(start = 0) "total transmission ratio";
          Real r_gear;
          //SI.Torque Tc;
          SI.Torque Tt;
          SI.Torque Tp;
          SI.Torque Tf;
          SI.Torque Td;
          SI.Torque Tw;
          //SI.AngularVelocity w_c;
          SI.AngularVelocity w_t;
          SI.AngularVelocity w_p;
          SI.AngularVelocity w_f;
          SI.AngularVelocity w_w;

          SI.Torque M_wheel_1;
          SI.Torque M_wheel_2;
          SI.Torque M_wheel_3;
          SI.Torque M_wheel_4;
          SI.AngularVelocity w_wheel_1;
          SI.AngularVelocity w_wheel_2;
          SI.AngularVelocity w_wheel_3;
          SI.AngularVelocity w_wheel_4;
          SI.AngularVelocity w_mean_wheels;
          SI.Torque rev_torque;
          Real startup_facotr( start = 0);
          output Real r_c;
        equation

          i_T = i_final * i_tm[agear];
           when Vx > upshift[pre(agear)] then
             agear = pre(agear)+1;
           elsewhen Vx < downshift[pre(agear)] then
             agear = pre(agear)-1;
           end when;

          Je*der(w_engine) = Te - T_impeller;
          w_engine = w_impeller;

          startup_facotr = if Aped > 0 then 1 else 0;
          T_turbine*startup_facotr = Tt;

          w_turbine = w_t*i_tm[agear];
          Tp = Tt*i_tm[agear];

          w_t = w_p;
          Tp = Tf;

          w_p = w_f*i_final;
          eff_tr*Tf*i_final = Td;

          Td = Tw;

          w_f = w_w;
          w_w = w_mean_wheels;
          w_mean_wheels = (w_wheel_1+w_wheel_2+w_wheel_3+w_wheel_4)/4;
          rev_torque = -1*r_c*max(Tt,0)*i_tm[6]*i_final;
          r_c = if r_gear > 0 then 1 else 0;
          M_wheel_2 = max(Tw*0.25,0);// torque equally distribued to rear wheel
          M_wheel_1 = max(Tw*0.25,0);
          M_wheel_3 = max(Tw*0.25,0);     // torque equally distribued to rear wheel
          M_wheel_4 = max(Tw*0.25,0);                             // torque equally distribued to front wheel

            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        end transmission_modular;
      end Powertrain;

      model brakes
        // XC90 Advanced Model
        Real brakeped;
        Real Tbw;
        Real Tbwact;
        parameter Real b_delay = 3.5;
        parameter Real Tbmax = 1800;
      equation
        Tbwact = brakeped*Tbmax*0.01;
        der(Tbw) = b_delay*(Tbwact - Tbw);
         annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end brakes;

      package Steering

        model try_steering
          /* 
  This file is used from vehlib2.
*/

          /*
  // visible variables
  Modelica.SIunits.Angle DeltaWheel_1(start = 0)
    " steering angle of the front left wheel";
  Modelica.SIunits.Angle DeltaWheel_2(start = 0);
  Modelica.SIunits.Angle DeltaWheel_3(start = 0);
  Modelica.SIunits.Angle DeltaWheel_4(start = 0);
  Modelica.SIunits.Angle Delta(start = 0) "LEFT TURN>0; RIGHT TURN<0; +-6 RAD";
  Modelica.SIunits.Torque Stw_torque "The steering wheel torque";
  Modelica.SIunits.Force F_y_1(start = 0) "lateral force front left wheel";
  Modelica.SIunits.Force F_y_2(start = 0);
  Modelica.SIunits.Force F_y_3(start = 0);
  Modelica.SIunits.Force F_y_4(start = 0);
  Modelica.SIunits.Force F_x_1(start = 0);
  Modelica.SIunits.Force F_x_2(start = 0);
  Modelica.SIunits.Force F_z_1;
  Modelica.SIunits.Force F_z_2;
  Modelica.SIunits.Torque M_z_1(start = 0);
  Modelica.SIunits.Torque M_z_2(start = 0);
  Modelica.SIunits.Torque M_z_3(start = 0);
  Modelica.SIunits.Torque M_z_4(start = 0);
  Modelica.SIunits.Angle roll_angle(start = 0) "the vehicles roll angle";
  Modelica.SIunits.Force F_rolling;
  parameter Modelica.SIunits.Length L1
    "Distance between COG and front axle,[m]";
  parameter Modelica.SIunits.Length L2 "Distance between COG and rear axle,[m]";
  parameter Modelica.SIunits.Length R_nom
    "tyre nominal radius [m], for 205/55 R16 tires";
  parameter Real Stw_friction_coeff = 0.7
    "Coulomb friction coefficient of steering wheel";
  parameter Real Stw_damp_coeff = 0.5 "Damping coeffficient of steering wheel";
  parameter Real dc = R_nom * sin(abs(Caster)) "caster distance";
  constant Real pi = Modelica.Constants.pi;
  // Steering system parameters
  parameter Real SR = 15.9 "Steering ratio";
  parameter Real SAL = 0.22 "Steering arm lever, aprox value [m]";
  parameter Real rp = 0.0138 "Steering system pinion radius, aprox value [m]";
  parameter Real roll_steer_front = 0.1
    "Roll steer coeff. for a front wheel, towards toe-in for jounce and towards toe-out for bounce";
  parameter Real roll_steer_rear = 0.04
    "Roll steer coeff. for a rear wheel, towards toe-in for jounce and towards toe-out for bounce";
  parameter Real Comp_Fy_front = 1.2217e-006
    "Lateral force compliance for a front wheel, toe-out for the outer wheel and toe-in for the inner wheel [rad/N]--default value (Chalmers VDA): 1.2217e-6 [rad/N] ";
  parameter Real Comp_Fy_rear = 5.236e-007
    " Lateral force compliance for a rear wheel, toe-out for the outer wheel and toe-in for the inner wheel [rad/N]--default value (Chalmers VDA): 5.2360e-7 [rad/N]";
  parameter Real Comp_Mz_front = 2.7925e-005
    "Aligning moment for a front wheel [rad/Nm]--default value (Chalmers VDA): 2.7925e-5 [rad/Nm]";
  parameter Real Comp_Mz_rear = 8.726700000000001e-006
    "Aligning moment for a rear wheel [rad/Nm]--//default value (Chalmers VDA): 8.7267e-6 [rad/Nm]";
  parameter Real static_toe_front = 0.3 * pi / 180
    "static toe angle [rad]; >0 ----->  Toe in: 0.3 degress";
  parameter Real static_toe_rear = 0.25 * pi / 180
    "static toe angle [rad]; <0 -----> Toe out";
  parameter Real r_KP = 0.015 "scrub radius";
  parameter Real Caster = -0.095 "caster angle [rad]";
  parameter Real KP = 0.08699999999999999 "King pin angle [rad]";
  parameter Real C = 100;
  parameter Real m_rack = 30 "rack mass";
  parameter Real A_servo = 0.001 "Area of the piston";
  parameter Real D_rack = 10 "rack damping";
  parameter Real K_torsionbar = 100 "Torsion bar stiffness";
  parameter Real D_torsionbar = 0.3 "Torsion bar damping";
  parameter Real t = 0.01 "tire caster";
  parameter Real A = 10000 * 4 "10000.0 * 4 45000 Assistance coefficient";
  parameter Modelica.SIunits.Angle Delta_max = 540 * pi / 180
    "maximum/minimum steering angle";
  Real Fr;
  Real t_1 "Pneumatic trail wheel1";
  Real t_2 "Pneumatic trail wheel2";
protected 
  Modelica.SIunits.Torque M1
    "Resistant torque produced by front left wheel around steeering axis";
  Modelica.SIunits.Torque M2
    "Resistant torque produced by front right wheel around steeering axis";
  //Modelica.SIunits.Force Fr1 "Resistant force produced by front left wheel on the rack";
  //Modelica.SIunits.Force Fr2 "Resistant force produced by front right wheel on the rack";
  Modelica.SIunits.Force Fr1
    "Resistant force produced by front left and right wheel on the rack";
  Modelica.SIunits.Force Fr2;
  Modelica.SIunits.Angle teta_bar(start = 0) "Torsion bar angular displacement";
  Modelica.SIunits.Angle teta_p "pinion angle";
  //Modelica.SIunits.Angle Delta_filt "Filtered version of the steering input";
  Modelica.SIunits.Pressure P_servo "servo assistance pressure";
  Modelica.SIunits.Position x_rack "rack displacement";
  Modelica.SIunits.Velocity v_rack "rack velocity";
  Modelica.SIunits.Force F_servo "Servo assistance force";
  //Modelica.SIunits.Force Fp "Force acting on the pinion";
  Modelica.SIunits.Force FX1
    "Total front left tire force in the longitudinal direction";
  Modelica.SIunits.Force FX2
    "Total front right tire force in the longitudinal direction";
  Modelica.SIunits.Force FY1
    "Total front left tire force in the lateral direction";
  Modelica.SIunits.Force FY2
    "Total front right tire force in the lateral direction";
  Modelica.SIunits.Force F_roll_res1
    "% of rolling resistance force acting on wheels 1 and 2";
  Modelica.SIunits.Force F_roll_res3
    "% of rolling resistance force acting on wheels 3 and 4";
  Modelica.SIunits.AngularVelocity Delta_der
    "Filterd version of the drivative of the swa";
  Real d_int "internal filter state of der(swa)";
  parameter Modelica.SIunits.Time T_dswa = 0.01
    "time constant for der(swa) filer";
  parameter Real bcc = 1.5 "Boost curve cut from qubic to linear part";
equation 
  //a fix to force relative degree down, a filtered version of the swa
  // implements a version of: 0.05*Delta_der+Delta_der = der(Delta) without derivating Delta
  T_dswa * der(d_int) + d_int = min(max(Delta, -Delta_max), Delta_max) / T_dswa;
  Delta_der = min(max(Delta, -Delta_max), Delta_max) / T_dswa - d_int;
  //der(Delta_filt) = Delta_der;
  // Wheel steer angles*/
          /*
                                                                                                                                                                                                                                                                                                  DeltaWheel_1 = -static_toe_front + teta_p / SR - Comp_Fy_front * F_y_1 + Comp_Mz_front * (M_z_1 - F_y_1 * dc) + roll_angle * roll_steer_front;
                                                                                                                                                                                                                                                                                                  DeltaWheel_2 = static_toe_front + teta_p / SR - Comp_Fy_front * F_y_2 + Comp_Mz_front * (M_z_2 - F_y_2 * dc) + roll_angle * roll_steer_front;
                                                                                                                                                                                                                                                                                                  DeltaWheel_3 = -static_toe_rear - Comp_Fy_rear * F_y_3 + Comp_Mz_rear * (M_z_3 - F_y_3 * dc) + roll_angle * roll_steer_rear;
                                                                                                                                                                                                                                                                                                  DeltaWheel_4 = static_toe_rear - Comp_Fy_rear * F_y_4 + Comp_Mz_rear * (M_z_4 - F_y_4 * dc) + roll_angle * roll_steer_rear;
                                                                                                                                                                                                                                                                                                  */
          // this dynamics was introduced to not get algebraic loops, there should be dynamics here, but probably more fancy than this...
          //der(DeltaWheel_1) = -100 * (DeltaWheel_1 - ((-static_toe_front) + teta_p / SR - Comp_Fy_front * F_y_1 + Comp_Mz_front * (M_z_1 - F_y_1 * dc) + roll_angle * roll_steer_front));
          //der(DeltaWheel_2) = -100 * (DeltaWheel_2 - (static_toe_front + teta_p / SR - Comp_Fy_front * F_y_2 + Comp_Mz_front * (M_z_2 - F_y_2 * dc) + roll_angle * roll_steer_front));
          /*
  der(DeltaWheel_1) = -50 * (DeltaWheel_1 - ((-static_toe_front) + atan(sin(teta_p / SR) / (cos(teta_p / SR) + 1.55 / 2 / 2.6 * sin(teta_p / SR))) - Comp_Fy_front * F_y_1 + Comp_Mz_front * (M_z_1 - F_y_1 * dc) + roll_angle * roll_steer_front));
  der(DeltaWheel_2) = -50 * (DeltaWheel_2 - (static_toe_front + atan(sin(teta_p / SR) / (cos(teta_p / SR) - 1.55 / 2 / 2.6 * sin(teta_p / SR))) - Comp_Fy_front * F_y_2 + Comp_Mz_front * (M_z_2 - F_y_2 * dc) + roll_angle * roll_steer_front));
  der(DeltaWheel_3) = -50 * (DeltaWheel_3 - ((-static_toe_rear) - Comp_Fy_rear * F_y_3 + Comp_Mz_rear * (M_z_3 - F_y_3 * dc) + roll_angle * roll_steer_rear));
  der(DeltaWheel_4) = -50 * (DeltaWheel_4 - (static_toe_rear - Comp_Fy_rear * F_y_4 + Comp_Mz_rear * (M_z_4 - F_y_4 * dc) + roll_angle * roll_steer_rear));
  F_roll_res1 = F_rolling * L2 / (2 * (L1 + L2));
  F_roll_res3 = F_rolling * L1 / (2 * (L1 + L2));
  FX1 = (F_x_1 - F_roll_res1) * cos(DeltaWheel_1) - F_y_1 * sin(DeltaWheel_1);
  FX2 = (F_x_2 - F_roll_res1) * cos(DeltaWheel_2) - F_y_2 * sin(DeltaWheel_2);
  FY1 = F_y_1 * cos(DeltaWheel_1) + (F_x_1 - F_roll_res1) * sin(DeltaWheel_1);
  FY2 = F_y_2 * cos(DeltaWheel_2) + (F_x_2 - F_roll_res1) * sin(DeltaWheel_2);
  //t_1 = -M_z_1 / max(abs(F_y_1), 0.01) * sign(F_y_1);
  //t_2 = -M_z_2 / max(abs(F_y_2), 0.01) * sign(F_y_2);
  //t_1 = -M_z_1/ F_y_1;
  //t_2 = -M_z_2 /F_y_2;
  M1 = (-FX1 * cos(Caster) * (r_KP * cos(KP) + R_nom * sin(KP))) + FY1 * cos(KP) * (R_nom * sin(Caster) + t_1 * cos(Caster)) + F_z_1 * sin(KP) * sin(DeltaWheel_1) * cos(Caster) * (r_KP + R_nom * tan(KP) * cos(KP));
  M2 = FX2 * cos(Caster) * (r_KP * cos(KP) + R_nom * sin(KP)) + FY2 * cos(KP) * (R_nom * sin(Caster) + t_2 * cos(Caster)) + F_z_2 * sin(KP) * sin(DeltaWheel_2) * cos(Caster) * (r_KP + R_nom * tan(KP) * cos(KP));
  //M1 = -FX1 * cos(Caster) * (r_KP * cos(KP) + R_nom * sin(KP)) + FY1 * cos(KP) * (R_nom * sin(Caster) + t * cos(Caster)) + F_z_1 * sin(KP) * sin(DeltaWheel_1) * cos(Caster) * (r_KP + R_nom * tan(KP) * cos(KP));
  //M2 = FX2 * cos(Caster) * (r_KP * cos(KP) + R_nom * sin(KP)) + FY2 * cos(KP) * (R_nom * sin(Caster) + t * cos(Caster)) + F_z_2 * sin(KP) * sin(DeltaWheel_2) * cos(Caster) * (r_KP + R_nom * tan(KP) * cos(KP));
  //P_servo = A * Stw_torque ^ 2 * sign(Stw_torque);
  // this is a glued quadratic to linear smooth boostcurve that does not causes numerical problems for large steering angles.
  P_servo = A * (if noEvent(abs(Stw_torque) < bcc) then Stw_torque ^ 3 else 3 * bcc ^ 2 * Stw_torque - 2 * bcc ^ 3 * sign(Stw_torque));
  Fr = Fr1 + Fr2;
  //P_servo = A * Stw_torque ^ 3;
  F_servo = P_servo * A_servo;
  Fr1 = -M1 / SAL;
  Fr2 = -M2 / SAL;
  teta_bar = Delta - x_rack / rp;
  teta_p = x_rack / rp;
  v_rack = der(x_rack);
  //a_rack = der(v_rack);
  //Fp = m_rack * a_rack + D_rack * v_rack + Fr1 + Fr2 + F_servo;
  //Fp = (K_torsionbar * teta_bar + D_torsionbar * (min(max(der(Delta_filt), -100), 100) - v_rack / rp)) / rp;
  //Fp = (K_torsionbar * teta_bar + D_torsionbar * (Delta_der - v_rack / rp)) / rp;
  m_rack * der(v_rack) = D_rack * v_rack - Fr1 - Fr2 - F_servo - Stw_torque / rp;
  //m_rack * a_rack =  - Fr1 - Fr2 - F_servo + Fp;
  //0.00035*der(Delta_der) = -D_torsionbar*(Delta_der-v_rack/rp)+Fp*rp-K_torsionbar*teta_bar;
  //0 = (-D_torsionbar * (Delta_der - v_rack / rp)) + Fp * rp - K_torsionbar * teta_bar;
  //0 = Fp*rp-K_torsionbar*teta_bar;
  //Stw_torque = -Fp * rp;
  //Stw_torque = -(D_torsionbar*der(teta_bar)+K_torsionbar*teta_bar);
  Stw_torque = -(D_torsionbar * (Delta_der - v_rack / rp) + K_torsionbar * teta_bar); */
          import SI = Modelica.SIunits;
          constant Real pi = Modelica.Constants.pi;
          parameter SI.Angle static_toe_front = 0*pi/180 "static front toe";
          parameter SI.Angle static_toe_rear = 0*pi/180 "static rear toe";
          parameter Real i_s = 16 "steering ratio";
          parameter Real roll_steer_front = 0.1
            "Roll steer coeff. for a front wheel, towards toe-in for jounce and towards toe-out for bounce";
          parameter Real roll_steer_rear = 0.04
            "Roll steer coeff. for a rear wheel, towards toe-in for jounce and towards toe-out for bounce";
          parameter Real Comp_Fy_front = 1.2217e-06
            "Lateral force compliance for a front wheel, toe-out for the outer wheel and toe-in for the inner wheel [rad/N]--default value (Chalmers VDA): 1.2217e-6 [rad/N] ";
          parameter Real Comp_Fy_rear = 5.236e-07
            " Lateral force compliance for a rear wheel, toe-out for the outer wheel and toe-in for the inner wheel [rad/N]--default value (Chalmers VDA): 5.2360e-7 [rad/N]";
          parameter Real Comp_Mz_front = 2.7925e-05
            "Aligning moment for a front wheel [rad/Nm]--default value (Chalmers VDA): 2.7925e-5 [rad/Nm]";
          parameter Real Comp_Mz_rear = 8.7267e-06    "Aligning moment for a rear wheel [rad/Nm]--//default value (Chalmers VDA): 8.7267e-6 [rad/Nm]";

          Real DeltaWheel_1 " steering angle of the front wheel 1 rad";
          Real DeltaWheel_2 " steering angle of the front wheel 2 rad";
          Real DeltaWheel_3 " steering angle of the rear wheel 3 rad";
          Real DeltaWheel_4 " steering angle of the rear wheel 4 rad";
          Real Delta "Steering wheel angle";
        //   SI.Force F_y_1;
        //   SI.Force F_y_2;
        //   SI.Force F_y_3;
        //   SI.Force F_y_4;
        //   SI.Angle Roll_angle;
        equation
          /*DeltaWheel_1 = -static_toe_front + Delta / i_s - Comp_Fy_front * F_y_1 + Roll_angle * roll_steer_front;
                                                                         /* + Comp_Mz_front * M_z_1*/
          /*DeltaWheel_2 = static_toe_front + Delta / i_s - Comp_Fy_front * F_y_2 + Roll_angle * roll_steer_front;
                                                                       /*+ Comp_Mz_front * M_z_2*/
          //DeltaWheel_3 = static_toe_rear - Comp_Fy_rear * F_y_3 + Comp_Mz_rear * M_z_3 + Roll_angle * roll_steer_rear;
          //DeltaWheel_4 = -static_toe_rear - Comp_Fy_rear * F_y_4 + Comp_Mz_rear * M_z_4 + Roll_angle * roll_steer_rear;
          //DeltaWheel_3 = -static_toe_rear - Comp_Fy_rear * F_y_3 + Roll_angle * roll_steer_rear;
                                                                 /*+ Comp_Mz_rear * M_z_3*/
          //DeltaWheel_4 = static_toe_rear - Comp_Fy_rear * F_y_4 + Roll_angle * roll_steer_rear;
                                                                /*+ Comp_Mz_rear * M_z_4*/
          DeltaWheel_1 =   Delta / i_s;
          DeltaWheel_2 =   Delta / i_s;
          DeltaWheel_3 =   0;
          DeltaWheel_4 =   0;

          annotation ();
        end try_steering;

        model steering_new
        import SI = Modelica.SIunits;

          parameter Real i_s = 16 "steering ratio";
          parameter Real kt =  115 "Torsion bar stiffness [Nm/rad]";
          parameter Real Jw = 20 "lumped inertia for the wheel and steering rack";
          parameter Real cw = 200 "damping of rack and tire [Nms/rad]";
          parameter Real error = 1*1e-3;
          parameter SI.Length TW;
          parameter SI.Length L;
          parameter Real C_f = 150000;
          parameter Real C_r = 150000;
          parameter Real static_toe_front = 0.00;
          SI.Torque tau_k "torsion bar torque[Nm]";
          SI.Torque Mz "aligning torque [Nm]";
          SI.Angle Delta "steering wheel angle [rad]";
          SI.Angle delta_wh "wheel angle[rad]";
          SI.Angle delta_p "pinion angle [rad]";
          SI.Angle DeltaWheel_1 "left wheel angle";
          SI.Angle DeltaWheel_2 "right wheel angle";
          SI.Angle DeltaWheel_3 "left wheel angle";
          SI.Angle DeltaWheel_4 "right wheel angle";
        //   SI.AngularVelocity ddelta_swa "angular velocity of steering wheel angle[rad/s]";
          SI.AngularVelocity ddelta_wh "angluar velocity of wheel[rad/s]";
        //   SI.AngularAcceleration dddelta_swa "angular acceleration of steeing wheel angle [rad/s^2]";
          SI.AngularAcceleration dddelta_wh "angluar acceleration of wheel angle [rad/s^2]";
        equation
        //   ddelta_swa = der(Delta);
        //   dddelta_swa = der(ddelta_swa);

          ddelta_wh = der(delta_wh);
          dddelta_wh = der(ddelta_wh);

          Jw*dddelta_wh =  tau_k*i_s - Mz - cw*ddelta_wh;
          tau_k = kt*(Delta - delta_p);
          DeltaWheel_1 = delta_p/i_s;
          DeltaWheel_2 = if ((delta_wh > error) or (delta_wh < -error)) then atan(1 / ( 1 / tan(DeltaWheel_1) + TW / L)) else DeltaWheel_1;
          delta_wh = (DeltaWheel_1 + DeltaWheel_2)/2;
          DeltaWheel_3 = 0;
          DeltaWheel_4 = 0;

          annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        end steering_new;
      end Steering;
    end Vehicle_control_and_actuators;
  end Components;

  package SimulationExperiment "file for FMU"
    model simulink_new
      import SI = Modelica.SIunits;
      //parameters for the two track and assembly of all components
      parameter SI.Mass m = 2194 "mass of the full vehicle";
      parameter SI.Mass m_usp = 260.516 "total unsprung mass";
      parameter SI.Mass mu_f = m_usp * L2 / (L1 + L2) / 2
        " 48.509 unsprung mass front per wheel";
      parameter SI.Mass mu_r = m_usp * L1 / (L1 + L2) / 2
        "46.093 unsprung mass rear per wheel";
      parameter SI.Mass m_s = m - 2 * mu_f - 2 * mu_r "sprung mass";
      parameter SI.Length R_nom =  0.347 "Nominal road wheel radius";
      parameter SI.Length h_CG = 0.68 "Centre of Gravity height,[m]";
      parameter SI.Length L1 = 1.41
        "Distance between COG and front axle,[m]";
      parameter SI.Length L2 = 1.576
        "Distance between COG and rear axle,[m]";
      parameter SI.Length TW_f = 1.722 "front track width,[m]";
      parameter SI.Length TW_r = 1.730 "rear track width,[m]";
      parameter SI.Inertia I_z = 4770
        "Yaw moment of inertia (about z-axis at CG)";
      parameter SI.Inertia I_y = 4446.3
        "Pitch moment of inertia (about y-axis at CG)";
      parameter SI.Inertia I_x = 894.402
        "Roll moment of inertia (about x-axis at CG)";
      parameter SI.Distance h_frc = 0.115
        "0.25 Front suspension roll centre heigth (m)";
      parameter SI.Distance h_rrc = 0.12
        "0.3 Rear suspension roll centre heigth (m)";
      parameter SI.Distance h_pc = 0.29;
      constant Real pi = Modelica.Constants.pi;
      parameter Real K0_f = 32000/2 "Spring suspension stiffness [N/m] front";
      parameter Real K0_r = 27000/2 "Spring stiffness [N/m] rear";
      parameter Real Dfb_f = 2750 "damper fast bounce front";
      parameter Real Dfr_f = 900;
      parameter Real Dsb_f = 10000.0;
      parameter Real Dsr_f = 5500 "damper slow rebounce front";
      parameter Real v_rebound_f = -0.132 "breakpoint fast/slow front";
      parameter Real v_bump_f = 0.131;
      parameter Real Dfb_r = 3137 "damper fast rebounce front";
      parameter Real Dfr_r = 3137;
      parameter Real Dsb_r = 3137;
      parameter Real Dsr_r = 3137 "damper slow rebounce front";
      parameter Real v_rebound_r = -0.132 "breakpoint fast/slow front";
      parameter Real v_bump_r = 0.131;
      parameter Real k_antiroll_r= 367 * 180 / pi*TW_r/2
        "anti-rollbar stifness rear";
      parameter Real k_antiroll_f = 652 * 180 / pi*TW_f/2 " --";
      // Tire model parameters
      parameter Real cx = 25000 "Tire longitudinal stiffness";
      parameter Real cy = 18000 "Tire vertical stiffness";
      parameter Real Kt= 250e3 "Carcass vertical stiffness";
      parameter Real Ct=10e3 "Carcass vertical damping";
      parameter Integer st_agear = 1;
      parameter Real vx0 = 0;
      Components.Wheels_and_suspension.brush_tire_model_2 wheel1(
        rw=0.3125,
        vx0=vx0,
        Fz0=suspension.F_z_f0,
        cpx=25000,
        cpy=18000);
      Components.Wheels_and_suspension.brush_tire_model_2 wheel2(
        rw=0.3125,
        vx0=vx0,
        Fz0=suspension.F_z_f0,
        cpx=cx,
        cpy=cy);
      Components.Wheels_and_suspension.brush_tire_model_2 wheel3(
        rw=0.3125,
        vx0=vx0,
        Fz0=suspension.F_z_r0,
        cpx=cx,
        cpy=cy);
      Components.Wheels_and_suspension.brush_tire_model_2 wheel4(
        rw=0.3125,
        vx0=vx0,
        Fz0=suspension.F_z_r0,
        cpx=25000,
        cpy=18000);
      Components.Vehicle_motion.chassis chassis(m=m, vxstart=vx0);
      Advanced_XC90_dymola.Components.Wheels_and_suspension.Suspension.suspension
        suspension(
        L1=L1,
        L2=L2,
        TW_f=TW_f,
        TW_r=TW_r,
        h_CG=h_CG,
        m=m,
        mu_f=mu_f,
        mu_r=mu_r,
        R_nom=wheel1.rw,
        K0_f=K0_f,
        Dfb_f=Dfb_f,
        Dfr_f=Dfr_f,
        Dsb_f=Dsb_f,
        Dsr_f=Dsr_f,
        v_rebound_f=v_rebound_f,
        v_bump_f=v_bump_f,
        k_antiroll_f=k_antiroll_f,
        K0_r=K0_r,
        Dfb_r=Dfb_r,
        Dfr_r=Dfr_r,
        Dsb_r=Dsb_r,
        Dsr_r=Dsr_r,
        v_rebound_r=v_rebound_r,
        v_bump_r=v_bump_r,
        k_antiroll_r=k_antiroll_r,
        h_frc=h_frc,
        h_rrc=h_rrc,
        h_pc=h_pc,
        Kt=Kt,
        Ct=Ct);
      Advanced_XC90_dymola.Components.Vehicle_control_and_actuators.Steering.steering_new
        steering(L=(L1 + L2), TW=TW_f);
    //     Modular_5.Steering.try_steering steering;

      Advanced_XC90_dymola.Components.Vehicle_control_and_actuators.Powertrain.transmission
        powertrain(st_agear=st_agear);
      Components.Vehicle_control_and_actuators.brakes brake;
      SI.Position z_mean "mean value of height of wheel positions";
      SI.Position zr[4] "position of the road under all 4 wheels with zero mean";
      parameter Real zwheel[4] = {0,0,0,0};
      input Real Aped;
      input Real Bped;
      input Real swa;
      input Real r_gear;
      output Real ax_sim;
      output Real ax;
      output Real decl;
      output Real vx;
      output Real yaw_angle;
      output Real x;
      output Real y;
      output Real ay;
      output Real delta;
      output Integer agear;
      output Real vy;
      output Real yaw_velocity;
      output Real w_eng;
      output Real Tbw;
      output Real Tw;
      output Real Tbmax;
      output Real yaw_acc;
    equation
        //road
        z_mean = (zwheel[1] + zwheel[2] + zwheel[3] + zwheel[4]) / 4;
        zr[1] = zwheel[1] - z_mean;
        zr[2] = zwheel[2] - z_mean;
        zr[3] = zwheel[3] - z_mean;
        zr[4] = zwheel[4] - z_mean;
        chassis.road_slope = atan((zr[1] + zr[2] - zr[3] - zr[4]) / 2 / (L1 + L2));
        chassis.road_bank = atan((zr[1] + zr[3] - zr[2] - zr[4]) / 2 / (TW_f + TW_r) / 2);
      //Forces
      chassis.Fxf1 = wheel1.F_x;
      chassis.Fxf2 = wheel2.F_x;
      chassis.Fxr3 = wheel3.F_x;
      chassis.Fxr4 = wheel4.F_x;
      chassis.Fyf1 = wheel1.F_y;
      chassis.Fyf2 = wheel2.F_y;
      chassis.Fyr3 = wheel3.F_y;
      chassis.Fyr4 = wheel4.F_y;

      chassis.ext_Fx = 0;
      chassis.ext_Fy = 0;
      chassis.ext_Fz = 0;
      chassis.ext_Mx = 0;
      chassis.ext_My = 0;
      chassis.ext_Mz = 0;
      //Velocity of vehicle and wheel

         chassis.vxw1 = wheel1.Vx;
         chassis.vxw2 = wheel2.Vx;
         chassis.vxw3 = wheel3.Vx;
         chassis.vxw4 = wheel4.Vx;

         chassis.vyw1 = wheel1.Vy;
         chassis.vyw2 = wheel2.Vy;
         chassis.vyw3 = wheel3.Vy;
         chassis.vyw4 = wheel4.Vy;

         //angle
         chassis.del_w1 = wheel1.DeltaWheel;
         chassis.del_w2 = wheel2.DeltaWheel;
         chassis.del_w3 = wheel3.DeltaWheel;
         chassis.del_w4 = wheel4.DeltaWheel;

         steering.DeltaWheel_1 = wheel1.DeltaWheel;
         steering.DeltaWheel_2 = wheel2.DeltaWheel;
         steering.DeltaWheel_3 = wheel3.DeltaWheel;
         steering.DeltaWheel_4 = wheel4.DeltaWheel;
          steering.Mz = (wheel1.Mz + wheel2.Mz)/2;
         steering.Delta = swa;
         // suspension
         suspension.pitch_angle = chassis.phi;
         suspension.roll_angle = chassis.theta;
         //chassis and suspension forces
         suspension.FC1 = chassis.FC1;
         suspension.FC2 = chassis.FC2;
         suspension.FC3 = chassis.FC3;
         suspension.FC4 = chassis.FC4;

         // the road profile at each wheel
         suspension.zr1 = zr[1];
         suspension.zr2 = zr[2];
         suspension.zr3 = zr[3];
         suspension.zr4 = zr[4];
        // forces to the suspension from the wheels
         wheel1.F_y = suspension.F_y_1;
         wheel2.F_y = suspension.F_y_2;
         wheel3.F_y = suspension.F_y_3;
         wheel4.F_y = suspension.F_y_4;
         wheel1.F_x = suspension.F_x_1;
         wheel2.F_x = suspension.F_x_2;
         wheel3.F_x = suspension.F_x_3;
         wheel4.F_x = suspension.F_x_4;
         wheel1.Fz = suspension.F_z_1;
         wheel2.Fz = suspension.F_z_2;
         wheel3.Fz = suspension.F_z_3;
         wheel4.Fz = suspension.F_z_4;
         // vertical motion of the COG
         suspension.d_roll = chassis.d_roll;
         suspension.d_pitch = chassis.d_pitch;
         suspension.z = chassis.z;

         //powertrain
         powertrain.Aped = Aped;
         powertrain.w_wheel_1 = wheel1.omega;
         powertrain.w_wheel_2 = wheel2.omega;
         powertrain.w_wheel_3 = wheel3.omega;
         powertrain.w_wheel_4 = wheel4.omega;
         powertrain.Vx = chassis.vx;
         powertrain.r_gear = r_gear;
         vx = chassis.vx;
         wheel1.Drive_torque = if r_gear == 1 then powertrain.rev_torque else powertrain.M_wheel_1;
         wheel2.Drive_torque = if r_gear == 1 then powertrain.rev_torque else powertrain.M_wheel_2;
         wheel3.Drive_torque = if r_gear == 1 then powertrain.rev_torque else powertrain.M_wheel_3;
         wheel4.Drive_torque = if r_gear == 1 then powertrain.rev_torque else powertrain.M_wheel_4;

         //brakes
         brake.brakeped = Bped;
         wheel1.Brake_torque = brake.Tbw;
         wheel2.Brake_torque = brake.Tbw;
         wheel3.Brake_torque = brake.Tbw;
         wheel4.Brake_torque = brake.Tbw;
         ax_sim = chassis.axp;
         decl = chassis.decl;
         ax = chassis.ax;
         ay = chassis.ay;
         delta = chassis.del_w1;
         agear = powertrain.agear;
         yaw_velocity = chassis.psidot;
         Tbw = brake.Tbw;
         w_eng = powertrain.w_engine;
         Tbmax = brake.Tbmax;
         Tw = powertrain.Tw;
         yaw_angle = chassis.psi;
         yaw_acc = chassis.psiddot;
         x = chassis.x;
         y = chassis.y;
         vy = chassis.vy;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end simulink_new;
  end SimulationExperiment;
  annotation (uses(Modelica(version="3.2.2")));
end Advanced_XC90_dymola;
