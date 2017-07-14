function QuadHamiSFunctionArticle(block)
setup(block);

function setup(block)

  % Register the number of ports.
  %------
  block.NumInputPorts  = 5;
  %------
  block.NumOutputPorts = 12;
  
  % Set up the port properties to be inherited or dynamic.
  
  for i = 1:4; % These are the motor inputs
  block.InputPort(i).Dimensions        = 1;
  block.InputPort(i).DirectFeedthrough = false;
  block.InputPort(i).SamplingMode      = 'Sample';
  end
  %------
  % This is the disturbance input
  block.InputPort(5).Dimensions        = 6; % torques x,y,z; forces x,y,z.
  block.InputPort(5).DirectFeedthrough = false;
  block.InputPort(5).SamplingMode      = 'Sample';
  %------
  for i = 1:12;
  block.OutputPort(i).Dimensions       = 1;
  block.OutputPort(i).SamplingMode     = 'Sample';
  end


  % Register the parameters.
  block.NumDialogPrms     = 2;
  
  % Set up the continuous states.
  block.NumContStates = 12;

  block.SampleTimes = [0 0];
  
  block.SetAccelRunOnTLC(false);
  
  block.SimStateCompliance = 'DefaultSimState';
  
  block.RegBlockMethod('CheckParameters', @CheckPrms);

  block.RegBlockMethod('InitializeConditions', @InitializeConditions);
  
  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Derivatives', @Derivatives);
  
% -------------------------------------------------------------------
% The local functions below are provided to illustrate how you may implement
% the various block methods listed above.
% -------------------------------------------------------------------

 function CheckPrms(block)
     quad   = block.DialogPrm(1).Data;
     IC     = block.DialogPrm(2).Data;

function InitializeConditions(block)
% Initialize 12 States

IC = block.DialogPrm(2).Data;


Phi = IC.Phi*pi/180; The = IC.The*pi/180; Psi = IC.Psi*pi/180;
X = IC.X; Y = IC.Y; Z = IC.Z;

init = [X,Y,Z,Phi,The,Psi,0,0,0,0,0,0];
for i=1:12
block.OutputPort(i).Data = init(i);
block.ContStates.Data(i) = init(i);
end

function Outputs(block)
for i = 1:12;
  block.OutputPort(i).Data = block.ContStates.Data(i);
end

function Derivatives(block)
quadrotor = block.DialogPrm(1).Data;

X = block.ContStates.Data(1);
Y = block.ContStates.Data(2);
Z = block.ContStates.Data(3);
% Phi The Psi in radians
Phi = block.ContStates.Data(4);
The = block.ContStates.Data(5);
Psi = block.ContStates.Data(6);
% U V W in units of m/s
m_X = block.ContStates.Data(7);
m_Y = block.ContStates.Data(8);
m_Z = block.ContStates.Data(9);
% X Y Z in units of m
m_Phi = block.ContStates.Data(10);
m_The = block.ContStates.Data(11);
m_Psi = block.ContStates.Data(12);
% w values in rev/min! NOT radians/s!!!!
w1 = block.InputPort(1).Data;
w2 = block.InputPort(2).Data;
w3 = block.InputPort(3).Data;
w4 = block.InputPort(4).Data;
w  = [w1; w2; w3; w4];
%aticle coeffience
D_x = -1.2;D_y = -1.2;D_z = -1.5;
D_phi = -0.03;D_the = -0.03;D_psi=-0.05;

% D_x = 0;D_y = 0;D_z = 0;
% D_phi = 0;D_the = 0;D_psi=0;
%------
% Dist_F = block.InputPort(5).Data(1:3); %惯性系下的阻力
% Dist_tau   = block.InputPort(5).Data(4:6);
%------
dX = 1/quadrotor.mass*m_X;
dY = 1/quadrotor.mass*m_Y;
dZ = 1/quadrotor.mass*m_Z;
Jx = quadrotor.Jx;Jy = quadrotor.Jy;Jz = quadrotor.Jz;
dPhi = (Jy*Jz+(Jx-Jz)*Jy*sin(The)^2-(Jy-Jz)*Jx*sin(Phi)^2*sin(The)^2)*m_Phi/(Jx*Jy*Jz*cos(The)^2)+...
       (-(Jy-Jz)*cos(Phi)*sin(Phi)*sin(The)*m_The)/(Jy*Jz*cos(The))+...
       (Jy*cos(Phi)^2+Jz*sin(Phi)^2)*sin(The)*m_Psi/(Jy*Jz*cos(The)^2);
dThe = -(Jy-Jz)*cos(Phi)*sin(Phi)*sin(The)*m_Phi/(Jy*Jz*cos(The))+...
       (Jy*sin(Phi)^2+Jz*cos(Phi)^2)*m_The/(Jy*Jz)+...
       (-(Jy-Jz)*cos(Phi)*sin(Phi))*m_Psi/(Jy*Jz*cos(The));
dPsi = (Jy*cos(Phi)^2+Jz*sin(Phi)^2)*sin(The)*m_Phi/(Jy*Jz*cos(The)^2)+...
       (-(Jy-Jz)*cos(Phi)*sin(Phi))*m_The/(Jy*Jz*cos(The))+...
       (Jy*cos(Phi)^2+Jz*sin(Phi)^2)*m_Psi/(Jy*Jz*cos(The)^2);

Dist_F = [D_x*dX;D_y*dY;D_z*dZ];
Dist_tau = [D_phi*dPhi;D_the*dThe;D_psi*dPsi];
P = dPhi-sin(The)*dPsi;
Q = cos(Phi)*dThe + cos(The)*sin(Phi)*dPsi;

% tau_motorGyro = [Q*quadrotor.Jm*2*pi/60*(-w1-w3+w2+w4); P*quadrotor.Jm*2*pi/60*(w1+w3-w2-w4); 0]; % Note: 2*pi/60 required to convert from RPM to radians/s
tau_motorGyro = [0;0;0];
Mb = (quadrotor.dctcq*(w.^2))+ tau_motorGyro + (Dist_tau);  % Mb = [tau1 tau2 tau3]'总外力矩

Fb = [0; 0; -sum(quadrotor.ct*(w.^2))];   %[0, 0, sum(ct*w.^2)]'推力

dm_X = (cos(Psi)*sin(The)*cos(Phi)+sin(Psi)*sin(Phi))*Fb(3)+Dist_F(1);
dm_Y = (sin(Psi)*sin(The)*cos(Phi)-cos(Psi)*sin(Phi))*Fb(3)+Dist_F(2);
dm_Z = cos(The)*cos(Phi)*Fb(3)+quadrotor.mass*quadrotor.g+Dist_F(3);

dm_Phi = -(Jy-Jz)*(dThe^2*sin(Phi)*cos(Phi)-dThe*dPsi*cos(2*Phi)*cos(The)-dPsi^2*sin(Phi)*cos(Phi)*cos(The)^2)+Mb(1);
dm_The = (Jx-Jy*sin(Phi)^2-Jz*cos(Phi)^2)*dPsi^2*sin(The)*cos(The)-(Jy-Jz)*dThe*dPsi*sin(Phi)*cos(Phi)*sin(The)-dPhi*dPsi*Jx*cos(The)+Mb(2);
dm_Psi = Mb(3);
% Rough rule to impose a "ground" boundary...could easily be improved...
if ((Z>=0) && (dm_Z>=0)) % better  version then before?
    dm_Z = 0;
    block.ContStates.Data(9) = 0;
end
f = [dX,dY,dZ,dPhi,dThe,dPsi,dm_X,dm_Y,dm_Z,dm_Phi,dm_The,dm_Psi].';
  %This is the state derivative vector
block.Derivatives.Data = f;


%endfunction
