function y = Para_opt(x)
% assignin('base','K1',x(1));
% assignin('base','K2',x(2));
% assignin('base','K3',x(3));
% assignin('base','K4',x(4));
% assignin('base','K5',x(5));
% assignin('base','K6',x(6));
assignin('base','K7',x(1));
assignin('base','K8',x(2));
assignin('base','K9',x(3));
assignin('base','K10',x(4));
assignin('base','K11',x(5));
assignin('base','K12',x(6));

[t,xx,yy] = sim('PC_Quadcopter_Simulation_Lyapnov_Hamilton.slx',[0,10]);
y = 1/yy(end);

