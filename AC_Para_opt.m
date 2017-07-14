function y = Para_opt(x)
assignin('base','K1',x(1));
assignin('base','K2',x(2));
assignin('base','K3',x(3));
assignin('base','K4',x(4));
assignin('base','K5',x(5));
assignin('base','K6',x(6));


[t,xx,yy] = sim('AC_Quadcopter_Simulation_Lyapnov_Hamilton.slx',[0,5]);
y = 1/yy(end);

