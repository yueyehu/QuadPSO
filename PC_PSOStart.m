%打开计时器
tic;
%
Scope=[0.1 4
       0.1 4
       0.1 4
       0.1 4
       0.1 4
       0.1 4
       ];
   
[PID,minmax] = PsoProcess(20,6,Scope,@InitSwarm,@BaseStepPso,@PC_Para_opt,10)

toc;
