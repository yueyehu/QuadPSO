%�򿪼�ʱ��
tic;
%
Scope=[10  20
       5.0 10
       10  20
       5.0 10
       0.1 5
       0.1 5
       ];
   
[PID,minmax] = PsoProcess(20,6,Scope,@InitSwarm,@BaseStepPso,@AC_Para_opt,10)

toc;
