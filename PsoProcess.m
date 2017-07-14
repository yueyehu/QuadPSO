function [Result,MinMaxMeanAdapt]=PsoProcess(SwarmSize,ParticleSize,ParticleScope,InitFunc,StepFindFunc,AdaptFunc,LoopCount)
%����������һ��ѭ��n�ε�PSO�㷨�������̣�����������е���С������ƽ����Ӧ��
%[Result,MinMaxMeanAdapt]=PsoProcess(SwarmSize,ParticleSize,ParticleScope,InitFunc,StepFindFunc,AdaptFunc,LoopCount)
%���������SwarmSize:��Ⱥ��С�ĸ���
%���������ParticleSize��һ�����ӵ�ά��
%���������ParticleScope:һ�������������и�ά�ķ�Χ��
% ParticleScope��ʽ:
% 3ά���ӵ�ParticleScope��ʽ:
%  [x1Min,x1Max
%   x2Min,x2Max
%   x3Min,x3Max]
%
%�������:InitFunc:��ʼ������Ⱥ����
%�������:StepFindFunc:���������ٶȣ�λ�ú���
%���������AdaptFunc����Ӧ�Ⱥ���
%���������LoopCount�������Ĵ�����ȱʡ����100��
%
%����ֵ��ResultΪ����������õ������Ž�
%����ֵ��MinMaxMeanAdaptΪ�������������õ�����С������ƽ����Ӧ��
%
%�쳣�����ȱ�֤���ļ���Matlab������·���У�Ȼ��鿴��ص���ʾ��Ϣ��
%
%�����ˣ�Guide
%����ʱ�䣺2011.8.9
%�ο����ף�XXXXX

%�ݴ����
if nargin<4
     error('����Ĳ�����������')
end

[row,colum]=size(ParticleSize);
if row>1||colum>1
         error('��������ӵ�ά��������һ��1��1�е����ݡ�');
end
[row,colum]=size(ParticleScope);
if row~=ParticleSize||colum~=2
         error('��������ӵ�ά����Χ����');
end

%����ȱʡ
if nargin<7
    LoopCount=100;
end

%��ʼ����Ⱥ                 
[ParSwarm,OptSwarm] = InitFunc(SwarmSize,ParticleSize,ParticleScope,AdaptFunc);
       
%��ʼ�����㷨�ĵ���
for k=1:LoopCount
    %��ʾ�����Ĵ�����
    disp('----------------------------------------------------------')
    TempStr=sprintf('�� %g �ε���',k);
    disp(TempStr);
    disp('----------------------------------------------------------')

    %����һ���������㷨
    [ParSwarm,OptSwarm]=StepFindFunc(ParSwarm,OptSwarm,AdaptFunc,ParticleScope,0.95,0.4,LoopCount,k);

    %��¼ÿһ����ƽ����Ӧ��
    MeanAdapt(1,k)=mean(ParSwarm(:,2*ParticleSize+1));
end
%forѭ��������־

%��¼��С������ƽ����Ӧ��
MinMaxMeanAdapt=[min(MeanAdapt),max(MeanAdapt)];

%��¼���ε����õ������Ž��
XResult=OptSwarm(SwarmSize+1,1:ParticleSize);
YResult=AdaptFunc(XResult);
Result=[XResult,YResult];
