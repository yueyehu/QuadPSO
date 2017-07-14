function [Result,MinMaxMeanAdapt]=PsoProcess(SwarmSize,ParticleSize,ParticleScope,InitFunc,StepFindFunc,AdaptFunc,LoopCount)
%功能描述：一个循环n次的PSO算法完整过程，返回这次运行的最小与最大的平均适应度
%[Result,MinMaxMeanAdapt]=PsoProcess(SwarmSize,ParticleSize,ParticleScope,InitFunc,StepFindFunc,AdaptFunc,LoopCount)
%输入参数：SwarmSize:种群大小的个数
%输入参数：ParticleSize：一个粒子的维数
%输入参数：ParticleScope:一个粒子在运算中各维的范围；
% ParticleScope格式:
% 3维粒子的ParticleScope格式:
%  [x1Min,x1Max
%   x2Min,x2Max
%   x3Min,x3Max]
%
%输入参数:InitFunc:初始化粒子群函数
%输入参数:StepFindFunc:单步更新速度，位置函数
%输入参数：AdaptFunc：适应度函数
%输入参数：LoopCount：迭代的次数；缺省迭代100次
%
%返回值：Result为经过迭代后得到的最优解
%返回值：MinMaxMeanAdapt为本次完整迭代得到的最小与最大的平均适应度
%
%异常：首先保证该文件在Matlab的搜索路径中，然后查看相关的提示信息。
%
%编制人：Guide
%编制时间：2011.8.9
%参考文献：XXXXX

%容错控制
if nargin<4
     error('输入的参数个数错误。')
end

[row,colum]=size(ParticleSize);
if row>1||colum>1
         error('输入的粒子的维数错误，是一个1行1列的数据。');
end
[row,colum]=size(ParticleScope);
if row~=ParticleSize||colum~=2
         error('输入的粒子的维数范围错误。');
end

%设置缺省
if nargin<7
    LoopCount=100;
end

%初始化种群                 
[ParSwarm,OptSwarm] = InitFunc(SwarmSize,ParticleSize,ParticleScope,AdaptFunc);
       
%开始更新算法的调用
for k=1:LoopCount
    %显示迭代的次数：
    disp('----------------------------------------------------------')
    TempStr=sprintf('第 %g 次迭代',k);
    disp(TempStr);
    disp('----------------------------------------------------------')

    %调用一步迭代的算法
    [ParSwarm,OptSwarm]=StepFindFunc(ParSwarm,OptSwarm,AdaptFunc,ParticleScope,0.95,0.4,LoopCount,k);

    %记录每一步的平均适应度
    MeanAdapt(1,k)=mean(ParSwarm(:,2*ParticleSize+1));
end
%for循环结束标志

%记录最小与最大的平均适应度
MinMaxMeanAdapt=[min(MeanAdapt),max(MeanAdapt)];

%记录本次迭代得到的最优结果
XResult=OptSwarm(SwarmSize+1,1:ParticleSize);
YResult=AdaptFunc(XResult);
Result=[XResult,YResult];
