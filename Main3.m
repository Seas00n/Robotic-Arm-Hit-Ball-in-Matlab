%尝试制作反复打球的动画效果
%有Main.m文件发现小球会因为速度越来越快超出机械臂运动范围（在第3次和墙壁碰撞后就无法继续）
%固定相对高度，添加角度调节

clear all;clc;close all;
ball=Ball(1,[10,5],[-1,0]);%球
arm=Arm();%机械臂
world=World();%物理环境
Tr=Trajectory();%轨迹规划器
o=Observer(ball,arm);%视觉

MaxN=6;%和墙壁碰撞次数
v=zeros(5,1000);%存储数据供最后可视化使用
pointer=0;

%每轮以从墙壁出发为开始，以回弹到墙壁结束，内部大体思路相同
for n=1:MaxN
    %每轮循环首先根据小球的初始位置优化预测击打位置
    %要求击打位置需要使得小球回到墙壁时的速度尽可能与原来保持一致（优化使动能差最小），角度不会过大或过小，且至少与地面碰撞一次
    %优化后得到击打位置，由于小球水平方向速度不变，所以等效于给出击打发生的时间
    %将角度纳入优化范围，通过末端执行器击球角度的调整使得目标函数收敛
    [T,theta]=o.besthitpointwithangle(ball.p,ball.v,0.7);
    %把T均分为5份，每个i*T/5为关键时间戳，用于监督轨迹变化及时更新修正
    T=T/5:T/5:T;
    %前向迭代器C作用是：
    %1.根据关键帧的小球位置和速度预测击打点的位置
    %2.根据关键帧的机械臂位置和预测到的击打点位置关节空间线性插值得到机械臂在未来T/n的运动策略
    %3.迭代更新T/n的机械臂和小球的运动情况
    for i=1:5
        %i=1，前向预测T s,
        %i=2, 前向预测T/5*4 s
        [qa,pb]=C(ball,arm,Tr,o,T(5-i+1),5-i+1,theta);
        v(1:3,pointer+1:pointer+size(qa,2))=qa;
        v(4:5,pointer+1:pointer+size(pb,2))=pb;
        pointer=pointer+size(qa,2);
    end
    temp=pointer;
    %碰撞发生后，机械臂缓慢回移，小球按照动力学递推正常运动直到和墙壁碰撞
    while(1)
        ball.move(World.sample);
        ball.isbouncehappend([],1);
        v(1:3,pointer+1)=v(1:3,pointer)-[0.05;0;0];
        v(4:5,pointer+1)=ball.p;
        pointer=pointer+1;
        if(ball.boundsTimewall~=0)
            ball.boundsTimewall=0;
            arm.q=v(1:3,pointer);
            break;
        end
    end
end%整体大循环
    ax=gca;
    ax.Projection = 'orthographic';
    hold on;
    axis([-1.5 10 -1.5 15]);
    framesPerSecond = 20;
    r = rateControl(framesPerSecond);
    for i=1:pointer
        plot3(v(4,i),v(5,i),0,'o','LineWidth',0.05);
        show(arm.robot,v(1:3,i),'PreservePlot',false);
        waitfor(r);
        if(i==1)
            pause(5);
        end
    end

%%%%%%产生机械臂在节点时间的运动策略
function [qa,pb]=C(ball,Arm,Tr,o,T,n,theta)
    Ts=T/n/20;
    p0tool=Arm.ptool;%0s机械臂末端初始位置
    o0tool=Arm.otool;%0s机械臂末端姿态
    p0ball=ball.p; %0s时小球的位置
    v0ball=ball.v; %0s时小球的速度
    p1=o.forwardPrediction(p0ball,v0ball,T);
    p1(3)=theta;
    %[px;py;theta],预测T s后的位置和方向，即目标位置和目标角度
    %theta为主函数中优化得到的最佳击球角度，所有时间运动策略的最终目标角度都是theta
    pkeypoint=[linspace(p0tool(1),p1(1),n+1);...
               linspace(p0tool(2),p1(2),n+1);...
               linspace(o0tool,p1(3),n+1)];
    %机械臂从p0tool到pl用时T，其中每个T/n期望到达轨迹的n等分点
    qk=zeros(3,2);
    qk(:,1)=Arm.q;
    qk(:,2)=Arm.InverseK(pkeypoint(1:2,2),pkeypoint(3,2));%T/n后小球的第一个关键帧期望的关节角度
    %关节空间线性插值，20等分T/n插值使得小球轨迹更加顺滑
    Tr.calUsingCLI(qk(:,1:2));%3*21
    qa=Tr.waypoints(:,1:20);%qa即得到的T/n时间内的关节空间的轨迹
    pb=zeros(2,size(qa,2));%pb为小球在T/n时间内的轨迹
    pb(:,1)=p0ball;
    count=1;
    %n==1时处于碰撞发生的时间附近要专门写检测碰撞的逻辑
    %n!=1时，小球依据动力学正常递归
    if n~=1
        for i=Ts:Ts:T/n-Ts
            count=count+1;
            ball.move(Ts);
            ball.isbouncehappend([],1);
            pb(:,count)=ball.p;
        end
        Arm.q=qk(:,2);
        ball.move(Ts);
        ball.isbouncehappend([],1);
    else
        while(1)
            count=count+1;
            if count<=20
                Arm.q=Tr.waypoints(:,count);
            else
                Arm.q=Tr.waypoints(:,20);
                qa=[qa,Arm.q];
            end
            ball.move(Ts);
            ball.isbouncehappend(Arm,1);
            pb(:,count)=ball.p;
            if ball.boundsTimeArm==1
                ball.boundsTimeArm=0;
                break;
            end
        end
        pb=pb(:,1:count);
        qa=qa(:,1:count);
    end
end

