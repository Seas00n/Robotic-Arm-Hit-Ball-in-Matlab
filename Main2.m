%尝试制作颠球的动画效果
clear all;clc;close all;
ball=Ball(1,[10,5],[-2,0]);%球
arm=Arm();%机械臂
world=World();%物理环境
Tr=Trajectory();%轨迹规划器
o=Observer(ball,arm);%视觉

MaxN=6;%颠球次数
v=zeros(5,1000);%存储数据供最后可视化使用
pointer=0;

%%%要求击打位置需要使得小球回到墙壁时的速度尽可能与原来保持一致，角度不会过大或过小，且至少与地面碰撞一次
T=o.besthitpoint(ball.p,ball.v,0.3);
%%%%%%%%%%得到小球的与机械臂的预期碰撞时间T后，把T均分为5份
T=T/5:T/5:T;
%%%%%%%%%%
for i=1:5
    %i=1，前向预测T s,
    %i=2, 前向预测T/5*4 s
    [qa,pb]=B(ball,arm,Tr,o,T(5-i+1),5-i+1,0);
    v(1:3,pointer+1:pointer+size(qa,2))=qa;
    v(4:5,pointer+1:pointer+size(pb,2))=pb;
    pointer=pointer+size(qa,2);
end
%%%%%%%%
for n=2:MaxN
    T=abs(ball.vy)/World.g*2;
    theta=atan2(ball.vy,ball.vx);
    theta=(pi/2+theta)/2;
    %%%%%%%%%%得到小球的与机械臂的预期碰撞时间T后，把T均分为5份
    if T>=3
    T=T/5:T/5:T;
    %%%%%%%%%%
    for i=1:5
        %i=1，前向预测T s,
        %i=2, 前向预测T/5*4 s
        [qa,pb]=B(ball,arm,Tr,o,T(5-i+1),5-i+1,theta);
        v(1:3,pointer+1:pointer+size(qa,2))=qa;
        v(4:5,pointer+1:pointer+size(pb,2))=pb;
        pointer=pointer+size(qa,2);
    end
    else
    T=T/3:T/3:T;
    %%%%%%%%%%
    for i=1:3
        %i=1，前向预测T s,
        %i=2, 前向预测T/3*2 s
        [qa,pb]=B(ball,arm,Tr,o,T(3-i+1),3-i+1,0);
        v(1:3,pointer+1:pointer+size(qa,2))=qa;
        v(4:5,pointer+1:pointer+size(pb,2))=pb;
        pointer=pointer+size(qa,2);
    end
    end
end%整体大循环

%%%%%可视化
    ax=gca;
    ax.Projection = 'orthographic';
    hold on;
    axis([-1.5 10 -1.5 15]);
    framesPerSecond = 20;
    r = rateControl(framesPerSecond);
    for i=1:pointer
        plot3(v(4,i),v(5,i),0,'o','LineWidth',0.05);
        show(arm.robot,v(1:3,i),'PreservePlot',false);
        waitfor(r);cla
        if i==1
            pause(5);
        end
    end

%%%%%%产生机械臂在节点时间的运动策略
function [qa,pb]=B(ball,Arm,Tr,o,T,n,theta)
    Ts=T/n/20;
    p0tool=Arm.ptool;%0s机械臂末端初始位置
    o0tool=Arm.otool;%0s机械臂末端姿态
    p0ball=ball.p; %第一次预测使用的位置
    v0ball=ball.v; %第一次预测使用的速度
    p1=o.forwardPrediction(p0ball,v0ball,T);%[px;py;theta],预测T s后的位置和方向
    if theta~=0
        p1(3)=theta;
    end
    pkeypoint=[linspace(p0tool(1),p1(1),n+1);...
               linspace(p0tool(2),p1(2),n+1);...
               linspace(o0tool,p1(3),n+1)];
    qk=zeros(3,2);
    qk(:,1)=Arm.q;
    qk(:,2)=Arm.InverseK(pkeypoint(1:2,2),pkeypoint(3,2)); 
    Tr.calUsingCLI(qk(:,1:2));%3*21
    qa=Tr.waypoints(:,1:20);%根据第一次预测得到移动的策略
    pb=zeros(2,size(qa,2));
    pb(:,1)=p0ball;
    count=1;
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
            ball.isbouncehappend(Arm,2);
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

