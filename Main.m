%尝试制作反复打球的动画效果
clear all;clc;close all;
ball=Ball(1,[10,5],[-1.5,0]);%球
arm=Arm();%机械臂
world=World();%物理环境
Tr=Trajectory();%轨迹规划器
o=Observer(ball,arm);%视觉

MaxN=5;%和墙壁碰撞次数
v=zeros(5,1000);%存储数据供最后可视化使用
pointer=0;

%%%%%%%%每轮以从墙壁出发为开始，以回弹到墙壁结束，内部大体思路相同
for n=1:MaxN
    %%%%%%%%%每轮循环首先根据小球的初始位置预测小球到达希望击打位置的时间T
    %%%要求击打位置需要使得小球回到墙壁时的速度尽可能与原来保持一致，角度不会过大或过小，且至少与地面碰撞一次
     T=o.besthitpoint(ball.p,ball.v,0.45);
    %%%%%%%%%%得到小球的与机械臂的预期碰撞时间T后，把T均分为5份
    T=T/5:T/5:T;
    %%%%%%%%%%
    for i=1:5
        %i=1，前向预测T s,
        %i=2, 前向预测T/5*4 s
        [qa,pb]=A(ball,arm,Tr,o,T(5-i+1),5-i+1);
        v(1:3,pointer+1:pointer+size(qa,2))=qa;
        v(4:5,pointer+1:pointer+size(pb,2))=pb;
        pointer=pointer+size(qa,2);
    end
    temp=pointer;
    while(1)
        ball.move(World.sample);
        ball.isbouncehappend([],1);
%         ball.p;
        v(1:3,pointer+1)=v(1:3,pointer)-[0.05;0;0];
        v(4:5,pointer+1)=ball.p;
%         plot3(ball.p(1),ball.p(2),0,'o','LineWidth',0.05);
        pointer=pointer+1;
        if(ball.boundsTimewall~=0)
            ball.boundsTimewall=0;
            arm.q=v(1:3,pointer);
%             a=ball.p0;
%             ball.p0=v(4:5,pointer)-0.5*(a-v(4:5,pointer));
            break;
        end
    end
end%整体大循环
    ax=gca;
    ax.Projection = 'orthographic';
    hold on;
    axis([-1.5 10 -1.5 15]);
    framesPerSecond = 25;
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
function [qa,pb]=A(ball,Arm,Tr,o,T,n)
    Ts=T/n/20;
    p0tool=Arm.ptool;%0s机械臂末端初始位置
    o0tool=Arm.otool;%0s机械臂末端姿态
    p0ball=ball.p; %第一次预测使用的位置
    v0ball=ball.v; %第一次预测使用的速度
    p1=o.forwardPrediction(p0ball,v0ball,T);%[px;py;theta],预测T s后的位置和方向
    pkeypoint=[linspace(p0tool(1),p1(1),n+1);...
               linspace(p0tool(2),p1(2),n+1);...
               linspace(o0tool,p1(3),n+1)];
    qk=zeros(3,2);
    qk(:,1)=Arm.q;
    qk(:,2)=Arm.InverseK(pkeypoint(1:2,2),pkeypoint(3,2)); 
    Tr.calUsingCLI(qk(:,1:2));%3*101
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
                %qa=[qa,Arm.q];
            end
            ball.move(Ts);
            ball.isbouncehappend(Arm,1);
            pb(:,count)=ball.p;
            if ball.boundsTimeArm==1
                ball.boundsTimeArm=0;
                break;
            end
        end
    end
end

