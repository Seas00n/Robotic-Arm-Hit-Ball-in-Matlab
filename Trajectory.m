classdef Trajectory<handle
    properties
        waypoints
    end
    methods
        function obj=Trajectory()
        end
        function calUsingCLI(obj,qk)
           x=1:size(qk,2);
           y=qk;
           xp=1:0.05:size(x,2);
           obj.waypoints=pchip(x,y,xp);
        end
    end
    methods(Static)
        function [vtool,vx,vy]=bouncetotheoriginal(pbw,x,y,theta,vby)%%pbw为击打点到目标的位移，x,y,theta，为末端执行器的位姿，vby为碰撞速度在末端执行器坐标系下的y分量
            b=x*tan(theta)-tan(theta)*sin(theta)*vby-cos(theta)*vby-y;
            a=pbw(2)-tan(theta)*pbw(1);
            tb=(b+sqrt(b*b-2*World.g*a))/World.g;% + or -
            vtool=(pbw(1)/tb+vby*sin(theta)-x)/cos(theta);
            vx=pbw(1)/tb;
            vy=pbw(2)/tb+0.5*World.g*tb;
        end
        function [vtool,vx,vy]=bounceperpendicular(pbw,x,y,theta,vby)
            b=x*tan(theta)-tan(theta)*sin(theta)*vby-cos(theta)*vby-y;
            a=pbw(2)-tan(theta)*pbw(1);
            tb=(b+sqrt(b*b-2*World.g*a))/World.g;% + or -
            vtool=(pbw(1)/tb+vby*sin(theta)-x)/cos(theta);
            vx=pbw(1)/tb;
            vy=pbw(2)/tb+0.5*World.g*tb;
        end
    end
end