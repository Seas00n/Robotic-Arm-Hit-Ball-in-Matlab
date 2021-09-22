classdef Observer<handle
    properties
        ball
        arm
    end
    methods
        function obj=Observer(ball,arm)
            obj.ball=ball;
            obj.arm=arm;
            addlistener(ball,'bouncewithwall',@Observer.bouncewithwall);
            addlistener(ball,'bouncewithground',@Observer.bouncewithground);
           
        end
        function T=besthitpoint(obj,p,v,k)
            vy0=v(2);
            vx0=v(1);
            g=World.g;
            t=-(World.limx-p(1))/vx0;
            vy0=vy0+g*t;
            H=p(2)-vy0*t-0.5*g*t*t;
            %%%%%%第一次落地时间
            t1=(vy0+sqrt(vy0^2+2*g*H))/g;
            x1=vx0*t1;
            vy1=-World.ratio*(vy0-g*t1);
            vx1=vx0;
%             ed=0.5*m*(vy0^2-vy1^2);%%与地碰撞能量损失
            %%%%%第二次和机械臂碰撞
            %%%详细推导见derivation文件第二段
            syms h
            t2=(vy1+sqrt(vy1^2-2*g*h))/g;
            vy2=vy1-g*t2;
            vx2=vx1;
            ctheta=-vx2/sqrt(vx2^2+vy2^2);
            stheta=sqrt(1-ctheta^2);
            x=x1+vx2*t2;
            %%%反弹过程
            vbx=sqrt(x^2*g/(2*(-x*stheta/ctheta-H+h)));
            vby=vbx*stheta/ctheta;
            t=-x/vbx;
            e=0.8*vbx^2+0.2*vby^2;
            fun=matlabFunction(e);
            [hx,e]=fmincon(fun,0,[],[],[],[],0,k*vy1^2/2/g);
            x=double(subs(x,'h',hx));
            T=x/vx0;
        end
        
        function [T,thetax]=besthitpointwithangle(obj,p,v,k)
            vy0=v(2);
            vx0=v(1);
            g=World.g;
            t=-(World.limx-p(1))/vx0;
            vy0=vy0+g*t;
            H=p(2)-vy0*t-0.5*g*t*t;
            %%%%%%第一次落地时间
            t1=(vy0+sqrt(vy0^2+2*g*H))/g;
            x1=vx0*t1;
            vy1=-World.ratio*(vy0-g*t1);
            vx1=vx0;
            %%%%%第二次和机械臂碰撞
            %%%详细推导见derivation文件第三段
            %%%现在假设高度已经确定，使用角度
            h=k*vy1^2/2/g;
            t2=(vy1+sqrt(vy1^2-2*g*h))/g;
            vy2=vy1-g*t2;
            vx2=vx1;
            ctheta=-vx2/sqrt(vx2^2+vy2^2);
            stheta=sqrt(1-ctheta^2);
            thetav=atan2(stheta,ctheta);
            x=x1+vx2*t2;
            T=x/vx0;
            %%%反弹过程
            syms theta
%             vbx=sqrt(x^2*g/(2*(-x*stheta/ctheta-H+h)));
%             vby=vbx*stheta/ctheta;
%             t=-x/vbx;
%             e=0.8*vbx^2+0.2*vby^2;
            pbw=[-x;H-h];
            x=World.limx+x;
            y=H-h;
            vby=-vx2*sin(theta)+vy2*cos(theta);
            b=x*tan(theta)-tan(theta)*sin(theta)*vby-cos(theta)*vby-y;
            a=pbw(2)-tan(theta)*pbw(1);
            tb=(b+sqrt(b*b-2*World.g*a))/World.g;% + or -
            vx=pbw(1)/tb;
            vy=pbw(2)/tb+0.5*World.g*tb;
            vy=vy-g*tb;
            e=(vy^2+vx^2)-((vx0/0.8)^2+vy0^2);
            fun=matlabFunction(2*e);
            [thetax,e]=fmincon(fun,0,[],[],[],[],thetav-0.1745*2,pi/2);
            t=double(subs(t,'theta',thetax));
        end
        
        function x=forwardPrediction(obj,p,v,T)
            %使用物理根据当前小球的位置和速度估计t时间后小球的位置和姿态
            px=T*v(1)+p(1);
            py=p(2);
            vx=v(1);
            vy=v(2);
            t=0;vtemp=vy;ptemp=py;
            b=0;counts=0;
            while(t+b<T)
                t=t+b;
                vy=vtemp;
                py=ptemp;
                a=sqrt(vy^2+2*World.g*py);
                b=(a+vy)/World.g;
                vtemp=a*0.8;
                ptemp=0;
                counts=counts+1;
            end
            tl=T-t;
            if counts~=1
                py=vy*tl-0.5*World.g*tl*tl;
            else
                py=vy*tl-0.5*World.g*tl*tl+py;
            end
            vy=vy-World.g*tl;
            theta=atan2(vy,vx)+pi;
            x=[px;py;theta];
        end
    end
    methods(Static)
        function bouncewithwall(src,~)
            disp('Bounce with wall.');
            if(src.vx>0)
                t=(src.px-World.limx)/src.vx;
                src.px=World.limx;
            else
                t=src.px/src.vx;
                src.px=0;
            end
            src.vx=-World.ratio*src.vx;
            src.move(t);
            src.boundsTimewall=src.boundsTimewall+1;
        end
        function bouncewithground(src,~)
            disp('Bounce with ground.');
            v=sqrt(src.vy^2+2*World.g*src.py);
            t=(-src.vy-v)/World.g;
            src.vy=v*World.ratio;
            src.py=0;
            src.move(t);
            src.boundsTime=src.boundsTime+1;
        end
    end
end