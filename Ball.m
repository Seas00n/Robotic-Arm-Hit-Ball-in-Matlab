classdef Ball <handle
    properties
        mass
        px
        py
        vx
        vy
        p0
        v0
        boundsTime=0
        boundsTimewall=0
        boundsTimeArm=0;
        stopping=false
        rolling=false
    end
    properties(Dependent)
        p
        v
    end
    events
        bouncewithwall
        bouncewithground
    end
    methods
        function obj=Ball(mass,pi,vi)
            obj.mass=mass;
            obj.px=pi(1);
            obj.py=pi(2);
            obj.vx=vi(1);
            obj.vy=vi(2);
            obj.p0=[pi(1);pi(2)];
            obj.v0=vi;
        end
        function p=get.p(obj)
            p=[obj.px;obj.py];
        end
        function v=get.v(obj)
            v=[obj.vx;obj.vy];
        end
        function move(obj,T)
            if(~obj.rolling)
                obj.py=obj.py+obj.vy*T-0.5*World.g*T^2;
                obj.vy=obj.vy-World.g*T;
            end
            if(~obj.stopping)
                obj.px=obj.px+obj.vx*T;
                obj.vx=obj.vx;     
            end                         
        end
        % 触发事件
        function isbouncehappend(obj,arm,mode)
            if(obj.py<=0)
                notify(obj,'bouncewithground');
            elseif(obj.px<=0||obj.px>=World.limx)
                notify(obj,'bouncewithwall');
            end
            if(size(arm,1)~=0)
                pr=arm.invHtool*[obj.p;0;1];
                if ~isreal(pr)
                    pr
                end
                if(pr(1)<World.boundrange&&pr(2)>-arm.toolL/2&&pr(2)<arm.toolL/2)
                    pr
                    obj.bouncewitharm(arm,mode);
                end
            end
            obj.isroll();
            obj.isstop();
        end
        function isstop(obj)
            if(abs(obj.vx)<0.001&&obj.py==eps)
                obj.vx=0;
                obj.py=0;
                obj.stopping=true;
            end
        end
        function isroll(obj)
            if(abs(obj.vy)<0.001&&obj.py==eps)
                obj.vy=0;
                obj.py=0;
                obj.rolling=true;
            end
        end
        function bouncewitharm(obj,arm,mode)
            obj.boundsTime=obj.boundsTime+1;
            disp('Bounce with arm');
            pr=arm.invHtool*[obj.p;0;1];
            vr=arm.invHtool*[obj.v;0;1];
            gr=arm.invHtool*[0;-World.g;0;1];%把小球的位置，速度，加速度先转换到tool坐标系下
            %根据位移反算碰撞的速度和时间
            %碰撞发生在x方向
            y=pr(1)-World.boundrange;
            vbx=-sqrt(vr(1)^2-2*gr(1)*y);%碰撞前一刻x方向的速度大小
            t=(vr(1)-vbx)/gr(1);%碰撞发生距离采样点的时间
            vb=[obj.v(1);obj.v(2)-World.g*t;0;1];
            vby=vr(2)-gr(2)*t;%碰撞前一刻y方向的速度
            pb=[World.boundrange;pr(2)-vby*t-0.5*gr(2)*t^2;0;1];%碰撞发生的位置,在tool坐标系下
            pb=arm.Htool*pb;%碰撞发生的位置在基坐标系下
            pbw=[obj.p0;0;1]-pb;%到墙面的位移
            theta=arm.otool;
            x=arm.ptool(1);
            y=arm.ptool(2);
            if mode==1
                [arm.vtool,obj.vx,obj.vy]=Trajectory.bouncetotheoriginal(pbw,x,y,theta,vby);
            elseif mode==2
                obj.boundsTime
                if vb(1)<0 %向前打
                    pbw=[2-0.2*obj.boundsTime;0.05];
                else
                    pbw=[-2+0.2*obj.boundsTime;0.05];
                end
                [arm.vtool,obj.vx,obj.vy]=Trajectory.bounceperpendicular(pbw,x,y,theta,vby);
            end
%             vr=arm.Htool*[arm.vtool;vby;0;1];
            pr=pb;
            obj.px=pr(1);
            obj.py=pr(2);
            obj.move(t);
            obj.boundsTimeArm=1;
        end
    end
end