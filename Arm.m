classdef Arm<handle
    properties
        robot
        L1
        L2
        gik
        q=[0;0;0]
        vtool=10
        toolL=1.5
        heightAboveTable
        distanceFromTarget
        fixOrientation
        limitJointChange
    end
    properties(Dependent)
        ptool
        otool
        Htool
        invHtool
    end
    methods 
        function obj=Arm()
            obj.L1=1.3;obj.L2=1.3;
            L1=obj.L1;L2=obj.L2;
            robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);
            %%
            body = rigidBody('link1');
            joint = rigidBodyJoint('joint1', 'prismatic');
            setFixedTransform(joint,trvec2tform([0 0 0]));
            joint.PositionLimits=[-5,10];
            joint.JointAxis = [1 0 0];      
            body.Joint = joint;
            addBody(robot, body, 'base');
            %%
            body = rigidBody('link2');
            joint = rigidBodyJoint('joint2','revolute');
            setFixedTransform(joint, trvec2tform([0,0,0]));
            joint.JointAxis = [0 0 1];
            body.Joint = joint;
            addBody(robot, body, 'link1');
            body = rigidBody('link3');
            joint = rigidBodyJoint('joint3','revolute');
            setFixedTransform(joint, trvec2tform([L1, 0, 0]));
            joint.JointAxis = [0 0 1];
            body.Joint = joint;
            addBody(robot, body, 'link2');
            body = rigidBody('tool');
            joint = rigidBodyJoint('joint4','fixed');
            setFixedTransform(joint, trvec2tform([L2, 0, 0]));
            body.Joint = joint;
            addBody(robot, body, 'link3');
            
            body = rigidBody('l1');
            joint = rigidBodyJoint('joint5','fixed');
            setFixedTransform(joint, trvec2tform([0, -obj.toolL/2, 0]));
            body.Joint = joint;
            addBody(robot, body, 'tool');
             body = rigidBody('l2');
            joint = rigidBodyJoint('joint6','fixed');
            setFixedTransform(joint, trvec2tform([0, obj.toolL/2, 0]));
            body.Joint = joint;
            addBody(robot, body, 'tool');
            
            showdetails(robot);
            obj.robot=robot;
            obj.gik=generalizedInverseKinematics('RigidBodyTree',obj.robot,'ConstraintInputs',{'cartesian','position','orientation','joint'});
            obj.heightAboveTable=constraintCartesianBounds('tool');
            obj.heightAboveTable.Bounds=[0,inf;...
                                         0,inf;...
                                         -inf,inf];
            obj.distanceFromTarget=constraintPositionTarget('tool');
            obj.distanceFromTarget.PositionTolerance=0.3;
            obj.fixOrientation=constraintOrientationTarget('tool');
            obj.fixOrientation.OrientationTolerance=deg2rad(5);
            obj.limitJointChange=constraintJointBounds(obj.robot);
        end
        function q=InverseK(obj,p,o)
            qInitial=obj.q;
            obj.distanceFromTarget.TargetPosition=[p;0];
            obj.fixOrientation.TargetOrientation=tform2quat([
                cos(o),-sin(o),0  0;...
                sin(o),cos(o), 0  0;...
                0       0      1  0;
                0       0      0  1]);
            obj.limitJointChange.Bounds=[-5,15;
                                         -pi,pi;
                                         -pi,pi];
            q=obj.gik(qInitial,obj.heightAboveTable,obj.distanceFromTarget,obj.fixOrientation,obj.limitJointChange);
        end
        function q=InverseK2(obj,p,o,qInitial)
            ik = inverseKinematics('RigidBodyTree', obj.robot);
            weights = [0, 0, 0,1,1,0];
            endEffector = 'tool';
            a=[cos(o),-sin(o),0  p(1);...
                sin(o),cos(o),0  p(2);...
                0       0     1  0;
                0       0     0  1];
            q=ik(endEffector,a,weights,qInitial);
        end
        %给出末端执行器的位姿计算和基座的齐次变换矩阵
        function Htool=get.Htool(obj)% p=[px;py], o=theta
            R=[cos(obj.otool),-sin(obj.otool),0;...
                sin(obj.otool),cos(obj.otool),0;...
                0 0 1];
            D=[obj.ptool;0];
            Htool=[R D;0 0 0 1];
        end
        
        function ptool=get.ptool(obj)
            ptool=[obj.q(1)+obj.L1*cos(obj.q(2))+obj.L2*cos(obj.q(2)+obj.q(3));...
                obj.L1*sin(obj.q(2))+obj.L2*sin(obj.q(2)+obj.q(3))];
        end
        function otool=get.otool(obj)
            otool=obj.q(2)+obj.q(3);
        end
        function invHtool=get.invHtool(obj)
            R=obj.Htool(1:3,1:3).';
            D=-1*R*obj.Htool(1:3,4);
            invHtool=[R D;0 0 0 1];
        end
    end
    
end