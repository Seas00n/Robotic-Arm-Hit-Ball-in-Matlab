classdef World < handle
    properties(Constant)
        ratio=0.8;
        limx=10;
        limy=8;
        g=9.8;
        sample=0.05;
        boundrange=0.5;
    end
    methods
        function obj=World()
        end
    end
end