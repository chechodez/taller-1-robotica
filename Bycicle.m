classdef Bycicle 
    %DiffDrive Differential drive kinematic model
    %   Calculates forward and inverse kinematics for a differntial drive
    %   robot
    %   Author and Copyright Carlos Santacruz 2021

    properties
        % Wheel radius in meters [m]
        WheelRadius  = 1.0;

        
        %Distance from wheel back to wheel forward
        WheelBase   = 1.0;
    end
    
    methods
        function obj = Bycicle(wheelRadius,  wheelBase)
            %DiffDrive Class Constructor
            % Inputs: 
            %   wheelRadius: wheel radius [m]
            %   trackWidth: distance from wheel to wheel [m]
            %   wheelBase: distance wheel back to wheel forward
            
            % Assign parameters
            obj.WheelRadius = wheelRadius;
        
            obj.WheelBase = wheelBase;
        end
        
        function r_dxi = calcForwardKinematics(obj, wr,delta) 
            %CALCFORWARDKINEMATICS Calculates forward kinematics
            % Inputs:
            %    wr: back speed [rad/s]
            %    zita=angulo en radianes de la llanta delantera
            % Outputs:
                   
            %       vx: robot speed in robot frame [m/s]
            %       vy: robot angular vel in robot frame [rad/s]            
            %       W: velocidad angular
            vx = obj.WheelRadius*wr;
            vy = 0;
            w=obj.WheelRadius*wr*tan(delta)/(obj.WheelBase);
            r_dxi = [vx;vy;w];
        end
        
        function [wr, delta] = calcInverseKinematics(obj, r_dxi)
            %CALCINVERSEKINEMATICS Calculates forward kinematics
            % Inputs:
            %    r_dxi : [vx; 0 ; wz] robot velocity vector in robot base frame             
            %       vx: robot linear speed in robot frame  [m/s]
            %       wz: robot angular speed in robot frame  [rad/s]          
            % Outputs:
            %       wr: right wheel speed [rad/s]
            %       wl: left wheel speed [rad/s]
            
            vx = r_dxi(1);
            wz = r_dxi(3);

            wr = vx/obj.WheelRadius;
            delta = atan2((wz*obj.WheelBase),(vx));            
        end
        
        
    end
    
    
end

