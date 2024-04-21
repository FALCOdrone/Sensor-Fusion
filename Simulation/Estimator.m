classdef Estimator < handle 
    properties
        ekfCov
        ekfState
        ekfCov_at
        ekfCov_pred
        xt_at
        estAttitude
        dt
        Q
        Q_at
        R
        R_at
        R_GPS
        R_Mag
        Nstate
        imu_acc_bias
        imu_gyro_bias

        %for adaptive
        %adjustable process noice
        scaling_factor 
        scaling_factor_at
        epsilon_max_at
        epsilon_max
        count
        gps_residual
        attitude_imu_residual

        %fading memory filter
        alpha
    end
    
    methods
        function obj = Estimator(initial_state, initial_stddev, initial_attitude, imu_acc_bias, imu_gyro_bias)
            %ESTIMATOR Construct an instance of this class
            obj.Nstate = length(initial_state);

            obj.ekfState = initial_state;
            obj.ekfCov = initial_stddev;
            obj.xt_at = initial_attitude;
            obj.estAttitude = quat2eul(obj.xt_at', 'ZYX');
            
            obj.dt = 0.01;
            obj.Q = eye(7);   %model covariance
            obj.Q(1,1) = 0.5^2 ;      
            obj.Q(2,2) = 0.5^2;     
            obj.Q(3,3) = 0.05^2;   
            obj.Q(4,4) = 0.5^2;    
            obj.Q(5,5) = 0.5^2; 
            obj.Q(6,6) = 0.05^2;    
            obj.Q(7,7) = 0.095^2;   %related to yaw
            obj.Q = obj.Q * obj.dt;
            obj.ekfCov_at = eye(4);
            obj.Q_at = eye(4) * 0.00001;

            obj.R_at = eye(4) * 100; % needs to be settled correctly consulting the datasheet
           
            obj.R_GPS = zeros(6,6); %GPS noise matrix
            %stimati dai dati
            obj.R_GPS(1,1) = 0.16;      %precedentemente messi a 0.1^2
            obj.R_GPS(2,2) = 0.3;       %precedentemente messi a 0.1^2
            obj.R_GPS(3,3) = 1.1;         %precedentemente messi a 0.3^2
            obj.R_GPS(4,4) = 0.1^4;     %come sopra
            obj.R_GPS(5,5) = 0.1^4;
            obj.R_GPS(6,6) = 0.1^3;

            obj.imu_acc_bias = imu_acc_bias;
            obj.imu_gyro_bias = imu_gyro_bias;

            %for adaptive
            %adjustable process noice
            obj.scaling_factor=1000; 
            obj.scaling_factor_at=1000;
            obj.epsilon_max=4;
            obj.epsilon_max_at=4;
            obj.count=0;
            %Fading memory filter
            obj.alpha= 1.05; %stay near 1 but >1 like 1.05
           

       
        end
        
        function predict(obj, acc, gyro, fading,residual) %fading parameter allow to use or not use the faind memory filter
            acc = acc - obj.imu_acc_bias;
            gyro = gyro - obj.imu_gyro_bias;
            attitude_estimation(obj, acc,gyro, fading, residual);
            M = QuatRotMat(obj.xt_at);
            inertial_acc = M*acc;
            inertial_acc(3) = inertial_acc(3) + 9.81;
            obj.ekfState(1:3) = obj.ekfState(1:3) + obj.ekfState(4:6)*obj.dt;
            obj.ekfState(4:6) = obj.ekfState(4:6) + inertial_acc*obj.dt;
            
            RbgPrime = GetRbgPrime(obj);
            gPrime = eye(7);
            gPrime(1, 4) = obj.dt;
            gPrime(2, 5) = obj.dt;
            gPrime(3, 6) = obj.dt;
        
            helper_matrix = RbgPrime * acc;
            gPrime(4:6, 7) = helper_matrix * obj.dt;
            
            obj.ekfCov = gPrime * obj.ekfCov * gPrime' + obj.Q;
                    
        end
        
        function attitude_estimation(obj,acc,gyro, fading, residual)
        
            accelPitch = asin(-acc(1) / (-9.81));
            accelRoll = asin(acc(2) / (-9.81 * cos(accelPitch)));
                
            B =  [0, -gyro(1), -gyro(2), -gyro(3);
              gyro(1), 0, -gyro(3), gyro(2);
              gyro(2), gyro(3), 0, -gyro(1);
              gyro(3), -gyro(2), gyro(1), 0];
        
            A = eye(4) + obj.dt * 0.5 * B;
            obj.xt_at = A * obj.xt_at;
        
            % update
            H_at = eye(4);
            
            if fading == 0
                obj.ekfCov_pred = A * obj.ekfCov_at * A' + obj.Q_at;
            elseif fading == 1 %fading memory filter
                obj.ekfCov_pred = obj.alpha^2 * A * obj.ekfCov_at * A' + obj.Q_at;
            end

            M = H_at * obj.ekfCov_pred * H_at' + obj.R_at;
            k_at = obj.ekfCov_pred * H_at'/M;
            
            euler = [obj.estAttitude(1), accelPitch, accelRoll];
            z = eul2quat(euler, 'ZYX');
            obj.xt_at = obj.xt_at + k_at * (z' - H_at * obj.xt_at);
            obj.ekfCov_at = obj.ekfCov_pred - k_at * H_at * obj.ekfCov_pred;
            obj.estAttitude = quat2eul(obj.xt_at', 'ZYX');
            %normalization of quaternions
            obj.xt_at=obj.xt_at/norm (obj.xt_at);

            % normalizetion of residuals
            S = H_at * obj.ekfCov_at * H_at' + obj.R_at;
            y = z' - H_at * obj.xt_at;
            obj.attitude_imu_residual = y' / S * y;
            if residual ==1
                if obj.attitude_imu_residual > obj.epsilon_max_at
                    obj.Q_at=obj.Q_at*obj.scaling_factor_at;
                    obj.count = obj.count + 1;
                elseif obj.count > 0
                    obj.Q_at = obj.Q_at / obj.scaling_factor_at;
                    obj.count= obj.count - 1;
                end
            end
            
        end
        
        function updateFromGps(obj, gps_pos,gps_vel,HDOP,VDOP, residual)
            z = [gps_pos; gps_vel];
            %using HDOP for  R_GPS
            %obj.R_GPS(1,1) = (HDOP*5)^1/2; %formule da verificare
            %obj.R_GPS(2,2) = (HDOP*5)^1/2;
            %obj.R_GPS(3,3) = (VDOP*5)^1/2;
            %obj.R_GPS(4,4) = (HDOP*5)^1/2;
            %obj.R_GPS(5,5) = (HDOP*5)^1/2;
            %obj.R_GPS(6,6) = (VDOP*5)^1/2;
            hPrime = eye(6,7);
            zFromX = obj.ekfState(1:6);
            update(obj, z, hPrime, obj.R_GPS, zFromX, residual);
        end
        
        function update(obj, z, H, R, zFromX, residual)
            toInvert = H * obj.ekfCov * H' + R;
            K = obj.ekfCov * H' / toInvert;
            obj.ekfState = obj.ekfState + K*(z - zFromX);
            obj.ekfCov = (eye(obj.Nstate) - K*H)*obj.ekfCov;

            if H == eye(6,7)
                obj.gps_residual = (z - H*obj.ekfState)' / toInvert * (z - H*obj.ekfState);      % normalized gps residual
            end 
            if residual==1
                if obj.gps_residual > obj.epsilon_max
                    obj.Q = obj.Q*obj.scaling_factor;
                    obj.count = obj.count + 1;
                elseif obj.count>0 
                    obj.Q = obj.Q/obj.scaling_factor;
                    obj.count = obj.count -1;
                end
            end
                
        end

        function RbgPrime = GetRbgPrime(obj)

            RbgPrime = zeros(3,3);
            RbgPrime(1, 1) = -cos(obj.estAttitude(2)) * sin(obj.estAttitude(1));
            RbgPrime(1, 2) = -sin(obj.estAttitude(3)) * sin(obj.estAttitude(2)) * sin(obj.estAttitude(1)) - cos(obj.estAttitude(3)) * cos(obj.estAttitude(1));
            RbgPrime(1, 3) = -cos(obj.estAttitude(3)) * sin(obj.estAttitude(2)) * sin(obj.estAttitude(1)) + sin(obj.estAttitude(3)) * cos(obj.estAttitude(1));
            RbgPrime(2, 1) = cos(obj.estAttitude(2)) * cos(obj.estAttitude(1));
            RbgPrime(2, 2) = sin(obj.estAttitude(3)) * sin(obj.estAttitude(2)) * cos(obj.estAttitude(1)) - cos(obj.estAttitude(3)) * sin(obj.estAttitude(1));
            RbgPrime(2, 3) = cos(obj.estAttitude(3)) * sin(obj.estAttitude(2)) * cos(obj.estAttitude(1)) + sin(obj.estAttitude(3)) * sin(obj.estAttitude(1));
        end
    end
end

