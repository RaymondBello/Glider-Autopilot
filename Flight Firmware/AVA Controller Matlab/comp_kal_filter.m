clc;
clear;
close all;

% Gyro Accel sim Parameters
rollTruek1 = 0.0;
rollTruek = 0.0;
rollVelTrue = 0.0;
gyroDriftTrue = 1.0;
gyroCalBiasTrue = 0.01;
gyroSignalNoise = 0.002;
accelSignalNoise = sqrt(0.03);

accelMeask1 = 0.0;
accelMeask = 0.0;
gyroMeask1 = 0.0;
gyroMeask = 0.0;

deltaT = 0.004;    %250Hz
time = 0.0;     %Init sim time
dataStore = zeros(5010,7);  %Data Store array

% Complementary filter parameters
angleRollk1 = 0.0;
angleRollk = 0.0;

gainK1 = 0.90;      %Gyro Gain
gainK2 = 0.10;      %Gyro Drift correction gain

% Kalman Filter Parameters
xk = zeros(1,2);
pk = [0.5 0 0 0.01];
K = zeros(1,2);
phi = [1 deltaT 0 1];
psi = [deltaT 0];
R = 0.03;
Q = [0.002^2 0 0 0];
H = [1 0];


% Simulation loop. Every 250 iters is 1 second of sim time

endPoint = 2500;
for i = 1:endPoint
    
    % Move gyro using velocity at specific times to simulaate flying 
    
    rollVelTrue = 0.0;
    if (i>=500) &&(i<750)
        rollVelTrue = 30.0;
    end
    if (i>=1250) && (i<1500)
        rollVelTrue = -40.0;
    end
    if (i>=1750) && (i<2000)
        rollVelTrue = 10.0;
    end
    
    % Create Gyro Data at each time step (k+1). 
    % Gyro Measures (deg/sec). Accel provides (g's) converted to degrees.
    
    rollTruek1 = rollTruek + rollVelTrue * deltaT;
    accelMeask1 = rollTruek1 + randn(1) * accelSignalNoise; %Degrees
    gyroReadk1 = rollVelTrue - gyroDriftTrue +rand(1) * gyroSignalNoise + gyroCalBiasTrue;
    
    gyroMeask1 = gyroReadk1 - gyroCalBiasTrue;      %Degrees per second
    
    % Completary to solve for roll using drift-corrcted measurements
    
    angleRollk1 = gainK1 * (angleRollk + gyroMeask1* deltaT) + gainK2 * (accelMeask1);
    
    % Kalman Filter to estimate roll and gyro bias. 
    
    uk = gyroMeask1;
    zk = accelMeask1;
    
    % xk1Minus = phi * xk + psi * uk;
    xk1Minus(1) = phi(1) * xk(1) + phi(2) * xk(2) + psi(1) * uk;
    xk1Minus(2) = phi(3) * xk(1) + phi(4) * xk(2) + psi(2) * uk;
    
    % pk1Minus = phi * pk * phi' + Q;
    pk1Minus(1) = (phi(1) * pk(1) + phi(2) * pk(3)) * phi(1) + (phi(1) * pk(2) + phi(2) * pk(4)) * phi(2) + Q(1);
    pk1Minus(2) = (phi(1) * pk(1) + phi(2) * pk(3)) * phi(3) + (phi(1) * pk(2) + phi(2) * pk(4)) * phi(4) + Q(2);
    pk1Minus(3) = (phi(3) * pk(1) + phi(4) * pk(3)) * phi(1) + (phi(3) * pk(2) + phi(4) * pk(4)) * phi(2) + Q(3);
    pk1Minus(4) = (phi(3) * pk(1) + phi(4) * pk(3)) * phi(3) + (phi(3) * pk(2) + phi(4) * pk(4)) * phi(4) + Q(4);
    
    % S = H * pk1Minus * H' + R;
    S = (H(1) * pk1Minus(1) + H(2) + pk1Minus(3)) * H(1) + (H(1) * pk1Minus(2) + H(2) * pk1Minus(4)) * H(1) + R;
    
    % K = pk1Minus * H' * ins(S);
    K(1) = (pk1Minus(1) * H(1) + pk1Minus(2) * H(2))/S;
    K(2) = (pk1Minus(3) * H(1) + pk1Minus(4) * H(2))/S;
    
    % xk1 = xk1Minus + K * (zk - H * xk1Minus);
    xk1(1) = xk1Minus(1) + K(1) * (zk- (H(1) * xk1Minus(1) + H(2) * xk1Minus(2)));
    xk1(2) = xk1Minus(2) + K(2) * (zk - (H(1) * xk1Minus(1) + H(2) * xk1Minus(2)));
    
    % pk1 = (eye(2,2) - K * H) * pk1Minus;
    pk1(1,1) = (1 - K(1) * H(1)) * pk1Minus(1) + (0-K(1) * H(2)) * pk1Minus(3);
    pk1(1,2) = (1 - K(1) * H(1)) * pk1Minus(2) + (0-K(1) * H(2)) * pk1Minus(4);
    pk1(2,1) = (1 - K(2) * H(1)) * pk1Minus(1) + (0-K(2) * H(2)) * pk1Minus(3);
    pk1(2,1) = (1 - K(2) * H(1)) * pk1Minus(2) + (0-K(2) * H(2)) * pk1Minus(4);
    
            
    % Data Store
    dataStore(i,:) = [time rollTruek1 accelMeask1 gyroMeask1 angleRollk1 xk1(1) xk1(2)];
    
    % Reset Values for next iteration
    angleRollk = angleRollk1;
    rollTruek = rollTruek1;
    time = time + deltaT;
    
    xk = xk1;
    pk = pk1;
    
end

%Plot Results
plot(dataStore(1:endPoint, 1), dataStore(1:endPoint, 2), 'r');
hold
plot(dataStore(1:endPoint, 1), dataStore(1:endPoint, 5), 'b');
plot(dataStore(1:endPoint, 1), dataStore(1:endPoint, 6), 'g');    
grid on
axis([0 10 -15 35])
xlabel('Time (sec)')
ylabel('Angle (deg)')
%legend('True Angle')
%legend('True Angle','Comp Filter')
legend('True Angle', 'Comp Filter', 'Kalman Filter')
    

