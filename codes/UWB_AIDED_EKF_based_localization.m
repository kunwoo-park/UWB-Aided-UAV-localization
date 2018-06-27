% UWB_AIDED_EKF_based_localization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This code was made by Kunwoo Park, York University
%2018.06.11
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1. import data

import_filename = 'data_csrs_wo_corio_ver11.csv'; %'data_csrs_w_corio_ver10.csv';
All_data = csvread(import_filename);
length_All_data=size(All_data,1)/2;

%{ Current data import method is memory-intensive and it should be modified later.%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2. state vector   

% *velocity: V, *biases of accelerometers: A_b, *biases of gyroscopes: W_b
% *position: B, *orientation: q (Tait bryan angle)
% x=[V,A_b,W_b,B,q]' 15state

% control of pose state 
% A_S Accelerometer measurements 
% W_S Gyroscope measurements

V=[1,0,0.1]';
A_b=[0,0,0]';
W_b=[0,0,0]';
B=[0.01,0.000017,0.001]';
q=[0,0,-0.00349100]';  

x=[V;A_b;W_b;B;q];

P=diag(ones(1,15))*1; %Covariance Matrix of current state - Detailed model should be applied

% Noise Model
std_V_vec=[0.2,0.2,0.2]; %sigma_x,sigma_y,sigma_z
std_A_b_vec=[0.3,0.3,0.3];
std_W_b_vec=[0.5,0.5,0.5]/180*pi();
std_q_vec=[0.1,0.1,0.1];
Q=diag([std_V_vec.^2,std_A_b_vec.^2,std_W_b_vec.^2,std_q_vec.^2]);

std_UWB=0.05; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3. initailizing
time_previous=0;
x_All = zeros(20001,25); % output file format: time, estimated rotation matrix, estimated position,reference rotation matrix, reference position)
x_All(1,:)=[0,All_data(6,2:10),All_data(8,2:4),All_data(6,2:10),All_data(8,2:4)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 4. EKF
figure
curve1 = animatedline('LineWidth',2);

for i=1:length_All_data
  measurement_type=All_data(2*i-1,1);
  time=All_data(2*i,1);
  dt=time-time_previous;

  switch measurement_type
       
      case 2 %IMU
            A_S_control=All_data(2*i,2:4)'; % to add random noise  + randn([3,1]).*std_A_b_vec'; 
            W_S_control=-All_data(2*i,5:7)'; % to add random noise  + randn([3,1]).*std_W_b_vec'; 
            
%prediction
A=NumJacob(@transition_function,x,A_S_control,W_S_control,dt,zeros(12,1));
W=NumJacob(@transition_function2,zeros(12,1),A_S_control,W_S_control,dt,x);

P=A*P*A'+W*Q*W';
x=transition_function(x,A_S_control,W_S_control,dt,zeros(12,1));


index=time*100+1;  
index=cast(index,'int16');
x_All(index,11:13)=x(10:12,1)';  
rotation_mat=Rotation_e2i(x(13),x(14),x(15));
x_All(index,2:4)=rotation_mat(1,1:3);
x_All(index,5:7)=rotation_mat(2,1:3);
x_All(index,8:10)=rotation_mat(3,1:3);

        case 0 %UWB  

            UWB_ID=All_data(2*i,2);
            z=All_data(2*i,3);%+R*randn(1);
            switch UWB_ID
              case 100
                  anchor_position=[-5;-5;1.0];
              case 101
                  anchor_position=[ 5;-5; 1.2];
              case 102
                  anchor_position=[-5; 5; 1.4];
              case 103
                  anchor_position= [5; 5;1.6];
            end
            
% correction            
            z1 = norm(x(10:12)' - anchor_position');
H=1/z1*[zeros(1,9), x(10:12)' - anchor_position', zeros(1,3) ];


%H*P*H'
P12=P*H';                   %cross covariance
K=P12/(H*P12+std_UWB);       %Kalman filter gain


x=x+K*(z-z1);            %state estimate
P=P-K*P12';               %state covariance matrix

index=(time*100+1)*1;  
index=cast(index,'int16');
x_All(index,11:13)=x(10:12,1)';    
rotation_mat=Rotation_e2i(x(13),x(14),x(15));
x_All(index,2:4)=rotation_mat(1,1:3);
x_All(index,5:7)=rotation_mat(2,1:3);
x_All(index,8:10)=rotation_mat(3,1:3);
        case 3  %height
        
        case 10  %height
        index=time*100+1;  
        index=cast(index,'int16');
            x_All(index,14:22)=All_data(2*i,2:10);
        
        case 11  %height
          index=time*100+1;  
            index=cast(index,'int16');
            x_All(index,23:25)=All_data(2*i,2:4);
            
       
        otherwise
            disp('error');
 end

time_previous=time;
    addpoints(curve1,x(10),x(11),x(12));
xlabel('x');ylabel('y');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 5. save result

csvwrite('result_EKF_data_csrs_wo_corio_ver11.csv',x_All);