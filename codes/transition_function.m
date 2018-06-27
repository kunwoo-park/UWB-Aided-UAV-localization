function y = transition_function(x,A_S_control,W_S_control,dt,errors)
 
V_previous=x(1:3,1);
A_b_previous=x(4:6,1);
W_b_previous=x(7:9,1);
B_previous=x(10:12,1);
q_previous=x(13:15,1);

epsilon_V=errors(1:3,1);
epsilon_A_b=errors(4:6,1);
epsilon_W_b=errors(7:9,1);
epsilon_q=errors(10:12,1);



% ******state vector******  
% *velocity: V, *biases of accelerometers: A_b, *biases of gyroscopes: W_b
% *position: B, *orientation: q (Euler or quarternion)
% x=[V,A_b,W_b,B,q]' 15state(Euler) or 16 state(quarternion)

% *****control of pose state******
% A_S Accelerometer measurements 
% W_S Gyroscope measurements



R_body2global_previous=Rotation_e2i(q_previous(1),q_previous(2),q_previous(3));

V = V_previous + R_body2global_previous*(A_S_control+A_b_previous+epsilon_A_b)*dt+epsilon_V;
% A_b,W_b remain constant
A_b=A_b_previous+epsilon_A_b;
W_b=W_b_previous+epsilon_W_b;

B= B_previous + (V_previous+epsilon_V)*dt+0.5* R_body2global_previous*(A_S_control+A_b+epsilon_A_b)*dt^2;
rotated_angle=(W_S_control+W_b+epsilon_q)*dt;
R_body2global=R_body2global_previous*Rotation_e2i(rotated_angle(1),rotated_angle(2),rotated_angle(3));

omega=atan2(-R_body2global(3,2),R_body2global(3,3));
fi=atan2(R_body2global(3,1),sqrt(R_body2global(3,2)^2+R_body2global(3,3)^2));
kappa= atan2(-R_body2global(2,1),R_body2global(1,1));

q=[omega;fi;kappa];

y=[V;A_b_previous;W_b_previous;B;q];

end

