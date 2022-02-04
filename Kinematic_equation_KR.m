
%%THE FOLLOWLING FUNCTION CAN COMPUTE THE APPROXIMATE KINEMATIC MODEL OF
%%N-SECTION CONTINUUM ROBOT BASED ON THE FOLLOWING ARTICLES
% (1) Jones, B. A., & Walker, I. D. (2006). Kinematics for multisection continuum robots. IEEE Transactions on Robotics, 22(1), 43-55.
% (2)Amouri, A., Zaatri, A., & Mahfoudi, C. (2018). Dynamic modeling of a class of continuum manipulators in A fixed orientation. Journal of Intelligent & Robotic Systems, 91(3), 413-424.
% (3)Seleem, I. A., El-Hussieny, H., Assal, S. F., & Ishii, H. (2020). Development and stability analysis of an imitation learning-based pose planning approach for multi-section continuum robot. IEEE Access, 8, 99366-99379. 
% (4) Seleem, I. A., Assal, S. F., Ishii, H., & El-Hussieny, H. (2019). Guided pose planning and tracking for multi-section continuum robots considering robot dynamics. IEEE Access, 7, 166690-166703.
% THE EXACT KINEMATIC MODEL OF (1) SUFFERS FROM SINGULARITY IF THE ROBOTIC
% ARM IN STRAIGHT CONFIGURATION.
% TAYLOR EXPANSION SERIES WERE APPLIED TO OVERCOME THE SINGULARITY

%% FIRST THE BY RUNNING THE FUNCTION, YOU SHOULD ENTER NUMBER OF SECTIONS (n), OUTPUT THE OVERALL HOMOGENEOUS TRANSFORMATION MATRIX (Hend0)
%% all rights reserved to the author
%% please if you will use this code, cite the aforementioned paper from (1) to (4) to keep the author rights

function Hend0=Kinematic_equation_KR()

tic

syms h                     %% LENGTH OF NEUTRAL AXIS
syms theta kappa  'real'   %% CONFIGURATION PARAMETERES
syms L1 L2 L3 L(n) n  real %% CABLE LENGTHS
syms r                     %% DISTANCE FROM THE NEUTRAL AXIS TO EACH CABLE (radius of curvature) 

prompt = 'What is the original value? ';
n = input(prompt);

%%ROTATION MATRIX
R=[(cos(theta)^2*(cos(kappa*h)-1)+1),sin(theta)*cos(theta)*(cos(kappa*h)-1),-cos(theta)*sin(kappa*h);...
    sin(theta)*cos(theta)*(cos(kappa*h)-1),(cos(theta)^2*(1-cos(kappa*h))+cos(kappa*h)),-sin(theta)*sin(kappa*h);...
    cos(theta)*sin(kappa*h),sin(theta)*sin(kappa*h),cos(kappa*h)]; %% Rotation matrix


%%Taylor Expansion Is Applied To Overcome The Singularity At Vertical Position

X1= simplify((cos(theta)*(taylor(cos(kappa*h),'ExpansionPoint',0,'Order', 11)-1))/kappa);
Y1= simplify((sin(theta)*(taylor(cos(kappa*h),'Order', 11)-1))/kappa);
Z1 = simplify(taylor(sin(kappa*h),'Order', 11)/kappa);

%%HOMOGENOUS TRANSFROAMTION MATRIX

H=[R,[X1;Y1;Z1];
   0, 0, 0, 1];

%%SUBSTITUTE CONFIGURATION PARAMETERS AS A FUNCTION OF THE LENGTHS VALUE 
hl=((n*r*(L1+L2+L3))/sqrt(L1.^2+L2.^2+L3.^2-L1*L2-L2*L3-L1*L3))*asin(sqrt(L1.^2+L2.^2+L3.^2-L1*L2-L2*L3-L1*L3)/(3*n*r));
kappal=((2*sqrt(L1.^2+L2.^2+L3.^2-L1*L2-L2*L3-L1*L3))/(r*(L1+L2+L3)));
thetal=atan2(3*(L2-L3),(sqrt(3)*(L3+L2-2*L1)));


Hend_0=subs(H,[h kappa theta],[hl kappal thetal]);




if(n==0)
    disp('YOU ENTERED WRONG SECTION NUMBER')
    return;
end

if(n==1)
   
    Hend_0=subs([R,[X1;Y1;Z1];0, 0, 0, 1],[h kappa theta],[hl kappal thetal]); %% HOMOGENEOUS TRANSFROMATION MATRIX OF ONE SECTION
                                                                                %  FROM END EFFECTOR TO BASE FRAME
end


if (n>1)
    H=cell(1,n-1); %% CREATE CELL TO SAVE THE GENERATED HOMOGENEOUS TRANSFROMATION MATRIX
    i=0;
    for j=1:1:n-1
    
        H{j}=subs(Hend_0,[L1 L2 L3],[L(j+i+3) L(j+i+4) L(j+i+5)]);
      
        Hend0=Hend_0*[H{j}];   %% HOMOGENEOUS TRANSFROMATION MATRIX OF n-SECTION
                               %  FROM END EFFECTOR TO BASE FRAME
                               % [H{j}] IS USED TO CONVERT THE GENERATED
                               % CELL TO MATRIX FOR VALID MULTIPLICATION
        i=i+2;
    end

end

toc

end











