% Code to to model geometrically exact concentric tube robots by Rucker et
% al.
% Author: Keshav Iyengar

% Initial value of twist and joint inputs
uz_0 = [0 0 0]; % uz_0 is a column vector. uz is initial torsion
q = [0; 0; 0; pi/2; -pi; 0]; % q1 to q3 is extension and q6 is rotation of each tube from base

[r1,r2,r3,Uz] = CTRModel(q, uz_0);
plot3(r1(:,1),r1(:,2),r1(:,3),'linewidth',1)
hold on
plot3(r2(:,1),r2(:,2),r2(:,3),'linewidth',2)
hold on
plot3(r3(:,1),r3(:,2),r3(:,3),'linewidth',3)
axis equal

function [r1, r2, r3, Uz] = CTRModel(q, uz_0)
    % Define length of tubes
    l = 1e-3 .* [431; 332; 174];
    % Define lengtoh of curved part of tubes
    l_k = 1e-3 .* [103; 113; 134];
    
    % Material and physical tube parameters
    % Young's modulus
    E = [6.4359738368e+10, 5.2548578304e+10, 4.7163091968e+10];
    % Polar moment of interia
    J = 1.0e-11 *[ 0.0120, 0.0653, 0.1686];
    % Second moment of interia
    I = 1.0e-12 *[ 0.0601, 0.3267, 0.8432 ];
    % Shear modulus
    G = [2.5091302912e+10, 2.1467424256e+10, 2.9788923392e+10]; 
    % Curvature vector x-component
    Ux = [21.3 13.108 3.5];
    % Curvature vector y-component
    Uy = [0 0 0];
    % Number of tubes
    n = 3;
    
    % q1 to q3 is extension and q6 is rotation of each tube from base
    % q_0 = [-0.2858; -0.2025; -0.0945; 0; 0; 0]; % Starting q values
    q_0 = [0; 0; 0.0; 0; 0; 0]; % Starting q values
    B = q(1:3) + q_0(1:3);
    % Starting angles
    alpha = (q(4:6) + q_0(4:6)) - B .* transpose(uz_0);
    alpha_1=alpha(1);

    
    % Segment tubes
    [L,d_tip,EE,UUx,UUy] = SegmentRobot(E,Ux,Uy,I, G, J, l,B,l_k);
    SS = L;
    for i=1:length(L)
        SS(i)=sum(L(1:i));
        %plot((B(1)+SS(i))*ones(1,10),1:10,'b', 'LineWidth',2)
        %hold on
    end
    % S is segmented abssica of tube after template
    S = SS(SS+min(B)>0)+min(B);
    E = zeros(n,length(S)); Ux=E; Uy=E;
    for i=1:n
       E(i,:) = EE(i,SS+min(B)>0); 
       Ux(i,:) = UUx(i,SS+min(B)>0);
       Uy(i,:) = UUy(i,SS+min(B)>0);
    end
    % each (i,j) element of above matrices correspond to the jth segment of
    % ith tube, 1st tube is the most inner
    
    span = [0 S]; % vector of x-coordinates of starting at zero. 
    % solved length, curvatures and twist angles
    Length = []; r = []; U_z = []; a = [];
    r0 = [0;0;0];
    R0 = [cos(alpha_1) sin(alpha_1) 0; -sin(alpha_1) cos(alpha_1) 0; 0 0 1];
    R0 = reshape(R0, [9,1]);
    
    for segment=1:length(S)
       s_span = [span(segment) span(segment+1) - realmin];
       y0_1 = [r0; R0];
       y0_2 = zeros(2*n, 1);
       y0_2(n+1:2*n) = alpha;
       y0_2(1:n) = uz_0;
       y_0 = [y0_2; y0_1];
       [s,y] = ode23(@(s,y) ode(s,y,Ux(:,segment),Uy(:,segment), ...
               E(:,segment).*transpose(I),G.*J,n), s_span, y_0);
       % first n elements of y are curvatures along z, e.g., y= [ u1_z  u2_z ... ]
       % last n elements of y are twist angles, alpha_i
       shape = [y(:,2*n+1),y(:,2*n+2),y(:,2*n+3)];
       Length = [Length; s];
       r = [r; shape];
       U_z = [U_z; y(:,1:n)];
       a = [a; y(:,n+1:2*n)];  % twist angle
       r0 = transpose(shape(end,:));
       R0 = transpose(y(end,2*n+4:2*n+12));
       uz_0 = transpose(U_z(end,:));
       alpha = transpose(a(end,:));
    end
    
    Uz = zeros(n,1);
    for i=1:n
       [~, index] = min(abs(Length - d_tip(i) + realmin));
       Uz(i) = U_z(index, i);
    end
    
    r1 = r;
    [~, tube2_end] = min(abs(Length-d_tip(2)));
    r2=[r(1:tube2_end,1),r(1:tube2_end,2),r(1:tube2_end,3)];
    [~, tube3_end] = min(abs(Length-d_tip(3)));
    r3=[r(1:tube3_end,1),r(1:tube3_end,2),r(1:tube3_end,3)];
end

function dyds = ode(~,y, Ux, Uy, EI, GJ, n)
    dyds = zeros(2*n+12, 1);
    % first n elements of y are curvatures along z, e.g., y= [ u1_z  u2_z ... ]
    % second n elements of y are twist angles, alpha_i
    % last 12 elements are r (position) and R (orientations), respectively
    
    % caclulate 1st tube curvature x and y direction
    ux = zeros(n,1); uy = zeros(n,1);
    
    % calculating tube's curvatures in x and y direction
    for i=1:n  
    ux(i)= (1/(EI(1)+EI(2)+EI(3)))* (...
          EI(1)*Ux(1)*cos(y(n+i)-y(n+1))+ EI(1)*Uy(1)*sin(y(n+i)-y(n+1))  + ...
          EI(2)*Ux(2)*cos(y(n+i)-y(n+2))+ EI(2)*Uy(2)*sin(y(n+i)-y(n+2))  + ...
          EI(3)*Ux(3)*cos(y(n+i)-y(n+3))+ EI(3)*Uy(3)*sin(y(n+i)-y(n+3)) );
    uy(i)= (1/(EI(1)+EI(2)+EI(3)))* (...
          -EI(1)*Ux(1)*sin(y(n+i)-y(n+1))+ EI(1)*Uy(1)*cos(y(n+i)-y(n+1))  + ...
          -EI(2)*Ux(2)*sin(y(n+i)-y(n+2))+ EI(2)*Uy(2)*cos(y(n+i)-y(n+2))  + ...
          -EI(3)*Ux(3)*sin(y(n+i)-y(n+3))+ EI(3)*Uy(3)*cos(y(n+i)-y(n+3)) );
    end
    
    % odes for twist
    GJ = transpose(GJ);
    GJ(EI==0) = 0;
    for i=1:n
       if EI(i)==0
           dyds(i) = 0; % ui_z
           dyds(n+i) = 0; % alpha_i
       else
           dyds(i)=  ((EI(i))/(GJ(i))) * (ux(i)* Uy(i) - uy(i)* Ux(i)); % ui_z
           dyds(n+i)=  y(i); % alpha_i
       end
    end
    
    e3 = [0;0;1];
    uz = y(1:n);
    % y(1) to y(3) are position of point materials
    % y(4) to y(12) are rotation matrix elements
    R1 = [y(2*n+4) y(2*n+5) y(2*n+6);
          y(2*n+7) y(2*n+8) y(2*n+9);
          y(2*n+10) y(2*n+11) y(2*n+12)];
      
    u_hat = [0 -uz(1) uy(1); uz(1) 0 -ux(1); -uy(1) ux(1) 0];
    % odes
    dr1 = R1*e3;
    dR1 = R1*u_hat;
    
    dyds(2*n+1)=dr1(1);dyds(2*n+2)=dr1(2);dyds(2*n+3)=dr1(3);
    dR=dR1'; 
    dR=dR(:);
    for i=4:12
       dyds(2*n+i)=dR(i-3);
    end
end

function [L,d1,E,Ux,Uy,I,G,J] = SegmentRobot(E,Ux,Uy,I,G,J,l,B,l_k)
% All vectors must be sorted, starting element belongs to the most inner tube
% E, U, I, G, J stifness, curvature, inertia, torsion constant, and 
% second moment of inertia vectors for each tube
% l vector of tube length
% B  vector of tube movements with respect to template position,
% i.e., s=0 (always negative)
% l_k vecot oftube's curved part length

k = length(l);

d1 = l + B; % position of tip of the tubes
d2 = d1 - l_k; % position where bending begins.
points = [0 transpose(B) transpose(d2) transpose(d1)];
[L, index] = sort(points);
L = 1e-5 * floor(1e5 * diff(L)); % length of each segment

EE = zeros(3, length(L));
II = EE; GG = EE; JJ = EE; UUx = EE; UUy = EE;

% Iterating through tubes (strange for loop here)
for i=1:k
    a=find(index==i+1);   % find where tube begins
    b=find(index==1*k+i+1); % find where tube curve starts
    c=find(index==2*k+i+1); % find where tube ends

    if L(a)==0; a=a+1;  end
    if L(b)==0; b=b+1;  end
    if c<=length(L)
        if L(c)==0; c=c+1; end
    end
    
    EE(i,a:c-1)=E(i);
    II(i,a:c-1)=I(i);
    GG(i,a:c-1)=G(i);
    JJ(i,a:c-1)=J(i);
    UUx(i,b:c-1)=Ux(i);
    UUy(i,b:c-1)=Uy(i);
end
    l=L(~(L==0));  % get rid of zero lengths
    E=zeros(k,length(l)); I=E; G=E; J=E; Ux=E; Uy=E;
     for i=1:k
        E(i,:)=EE(i,~(L==0)); I(i,:)=II(i,~(L==0)); G(i,:)=GG(i,~(L==0));
        J(i,:)=JJ(i,~(L==0)); Ux(i,:)=UUx(i,~(L==0)); Uy(i,:)=UUy(i,~(L==0));
     end
    L=L(~(L==0));
end