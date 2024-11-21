clear
clc

fk3001_JAKUB([0; 0; 0; 0])

%% LAB 2 code used for Jacobian finding in RADIANS
function T = dh2mat(q)
    T = [cos(q(1)) -sin(q(1))*cos(q(4)) sin(q(1))*sin(q(4)) q(3)*cos(q(1));
        sin(q(1)) cos(q(1))*cos(q(4)) -cos(q(1))*sin(q(4)) q(3)*sin(q(1));
        0 sin(q(4)) cos(q(4)) q(2);
        0 0 0 1];
end

% Takes in an nx4 matrix representing a DH table and the
% transformation matrix from frame 0 to frame 1
% Returns the composite symbolic transformation matrix from frame 0
% to the end effector frame
function T = dh2fk(q, t01)
    theta = sym('theta%d', [1 size(q, 1)]);
    d = sym('d%d', [1 size(q, 1)]);
    a = sym('a%d', [1 size(q, 1)]);
    alph = sym('alph%d', [1 size(q, 1)]);
    
    T = t01;
    
    % Iterate through rows of DH and multiply T by the next
    % transformation
    for i = 1:size(q, 1)
        T = T * dh2mat([theta(i) d(i) a(i) alph(i)]);
    end
end

% Takes an nx1 vector representing the n joint angles of the arm (4)
% Returns a 4x4 transformation matrix representing the position and
% orientation of the tip frame with respect to the base frame
function T = fk3001_JAKUB(q)
    % Get link lengths in mm
    L0 = 36.076;
    L1 = 96.326 - L0;
    L2 = sqrt(128^2 + 24^2);
    L3 = 124;
    L4 = 133.4;     % Get the DH parameter table for the arm

    % theta, d, a, alpha
    DH = [q(1) L1 0 -pi/2;      % used to be alpha=-90
        q(2)-acos(24/L2) 0 L2 0;
        q(3)+acos(24/L2) 0 L3 0;
        q(4) 0 L4 0];

    % Get T01
    T01 = dh2mat([0 L0 0 0]);

    % Use dh2fk() to get the symbolic transformation matrix
    sym_tm = dh2fk(DH, T01); % symbolic matrix 0 to EE

    % Turn that into a numerical transformation matrix
    % Recreate the symbols we'll be substituting in
    theta = sym('theta%d', [1 size(q, 1)]);
    d = sym('d%d', [1 size(q, 1)]);
    a = sym('a%d', [1 size(q, 1)]);
    alph = sym('alph%d', [1 size(q, 1)]);
    
    % Substitute in for symbolic d, a, and alpha
    function finished = subsitute(symbolic)        
        finished = subs(symbolic, d(1), DH(1, 2));
        
        % Iterate through d's
        for i = 2:size(q, 1)
            finished = subs(finished, d(i), DH(i, 2));
        end
        
        % Iterate through a's
        for i = 1:size(q, 1)
            finished = subs(finished, a(i), DH(i, 3));
        end
        
        % Iterate through alpha's
        for i = 1:size(q, 1)
            finished = subs(finished, alph(i), DH(i, 4));
        end
    end

    T = subsitute(sym_tm); % T0EE with only thetas

    % Get x, y, and z values of T0EE
    EE_X_sym = T(1, 4);
    EE_Y_sym = T(2, 4);
    EE_Z_sym = T(3, 4);

    X_partial = jacobian(EE_X_sym, [theta(1), theta(2), theta(3), theta(4)]);
    Y_partial = jacobian(EE_Y_sym, [theta(1), theta(2), theta(3), theta(4)]);
    Z_partial = jacobian(EE_Z_sym, [theta(1), theta(2), theta(3), theta(4)]);

    Jp = [X_partial;
          Y_partial;
          Z_partial];
    
    % calculate the J0 of the jacobian
    T1 = T01;
    T1_temp =  T1(1:3, 3); %Zi for T01
    T2 = T1*dh2mat([theta(1) d(1) a(1) alph(1)]);
    T2_temp = T2(1:3, 3);   %Zi for T02
    T3 = T2*dh2mat([theta(2) d(2) a(2) alph(2)]);
    T3_temp = T3(1:3, 3);   %Zi for T03
    T4 = T3*dh2mat([theta(3) d(3) a(3) alph(3)]);
    T4_temp = T4(1:3, 3);   %Zi for T04

    % use the subsitute function to numerically substitute all variables but thetas
    T1_subs = subsitute(T1_temp);
    T2_subs = subsitute(T2_temp);
    T3_subs = subsitute(T3_temp);
    T4_subs = subsitute(T4_temp);

    Jo = [T1_subs(1), T2_subs(1), T3_subs(1), T4_subs(1);
          T1_subs(2), T2_subs(2), T3_subs(2), T4_subs(2);
          T1_subs(3), T2_subs(3), T3_subs(3), T4_subs(3)];

    % WHEN USING THESE DERIVATIONS PASS IN THE THETAS IN RADIANS AND ADD acos(24/L2) to q(2) and subtract it from q(3)
    % construct the 6x4 Jacobian
    Jacobian = [Jp; Jo];
                
    T = Jacobian;
end