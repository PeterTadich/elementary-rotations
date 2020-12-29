// mcer = matrix computations elementary rotations

// ECMAScript module

// Elementary rotations. REF: Robotics Modelling, Planning and Control, Page 41

// Rj,i - rotation matrix of Frame i with respect to Frame j (page 46)
//  - p1 = R1,2 * p2

// R0,2 = R0,1 * R1,2 - composition of successive rotations (page 46)
//  - where R0,1 is the rotation matrix of frame O-x1y1z1 with respect to the fixed frame O-x0y0z0 (page 46)

import * as hlao from 'matrix-computations';
import * as mtojs from 'matlab-javascript';

function Rx_elementary(gamma){
    //var a = [[1], [0], [0]];
    //console.log(hlao.matrix_multiplication(Ry_elementary(Math.PI),a));
    //--> returns [[-1], [0], [0]]

    var r11 = 1.0; var r12 = 0.0; var r13 = 0.0;
    var r21 = 0.0;
    var r22 = Math.cos(gamma);
    var r23 = -1.0*Math.sin(gamma);
    var r31 = 0.0;
    var r32 = Math.sin(gamma);
    var r33 = Math.cos(gamma);
    
    var Rx = [
        [r11, r12, r13],
        [r21, r22, r23],
        [r31, r32, r33]
    ];
    
    return Rx;
}

function Ry_elementary(beta){
    //var a = [[1], [0], [0]];
    //console.log(hlao.matrix_multiplication(Ry_elementary(Math.PI),a));
    //--> returns [[-1], [0], [0]]

    var r11 = Math.cos(beta);
    var r12 = 0.0;
    var r13 = Math.sin(beta);
    var r21 = 0.0; var r22 = 1.0; var r23 = 0.0;
    var r31 = -1.0*Math.sin(beta);
    var r32 = 0.0;
    var r33 = Math.cos(beta);
    
    var Ry = [
        [r11, r12, r13],
        [r21, r22, r23],
        [r31, r32, r33]
    ];
    
    return Ry;
}

function Rz_elementary(alpha){
    //var a = [[1], [0], [0]];
    //console.log(hlao.matrix_multiplication(Rz_elementary(Math.PI),a));
    //--> returns [[-1], [0], [0]]

    var r11 = Math.cos(alpha);
    var r12 = -1.0*Math.sin(alpha);
    var r13 = 0.0;
    var r21 = Math.sin(alpha);
    var r22 = Math.cos(alpha);
    var r23 = 0.0;
    var r31 = 0.0; var r32 = 0.0; var r33 = 1.0;
    
    var Rz = [
        [r11, r12, r13],
        [r21, r22, r23],
        [r31, r32, r33]
    ];
    
    return Rz;
}

//roll, pitch, yaw to rotation matrix (radians)
//IMPORTANT: returns solution for sequential rotations about Z, Y, X axes (Paul book)
//REF: Robotics, Vision and Control. Page 30
/*
MATLAB
R = rpy2r(0.1,0.2,0.3,'zyx')
R =
    0.9752   -0.0370    0.2184
    0.0978    0.9564   -0.2751
   -0.1987    0.2896    0.9363
JavaScript
var R = rpy2r(0.1, 0.2, 0.3); //'ZYX'
print_rotation_matrix(R,4);
R = [
    [0.9752, -0.0370, 0.2184],
    [0.0978, 0.9564, -0.2751],
    [-0.1987, 0.2896, 0.9363]
]
JavaScript
var R = rpy2r(0.1, 0.2, 0.3); //'XYZ' (as to page 30)
print_rotation_matrix(R,4);
R = [
    [0.9363, -0.2896, 0.1987],
    [0.3130, 0.9447, -0.0978],
    [-0.1593, 0.1538, 0.9752]
]
*/
function rpy2r(roll, pitch, yaw){
    //IMPORTANT: rotations taken with respect to the current frame
    return(
            hlao.matrix_multiplication(
                hlao.matrix_multiplication(
                    Rz_elementary(roll),
                    Ry_elementary(pitch)
                ),
                Rx_elementary(yaw)
            )
        ); //'ZYX' (Paul 1981) (roll about 'z')
    //return matrix_multiplication(matrix_multiplication(Rx_elementary(roll),Ry_elementary(pitch)),Rz_elementary(yaw)); //'XYZ' (roll about 'x') equ.(2.14) ref: Robotics, Vision and Control. Page 30
}

/*
JavaScript
var R = rpy2r_RMPC(0.3, 0.2, 0.1); //'ZYX'
print_rotation_matrix(R,4);
[
    [0.9752, -0.0370, 0.2184],
    [0.0978, 0.9564, -0.2751],
    [-0.1987, 0.2896, 0.9363]
]
*/
function rpy2r_RMPC(psi,vartheta,varphi){
    //ref: Robotics Modelling, Planning and Control, Page 51
    //   - see function 'rpy2r_RMPC_check()'
    return(
        hlao.matrix_multiplication(
            hlao.matrix_multiplication(
                Rz_elementary(varphi),
                Ry_elementary(vartheta)
            ),
            Rx_elementary(psi)
        )
    ); //equ.(2.21) premultiplication
}

/*
JavaScript
var R = rpy2r_RMPC_check(0.3, 0.2, 0.1); //'ZYX'
print_rotation_matrix(R,4);
[
    [0.9752, -0.0370, 0.2184],
    [0.0978, 0.9564, -0.2751],
    [-0.1987, 0.2896, 0.9363]
]
*/
function rpy2r_RMPC_check(psi,vartheta,varphi){ //x,y,z
    //ref: Robotics Modelling, Planning and Control, Page 51
    //x (yaw)   - psi
    //y (pitch) - vartheta (ref: http://gplusnick.com/greek-letters-latex.html)
    //z (roll)  - varphi
    //IMPORTANT: rotation defined with respect to a 'fixed' frame attached to the centre of mass of the craft
    //   - order of rotation as as x - y - z (successive rotations are relative to fixed frame (yaw-pitch-roll relative to the fixed frames))
    //     first a yaw about x0 through an angle 'psi', then pitch about the y0 by an angle 'vartheta', and finally roll about the z0 by an angle 'varphi' (ref: Spong - Robot modeling and Control page 49)
    //     or roll-pitch-yaw, in that order, each taken with respect to the 'current' frame
    var c_psi = Math.cos(psi);
    var s_psi = Math.sin(psi);
    var c_vartheta = Math.cos(vartheta);
    var s_vartheta = Math.sin(vartheta);
    var c_varphi = Math.cos(varphi);
    var s_varphi = Math.sin(varphi);
    
    return [
        [c_varphi*c_vartheta, (c_varphi*s_vartheta*s_psi - s_varphi*c_psi), (c_varphi*s_vartheta*c_psi + s_varphi*s_psi)],
        [s_varphi*c_vartheta, (s_varphi*s_vartheta*s_psi + c_varphi*c_psi), (s_varphi*s_vartheta*c_psi - c_varphi*s_psi)],
        [    -1.0*s_vartheta,                             c_vartheta*s_psi,                             c_vartheta*c_psi]
    ];
}

//Calc. angle between two vectors.
//     ref: http://math.stackexchange.com/questions/293116/rotating-one-3d-vector-to-another
//MATLAB check.
/*
LPSISSP = [0,27.942,186.238]'
CPV366SP = [-118.0,0.0,17.0]'
R = vrrotvec2mat(vrrotvec(LPSISSP,CPV366SP))
%or
a = LPSISSP
b = CPV366SP
%function [ R ] = RotAtoB( a,b )
    x = [a(2)*b(3) - b(2)*a(3);a(3)*b(1) - b(3)*a(1);a(1)*b(2) - b(1)*a(2)];
    x = x/norm(x);
    theta = acos(a'*b/(norm(a)*norm(b)));
    A = [0    -x(3)  x(2)
         x(3)   0   -x(1)
        -x(2)  x(1)   0  ];
    R = eye(3) + sin(theta)*A + (1-cos(theta))*A^2;
%end
R*a
*/
/*
var R = [
    [0.1414,   -0.1650,   -0.9761],
    [0.1287,    0.9807,   -0.1471],
    [0.9815,   -0.1048,    0.1599]
];
*/
function Rfrom2vectors(a,b){
    //Step 1. Find axis.
    var x = hlao.matrix_multiplication_scalar(
                hlao.vector_cross(a,b),
                1.0/(hlao.matrix_norms(hlao.vector_cross(a,b)))
            );
    //console.log('x: ' + x);
    //Step 2. Find angle.
    var theta = Math.acos(hlao.vector_dot(a,b)/(hlao.matrix_norms(a) * hlao.matrix_norms(b)));
    //console.log('theta: ' + theta);
    //Step 3. Find rotation.
    var A = [
        [         0.0, -1.0*x[2][0],      x[1][0]],
        [     x[2][0],          0.0, -1.0*x[0][0]],
        [-1.0*x[1][0],      x[0][0],          0.0]
    ];
    var E = hlao.identity_matrix(3);
    //   - Rodrigues' rotation formula.
    var R = hlao.matrix_arithmetic(
                E,
                hlao.matrix_arithmetic(
                    hlao.matrix_multiplication_scalar(
                        A,
                        Math.sin(theta)
                    ),
                    hlao.matrix_multiplication_scalar(
                        hlao.matrix_multiplication(A,A),
                        (1.0 - Math.cos(theta))
                    ),
                    '+'
                ),
                '+'
            );
    //console.log('R: ' + R);
    return R;
}

//requires:
//   z.js
//   matrixAlgebra.js
//   elementaryRotations.js
/*
var ORO = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ];
var OR1 = Rz_elementary(Math.PI/64.0);
console.log(Math.PI/64.0);
*/
/*
var w = rot2omega(OR1);
console.log(w[2][0]);
var w_check = infinitesimalRotation(ORO,OR1); //ref: "../DenavitHartenberg/DenavitHartenberg.js"
console.log(w_check[2][0]);
*/
//var rpy = [[(Math.PI/2.0)*Math.random()],[(Math.PI/2.0)*Math.random()],[(Math.PI/2.0)*Math.random()]];
//ref: Introduction to Humanoid Robotics (page 36, 67)
//see also rot2omega.js
function rot2omega(R){
    var el = [
        [R[2][1] - R[1][2]],
        [R[0][2] - R[2][0]],
        [R[1][0] - R[0][1]]
    ];
    
    var norm_el = mtojs.matrix_norms(el,'2');
    
    if(norm_el > 0.001){
        var w = 
            hlao.vector_multiplication_scalar(
                el,
                Math.atan2(norm_el, mtojs.trace(R)-1)/norm_el
            );
    } else if((R[0][0] > 0.0)&&(R[1][1] > 0.0)&&(R[2][2] > 0.0)) {
        var w = [[0.0],[0.0],[0.0]];
    } else {
        var w = 
            hlao.vector_multiplication_scalar(
                [[R[0][0]+1],[R[1][1]+1],[R[2][2]+1]],
                Math.PI/2.0
            );
    }
    
    return w;
}

/*
For small dt can also use --> 'skew(w)*dt':
matrix_multiplication_scalar(S(w),dt)
ref: eq 3.7 Robotics Vision and Control
*/
//ref: Intro to Humanoid Robotics page 35 (eq 2.37)
function Rodrigues(w,dt){
    var n = w.length;
    var eps = 2.2204e-16;
    var mag = Math.sqrt(w[0][0]*w[0][0] + w[1][0]*w[1][0] + w[2][0]*w[2][0]);
    if(mag < eps) var R = hlao.identity_matrix(n);
    else {
        var wn = hlao.matrix_multiplication_scalar(w,1.0/mag); //angular velocity unit vector
        var th = mag*dt; //amount of rotation (radians)
        var w_wedge = [ //skew
            [          0.0, -1.0*wn[2][0],      wn[1][0]],
            [     wn[2][0],           0.0, -1.0*wn[0][0]],
            [-1.0*wn[1][0],      wn[0][0],           0.0]
        ];
        var R = hlao.matrix_arithmetic(
                    hlao.identity_matrix(n),
                    hlao.matrix_arithmetic(
                        hlao.matrix_multiplication_scalar(w_wedge,Math.sin(th)),
                        hlao.matrix_multiplication_scalar(hlao.matrix_multiplication(w_wedge,w_wedge),(1.0 - Math.cos(th))),
                        '+'
                    ),
                    '+'
                );
    }
    return(R);
}

function print_rotation_matrix(R,x){
    //print the rotation matrix
    //   - 'R' the rotation matrix
    //   - 'x' the number of digits after the decimal point to print
    var Rstr = "";
    //var dim = size(R);
    //var m = dim[0]; //number of rows
    //var n = dim[1]; //number of columns
    var m; var n;
    [m,n] = [R.length,R[0].length];
    Rstr = Rstr + '[\n';
    for(var i=0;i<m;i=i+1){ //row
        Rstr = Rstr + '    [';
        for(var j=0;j<n;j=j+1){ //col
            if(j < (n-1)) Rstr = Rstr + R[i][j].toFixed(x) + ', ';
            else Rstr = Rstr + R[i][j].toFixed(x);
        }
        if(i < (m-1)) Rstr = Rstr + '],\n';
        else Rstr = Rstr + ']\n';
    }
    Rstr = Rstr + ']\n';
    console.log(Rstr);
}

export {
    Rx_elementary,
    Ry_elementary,
    Rz_elementary,
    rpy2r,
    rpy2r_RMPC,
    Rfrom2vectors,
    rot2omega,
    Rodrigues,
    print_rotation_matrix
};