#include "mex.hpp"
#include "mexAdapter.hpp"
#include "MatlabDataArray.hpp"
#include <vector>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <iomanip>

#define pi 3.1415926535

using namespace matlab::data;
using matlab::mex::ArgumentList;

class MexFunction : public matlab::mex::Function {
public:
    ArrayFactory f;
    TypedArray<double> ALPHA = f.createArray<double>( {8,1} );
    TypedArray<double> A     = f.createArray<double>( {8,1} );
    TypedArray<double> D     = f.createArray<double>( {8,1} );

    void operator()( ArgumentList outputs, ArgumentList inputs ) {
        checkArguments( outputs, inputs );

        TypedArray<double> pe = std::move( inputs[0] );
        TypedArray<double> Re = std::move( inputs[1] );
        ALPHA = std::move( inputs[2] );
        A     = std::move( inputs[3] );
        D     = std::move( inputs[4] );

        bool gripper = false;
        // If in the D-H table is given also the transformation T6_ee, it has to be considered by kinematics
        if ( D.getNumberOfElements() > 7 )
            gripper = true;

        TypedArray<double> Te = f.createArray<double>( {4,4}, {Re[0][0], Re[1][0], Re[2][0], 0,
                                                               Re[0][1], Re[1][1], Re[2][1], 0,
                                                               Re[0][2], Re[1][2], Re[2][2], 0,
                                                                  pe[0],    pe[1],    pe[2], 1} );
        TypedArray<double> z_6 = f.createArray<double>( {3,1}, {Re[0][2], Re[1][2], Re[2][2]} );
        
        // THETA 1 ========================================================================================
        TypedArray<double> p05 = f.createArray<double>( {4,1} );
        if ( gripper )
            p05 = matrixMult( Te, f.createArray<double>( {4,1}, {0, 0, -D[6]-D[7], 1} ) );
        else
            p05 = matrixMult( Te, f.createArray<double>( {4,1}, {0, 0, -D[6], 1} ) );

        double th1_1 = atan2( p05[1], p05[0] );
        double th1_2 = th1_1 - pi;
        // std::cout << "th1_1: " << th1_1 << ", th1_2: " << th1_2 << std::endl;

        // THETA 3 ========================================================================================
        double phi  = atan2( D[4], A[3] );
        double d4_p =  sqrt( D[4]*D[4] + A[3]*A[3] );
        
        // theta3 1&2 with theta1_1
        TypedArray<double> T01_1 = matrixMult( rot_traslZ( th1_1, D[1] ), rot_traslX( ALPHA[0], A[1] ) );
        TypedArray<double> p15_1 = matrixMult( homogeneousMatrixInverse( T01_1), f.createArray<double>( {4,1}, {p05[0], p05[1], p05[2], 1} ) );
        
        double F_2_1       = p15_1[0]*p15_1[0] + p15_1[2]*p15_1[2];
        double cos_gamma_1 = (F_2_1 - A[2]*A[2] - d4_p*d4_p) / (-2*A[2]*d4_p);
        // check if the position is reachable: |cos(gamma)| <= 1. If not, the configuration with this theta1 is not feasible
        bool impossible_conf_1 = abs(cos_gamma_1) > 1;
        double gamma_1, th3_1, th3_2;
        if ( impossible_conf_1 )
        {
            std::cout << "Impossible configuration given. Exiting." << std::endl;
            exit(1);
        }
        else
        {
            gamma_1 = atan2( sqrt(1-cos_gamma_1*cos_gamma_1), cos_gamma_1 );
            th3_1   =  gamma_1 + phi - pi;
            th3_2   = -gamma_1 + phi - pi;
        }

        // theta3 3&4 with theta1_2
        TypedArray<double> T01_2 = matrixMult( rot_traslZ( th1_2, D[1] ), rot_traslX( ALPHA[0], A[1] ) );
        TypedArray<double> p15_2 = matrixMult( homogeneousMatrixInverse( T01_2), f.createArray<double>( {4,1}, {p05[0], p05[1], p05[2], 1} ) );

        double F_2_2       = p15_2[0]*p15_2[0] + p15_2[2]*p15_2[2];
        double cos_gamma_2 = (F_2_2 - A[2]*A[2] - d4_p*d4_p) / (-2*A[2]*d4_p);
        // check if the position is reachable: |cos(gamma)| <= 1. If not, the configuration with this theta1 is not feasible
        bool impossible_conf_2 = abs(cos_gamma_2) > 1;
        double gamma_2, th3_3, th3_4;
        if ( impossible_conf_2 )
        {
            std::cout << "Impossible configuration given. Exiting." << std::endl;
            exit(1);
        }
        else
        {
            gamma_2 = atan2( sqrt(1-cos_gamma_2*cos_gamma_2), cos_gamma_2 );
            th3_3   =  gamma_2 + phi - pi;
            th3_4   = -gamma_2 + phi - pi;
        }
        

        // THETA 2 ========================================================================================
    
        // theta2 1&2 with theta1_1
        double beta_1      = atan2( p15_1[2], p15_1[0] );
        double cos_alpha_1 = (F_2_1+A[2]*A[2]-d4_p*d4_p) / (2*sqrt( F_2_1 )*A[2]);
        double alpha_1     = atan2( sqrt(1-cos_alpha_1*cos_alpha_1), cos_alpha_1 );

        double th2_1 =  alpha_1 + beta_1;
        double th2_2 = -alpha_1 + beta_1;

        // theta2 3&4 with theta1_2
        double beta_2      = atan2( p15_2[2], p15_2[0] );
        double cos_alpha_2 = (F_2_2 + A[2]*A[2] - d4_p*d4_p) / (2*sqrt( F_2_2 )*A[2]);
        double alpha_2     = atan2( sqrt( 1-cos_alpha_2*cos_alpha_2 ), cos_alpha_2 );
        
        double th2_3 =  alpha_2 + beta_2;
        double th2_4 = -alpha_2 + beta_2;
    

        // THETA 5 ========================================================================================
        TypedArray<double> T03_1 = matrixMult( matrixMult( transf_i_1_i( 1, th1_1 ), transf_i_1_i( 2, th2_1 ) ), transf_i_1_i( 3, th3_1 ) );
        TypedArray<double> T03_2 = matrixMult( matrixMult( transf_i_1_i( 1, th1_1 ), transf_i_1_i( 2, th2_2 ) ), transf_i_1_i( 3, th3_2 ) );
        
        TypedArray<double> y_3m_1 = f.createArray<double>( {3,1}, {-T03_1[0][1], -T03_1[1][1], -T03_1[2][1]} );
        TypedArray<double> y_3m_2 = f.createArray<double>( {3,1}, {-T03_2[0][1], -T03_2[1][1], -T03_2[2][1]} );
    
        double cos_th5_1 = dot( y_3m_1, z_6 );
        double cos_th5_2 = dot( y_3m_2, z_6 );
    
        double th5_1 = atan2(  sqrt(1-cos_th5_1*cos_th5_1), cos_th5_1 );
        double th5_2 = -th5_1;
        double th5_3 = atan2(  sqrt(1-cos_th5_2*cos_th5_2), cos_th5_2 );
        double th5_4 = -th5_3;
    
        TypedArray<double> T03_3 = matrixMult( matrixMult( transf_i_1_i( 1, th1_2 ), transf_i_1_i( 2, th2_3 ) ), transf_i_1_i( 3, th3_3 ) );
        TypedArray<double> T03_4 = matrixMult( matrixMult( transf_i_1_i( 1, th1_2 ), transf_i_1_i( 2, th2_4 ) ), transf_i_1_i( 3, th3_4 ) );
    
        TypedArray<double> y_3m_3 = f.createArray<double>( {3,1}, {-T03_3[0][1], -T03_3[1][1], -T03_3[2][1]} );
        TypedArray<double> y_3m_4 = f.createArray<double>( {3,1}, {-T03_4[0][1], -T03_4[1][1], -T03_4[2][1]} );
    
        double cos_th5_3 = dot( y_3m_3, z_6 );
        double cos_th5_4 = dot( y_3m_4, z_6 );
        
        double th5_5 = atan2(  sqrt(1-cos_th5_3*cos_th5_3), cos_th5_3 );
        double th5_6 = -th5_5;
        double th5_7 = atan2(  sqrt(1-cos_th5_4*cos_th5_4), cos_th5_4 );
        double th5_8 = -th5_7;
    

        // THETA 4&6 ======================================================================================
        TypedArray<double> T36_1 = matrixMult( homogeneousMatrixInverse( T03_1 ), Te );
        TypedArray<double> T36_2 = matrixMult( homogeneousMatrixInverse( T03_2 ), Te );

        double th6_1, th6_2, th6_3, th6_4;
        double th4_1, th4_2, th4_3, th4_4;
        if ( !(-0.000001 < th5_1 && th5_1 < 0.000001 || -0.000001 < th5_2 && th5_2 < 0.000001) )
        {
            th4_1 = atan2( T36_1[2][2]/sin(th5_1),  T36_1[0][2]/sin(th5_1) );
            th4_2 = atan2( T36_1[2][2]/sin(th5_2),  T36_1[0][2]/sin(th5_2) );
            th6_1 = atan2( T36_1[1][1]/sin(th5_1), -T36_1[1][0]/sin(th5_1) ) - pi;
            th6_2 = atan2( T36_1[1][1]/sin(th5_2), -T36_1[1][0]/sin(th5_2) ) - pi;
        }
        else
        {
            std::cout << "Singular configuration on theta 5. Exiting." << std::endl;
            exit(1);
        }

        if ( !(-0.000001 < th5_3 && th5_3 < 0.000001 || -0.000001 < th5_4 && th5_4 < 0.000001) )
        {
            th4_3 = atan2( T36_2[2][2]/sin(th5_3),  T36_2[0][2]/sin(th5_3) );
            th4_4 = atan2( T36_2[2][2]/sin(th5_4),  T36_2[0][2]/sin(th5_4) );
            th6_3 = atan2( T36_2[1][1]/sin(th5_3), -T36_2[1][0]/sin(th5_3) ) - pi;
            th6_4 = atan2( T36_2[1][1]/sin(th5_4), -T36_2[1][0]/sin(th5_4) ) - pi;
        }
        else
        {
            std::cout << "Singular configuration on theta 5. Exiting." << std::endl;
            exit(1);
        }
        

        TypedArray<double> T36_3 = matrixMult( homogeneousMatrixInverse( T03_3 ), Te );
        TypedArray<double> T36_4 = matrixMult( homogeneousMatrixInverse( T03_4 ), Te );
        
        double th6_5, th6_6, th6_7, th6_8;
        double th4_5, th4_6, th4_7, th4_8;
        if ( !(-0.000001 < th5_5 && th5_5 < 0.000001 || -0.000001 < th5_6 && th5_6 < 0.000001) )
        {
            th4_5 = atan2( T36_3[2][2]/sin(th5_5),  T36_3[0][2]/sin(th5_5) );
            th4_6 = atan2( T36_3[2][2]/sin(th5_6),  T36_3[0][2]/sin(th5_6) );
            th6_5 = atan2( T36_3[1][1]/sin(th5_5), -T36_3[1][0]/sin(th5_5) ) - pi;
            th6_6 = atan2( T36_3[1][1]/sin(th5_6), -T36_3[1][0]/sin(th5_6) ) - pi;
        }
        else
        {
            std::cout << "Singular configuration on theta 5. Exiting." << std::endl;
            exit(1);
        }

        if ( !(-0.000001 < th5_7 && th5_7 < 0.000001 || -0.000001 < th5_8 && th5_8 < 0.000001) )
        {
            th4_7 = atan2( T36_4[2][2]/sin(th5_7),  T36_4[0][2]/sin(th5_7) );
            th4_8 = atan2( T36_4[2][2]/sin(th5_8),  T36_4[0][2]/sin(th5_8) );
            th6_7 = atan2( T36_4[1][1]/sin(th5_7), -T36_4[1][0]/sin(th5_7) ) - pi;
            th6_8 = atan2( T36_4[1][1]/sin(th5_8), -T36_4[1][0]/sin(th5_8) ) - pi;
        }
        else
        {
            std::cout << "Singular configuration on theta 5. Exiting." << std::endl;
            exit(1);
        }
        
        
        TypedArray<double> TH    = f.createArray<double>( {8,6}, { th1_1, th1_1, th1_1, th1_1, th1_2, th1_2, th1_2, th1_2,
                                                                   th2_1, th2_1, th2_2, th2_2, th2_3, th2_3, th2_4, th2_4,
                                                                   th3_1, th3_1, th3_2, th3_2, th3_3, th3_3, th3_4, th3_4,
                                                                   th4_1, th4_2, th4_3, th4_4, th4_5, th4_6, th4_7, th4_8,
                                                                   th5_1, th5_2, th5_3, th5_4, th5_5, th5_6, th5_7, th5_8,
                                                                   th6_1, th6_2, th6_3, th6_4, th6_5, th6_6, th6_7, th6_8
                                                                   } );
        outputs[0] = TH;
    }

    double dot( TypedArray<double> A, TypedArray<double> B )
    {
        double tot = 0;
        for ( int i=0; i<A.getNumberOfElements(); ++i )
            tot += A[i] * B[i];
        
        return tot;
    }

    TypedArray<double> rot_traslZ( double th, double offset )
    {
        return f.createArray<double>( {4,4}, {  cos(th), sin(th),      0, 0, 
                                               -sin(th), cos(th),      0, 0, 
                                                      0,       0,      1, 0, 
                                                      0,       0, offset, 1 } );
    }

    TypedArray<double> rot_traslX( double th, double offset )
    {
        return f.createArray<double>( {4,4}, {     1,        0,       0, 0, 
                                                   0,  cos(th), sin(th), 0, 
                                                   0, -sin(th), cos(th), 0, 
                                              offset,        0,       0, 1 } );
    }
    
    TypedArray<double> transf_i_1_i( int i, double th )
    {
        return f.createArray<double>( {4,4}, {  cos(th), sin(th)*cos(ALPHA[i-1]), sin(th)*sin(ALPHA[i-1]), 0, 
                                               -sin(th), cos(th)*cos(ALPHA[i-1]), cos(th)*sin(ALPHA[i-1]), 0, 
                                                      0,        -sin(ALPHA[i-1]),         cos(ALPHA[i-1]), 0, 
                                                 A[i-1],   -D[i]*sin(ALPHA[i-1]),    D[i]*cos(ALPHA[i-1]), 1 } );
    }

    TypedArray<double> matrixMult( TypedArray<double> A, TypedArray<double> B )
    {
        ArrayDimensions d1 = A.getDimensions();
        ArrayDimensions d2 = B.getDimensions();
        ArrayDimensions d  = {d1[0],d2[1]};

        TypedArray<double> res = f.createArray<double>( d );

        for ( int i=0; i<d[0]; ++i )
        {
            for ( int j=0; j<d[1]; ++j )
            {
                for ( int n=0; n<d[0]; ++n )
                {
                    res[i][j] += A[i][n] * B[n][j];
                }
            }
        }

        return res;
    }

    TypedArray<double> matrixTranspose( TypedArray<double> A )
    {
        ArrayDimensions d          = A.getDimensions();
        TypedArray<double> A_trans = f.createArray<double>( d );
        
        for ( int i=0; i<d[0]; ++i )
        {
            for ( int j=0; j<d[1]; ++j )
            {
                A_trans[i][j] = A[j][i];
            }
        }

        return A_trans;
    }

    TypedArray<double> homogeneousMatrixInverse( TypedArray<double> T )
    {
        TypedArray<double> R   = f.createArray<double>( {3,3}, {T[0][0], T[1][0], T[2][0],
                                                                T[0][1], T[1][1], T[2][1],
                                                                T[0][2], T[1][2], T[2][2]} );
        TypedArray<double> R_t = matrixTranspose( R );

        TypedArray<double> p   = f.createArray<double>( {3,1}, {-T[0][3], -T[1][3], -T[2][3]} );

        TypedArray<double> R_t_p = matrixMult( R_t, p );

        TypedArray<double> T_inv    = f.createArray<double>( {4,4}, {R_t[0][0], R_t[1][0], R_t[2][0], 0,
                                                                     R_t[0][1], R_t[1][1], R_t[2][1], 0,
                                                                     R_t[0][2], R_t[1][2], R_t[2][2], 0,
                                                                      R_t_p[0],  R_t_p[1],  R_t_p[2], 1} );
 
        return T_inv;
    }

    void printMatrix( TypedArray<double> A )
    {
        ArrayDimensions d = A.getDimensions();
        // fix the number of decimal digits
        std::cout << std::fixed << std::setprecision( 4 );  // to 2
        std::cout << std::endl;
        for ( int i=0; i<d[0]; ++i )
        {
            for ( int j=0; j<d[1]; ++j )
            {
                std::cout << std::setw(8) << A[i][j] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    void checkArguments( ArgumentList outputs, ArgumentList inputs ) {
        // Get pointer to engine
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();

        // Check first argument 'pe': input must be double array
        if ( inputs[0].getType() != ArrayType::DOUBLE ||
             inputs[0].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "First input 'pe' must be double array" ) } ) );
        }

        // Check second argument 'Re': input must be double array
        if ( inputs[1].getType() != ArrayType::DOUBLE ||
             inputs[1].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Second input 'Re' must be double array" ) } ) );
        }

        // Check third argument 'ALPHA': input must be double array
        if ( inputs[2].getType() != ArrayType::DOUBLE ||
             inputs[2].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Third input 'ALPHA' must be double array" ) } ) );
        }

        // Check fourth argument 'A': input must be double array
        if ( inputs[3].getType() != ArrayType::DOUBLE ||
             inputs[3].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Fourth input 'A' must be double array" ) } ) );
        }

        // Check fifth argument 'D': input must be double array
        if ( inputs[4].getType() != ArrayType::DOUBLE ||
             inputs[4].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Fifth input 'D' must be double array" ) } ) );
        }
        
        // Check number of outputs: output must be 1 element -> DK homogeneous matrix
        if ( outputs.size() > 1 ) {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar("Only one output is returned") } ) );
        }
    }
};
