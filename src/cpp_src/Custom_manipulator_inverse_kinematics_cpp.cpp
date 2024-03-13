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

    void operator()( ArgumentList outputs, ArgumentList inputs ) {
        checkArguments( outputs, inputs );

        TypedArray<double> pe    = std::move( inputs[0] );
        TypedArray<double> Re    = std::move( inputs[1] );
        TypedArray<double> ALPHA = std::move( inputs[2] );
        TypedArray<double> A     = std::move( inputs[3] );
        TypedArray<double> D     = std::move( inputs[4] );
        TypedArray<double> TH    = std::move( inputs[5] );
        
        TypedArray<double> Te    = f.createArray<double>( {4,4}, {Re[0][0], Re[1][0], Re[2][0], 0,
                                                                  Re[0][1], Re[1][1], Re[2][1], 0,
                                                                  Re[0][2], Re[1][2], Re[2][2], 0,
                                                                     pe[0],    pe[1],    pe[2], 1} );
        
        // THETA 1 ========================================================================================
        TypedArray<double> p05 = matrixMult( Te, f.createArray<double>( {4,1}, {0, 0, -D[6]-D[7], 1} ) );
        
        double th1_1 = atan2( p05[1], p05[0] );
        double th1_2 = th1_1 + pi;
        // std::cout << "th1_1: " << th1_1 << ", th1_2: " << th1_2 << std::endl;


        // THETA 2 ========================================================================================
        double d_34_2    = D[4]*D[4] + A[3]*A[3];
        double d_34      = sqrt( d_34_2 );
        
        TypedArray<double> T01   = transf_i_1_i( 1, th1_1, ALPHA, A, D, TH );
        TypedArray<double> p15_1 = matrixMult( homogeneousMatrixInverse( T01 ), p05 );
        double l_p15             = sqrt( p15_1[2]*p15_1[2] + p15_1[0]*p15_1[0] );
        double cos_gamma         = (d_34_2 + A[2]*A[2] - l_p15*l_p15) / (2*d_34*A[2]);
        double gamma_1           = atan2( sqrt(1-cos_gamma*cos_gamma), cos_gamma );
        double sin_alpha         = d_34*sin( gamma_1 ) / l_p15;
        double alpha             = atan2( sin_alpha, sqrt(1-sin_alpha*sin_alpha) );
        double phi               = atan2( p15_1[2], p15_1[0] );

        double th2_1             = phi - alpha;
        double th2_2             = phi + alpha;

        T01                      = transf_i_1_i( 1, th1_2, ALPHA, A, D, TH );
        TypedArray<double> p15_2 = matrixMult( homogeneousMatrixInverse( T01 ), p05 );
        l_p15                    = sqrt( p15_1[2]*p15_1[2] + p15_1[0]*p15_1[0] );
        cos_gamma                = (d_34_2 + A[2]*A[2] - l_p15*l_p15) / (2*d_34*A[2]);
        double gamma_2           = atan2( sqrt(1-cos_gamma*cos_gamma), cos_gamma );
        sin_alpha                = d_34*sin( gamma_2 ) / l_p15;
        alpha                    = atan2( sin_alpha, sqrt(1-sin_alpha*sin_alpha) );
        phi                      = atan2( p15_2[2], p15_2[0] );

        double th2_3             = phi - alpha;
        double th2_4             = phi + alpha;
        // std::cout << "th2_1: " << th2_1 << ", th2_2: " << th2_2 << ", th2_3: " << th2_3 << ", th2_4: " << th2_4 << std::endl;


        // THETA 3 ========================================================================================
        double csi      = atan2( D[4] / d_34, A[3] / d_34 );
        double beta_1   = pi - gamma_1;
        double beta_2   = pi - gamma_2;
        
        double th3_1    =  beta_1 + csi - pi/2;
        double th3_2    = -beta_1 + csi - pi/2;
        double th3_3    =  beta_2 + csi - pi/2;
        double th3_4    = -beta_2 + csi - pi/2;
        // std::cout << "th3_1: " << th3_1 << ", th3_2: " << th3_2 << ", th3_3: " << th3_3 << ", th3_4: " << th3_4 << std::endl;


        // THETA 5 ========================================================================================
        TypedArray<double> T03     = matrixMult( matrixMult( transf_i_1_i( 1, th1_1, ALPHA, A, D, TH ), transf_i_1_i( 2, th2_1, ALPHA, A, D, TH ) ), transf_i_1_i( 3, th3_1, ALPHA, A, D, TH ) );
        TypedArray<double> T36_11  = matrixMult( homogeneousMatrixInverse( T03 ), Te );
        double cos_th5 = -T36_11[1][2];
        double th5_1   = atan2(  sqrt(1-cos_th5*cos_th5), cos_th5 );
        double th5_2   = atan2( -sqrt(1-cos_th5*cos_th5), cos_th5 );

        T03                        = matrixMult( matrixMult( transf_i_1_i( 1, th1_1, ALPHA, A, D, TH ), transf_i_1_i( 2, th2_2, ALPHA, A, D, TH ) ), transf_i_1_i( 3, th3_2, ALPHA, A, D, TH ) );
        TypedArray<double> T36_12  = matrixMult( homogeneousMatrixInverse( T03 ), Te );
        cos_th5        = -T36_12[1][2];
        double th5_3   = atan2(  sqrt(1-cos_th5*cos_th5), cos_th5 );
        double th5_4   = atan2( -sqrt(1-cos_th5*cos_th5), cos_th5 );
        
        T03                        = matrixMult( matrixMult( transf_i_1_i( 1, th1_2, ALPHA, A, D, TH ), transf_i_1_i( 2, th2_3, ALPHA, A, D, TH ) ), transf_i_1_i( 3, th3_3, ALPHA, A, D, TH ) );
        TypedArray<double> T36_21  = matrixMult( homogeneousMatrixInverse( T03 ), Te );
        cos_th5        = -T36_21[1][2];
        double th5_5   = atan2(  sqrt(1-cos_th5*cos_th5), cos_th5 );
        double th5_6   = atan2( -sqrt(1-cos_th5*cos_th5), cos_th5 );
        
        T03                        = matrixMult( matrixMult( transf_i_1_i( 1, th1_2, ALPHA, A, D, TH ), transf_i_1_i( 2, th2_4, ALPHA, A, D, TH ) ), transf_i_1_i( 3, th3_4, ALPHA, A, D, TH ) );
        TypedArray<double> T36_22  = matrixMult( homogeneousMatrixInverse( T03 ), Te );
        cos_th5        = -T36_22[1][2];
        double th5_7   = atan2(  sqrt(1-cos_th5*cos_th5), cos_th5 );
        double th5_8   = atan2( -sqrt(1-cos_th5*cos_th5), cos_th5 );
        // std::cout << "th5_1: " << th5_1 << ", th5_2: " << th5_2 << ", th5_3: " << th5_3 << ", th5_4: " << th5_4 << std::endl;
        
        // THETA 4&6 ======================================================================================
        double th6_1 = 0.0, th6_2 = 0.0, th6_3 = 0.0, th6_4 = 0.0, th6_5 = 0.0, th6_6 = 0.0, th6_7 = 0.0, th6_8 = 0.0;
        double th4_1 = 0.0, th4_2 = 0.0, th4_3 = 0.0, th4_4 = 0.0, th4_5 = 0.0, th4_6 = 0.0, th4_7 = 0.0, th4_8 = 0.0;
        
        if ( abs(sin(th5_1)) > 0.001 || abs(sin(th5_3)) > 0.001 || abs(sin(th5_5)) > 0.001 || abs(sin(th5_7)) > 0.001 )
        {
            if ( abs(sin(th5_1)) > 0.001 )
            {
                th4_1 = atan2(  T36_11[2][2]/sin(th5_1), T36_11[0][2]/sin(th5_1) );
                th4_2 = atan2(  T36_11[2][2]/sin(th5_2), T36_11[0][2]/sin(th5_2) );
                th6_1 = atan2( -T36_11[1][1]/sin(th5_1), T36_11[1][0]/sin(th5_1) );
                th6_2 = atan2( -T36_11[1][1]/sin(th5_2), T36_11[1][0]/sin(th5_2) );
            }
            else
            {
                std::cout << "Singularity encountered. Impossible configuration. Exiting.." << std::endl;
                exit(1);
            }
            if ( abs(sin(th5_3)) > 0.001 )
            {
                th4_3 = atan2(  T36_12[2][2]/sin(th5_3), T36_12[0][2]/sin(th5_3) );
                th4_4 = atan2(  T36_12[2][2]/sin(th5_4), T36_12[0][2]/sin(th5_4) );
                th6_3 = atan2( -T36_12[1][1]/sin(th5_3), T36_12[1][0]/sin(th5_3) );
                th6_4 = atan2( -T36_12[1][1]/sin(th5_4), T36_12[1][0]/sin(th5_4) );
            }
            else
            {
                std::cout << "Singularity encountered. Impossible configuration. Exiting.." << std::endl;
                exit(1);
            }
            if ( abs(sin(th5_5)) > 0.001 )
            {
                th4_5 = atan2(  T36_21[2][2]/sin(th5_5), T36_21[0][2]/sin(th5_5) );
                th4_6 = atan2(  T36_21[2][2]/sin(th5_6), T36_21[0][2]/sin(th5_6) );
                th6_5 = atan2( -T36_21[1][1]/sin(th5_5), T36_21[1][0]/sin(th5_5) );
                th6_6 = atan2( -T36_21[1][1]/sin(th5_6), T36_21[1][0]/sin(th5_6) );
            }
            else
            {
                std::cout << "Singularity encountered. Impossible configuration. Exiting.." << std::endl;
                exit(1);
            }
            if ( abs(sin(th5_7)) > 0.001 )
            {
                th4_7 = atan2(  T36_22[2][2]/sin(th5_7), T36_22[0][2]/sin(th5_7) );
                th4_8 = atan2(  T36_22[2][2]/sin(th5_8), T36_22[0][2]/sin(th5_8) );
                th6_7 = atan2( -T36_22[1][1]/sin(th5_7), T36_22[1][0]/sin(th5_7) );
                th6_8 = atan2( -T36_22[1][1]/sin(th5_8), T36_22[1][0]/sin(th5_8) );
            }
            else
            {
                std::cout << "Singularity encountered. Impossible configuration. Exiting.." << std::endl;
                exit(1);
            }
        }
        // std::cout << "th4_1: " << th4_1 << ", th4_2: " << th4_2 << ", th4_3: " << th4_3 << ", th4_4: " << th4_4 << std::endl;
        // std::cout << "th4_5: " << th4_5 << ", th4_6: " << th4_6 << ", th4_7: " << th4_7 << ", th4_8: " << th4_8 << std::endl;

        // std::cout << "th6_1: " << th6_1 << ", th6_2: " << th6_2 << ", th6_3: " << th6_3 << ", th6_4: " << th6_4 << std::endl;
        // std::cout << "th6_5: " << th6_5 << ", th6_6: " << th6_6 << ", th6_7: " << th6_7 << ", th6_8: " << th6_8 << std::endl;
        
        
        TypedArray<double> H    = f.createArray<double>( {8,6}, { th1_1, th1_1, th1_2, th1_2, th1_1, th1_1, th1_2, th1_2,
                                                                  th2_1, th2_2, th2_3, th2_4, th2_1, th2_2, th2_3, th2_4,
                                                                  th3_1, th3_2, th3_3, th3_4, th3_1, th3_2, th3_3, th3_4,
                                                                  th4_1, th4_3, th4_5, th4_7, th4_2, th4_4, th4_6, th4_8,
                                                                  th5_1, th5_3, th5_5, th5_7, th5_2, th5_4, th5_6, th5_8,
                                                                  th6_1, th6_3, th6_5, th6_7, th6_2, th6_4, th6_6, th6_8
                                                                   } );

        outputs[0] = H;
    }

    TypedArray<double> transf_i_1_i( int i, double q, TypedArray<double> ALPHA, TypedArray<double> A, TypedArray<double> D, TypedArray<double> TH )
    {
        return f.createArray<double>( {4,4}, {  cos(q+TH[i]), sin(q+TH[i])*cos(ALPHA[i-1]), sin(q+TH[i])*sin(ALPHA[i-1]), 0, 
                                               -sin(q+TH[i]), cos(q+TH[i])*cos(ALPHA[i-1]), cos(q+TH[i])*sin(ALPHA[i-1]), 0, 
                                                           0,             -sin(ALPHA[i-1]),              cos(ALPHA[i-1]), 0, 
                                                      A[i-1],        -D[i]*sin(ALPHA[i-1]),         D[i]*cos(ALPHA[i-1]), 1 } );
    }

    // Matrix multiplication for square matrices
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

        // Check first argument 'Pe': input must be double array
        if ( inputs[0].getType() != ArrayType::DOUBLE ||
             inputs[0].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "First input 'th' must be double array" ) } ) );
        }

        // Check second argument 'Re': input must be double array
        if ( inputs[0].getType() != ArrayType::DOUBLE ||
             inputs[0].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "First input 'th' must be double array" ) } ) );
        }

        // Check third argument 'ALPHA': input must be double array
        if ( inputs[1].getType() != ArrayType::DOUBLE ||
             inputs[1].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Second input 'alpha' must be double array" ) } ) );
        }

        // Check fourth argument 'A': input must be double array
        if ( inputs[2].getType() != ArrayType::DOUBLE ||
             inputs[2].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Third input 'a' must be double array" ) } ) );
        }

        // Check fifth argument 'D': input must be double array
        if ( inputs[3].getType() != ArrayType::DOUBLE ||
             inputs[3].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Fourth input 'd' must be double array" ) } ) );
        }

        // Check fifth argument 'TH': input must be double array
        if ( inputs[4].getType() != ArrayType::DOUBLE ||
             inputs[4].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Fourth input 'th' must be double array" ) } ) );
        }
        
        // Check number of outputs: output must be 1 element -> DK homogeneous matrix
        if ( outputs.size() > 1 ) {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar("Only one output is returned") } ) );
        }
    }
};
