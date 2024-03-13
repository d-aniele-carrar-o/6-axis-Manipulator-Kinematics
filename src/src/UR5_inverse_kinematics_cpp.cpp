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

        bool gripper = false;
        // If in the D-H table is given also the transformation T6_ee, it has to be considered by kinematics
        if ( D.getNumberOfElements() > 7 )
            gripper = true;

        TypedArray<double> Te    = f.createArray<double>( {4,4}, {Re[0][0], Re[1][0], Re[2][0], 0,
                                                                  Re[0][1], Re[1][1], Re[2][1], 0,
                                                                  Re[0][2], Re[1][2], Re[2][2], 0,
                                                                     pe[0],    pe[1],    pe[2], 1} );
        
        // THETA 1 ========================================================================================
        TypedArray<double> p05 = f.createArray<double>( {4,1} );
        if ( gripper )
            p05 = matrixMult( Te, f.createArray<double>( {4,1}, {0, 0, -D[6]-D[7], 1} ) );
        else
            p05 = matrixMult( Te, f.createArray<double>( {4,1}, {0, 0, -D[6], 1} ) );

        double phi1     = atan2( p05[1], p05[0] );
        double cos_phi2 = D[4] / sqrt( p05[1]*p05[1] + p05[0]*p05[0] );
        double phi2     = atan2( sqrt(1-cos_phi2*cos_phi2), cos_phi2 );
        
        double th1_1 = phi1 - phi2 + pi/2;
        double th1_2 = phi1 + phi2 + pi/2;
        // std::cout << "th1_1: " << th1_1 << ", th1_2: " << th1_2 << std::endl;

        // THETA 5 ========================================================================================
        double cos_th5_1, cos_th5_2;
        if ( gripper )
        {
            cos_th5_1 = (pe[0]*sin(th1_1) - pe[1]*cos(th1_1) - D[4]) / (D[6]+D[7]);
            cos_th5_2 = (pe[0]*sin(th1_2) - pe[1]*cos(th1_2) - D[4]) / (D[6]+D[7]);
        }
        else
        {
            cos_th5_1 = (pe[0]*sin(th1_1) - pe[1]*cos(th1_1) - D[4]) / D[6];
            cos_th5_2 = (pe[0]*sin(th1_2) - pe[1]*cos(th1_2) - D[4]) / D[6];
        }
        double th5_1 = atan2( sqrt(1 - cos_th5_1*cos_th5_1), cos_th5_1 );
        double th5_2 = -th5_1;
        double th5_3 = atan2( sqrt(1 - cos_th5_2*cos_th5_2), cos_th5_2 );
        double th5_4 = -th5_3;
        // std::cout << "th5_1: " << th5_1 << ", th5_2: " << th5_2 << ", th5_3: " << th5_3 << ", th5_4: " << th5_4 << std::endl;
        
        // THETA 6 ========================================================================================
        double th6_1 = 0.0;
        double th6_2 = 0.0;
        double th6_3 = 0.0;
        double th6_4 = 0.0;
        
        if ( abs(sin(th5_1)) > 0.001 || abs(sin(th5_2)) > 0.001 || abs(sin(th5_3)) > 0.001 || abs(sin(th5_4)) > 0.001 )
        {
            TypedArray<double> Te_1 = homogeneousMatrixInverse( Te );
            TypedArray<double> Xhat = f.createArray<double>( {3,1}, {Te_1[0][0], Te_1[1][0], Te_1[2][0]} );
            TypedArray<double> Yhat = f.createArray<double>( {3,1}, {Te_1[0][1], Te_1[1][1], Te_1[2][1]} );
            
            if ( abs(sin(th5_1)) > 0.001 )
            {
                double sin_th6_1 = (-Xhat[1]*sin(th1_1) + Yhat[1]*cos(th1_1)) / sin(th5_1);
                double cos_th6_1 = ( Xhat[0]*sin(th1_1) - Yhat[0]*cos(th1_1)) / sin(th5_1);
                th6_1 = atan2( sin_th6_1, cos_th6_1 );
            }
            if ( abs(sin(th5_2)) > 0.001 )
            {
                double sin_th6_2 = (-Xhat[1]*sin(th1_1) + Yhat[1]*cos(th1_1)) / sin(th5_2);
                double cos_th6_2 = ( Xhat[0]*sin(th1_1) - Yhat[0]*cos(th1_1)) / sin(th5_2);
                th6_2 = atan2( sin_th6_2, cos_th6_2 );
            }
            if ( abs(sin(th5_3)) > 0.001 )
            {
                double sin_th6_3 = (-Xhat[1]*sin(th1_2) + Yhat[1]*cos(th1_2)) / sin(th5_3);
                double cos_th6_3 = ( Xhat[0]*sin(th1_2) - Yhat[0]*cos(th1_2)) / sin(th5_3);
                th6_3 = atan2( sin_th6_3, cos_th6_3 );
            }
            if ( abs(sin(th5_4)) > 0.001 )
            {
                double sin_th6_4 = (-Xhat[1]*sin(th1_2) + Yhat[1]*cos(th1_2)) / sin(th5_4);
                double cos_th6_4 = ( Xhat[0]*sin(th1_2) - Yhat[0]*cos(th1_2)) / sin(th5_4);
                th6_4 = atan2( sin_th6_4, cos_th6_4 );
            }
        }

        // std::cout << "th6_1: " << th6_1 << ", th6_2: " << th6_2 << ", th6_3: " << th6_3 << ", th6_4: " << th6_4 << std::endl;
        
        // THETA 3 ========================================================================================
        TypedArray<double> T56_1 = f.createArray<double>( {4,4} );
        TypedArray<double> T56_2 = f.createArray<double>( {4,4} );
        TypedArray<double> T56_3 = f.createArray<double>( {4,4} );
        TypedArray<double> T56_4 = f.createArray<double>( {4,4} );
        if ( gripper )
        {
            T56_1 = matrixMult( matrixMult( transf_i_1_i( 5, th5_1, ALPHA, A, D ), transf_i_1_i( 6, th6_1, ALPHA, A, D ) ), transf_i_1_i( 7, 0, ALPHA, A, D ) );
            T56_2 = matrixMult( matrixMult( transf_i_1_i( 5, th5_2, ALPHA, A, D ), transf_i_1_i( 6, th6_2, ALPHA, A, D ) ), transf_i_1_i( 7, 0, ALPHA, A, D ) );
            T56_3 = matrixMult( matrixMult( transf_i_1_i( 5, th5_3, ALPHA, A, D ), transf_i_1_i( 6, th6_3, ALPHA, A, D ) ), transf_i_1_i( 7, 0, ALPHA, A, D ) );
            T56_4 = matrixMult( matrixMult( transf_i_1_i( 5, th5_4, ALPHA, A, D ), transf_i_1_i( 6, th6_4, ALPHA, A, D ) ), transf_i_1_i( 7, 0, ALPHA, A, D ) );
        }
        else
        {
            T56_1 = matrixMult( transf_i_1_i( 5, th5_1, ALPHA, A, D ) , transf_i_1_i( 6, th6_1, ALPHA, A, D ) );
            T56_2 = matrixMult( transf_i_1_i( 5, th5_2, ALPHA, A, D ) , transf_i_1_i( 6, th6_2, ALPHA, A, D ) );
            T56_3 = matrixMult( transf_i_1_i( 5, th5_3, ALPHA, A, D ) , transf_i_1_i( 6, th6_3, ALPHA, A, D ) );
            T56_4 = matrixMult( transf_i_1_i( 5, th5_4, ALPHA, A, D ) , transf_i_1_i( 6, th6_4, ALPHA, A, D ) );
        }
        TypedArray<double> T01_1 = transf_i_1_i( 1, th1_1, ALPHA, A, D );
        TypedArray<double> T01_2 = transf_i_1_i( 1, th1_2, ALPHA, A, D );
        
        TypedArray<double> T14_1 = matrixMult( matrixMult( homogeneousMatrixInverse( T01_1 ), Te), homogeneousMatrixInverse(T56_1) );
        TypedArray<double> p41_1 = f.createArray<double>( {3,1}, {T14_1[0][3], T14_1[1][3], T14_1[2][3]} );
        double p41xz_1           = sqrt( p41_1[0]*p41_1[0] + p41_1[2]*p41_1[2] );
        
        TypedArray<double> T14_2 = matrixMult( matrixMult( homogeneousMatrixInverse( T01_1 ), Te ), homogeneousMatrixInverse( T56_2 ) );
        TypedArray<double> p41_2 = f.createArray<double>( {3,1}, {T14_2[0][3], T14_2[1][3], T14_2[2][3]} );
        double p41xz_2           = sqrt( p41_2[0]*p41_2[0] + p41_2[2]*p41_2[2] );
        
        TypedArray<double> T14_3 = matrixMult( matrixMult( homogeneousMatrixInverse( T01_2 ), Te ), homogeneousMatrixInverse( T56_3 ) );
        TypedArray<double> p41_3 = f.createArray<double>( {3,1}, {T14_3[0][3], T14_3[1][3], T14_3[2][3]} );
        double p41xz_3           = sqrt( p41_3[0]*p41_3[0] + p41_3[2]*p41_3[2] );
        
        TypedArray<double> T14_4 = matrixMult( matrixMult( homogeneousMatrixInverse( T01_2 ), Te ), homogeneousMatrixInverse( T56_4 ) );
        TypedArray<double> p41_4 = f.createArray<double>( {3,1}, {T14_4[0][3], T14_4[1][3], T14_4[2][3]} );
        double p41xz_4           = sqrt( p41_4[0]*p41_4[0] + p41_4[2]*p41_4[2] );
        
        double cos_th3_1 = -(A[2]*A[2] + A[3]*A[3] - p41xz_1*p41xz_1) / (2*A[2]*A[3]);
        double cos_th3_2 = -(A[2]*A[2] + A[3]*A[3] - p41xz_2*p41xz_2) / (2*A[2]*A[3]);
        double cos_th3_3 = -(A[2]*A[2] + A[3]*A[3] - p41xz_3*p41xz_3) / (2*A[2]*A[3]);
        double cos_th3_4 = -(A[2]*A[2] + A[3]*A[3] - p41xz_4*p41xz_4) / (2*A[2]*A[3]);

        double th3_1 = -atan2( sqrt(1-cos_th3_1*cos_th3_1), cos_th3_1 );
        double th3_2 = -atan2( sqrt(1-cos_th3_2*cos_th3_2), cos_th3_2 );
        double th3_3 = -atan2( sqrt(1-cos_th3_3*cos_th3_3), cos_th3_3 );
        double th3_4 = -atan2( sqrt(1-cos_th3_4*cos_th3_4), cos_th3_4 );
        
        double th3_5 = -th3_1;
        double th3_6 = -th3_2;
        double th3_7 = -th3_3;
        double th3_8 = -th3_4;
        // std::cout << "th3_1: " << th3_1 << ", th3_2: " << th3_2 << ", th3_3: " << th3_3 << ", th3_4: " << th3_4 << std::endl;
        // std::cout << "th3_5: " << th3_5 << ", th3_6: " << th3_6 << ", th3_7: " << th3_7 << ", th3_8: " << th3_8 << std::endl;
        
        // THETA 2 ========================================================================================
        double phi_11    = atan2( -p41_1[2], -p41_1[0] );
        double sin_phi21 = (-A[3]*sin(th3_1)) / p41xz_1;
        double phi_21    = atan2( sin_phi21, sqrt(1 - sin_phi21*sin_phi21) );

        double phi_12    = atan2( -p41_2[2], -p41_2[0] );
        double sin_phi22 = (-A[3]*sin(th3_2)) / p41xz_2;
        double phi_22    = atan2( sin_phi22, sqrt(1 - sin_phi22*sin_phi22) );

        double phi_13    = atan2( -p41_3[2], -p41_3[0] );
        double sin_phi23 = (-A[3]*sin(th3_3)) / p41xz_3;
        double phi_23    = atan2( sin_phi23, sqrt(1 - sin_phi23*sin_phi23) );

        double phi_14    = atan2( -p41_4[2], -p41_4[0] );
        double sin_phi24 = (-A[3]*sin(th3_4)) / p41xz_4;
        double phi_24    = atan2( sin_phi24, sqrt(1 - sin_phi24*sin_phi24) );
        
        double th2_1 = phi_11 - phi_21;
        double th2_2 = phi_12 - phi_22;
        double th2_3 = phi_13 - phi_23;
        double th2_4 = phi_14 - phi_24;

        double phi_15    = atan2( -p41_1[2], -p41_1[0] );
        double sin_phi25 = (-A[3]*sin(th3_5)) / p41xz_1;
        double phi_25    = atan2( sin_phi25, sqrt(1 - sin_phi25*sin_phi25) );

        double phi_16    = atan2( -p41_2[2], -p41_2[0] );
        double sin_phi26 = (-A[3]*sin(th3_6)) / p41xz_2;
        double phi_26    = atan2( sin_phi26, sqrt(1 - sin_phi26*sin_phi26) );

        double phi_17    = atan2( -p41_3[2], -p41_3[0] );
        double sin_phi27 = (-A[3]*sin(th3_7)) / p41xz_3;
        double phi_27    = atan2( sin_phi27, sqrt(1 - sin_phi27*sin_phi27) );

        double phi_18    = atan2( -p41_4[2], -p41_4[0] );
        double sin_phi28 = (-A[3]*sin(th3_8)) / p41xz_4;
        double phi_28    = atan2( sin_phi28, sqrt(1 - sin_phi28*sin_phi28) );
        
        double th2_5 = phi_15 - phi_25;
        double th2_6 = phi_16 - phi_26;
        double th2_7 = phi_17 - phi_27;
        double th2_8 = phi_18 - phi_28;
        // std::cout << "th2_1: " << th2_1 << ", th2_2: " << th2_2 << ", th2_3: " << th2_3 << ", th2_4: " << th2_4 << std::endl;
        // std::cout << "th2_5: " << th2_5 << ", th2_6: " << th2_6 << ", th2_7: " << th2_7 << ", th2_8: " << th2_8 << std::endl;
        
        // THETA 4 ========================================================================================
        TypedArray<double> T34 = matrixMult( homogeneousMatrixInverse( matrixMult( transf_i_1_i(2,th2_1,ALPHA,A,D), transf_i_1_i(3,th3_1,ALPHA,A,D) ) ), T14_1 ) ;
        double th4_1           = atan2( T34[1][0], T34[0][0] );
        
        T34          = matrixMult( homogeneousMatrixInverse( matrixMult( transf_i_1_i(2,th2_2,ALPHA,A,D), transf_i_1_i(3,th3_2,ALPHA,A,D) ) ), T14_2 ) ;
        double th4_2 = atan2( T34[1][0], T34[0][0] );
        
        T34          = matrixMult( homogeneousMatrixInverse( matrixMult( transf_i_1_i(2,th2_3,ALPHA,A,D), transf_i_1_i(3,th3_3,ALPHA,A,D) ) ), T14_3 ) ;
        double th4_3 = atan2( T34[1][0], T34[0][0] );
        
        T34          = matrixMult( homogeneousMatrixInverse( matrixMult( transf_i_1_i(2,th2_4,ALPHA,A,D), transf_i_1_i(3,th3_4,ALPHA,A,D) ) ), T14_4 ) ;
        double th4_4 = atan2( T34[1][0], T34[0][0] );

        
        T34          = matrixMult( homogeneousMatrixInverse( matrixMult( transf_i_1_i(2,th2_5,ALPHA,A,D), transf_i_1_i(3,th3_5,ALPHA,A,D) ) ), T14_1 ) ;
        double th4_5 = atan2( T34[1][0], T34[0][0] );
        
        T34          = matrixMult( homogeneousMatrixInverse( matrixMult( transf_i_1_i(2,th2_6,ALPHA,A,D), transf_i_1_i(3,th3_6,ALPHA,A,D) ) ), T14_2 ) ;
        double th4_6 = atan2( T34[1][0], T34[0][0] );
        
        T34          = matrixMult( homogeneousMatrixInverse( matrixMult( transf_i_1_i(2,th2_7,ALPHA,A,D), transf_i_1_i(3,th3_7,ALPHA,A,D) ) ), T14_3 ) ;
        double th4_7 = atan2( T34[1][0], T34[0][0] );
        
        T34          = matrixMult( homogeneousMatrixInverse( matrixMult( transf_i_1_i(2,th2_8,ALPHA,A,D), transf_i_1_i(3,th3_8,ALPHA,A,D) ) ), T14_4 ) ;
        double th4_8 = atan2( T34[1][0], T34[0][0] );
        // std::cout << "th4_1: " << th4_1 << ", th4_2: " << th4_2 << ", th4_3: " << th4_3 << ", th4_4: " << th4_4 << std::endl;
        // std::cout << "th4_5: " << th4_5 << ", th4_6: " << th4_6 << ", th4_7: " << th4_7 << ", th4_8: " << th4_8 << std::endl;
        
        TypedArray<double> TH    = f.createArray<double>( {8,6}, { th1_1, th1_1, th1_2, th1_2, th1_1, th1_1, th1_2, th1_2,
                                                                   th2_1, th2_2, th2_3, th2_4, th2_5, th2_6, th2_7, th2_8,
                                                                   th3_1, th3_2, th3_3, th3_4, th3_5, th3_6, th3_7, th3_8,
                                                                   th4_1, th4_2, th4_3, th4_4, th4_5, th4_6, th4_7, th4_8,
                                                                   th5_1, th5_2, th5_3, th5_4, th5_1, th5_2, th5_3, th5_4,
                                                                   th6_1, th6_2, th6_3, th6_4, th6_1, th6_2, th6_3, th6_4
                                                                   } );
        outputs[0] = TH;
    }

    TypedArray<double> transf_i_1_i( int i, double th, TypedArray<double> ALPHA, TypedArray<double> A, TypedArray<double> D )
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
