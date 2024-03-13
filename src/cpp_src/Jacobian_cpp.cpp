#include "mex.hpp"
#include "mexAdapter.hpp"
#include "MatlabDataArray.hpp"
#include <vector>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <iomanip>

using namespace matlab::data;
using matlab::mex::ArgumentList;

class MexFunction : public matlab::mex::Function {
public:
    ArrayFactory f;

    void operator()( ArgumentList outputs, ArgumentList inputs ) {
        checkArguments( outputs, inputs );

        TypedArray<double> Te    = std::move( inputs[0] );
        TypedArray<double> Q     = std::move( inputs[1] );
        TypedArray<double> ALPHA = std::move( inputs[2] );
        TypedArray<double> A     = std::move( inputs[3] );
        TypedArray<double> D     = std::move( inputs[4] );
        TypedArray<double> TH    = std::move( inputs[5] );
        
        TypedArray<double> T0i   = f.createArray<double>( {4,4}, {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1} );
        TypedArray<double> J     = f.createArray<double>( {6,6} );
        TypedArray<double> pe    = f.createArray<double>( {3,1}, {Te[0][3], Te[1][3], Te[2][3]} );
        TypedArray<double> disti = f.createArray<double>( {3,1} );
        TypedArray<double> zetai = f.createArray<double>( {3,1} );
        TypedArray<double> cross = f.createArray<double>( {3,1} );
        
        int i = 1;
        for ( double q : Q )
        {
            T0i   = matrixMult( T0i, transf_i_1_i( i, q, ALPHA, A, D, TH ) );
            
            // pe - p0i
            disti[0] = pe[0] - T0i[0][3]; disti[1] = pe[1] - T0i[1][3]; disti[2] = pe[2] - T0i[2][3];
            
            // zeta0_i
            zetai[0] = T0i[0][2];         zetai[1] = T0i[1][2];         zetai[2] = T0i[2][2];

            // z_i x (pe-p0i)
            cross = crossProduct( zetai, disti );

            J[0][i-1] = cross[0]; J[1][i-1] = cross[1]; J[2][i-1] = cross[2]; 
            J[3][i-1] = zetai[0]; J[4][i-1] = zetai[1]; J[5][i-1] = zetai[2];
            ++i;
        }

        outputs[0] = J;
    }

    TypedArray<double> crossProduct( TypedArray<double> a, TypedArray<double> b )
    {
        return f.createArray<double>( {3,1}, { a[1]*b[2] - a[2]*b[1],
                                               a[2]*b[0] - a[0]*b[2],
                                               a[0]*b[1] - a[1]*b[0]
                                              } );
    }

    TypedArray<double> transf_i_1_i( int i, double q, TypedArray<double> ALPHA, TypedArray<double> A, TypedArray<double> D, TypedArray<double> TH )
    {
        return f.createArray<double>( {4,4}, {  cos(q+TH[i]), sin(q+TH[i])*cos(ALPHA[i-1]), sin(q+TH[i])*sin(ALPHA[i-1]), 0, 
                                               -sin(q+TH[i]), cos(q+TH[i])*cos(ALPHA[i-1]), cos(q+TH[i])*sin(ALPHA[i-1]), 0, 
                                                           0,             -sin(ALPHA[i-1]),              cos(ALPHA[i-1]), 0, 
                                                      A[i-1],        -D[i]*sin(ALPHA[i-1]),         D[i]*cos(ALPHA[i-1]), 1 } );
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

        // Check first argument 'Te': input must be double array
        if ( inputs[0].getType() != ArrayType::DOUBLE ||
             inputs[0].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "First input 'th' must be double array" ) } ) );
        }

        // Check first argument 'Q': input must be double array
        if ( inputs[1].getType() != ArrayType::DOUBLE ||
             inputs[1].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "First input 'Q' must be double array" ) } ) );
        }

        // Check second argument 'ALPHA': input must be double array
        if ( inputs[2].getType() != ArrayType::DOUBLE ||
             inputs[2].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Second input 'ALPHA' must be double array" ) } ) );
        }

        // Check third argument 'A': input must be double array
        if ( inputs[3].getType() != ArrayType::DOUBLE ||
             inputs[3].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Third input 'A' must be double array" ) } ) );
        }

        // Check fourth argument 'D': input must be double array
        if ( inputs[4].getType() != ArrayType::DOUBLE ||
             inputs[4].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Fourth input 'D' must be double array" ) } ) );
        }

        // Check fifth argument 'TH': input must be double array
        if ( inputs[5].getType() != ArrayType::DOUBLE ||
             inputs[5].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Fifth input 'TH' must be double array" ) } ) );
        }
        
        // Check number of outputs: output must be 1 element -> manipulator's Jacobian
        if ( outputs.size() > 1 ) {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar("Only one output is returned") } ) );
        }
    }
};
