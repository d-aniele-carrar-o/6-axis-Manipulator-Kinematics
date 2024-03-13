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

        TypedArray<double> TH    = std::move( inputs[0] );
        TypedArray<double> ALPHA = std::move( inputs[1] );
        TypedArray<double> A     = std::move( inputs[2] );
        TypedArray<double> D     = std::move( inputs[3] );
        
        TypedArray<double> Te     = f.createArray<double>( {4,4}, {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1} );
        TypedArray<double> Ti     = f.createArray<double>( {4,4}, {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1} );

        int i = 1;
        for ( double th : TH ) {
            Ti = transf_i_1_i( i, th, ALPHA, A, D );
            Te = matrixMult( Te, Ti );
            outputs[i-1] = Te;
            ++i;
        }
        outputs[i-1] = Te;
    }

    TypedArray<double> transf_i_1_i( int i, double th, TypedArray<double> ALPHA, TypedArray<double> A, TypedArray<double> D )
    {
        return f.createArray<double>( {4,4}, {  cos(th), sin(th)*cos(ALPHA[i-1]), sin(th)*sin(ALPHA[i-1]), 0, 
                                               -sin(th), cos(th)*cos(ALPHA[i-1]), cos(th)*sin(ALPHA[i-1]), 0, 
                                                      0,        -sin(ALPHA[i-1]),         cos(ALPHA[i-1]), 0, 
                                                 A[i-1],   -D[i]*sin(ALPHA[i-1]),    D[i]*cos(ALPHA[i-1]), 1 } );
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

        // Check first argument 'TH': input must be double array
        if ( inputs[0].getType() != ArrayType::DOUBLE ||
             inputs[0].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "First input 'th' must be double array" ) } ) );
        }

        // Check second argument 'ALPHA': input must be double array
        if ( inputs[1].getType() != ArrayType::DOUBLE ||
             inputs[1].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Second input 'alpha' must be double array" ) } ) );
        }

        // Check third argument 'A': input must be double array
        if ( inputs[2].getType() != ArrayType::DOUBLE ||
             inputs[2].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Third input 'a' must be double array" ) } ) );
        }

        // Check fourth argument 'D': input must be double array
        if ( inputs[3].getType() != ArrayType::DOUBLE ||
             inputs[3].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Fourth input 'd' must be double array" ) } ) );
        }
        
        // Check number of outputs: output must be 6 element -> DK homogeneous matrices: [T01, T12, T23, T34, T45, T56, Te]
        if ( outputs.size() > 6 ) {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar("Only 7 outputs are returned") } ) );
        }
    }
};
