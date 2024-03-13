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

        double i                 = inputs[0][0];
        double q                 = inputs[1][0];
        TypedArray<double> ALPHA = std::move( inputs[2] );
        TypedArray<double> A     = std::move( inputs[3] );
        TypedArray<double> D     = std::move( inputs[4] );
        TypedArray<double> TH    = std::move( inputs[5] );
        
        outputs[0] = transf_i_1_i( i, q, ALPHA, A, D, TH );
    }

    TypedArray<double> transf_i_1_i( int i, double q, TypedArray<double> ALPHA, TypedArray<double> A, TypedArray<double> D, TypedArray<double> TH )
    {
        return f.createArray<double>( {4,4}, {  cos(q+TH[i+1]), sin(q+TH[i+1])*cos(ALPHA[i]), sin(q+TH[i+1])*sin(ALPHA[i]), 0, 
                                               -sin(q+TH[i+1]), cos(q+TH[i+1])*cos(ALPHA[i]), cos(q+TH[i+1])*sin(ALPHA[i]), 0, 
                                                             0,               -sin(ALPHA[i]),                cos(ALPHA[i]), 0, 
                                                          A[i],        -D[i+1]*sin(ALPHA[i]),         D[i+1]*cos(ALPHA[i]), 1 } );
    }

    void checkArguments( ArgumentList outputs, ArgumentList inputs ) {
        // Get pointer to engine
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();

        // Check first argument 'i': input must be double array
        if ( inputs[0].getType() != ArrayType::DOUBLE ||
             inputs[0].getType() == ArrayType::COMPLEX_DOUBLE ||
             inputs[0].getNumberOfElements() != 1 )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "First input 'i' must be double scalar" ) } ) );
        }

        // Check second argument 'Q': input must be double array
        if ( inputs[1].getType() != ArrayType::DOUBLE ||
             inputs[1].getType() == ArrayType::COMPLEX_DOUBLE ||
             inputs[1].getNumberOfElements() != 1 )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Second input 'q' must be double scalar" ) } ) );
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

        // Check sixth argument 'TH': input must be double array
        if ( inputs[5].getType() != ArrayType::DOUBLE ||
             inputs[5].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Sixth input 'TH' must be double array" ) } ) );
        }
        
        // Check number of outputs: output must be 1 element -> transformation i-1 -> i
        if ( outputs.size() > 1 ) {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar("Only one output is returned") } ) );
        }
    }
};
