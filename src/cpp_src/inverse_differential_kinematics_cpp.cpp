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

        TypedArray<double> q_curr  = std::move( inputs[0] );
        TypedArray<double> Te      = std::move( inputs[1] );
        TypedArray<double> Td      = std::move( inputs[2] );
        TypedArray<double> vd      = std::move( inputs[3] );
        TypedArray<double> J       = std::move( inputs[4] );
        double dt                  = inputs[5][0];
        double max_vel             = inputs[6][0];

        double Kp = 3*max_vel, Ko = 3*max_vel;
        
        TypedArray<double> step     = getStep( Te, Td );
        TypedArray<double> J_inv    = MatrixInversion( J );
        TypedArray<double> v_dot_d  = f.createArray<double>( {6,1}, { vd[0]+Kp*step[0], vd[1]+Kp*step[1], vd[2]+Kp*step[2],
                                                                      vd[3]+Ko*step[3], vd[4]+Ko*step[4], vd[5]+Ko*step[5]
                                                                    } );

        double k = 0.00001;
        for ( double& j : J_inv )
            j += k;
        
        TypedArray<double> q_dot_d  = matrixMult( J_inv, v_dot_d );

        double v_max = 0.0;
        for ( double q : q_dot_d )
        {
            v_max = fmax( abs(q), v_max );
        }
        if ( v_max > max_vel )
        for ( double& q : q_dot_d )
        {
            q = q * max_vel / v_max;
        }

        outputs[0] = f.createArray<double>( {6,1}, { q_curr[0]+q_dot_d[0]*dt, q_curr[1]+q_dot_d[1]*dt, q_curr[2]+q_dot_d[2]*dt,
                                                     q_curr[3]+q_dot_d[3]*dt, q_curr[4]+q_dot_d[4]*dt, q_curr[5]+q_dot_d[5]*dt
                                                    } );
        outputs[1] = q_dot_d;
    }

    // calculate the cofactor of element (row,col)
    TypedArray<double> GetMinor( TypedArray<double> src, int row, int col, unsigned long order )
    {
        TypedArray<double> dest = f.createArray<double>( {order,order} );

        // indicate which col and row is being copied to dest
        int colCount=0, rowCount=0;
    
        for ( int i=0; i<order; ++i )
        {
            if ( i != row )
            {
                colCount = 0;
                for ( int j=0; j<order; ++j )
                {
                    // when j is not the element
                    if ( j != col )
                    {
                        dest[rowCount][colCount] = src[i][j];
                        ++colCount;
                    }
                }
                ++rowCount;
            }
        }
    
        return dest;
    }
    
    // Calculate the determinant recursively.
    double CalcDeterminant( TypedArray<double> mat, unsigned long order )
    {
        // order must be >= 0
        // stop the recursion when matrix is a single element
        if( order == 1 )
            return mat[0][0];
    
        // the determinant value
        double det = 0;
    
        // allocate the cofactor matrix
        TypedArray<double> minor = f.createArray<double>( {order,order} );
    
        for ( int i=0; i<order; ++i )
        {
            // get minor of element (0,i)
            minor = GetMinor( mat, 0, i , order);
            // the recusion is here!
    
            det += (i%2==1?-1.0:1.0) * mat[0][i] * CalcDeterminant( minor, order-1 );
        }
    
        return det;
    }

    TypedArray<double> MatrixInversion( TypedArray<double> A )
    {
        unsigned long order = A.getDimensions()[0];
        TypedArray<double> Y = f.createArray<double>( {order,order} );

        // get the determinant of A
        double det = 1.0 / CalcDeterminant( A, order );
    
        TypedArray<double> minor = f.createArray<double>( {order,order} );
        
        for ( int j=0; j<order; ++j )
        {
            for ( int i=0; i<order; ++i )
            {
                // get the co-factor (matrix) of A(j,i)
                minor = GetMinor( A, j, i, order );

                Y[i][j] = det*CalcDeterminant( minor, order-1 );

                if( (i+j)%2 == 1 )
                    Y[i][j] = -Y[i][j];
            }
        }

        return Y;
    }

    TypedArray<double> getStep( TypedArray<double> T_i, TypedArray<double> T_f )
    {
        TypedArray<double> T = matrixMult( T_f, homogeneousMatrixInverse( T_i ) );
        TypedArray<double> R = f.createArray<double>( {3,3}, {T[0][0], T[1][0], T[2][0],
                                                              T[0][1], T[1][1], T[2][1],
                                                              T[0][2], T[1][2], T[2][2]} );
        TypedArray<double> R_t = matrixTranspose( R );

        TypedArray<double> R_d = f.createArray<double>( {3,3}, {R[0][0]-R_t[0][0], R[1][0]-R_t[1][0], R[2][0]-R_t[2][0],
                                                                R[0][1]-R_t[0][1], R[1][1]-R_t[1][1], R[2][1]-R_t[2][1],
                                                                R[0][2]-R_t[0][2], R[1][2]-R_t[1][2], R[2][2]-R_t[2][2]} );
        
        double theta = atan2( sqrt( R_d[2][1]*R_d[2][1] + R_d[0][2]*R_d[0][2] + R_d[1][0]*R_d[1][0] ), R[0][0]+R[1][1]+R[2][2]+1 );

        TypedArray<double> w = f.createArray<double>( {3,1}, {theta/(2*sin(theta))*R_d[2][1], theta/(2*sin(theta))*R_d[0][2], theta/(2*sin(theta))*R_d[1][0]} );
        if ( abs( theta ) < 0.00001 )
            w = f.createArray<double>( {3,1}, {0,0,0} );

        return f.createArray<double>( {6,1}, {T_f[0][3]-T_i[0][3], T_f[1][3]-T_i[1][3], T_f[2][3]-T_i[2][3], w[0], w[1], w[2]} );
    }

    TypedArray<double> matrixTranspose( TypedArray<double> A )
    {
        ArrayDimensions d          = A.getDimensions();
        TypedArray<double> A_trans = f.createArray<double>( {d[1],d[0]} );
        
        for ( int i=0; i<d[0]; ++i )
        {
            for ( int j=0; j<d[1]; ++j )
            {
                A_trans[j][i] = A[i][j];
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

        // Check first argument 'q_curr': input must be double array
        if ( inputs[0].getType() != ArrayType::DOUBLE ||
             inputs[0].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "First input 'q_curr' must be double array" ) } ) );
        }

        // Check second argument 'Te': input must be double array
        if ( inputs[1].getType() != ArrayType::DOUBLE ||
             inputs[1].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Second input 'Te' must be double array" ) } ) );
        }

        // Check third argument 'Td': input must be double array
        if ( inputs[2].getType() != ArrayType::DOUBLE ||
             inputs[2].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Third input 'Td' must be double array" ) } ) );
        }

        // Check fourth argument 'vd': input must be double array
        if ( inputs[3].getType() != ArrayType::DOUBLE ||
             inputs[3].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Fourth input 'vd' must be double array" ) } ) );
        }

        // Check fifth argument 'J': input must be double array
        if ( inputs[4].getType() != ArrayType::DOUBLE ||
             inputs[4].getType() == ArrayType::COMPLEX_DOUBLE )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Fifth input 'J' must be double array" ) } ) );
        }

        // Check sixth argument 'dt': input must be double scalar
        if ( inputs[5].getType() != ArrayType::DOUBLE ||
             inputs[5].getType() == ArrayType::COMPLEX_DOUBLE ||
             inputs[5].getNumberOfElements() != 1 )
        {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar( "Sixth input 'dt' must be double scalar" ) } ) );
        }

        // Check number of outputs: output must be 2 elements -> [q_d, q_dot_d]
        if ( outputs.size() > 2 ) {
            matlabPtr->feval( u"error", 0, std::vector<Array>( { f.createScalar("Only 2 outputs are returned - [q_d, q_dot_d]") } ) );
        }
    }
};
