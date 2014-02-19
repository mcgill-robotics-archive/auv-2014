#include "matrix_utils.h"
#include <math.h>

void scaleVector(double scalar, double *vector, int length)
{
	for (int i = 0; i < length; i++)
	{
		vector[i] *= scalar;
	}
}

void diagonalMatrix(double value, double* matrix, int width)
{

	for (int i = 0; i < width*width; i += width+1)
	{
		if (i%(width+1) == 0)
			matrix[i] = value;
		else
			matrix[i] = 0;
	}
}

void cholesky(double *A, double *L, int n) {
    for (int i = 0; i < n; i++)
    {
    	int j = 0;
        for (; j <= i; j++)
        {
            double sum = 0;
            for (int k = 0; k < j; k++)
            {
                sum += L[i * n + k] * L[j * n + k];
            }

            if (i == j)
            {
            	L[i * n + j] = sqrt(fmax(A[i * n + i] - sum, 1e-10));
            	//TODO: Log an error if the argument of sqrt would have been negative without the max
            }else
            {
            	L[i * n + j] = (A[i * n + j] - sum)/ L[j * n + j];
            }
        }

        for (;j<n; j++)
        {
        	L[i*n + j] = 0.0;
        }
    }
}

void vectorCopy(double A[],double B[], int length)
{

	for (int i = 0; i< length; i++)
	{
		B[i] = A[i];
	}
}

void addVectors(double A[], double B[], int length)
{//Adds A and B and stores result in A
	for ( int i = 0; i < length; i++)
	{
		A[i] += B[i];
	}
}

void subtractVectors(double A[], double B[], int length)
{//subtracts A and B and stores result in A
	for ( int i = 0; i < length; i++)
	{
		A[i] -= B[i];
	}
}

void subtractMultipleVectors(double *vectors, double *subtrahend, int num_vectors, int dim)
{
	for (int i = 0; i < num_vectors; i++)
	{
		subtractVectors(vectorIndex(vectors, i, dim), subtrahend, dim);
	}
}

void outerProductAdd(double* A, double* B, double* C, int dim1, int dim2)
{//computes the outer product of A and B and adds the result to C
	//A is a vector of length dim1
	//B is a vector of length dim2
	//C is a dim1 * dim2 matrix
	for (int i = 0; i < dim1; i++)
	{
		for (int j = 0; j < dim2; j++)
		{
			C[dim2*i+j] += A[i]*B[j];
		}
	}
}

void averageVectors(double* vectors, double *dest,
		int num_vectors, int vector_length)
{
	for (int i = 0; i < vector_length; i++)
	{
		dest[i] = 0;
	}
	for (int i = 0; i< num_vectors; i++)
	{
		addVectors(dest, vectorIndex(vectors, i, vector_length), vector_length);
	}
	scaleVector(1.0/num_vectors, dest, vector_length);
}

void averageOuterProduct(
		double* vectors1
		,double* vectors2
		,double *dest
		,int num_vectors
		,int dim1
		,int dim2)
{// Computes the average of the outer product of the vectors in vectors1 and
	//vectors2 and stores result in dest

	for (int i = 0; i < dim1*dim2; i++)
	{
		dest[i] = 0;
	}

	for (int i = 0; i < num_vectors; i++)
	{
		outerProductAdd(vectorIndex(vectors1, i, dim1),vectorIndex(vectors2, i, dim2), dest, dim1, dim2);
	}
	scaleVector(1.0/num_vectors, dest, dim1*dim2);
}

double *vectorIndex(double* vector, int index, int vector_length)
{//Returns a pointer to the desired sigma array
	return &(vector[index*vector_length]);
}

void solve(double * A, double *B, double *C, int dim1, int dim2)
{
	//This method solves the equation CB=A
	//given dim1*dim2 matrix A
	//and dim2*dim2 symmetric positive definite matrix B
	double *rootB = new double[dim2*dim2]();
	double *D = new double[dim2*dim1]();
	cholesky(B, rootB, dim2);

	//Now we have rootB*rootB^T = B
	//where rootB is lower triangular
	//and thus C*rootB*rootB^T = A
	//introduce matrix D = C*rootB
	//we now have D*rootB^T = A

	//col and row refer to the column and row of D being written
	//we solve for D
	for(int col = 0; col < dim2; col++)
	{
		for(int row = 0; row<dim1;row++)
		{
			double temp = 0;
			for(int k = 0; k < col; k++)
			{
				temp += D[row*dim2+k]*rootB[col*dim2+k];
			}
			D[row*dim2+col] = (A[row*dim2+col]-temp)/rootB[col*dim2+col];
		}
	}

	//Now we have C*rootB = D
	for(int col = dim2 - 1; col >= 0; col--)
	{
		for(int row = 0; row<dim1;row++)
		{
			double temp = 0;
			for(int k = dim2-1; k > col; k--)
			{
				temp += C[row*dim2+k]*rootB[k*dim2+col];
			}
			C[row*dim2+col] = (D[row*dim2+col]-temp)/rootB[col*dim2+col];
		}
	}

	delete rootB;
	delete D;
}

void leftMultiplyAdd(double* A, double* B, double* C, int dim1, int dim2, int dim3)
{
	//Multiply A*B and add the result to in C
	//A: dim1 x dim2
	//B: dim2 x dim3
	//C: dim1 x dim3

	for(int row = 0; row < dim1; row++)
	{
		for(int col = 0; col < dim3; col++)
		{
			int temp = 0;
			for(int i = 0; i < dim2; i++)
			{
				temp += A[row*dim2 + i]*B[i*dim3 + col];
			}
			C[row*dim3 + col] += temp;
		}
	}
}

void transposedMultiplyAdd(double* A, double* B, double* C, int dim1, int dim2, int dim3)
{
	//Multiply A*B^T and add the result to C
	//A: dim1 x dim2
	//B: dim3 x dim2
	//C: dim1 x dim3

	for(int row = 0; row < dim1; row++)
	{
		for(int col = 0; col < dim3; col++)
		{
			int temp = 0;
			for(int i = 0; i < dim2; i++)
			{
				temp += A[row*dim2 + i]*B[col*dim2 + i];
			}
			C[row*dim3 + col] += temp;
		}
	}
}

void addDiagonal(double* matrix, double value, int dim)
{
	//Adds a diagonal matrix with value value
	//to dim x dim matrix matrix

	for (int i = 0; i < dim*dim; i += dim + 1)
	{
		matrix[i] += value;
	}
}
