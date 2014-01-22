/*
 * matrix_utils.cpp
 *
 *  Created on: Jan 22, 2014
 *      Author: mkrogius
 */
#include "matrix_utils.h"
#include <math.h>

/*//TODO: Should I be declaring these matrices with new? Probably
void zeroVector(double* vector, int length)
{
	for (int i = 0; i < length; i++)
	{
		vector[i] = 0.0;
	}
}

double* identityMatrix(int width)
{
	int num_elements = width*width;
	double matrix [num_elements] = {};

	matrix[0] = 1.0;
	for (int i = 0; i < num_elements; i += width + 1)
	{
		matrix[i] = 1;
	}

	return matrix;
}*/

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
		matrix[i] = value;
	}
}

void cholesky(double *A, double *L, int n) {
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < (i+1); j++)
        {
            double s = 0;
            for (int k = 0; k < j; k++)
            {
                s += L[i * n + k] * L[j * n + k];
            }

            L[i * n + j] = (i == j) ? sqrt(A[i * n + i] - s) : (1.0 / L[j * n + j] * (A[i * n + j] - s));
        }
    }
}

void matrixCopy(double A[],double B[], int dim)
{
	int size = dim*dim;

	for (int i = 0; i< size; i++)
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

void outerProductAdd(double* A, double* B, double* C, int dim)
{//computes the outer product of A and B and adds the result to C
	for (int i = 0; i < dim; i++)
	{
		for (int j = 0; j < dim; j++)
		{
			C[dim*i+j] = A[i]*B[j];
		}
	}
}

void averageVectors(double *dest, double* vectors,
		int num_vectors, int vector_length)
{
	for (int i = 0; i< num_vectors; i++)
	{
		addVectors(dest, vectorIndex(vectors, i, vector_length), vector_length);
	}
	scaleVector(1.0/num_vectors, dest, vector_length);
}

void averageOuterProductOfVectors(double *dest, double* vectors,
		int num_vectors, int dim)
{// Computes the average of the outer product of the vectors in vectors
	//with themselves and stores result in dest

	for (int i = 0; i < dim*dim; i++)
	{
		dest[i] = 0;
	}

	for (int i = 0; i < num_vectors; i++)
	{
		outerProductAdd(vectorIndex(vectors, i, dim),vectorIndex(vectors, i, dim), dest, dim);
	}
	scaleVector(1.0/num_vectors, dest, dim*dim);
}

double *vectorIndex(double* vector, int index, int vector_length)
{//Returns a pointer to the desired sigma array
	return &(vector[index*vector_length]);
}

