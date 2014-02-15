/*
 * matrix_utils.h
 *
 *  Created on: Jan 22, 2014
 *      Author: mkrogius
 */

#ifndef MATRIX_UTILS_H_
#define MATRIX_UTILS_H_

//double* zeroVector(int length);
void scaleVector(double scalar, double *vector, int length);
//double* identityMatrix(int size);
void diagonalMatrix(double value, double* matrix, int dim);
void cholesky(double* matrix, double* outMatrix, int dim);
void matrixCopy(double *A,double *B, int dim);
void addVectors(double A[], double B[], int length);
void subtractVectors(double A[], double B[], int length);
double *vectorIndex(double* vector, int index, int vector_length);
void averageVectors(double *dest, double* vectors,
		int num_vectors, int vector_length);
void averageOuterProductOfVectors(
		double *dest
		,double* vectors1
		,double* vectors2
		,int num_vectors
		,int dim1
		,int dim2);
void subtractMultipleVectors(double *vectors, double *subtrahend
		, int num_vectors, int dim);
void solve(double * A, double *B, double *C, int dim1, int dim2);
void leftMultiplyAdd(double* A, double* B, double* C, int dim1, int dim2, int dim3);
void transposedMultiplyAdd(double* A, double* B, double* C, int dim1, int dim2, int dim3);

#endif /* MATRIX_UTILS_H_ */
