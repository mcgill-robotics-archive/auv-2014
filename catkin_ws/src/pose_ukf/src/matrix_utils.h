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
double *vectorIndex(double* vector, int index, int vector_length);
void averageVectors(double *dest, double* vectors,
		int num_vectors, int vector_length);
void averageOuterProductOfVectors(double *dest, double* vectors,
		int num_vectors, int dim);


#endif /* MATRIX_UTILS_H_ */
