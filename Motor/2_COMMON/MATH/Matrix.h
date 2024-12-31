/**************************************************************************************************
*     File Name :                        Matrix.h
*     Library/Module Name :              MATH
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             ���������ͷ�ļ�
**************************************************************************************************/
#ifndef Matrix_H
#define Matrix_H

#include "Math.h"
#include "string.h"
#include "stdlib.h"

#define _IN
#define _OUT
#define _IN_OUT

#define _ERROR_NO_ERROR		                                        0X00000000   //�޴���
#define _ERROR_FAILED_TO_ALLOCATE_HEAP_MEMORY	                    0X00000001   //������ڴ�ʧ��
#define _ERROR_SVD_EXCEED_MAX_ITERATIONS							0X00000002   //svd��������������
#define _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL	                    0X00000003   //�������������������
#define _ERROR_MATRIX_MULTIPLICATION								0X00000004   //����˷�����(��һ����������������ڵڶ�����������)
#define _ERROR_MATRIX_MUST_BE_SQUARE								0X00000005   //�������Ϊ����
#define _ERROR_MATRIX_NORM_TYPE_INVALID								0X00000006   //����ģ������Ч
#define _ERROR_MATRIX_EQUATION_HAS_NO_SOLUTIONS						0X00000007   //���󷽳��޽�
#define _ERROR_MATRIX_EQUATION_HAS_INFINITY_MANNY_SOLUTIONS	        0X00000008   //���󷽳���������
#define _ERROR_QR_DECOMPOSITION_FAILED								0X00000009   //QR�ֽ�ʧ��
#define _ERROR_CHOLESKY_DECOMPOSITION_FAILED						0X0000000a   //cholesky�ֽ�ʧ��
#define _ERROR_IMPROVED_CHOLESKY_DECOMPOSITION_FAILED				0X0000000b   //improved cholesky�ֽ�ʧ��
#define _ERROR_LU_DECOMPOSITION_FAILED								0X0000000c   //LU�ֽ�ʧ��
#define _ERROR_CREATE_MATRIX_FAILED									0X0000000d   //��������ʧ��
#define _ERROR_MATRIX_TRANSPOSE_FAILED								0X0000000e   //����ת��ʧ��
#define _ERROR_CREATE_VECTOR_FAILED									0X0000000f   //��������ʧ��
#define _ERROR_VECTOR_DIMENSION_NOT_EQUAL 							0X00000010   //����ά������ͬ
#define _ERROR_VECTOR_NORM_TYPE_INVALID								0X00000011   //����ģ������Ч
#define _ERROR_VECTOR_CROSS_FAILED									0X00000012   //�������ʧ��
#define _ERROR_INPUT_PARAMETERS_ERROR								0X00010000   //�����������

typedef  unsigned int ERROR_ID;
typedef int INDEX;
typedef short FLAG;
typedef int INTEGER;
typedef float REAL;
typedef char* STRING;
typedef void VOID;

typedef  struct matrix
{
	INTEGER rows;
	INTEGER columns;
	REAL* p;
}MATRIX;

typedef struct matrix_node
{
	MATRIX* ptr;
	struct matrix_node* next;
} MATRIX_NODE;

typedef struct matrix_element_node
{
	REAL* ptr;
	struct matrix_element_node* next;
} MATRIX_ELEMENT_NODE;

typedef struct stacks
{
	MATRIX_NODE* matrixNode;
	MATRIX_ELEMENT_NODE* matrixElementNode;

	// ...
	// ������������ָ��
} STACKS;

/**********************************************************************************************
Function: init_stack
Description: ��ʼ��ջ
Input: ��
Output: ��
Input_Output: ջָ��
Return: ��
Author: CJYS
***********************************************************************************************/
VOID init_stack(_IN_OUT STACKS* S);

/**********************************************************************************************
Function: free_stack
Description: �ͷ�ջ
Input: ջָ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
VOID free_stack(_IN STACKS* S);

/**********************************************************************************************
Function: creat_matrix
Description: ��������
Input: ��������rows������columns
Output: �����ָ��errorID��ջָ��S
Input_Output: ��
Return: ����ָ��
Author: CJYS
***********************************************************************************************/
MATRIX* creat_matrix(_IN INTEGER rows, _IN INTEGER columns, _OUT ERROR_ID* errorID, _OUT STACKS* S);

/**********************************************************************************************
Function: creat_multiple_matrices
Description: �����������
Input: ��������rows������columns������count
Output: �����ָ��errorID��ջָ��S
Input_Output: ��
Return: ����ָ��
Author: CJYS
***********************************************************************************************/
MATRIX* creat_multiple_matrices(_IN INTEGER rows, _IN INTEGER columns, _IN INTEGER count, _OUT ERROR_ID* errorID, _OUT STACKS* S);

/**********************************************************************************************
Function: creat_zero_matrix
Description: ���������
Input: ��������rows������columns
Output: �����ָ��errorID��ջָ��S
Input_Output: ��
Return: ����ָ��
Author: CJYS
***********************************************************************************************/
MATRIX* creat_zero_matrix(_IN INTEGER rows, _IN INTEGER columns, _OUT ERROR_ID* errorID, _OUT STACKS* S);

/**********************************************************************************************
Function: creat_eye_matrix
Description: ������λ����
Input: ��������rows������columns
Output: �����ָ��errorID��ջָ��S
Input_Output: ��
Return: ����ָ��
Author: CJYS
***********************************************************************************************/
MATRIX* creat_eye_matrix(_IN INTEGER n, _OUT ERROR_ID* errorID, _OUT STACKS* S);

/**********************************************************************************************
Function: matrix_add
Description: ����A + ����B = ����C
Input: ����A,����B
Output: ����C
Input_Output: ��
Return: �����
Author: CJYS
***********************************************************************************************/
ERROR_ID matrix_add(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX *C);

/**********************************************************************************************
Function: matrix_subtraction
Description: ����A - ����B = ����C
Input: ����A,����B
Output: ����C
Input_Output: ��
Return: �����
Author: CJYS
***********************************************************************************************/
ERROR_ID matrix_subtraction(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C);

/**********************************************************************************************
Function: matrix_multiplication
Description: ����˷�C = A * B
Input: ����A,����B
Output: ����C
Input_Output: ��
Return: �����
Author: CJYS
***********************************************************************************************/
ERROR_ID matrix_multiplication(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C);

/**********************************************************************************************
Function: matrix_inverse
Description: ��������
Input: ����A
Output: ����A�������
Input_Output: ��
Return: �����
Author: CJYS
***********************************************************************************************/
ERROR_ID matrix_inverse(_IN MATRIX* A, _OUT MATRIX* invA);

/**********************************************************************************************
Function: matrix_transpose
Description: ����ת��
Input: ����A
Output: ����A��ת��
Input_Output: ��
Return: �����
Author: CJYS
***********************************************************************************************/
ERROR_ID matrix_transpose(_IN MATRIX* A, _OUT MATRIX* transposeA);

/**********************************************************************************************
Function: matrix_trace
Description: ����ļ�
Input: ����A
Output: ����A�ļ�
Input_Output: ��
Return: �����
Author: CJYS
***********************************************************************************************/
ERROR_ID matrix_trace(_IN MATRIX* A, _OUT REAL* trace);

/**********************************************************************************************
Function: lup_decomposition
Description: n��n�о���LUP�ֽ�PA = L * U
Input: n��n�о���A
Output: n��n�������Ǿ���L��n��n�������Ǿ���U��n��n���û�����P
Input_Output: ��
Return: �����
Author: CJYS
�ο���https://zhuanlan.zhihu.com/p/84210687
***********************************************************************************************/
ERROR_ID lup_decomposition(_IN MATRIX* A, _OUT MATRIX* L, _OUT MATRIX* U, _OUT MATRIX* P);

/**********************************************************************************************
Function: solve_matrix_equation_by_lup_decomposition
Description: LUP�ֽ����󷽳�AX=B,����AΪn��n�о���BΪn��m�о���XΪn��m�д������(д������B)
Input: n��n�о���A
Output: ��
Input_Output: n��m�о���B(��n��m�д������X)
Return: �����
Author: CJYS
***********************************************************************************************/
ERROR_ID solve_matrix_equation_by_lup_decomposition(_IN MATRIX* A, _IN_OUT MATRIX* B);

#endif /* Matrix_H */

