#ifndef LEAST_SQUARES_METHOD_HPP_INCLUDE
#define LEAST_SQUARES_METHOD_HPP_INCLUDE

#define Q_NUM 3             //�K�E�X�̏����@�ɂ����関�m���̐��i����n�Ɠ����l�ɂ��邱�Ɓj

#include <stdio.h>
#include <vector>

using namespace std;

class Lsm
{
public:
	Lsm();
	void gauss(float a[Q_NUM][Q_NUM + 1], float p[Q_NUM]);
	void sai(vector<float> x, vector<float> y, int num);
	float x[Q_NUM];
private:

};

Lsm::Lsm()
{

}

void Lsm::gauss(float a[Q_NUM][Q_NUM + 1], float xx[Q_NUM])
{
	int i, j, k, l, pivot;
	float p, q, m, b[1][Q_NUM + 1];

	for (i = 0; i < Q_NUM; i++) {
		m = 0;
		pivot = i;

		for (l = i; l<Q_NUM; l++) {
			if (fabs(a[l][i])>m) {   //i���̒��ň��Ԓl���傫���s���I��
				m = fabs(a[l][i]);
				pivot = l;
			}
		}

		if (pivot != i) {                          //pivot��i�ƈႦ�΁A�s�̓����ւ�
			for (j = 0; j < Q_NUM + 1; j++) {
				b[0][j] = a[i][j];
				a[i][j] = a[pivot][j];
				a[pivot][j] = b[0][j];
			}
		}
	}

	for (k = 0; k < Q_NUM; k++) {
		p = a[k][k];              //�Ίp�v�f���ۑ�
		a[k][k] = 1;              //�Ίp�v�f�͂P�ɂȂ邱�Ƃ��킩���Ă��邩��

		for (j = k + 1; j < Q_NUM + 1; j++) {
			a[k][j] /= p;
		}

		for (i = k + 1; i < Q_NUM; i++) {
			q = a[i][k];

			for (j = k + 1; j < Q_NUM + 1; j++) {
				a[i][j] -= q*a[k][j];
			}
			a[i][k] = 0;              //�O�ƂȂ邱�Ƃ��킩���Ă����Ƃ���
		}
	}

	//���̌v�Z
	for (i = Q_NUM - 1; i >= 0; i--) {
		x[i] = a[i][Q_NUM];
		for (j = Q_NUM - 1; j > i; j--) {
			x[i] -= a[i][j] * x[j];
		}
	}

}

void Lsm::sai(vector<float> x, vector<float> y, int num)
{
	int i, j, k;
	//float X, Y;
	float A[Q_NUM][Q_NUM + 1], xx[Q_NUM];

	/*������*/
	for (i = 0; i < Q_NUM; i++) {
		for (j = 0; j < Q_NUM + 1; j++) {
			A[i][j] = 0.0;
		}
	}

	/*�K�E�X�̏����@�ŉ����s���̍쐬*/
	for (i = 0; i < Q_NUM; i++) {
		for (j = 0; j < Q_NUM; j++) {
			for (k = 0; k < num; k++) {
				A[i][j] += pow(x[k], i + j);
			}
		}
	}
	for (i = 0; i < Q_NUM; i++) {
		for (k = 0; k < num; k++) {
			A[i][Q_NUM] += pow(x[k], i)*y[k];
		}
	}
	/*�K�E�X�̏����@�̎��s�i�z��xx�͉��A���Ȃ킿�������̌W���������邽�߂̂��́j*/
	gauss(A, xx);
}

#endif // LEAST_SQUARES_METHOD_HPP_INCLUDE
