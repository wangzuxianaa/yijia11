#ifndef __COMMON_H__
#define __COMMON_H__

struct PositionVector{
    double x, y, z;
};

// ָ�
struct TaskPoint {
	double x, y, z, angle;
	int  f1, f2, mode;
};

// �����
struct CommandPoint {
	int facade; // ����
	int layer; // ����
	int location; // ��λ
};

// ��ͼ��
struct MapPoint {
	double x, y, z, angle;
};



#endif