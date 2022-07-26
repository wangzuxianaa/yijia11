#ifndef __TASKPLANNER_H__
#define __TASKPLANNER_H__



#include<iostream>
#include<fstream>
#include<cstdlib>
#include<sys/time.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <vector>
#include <sstream>
#include <string.h>
#include <signal.h>
#include <algorithm>
#include <cmath>
#include "common.h"

using namespace std;



class TaskPlanner {
private:
	int m_countLayer; // ����Ĳ���
	int m_countLocation; // һ��Ŀ�λ��

	/*��ʼ��ɨ������ķ�ʽ��
	flagΪ1������ֻҪAGV�����̻�ģʽ��
	flagΪ2����ֻҪUAV�����̻�ģʽ��
	flagΪ3����AGV��UAV�������̻�ģʽ*/
	int flag;

	
	MapPoint m_pointCurrent;

	// �洢�����ֿ�ָ�
	vector<CommandPoint> m_mission_point;
	
	// ��������洢�ֿ�ָ�������1�������ڵ�һ�㣬���������ڵڶ���
	vector<vector<CommandPoint>> m_commandPointGroup;

	// �洢��ͼ��
	vector<vector<double>> m_dict;

	// ���յõ�������㼯�ϣ��������������ִ洢
	vector<vector<TaskPoint>> m_taskPointGroup;


public:
	TaskPlanner(const int countLayer, const int countlation, 
				string command_path, string dict_path,
				double x, double y, double z, double angle);
	~TaskPlanner();

	// ��ȡָ��㲢�洢��m_mission_point
	void rawCommandRead(string command_path);

	// ��ȡ��ͼӳ���
	void mapPointRead(string dict_path);

	static bool cmp(CommandPoint a, CommandPoint b);

	// ��ָ��ĵ��������
	void sortVector(vector<CommandPoint>& mission_point);
	

	// ��ָ�����������зָ���洢��m_commandPointGroup
	void splitByfacade(vector<CommandPoint>& mission_point);

	// ���ݵ��������ָ��ϵõ���������������
	vector<TaskPoint> sortSingleFacade(vector<CommandPoint>& singleCommandGroup, MapPoint& pointCurrent);

	// ���ݵ������浥��ָ���ģ���С��λ�õ��С��������״̬flag���õ������
	vector<TaskPoint> singleLayerGenerate(vector<CommandPoint>& singlelayerCommandGroup, MapPoint& pointCurrent, int flag);
	
	// ָ��͵�ͼ���ӳ��
	void mappingConvert(int facade, int layer, int location, double& x, double& y, double& z, double& angle);

	// ����ģ���pointCurrent���õ��Ͻ��ĵ�
	MapPoint compareTwoEnds(MapPoint mp1, MapPoint mp2, MapPoint pointCurrent);

	MapPoint mapPointGenerate(double x, double y, double z, double angle);

	TaskPoint taskPointGenerate(double x, double y, double z, double angle, int f1, int f2, int mode);

	vector<vector<TaskPoint>> getTaskPointGroup() { return m_taskPointGroup; }
};


#endif // !__TASKPLANNER_H__