#include "TaskPlanner.h"

TaskPlanner::TaskPlanner(const int countLayer, const int countLation,
	string command_path, string dict_path,
    double x, double y, double z, double angle) {
	m_countLayer = countLayer;
	m_countLocation = countLation;
    rawCommandRead(command_path);
    mapPointRead(dict_path);
    sortVector(m_mission_point);
    splitByfacade(m_mission_point);
    m_pointCurrent.x = x;
    m_pointCurrent.y = y;
    m_pointCurrent.z = z;
    m_pointCurrent.angle = angle;
    for (int i = 0; i < m_commandPointGroup.size(); i++) {
        vector<TaskPoint> currentGroup = sortSingleFacade(m_commandPointGroup[i], m_pointCurrent);
        m_taskPointGroup.push_back(currentGroup);
    }
}

void TaskPlanner::rawCommandRead(string command_path) {
    ifstream mission_task;
    unsigned int count = 0;
    mission_task.open(command_path);
    while (!mission_task.eof() && mission_task.peek() != EOF)
    {
        string nextline;
        getline(mission_task, nextline);
        if (nextline == "end")
            break;
        istringstream in(nextline);
        CommandPoint commandPoint;
        in >> commandPoint.facade >> commandPoint.layer >> commandPoint.location;
        m_mission_point.push_back(commandPoint);
    }
    mission_task.close();
}

void TaskPlanner::mapPointRead(string dict_path) {
    ifstream dict_file;
    dict_file.open(dict_path);
    while (!dict_file.eof() && dict_file.peek() != EOF)
    {
        string nextline;
        getline(dict_file, nextline);
        if (nextline == "end")
            break;
        istringstream in(nextline);
        vector<double> temp;
        double x, y, z, angle;
        in >> x >> y >> z >> angle;
        temp.push_back(x);
        temp.push_back(y);
        temp.push_back(z);
        temp.push_back(angle);
        m_dict.push_back(temp);
    }
    dict_file.close();
}

bool TaskPlanner::cmp(CommandPoint a, CommandPoint b) {
    if (a.facade == b.facade) {
        if (a.layer == b.layer) {
            return a.location < b.location;
        }
        return a.layer < b.layer;
    }
    return a.facade < b.facade;
}

void TaskPlanner::sortVector(vector<CommandPoint>& mission_point) {
    int size = mission_point.size();
    CommandPoint* array = new CommandPoint[size];
    for (int i = 0; i < size; i++) {
        array[i] = mission_point[i];
        cout << " facade: " << mission_point[i].facade << " layer: " << mission_point[i].layer << " location: " << mission_point[i].location << endl;
    }
    sort(array, array + size, cmp);
    for (int i = 0; i < size; i++) {
        mission_point[i] = array[i];
        cout << " facadeSorted: " << mission_point[i].facade << " layerSorted: " << mission_point[i].layer << " locationSorted: " << mission_point[i].location << endl;
    }
}

void TaskPlanner::splitByfacade(vector<CommandPoint>& mission_point) {
    vector<CommandPoint> currentGroup;
    int size = m_mission_point.size();
    for (int i = 0; i < size; i++) {
        if (i == 0 || mission_point[i].facade != mission_point[i - 1].facade) {
            vector<CommandPoint> preGroup(currentGroup);
            m_commandPointGroup.push_back(preGroup);
            currentGroup.clear();
            currentGroup.push_back(mission_point[i]);
        }
        else {
            currentGroup.push_back(mission_point[i]);
        }
    }
    m_commandPointGroup.push_back(currentGroup);
    m_commandPointGroup.erase(m_commandPointGroup.begin());
}

vector<TaskPoint> TaskPlanner::sortSingleFacade(vector<CommandPoint>& singleCommandGroup, MapPoint& pointCurrent){
    vector<TaskPoint> singleTaskPointGroup;
<<<<<<< HEAD
    vector<CommandPoint> singlelayerCommandGroup; // 单层货架指令集 
    vector<vector<CommandPoint>> singlefacadeCommandGroup; // 单个立面指令集
    TaskPoint taskPointFacadeFirst, taskPointFacadeLast, taskPointFacadeLastLast;
    bool searchAllFacades = false; // 是否搜索整个立面

    // 将一个立面指令集按层分开
=======
    vector<CommandPoint> singlelayerCommandGroup;
    vector<vector<CommandPoint>> singlefacadeCommandGroup;
    TaskPoint taskPointFacadeFirst, taskPointFacadeLast;
    bool searchAllFacades = false;
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
    singlelayerCommandGroup.push_back(singleCommandGroup[0]);
    for (int i = 1; i < singleCommandGroup.size(); i++) {
        if (singleCommandGroup[i - 1].layer == singleCommandGroup[i].layer) {
            singlelayerCommandGroup.push_back(singleCommandGroup[i]);
        }
        else {
            singlefacadeCommandGroup.push_back(singlelayerCommandGroup);
            singlelayerCommandGroup.clear();
            singlelayerCommandGroup.push_back(singleCommandGroup[i]);
        }
    }
    singlefacadeCommandGroup.push_back(singlelayerCommandGroup);

    for (int i = 0; i < singlefacadeCommandGroup.size(); i++) {
        for (int j = 0; j < singlefacadeCommandGroup[i].size(); j++) {
            cout << " facade: " << singlefacadeCommandGroup[i][j].facade << " layer: " << singlefacadeCommandGroup[i][j].layer << " location: " << singlefacadeCommandGroup[i][j].location << " ";
        }
        cout << endl;
    }


<<<<<<< HEAD
    // 是否搜索整个立面
=======
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
    for (int i = 0; i < singlefacadeCommandGroup.size(); i++) {
        for (int j = 0; j < singlefacadeCommandGroup[i].size(); j++) {
            if (singlefacadeCommandGroup[i][j].layer == -1) {
                searchAllFacades = true;
                break;
            }
        }
    }

    if (searchAllFacades) {
        cout << "SearchAllFacades" << endl;
        int index = 1;
        while(index < m_countLayer) {
            MapPoint mp1, mp2;
            mappingConvert(singleCommandGroup[0].facade - 1, index, 0, mp1.x, mp1.y, mp1.z, mp1.angle);//obtain mp1 based on command
            mappingConvert(singleCommandGroup[0].facade - 1, index, m_countLocation - 1, mp2.x, mp2.y, mp2.z, mp2.angle);//obtain mp2 based on cammand
            MapPoint mpNear = compareTwoEnds(mp1, mp2, pointCurrent);// obtain the nearer map point based on current point
            MapPoint mpFar;
            if (mpNear.x == mp1.x && mpNear.y == mp1.y) {
                mpFar = mapPointGenerate(mp2.x, mp2.y, mp2.z, mp2.angle);
            }
            else {
                mpFar = mapPointGenerate(mp1.x, mp1.y, mp1.z, mp1.angle);
            }
            TaskPoint taskPointNear, taskPointRevise, taskPointFar;
            if(index == 1) {
                taskPointNear = taskPointGenerate(mpNear.x, mpNear.y, mpNear.z, mpNear.angle, 0, 0, 1);
                taskPointRevise = taskPointGenerate(mpNear.x, mpNear.y, mpNear.z, mpNear.angle, 0, 0, 2);
                taskPointFar = taskPointGenerate(mpFar.x, mpFar.y, mpFar.z, mpFar.angle, 1, 1, 3);
            }
            else {
                taskPointNear = taskPointGenerate(mpNear.x, mpNear.y, mpNear.z, mpNear.angle, 0, 0, 1);
                taskPointRevise = taskPointGenerate(mpNear.x, mpNear.y, mpNear.z, mpNear.angle, 0, 0, 2);
                taskPointFar = taskPointGenerate(mpFar.x, mpFar.y, mpFar.z, mpFar.angle, 0, 1, 3);
            }
            singleTaskPointGroup.push_back(taskPointNear);
            singleTaskPointGroup.push_back(taskPointRevise);
            singleTaskPointGroup.push_back(taskPointFar);
            index++;
            pointCurrent = mapPointGenerate(mpFar.x, mpFar.y, mpFar.z, mpFar.angle);
        }
        searchAllFacades = false;
    }
    else {
        MapPoint FirstPoint, LastPoint;
        mappingConvert(singlefacadeCommandGroup[0][0].facade - 1, 0, 0, FirstPoint.x, FirstPoint.y, FirstPoint.z, FirstPoint.angle);
        mappingConvert(singlefacadeCommandGroup[0][0].facade - 1, 0, m_countLocation-1, LastPoint.x, LastPoint.y, LastPoint.z, LastPoint.angle);
<<<<<<< HEAD
        LastPoint.angle = 270;
=======
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
        MapPoint mpFirst = compareTwoEnds(FirstPoint, LastPoint, pointCurrent);
        MapPoint mpLast;
        if (mpFirst.x == FirstPoint.x && mpFirst.y == FirstPoint.y) {
            mpLast = mapPointGenerate(LastPoint.x, LastPoint.y, LastPoint.z, LastPoint.angle);
        }
        else {
            mpLast = mapPointGenerate(FirstPoint.x, FirstPoint.y, FirstPoint.z, FirstPoint.angle);
        }
        taskPointFacadeFirst = taskPointGenerate(mpFirst.x, mpFirst.y, mpFirst.z + 1.5, mpFirst.angle, 0, 0, 1);
        taskPointFacadeLast = taskPointGenerate(mpLast.x, mpLast.y, mpLast.z + 1.5, mpLast.angle, 0, 0, 1);
<<<<<<< HEAD
        taskPointFacadeLastLast = taskPointGenerate(mpLast.x, mpLast.y, mpLast.z + 1.5, mpLast.angle, 0, 0, 2);
=======
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
        singleTaskPointGroup.push_back(taskPointFacadeFirst);
        for (int i = 0; i < singlefacadeCommandGroup.size(); i++) {
            if (singlefacadeCommandGroup[i][0].layer == 1) {
                if (i + 1 < singlefacadeCommandGroup.size() && (singlefacadeCommandGroup[i + 1][0].location == -1 || singlefacadeCommandGroup[i][0].location == -1)) {
                    flag = 3;
                    singlefacadeCommandGroup[i + 1][0].location = -1;
                    vector<TaskPoint> v1 = singleLayerGenerate(singlefacadeCommandGroup[i + 1], pointCurrent, flag);
                    singleTaskPointGroup.insert(singleTaskPointGroup.end(), v1.begin(), v1.end());
                    i++;
                    cout << " flag: " << flag << endl;
                    cout << " current.x: " << pointCurrent.x << " current.y " << pointCurrent.y << " current.z: " << pointCurrent.z << endl;
                    continue;
                }
                flag = 1;
                vector<TaskPoint> v1 = singleLayerGenerate(singlefacadeCommandGroup[i], pointCurrent, flag);
                singleTaskPointGroup.insert(singleTaskPointGroup.end(), v1.begin(), v1.end());
            }
            else {
                flag = 2;
                vector<TaskPoint> v1 = singleLayerGenerate(singlefacadeCommandGroup[i], pointCurrent, flag);
                singleTaskPointGroup.insert(singleTaskPointGroup.end(), v1.begin(), v1.end());
            }
            cout << " flag: " << flag << endl;
            cout << " current.x: " << pointCurrent.x << " current.y " << pointCurrent.y << " current.z: " << pointCurrent.z << endl;
        }
    }
    int size = singleTaskPointGroup.size();
    TaskPoint taskPointFacadeEnd = taskPointGenerate(singleTaskPointGroup[size - 1].x, singleTaskPointGroup[size - 1].y, 1.5, singleTaskPointGroup[size - 1].angle, 0, 0, 1);
    singleTaskPointGroup.push_back(taskPointFacadeEnd);
    singleTaskPointGroup.push_back(taskPointFacadeLast);
<<<<<<< HEAD
    singleTaskPointGroup.push_back(taskPointFacadeLastLast);
=======
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
    return singleTaskPointGroup;
}

vector<TaskPoint> TaskPlanner::singleLayerGenerate(vector<CommandPoint>& singlelayerCommandGroup, MapPoint& pointCurrent, int flag) {
    vector<TaskPoint> singlelayerTaskPointGroup;
    double layerGap = 1.5;
    switch (flag) {
    case 1:
        if (singlelayerCommandGroup[0].location == -1) {
            CommandPoint command = singlelayerCommandGroup[0];
            MapPoint mp1, mp2;
            mappingConvert(command.facade - 1, command.layer - 1, 0, mp1.x, mp1.y, mp1.z, mp1.angle);//obtain mp1 based on command
            mappingConvert(command.facade - 1, command.layer - 1, m_countLocation - 1, mp2.x, mp2.y, mp2.z, mp2.angle);//obtain mp2 based on cammand
            MapPoint mpNear = compareTwoEnds(mp1, mp2, pointCurrent);// obtain the nearer map point based on current point
            MapPoint mpFar;
            //obtain the other map point
            if (mpNear.x == mp1.x && mpNear.y == mp1.y)
                mpFar = mapPointGenerate(mp2.x, mp2.y, mp2.z, mp2.angle);
            else
                mpFar = mapPointGenerate(mp1.x, mp1.y, mp1.z, mp1.angle);
            TaskPoint taskPointNear = taskPointGenerate(mpNear.x, mpNear.y, mpNear.z + layerGap, mpNear.angle, 0, 0, 1);
            TaskPoint taskPointFar = taskPointGenerate(mpFar.x, mpFar.y, mpFar.z + layerGap, mpFar.angle, 1, 0, 3);
            singlelayerTaskPointGroup.push_back(taskPointNear);
            singlelayerTaskPointGroup.push_back(taskPointFar);
            pointCurrent = mapPointGenerate(mpFar.x, mpFar.y, mpFar.z, mpFar.angle);
        }
        else {
            int n = singlelayerCommandGroup.size();
            vector<MapPoint> singleMapPointGroup;
            for (int i = 0; i < n; i++) {
                CommandPoint command = singlelayerCommandGroup[i];
                MapPoint mp;
                mappingConvert(command.facade - 1, command.layer - 1, command.location - 1, mp.x, mp.y, mp.z, mp.angle);//obtain every mp based on command
                singleMapPointGroup.push_back(mp);
<<<<<<< HEAD
            } 
=======
            }
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
            MapPoint mp1 = singleMapPointGroup[0];
            MapPoint mp2 = singleMapPointGroup[n - 1];
            MapPoint mpNear = compareTwoEnds(mp1, mp2, pointCurrent);// obtain the nearer map point based on current point
            bool isAscend = false; // check whether we need to sort the singleTaskPointGroup ascend or descend
            if (mpNear.x == mp1.x && mpNear.y == mp1.y)
                isAscend = true;
            if (isAscend) {
                for (int i = 0; i < n; i++) {
                    MapPoint mp = singleMapPointGroup[i];
                    TaskPoint targetTask = taskPointGenerate(mp.x, mp.y, mp.z + layerGap, mp.angle, 0, 0, 1);
                    singlelayerTaskPointGroup.push_back(targetTask);
                    targetTask = taskPointGenerate(mp.x, mp.y, mp.z + layerGap, mp.angle, 1, 0, 5);
                    singlelayerTaskPointGroup.push_back(targetTask);
                }
                pointCurrent = mapPointGenerate(singleMapPointGroup[n - 1].x, singleMapPointGroup[n - 1].y, singleMapPointGroup[n - 1].z, singleMapPointGroup[n - 1].angle);
            }
            else {
                for (int i = n - 1; i >= 0; i--) {
                    MapPoint mp = singleMapPointGroup[i];
                    TaskPoint targetTask = taskPointGenerate(mp.x, mp.y, mp.z + layerGap, mp.angle, 0, 0, 1);
                    singlelayerTaskPointGroup.push_back(targetTask);
                    targetTask = taskPointGenerate(mp.x, mp.y, mp.z + layerGap, mp.angle, 1, 0, 5);
                    singlelayerTaskPointGroup.push_back(targetTask);
                }
                pointCurrent = mapPointGenerate(singleMapPointGroup[0].x, singleMapPointGroup[0].y, singleMapPointGroup[0].z, singleMapPointGroup[0].angle);
            }
        }
        break;
    case 2:
        if (singlelayerCommandGroup[0].location == -1) {
            CommandPoint command = singlelayerCommandGroup[0];
            MapPoint mp1, mp2;
            mappingConvert(command.facade - 1, command.layer - 1, 0, mp1.x, mp1.y, mp1.z, mp1.angle);//obtain mp1 based on command
            mappingConvert(command.facade - 1, command.layer - 1, m_countLocation - 1, mp2.x, mp2.y, mp2.z, mp2.angle);//obtain mp2 based on cammand
            MapPoint mpNear = compareTwoEnds(mp1, mp2, pointCurrent);// obtain the nearer map point based on current point
            MapPoint mpFar;
            //obtain the other map point
            if (mpNear.x == mp1.x && mpNear.y == mp1.y)
                mpFar = mapPointGenerate(mp2.x, mp2.y, mp2.z, mp2.angle);
            else
                mpFar = mapPointGenerate(mp1.x, mp1.y, mp1.z, mp1.angle);
            TaskPoint taskPointNear = taskPointGenerate(mpNear.x, mpNear.y, mpNear.z, mpNear.angle, 0, 0, 1);
            TaskPoint taskPointRevise = taskPointGenerate(mpNear.x, mpNear.y, mpNear.z, mpNear.angle, 0, 0, 2);
            TaskPoint taskPointFar = taskPointGenerate(mpFar.x, mpFar.y, mpFar.z, mpFar.angle, 0, 1, 3);
            singlelayerTaskPointGroup.push_back(taskPointNear);
            singlelayerTaskPointGroup.push_back(taskPointRevise);
            singlelayerTaskPointGroup.push_back(taskPointFar);
            pointCurrent = mapPointGenerate(mpFar.x, mpFar.y, mpFar.z, mpFar.angle);
        }
        else {
            int n = singlelayerCommandGroup.size();
            vector<MapPoint> singleMapPointGroup;
            for (int i = 0; i < n; i++) {
                CommandPoint command = singlelayerCommandGroup[i];
                MapPoint mp;
                mappingConvert(command.facade - 1, command.layer - 1, command.location - 1, mp.x, mp.y, mp.z, mp.angle);//obtain every mp based on command
                singleMapPointGroup.push_back(mp);
            }
            MapPoint mp1 = singleMapPointGroup[0];
            MapPoint mp2 = singleMapPointGroup[n - 1];
            MapPoint mpNear = compareTwoEnds(mp1, mp2, pointCurrent);// obtain the nearer map point based on current point
            bool isAscend = false; // check whether we need to sort the singleTaskPointGroup ascend or descend
            if (mpNear.x == mp1.x && mpNear.y == mp1.y)
                isAscend = true;
            if (isAscend) {
                for (int i = 0; i < n; i++) {
                    MapPoint mp = singleMapPointGroup[i];
<<<<<<< HEAD
                    TaskPoint targetTask = taskPointGenerate(mp.x, mp.y, layerGap, mp.angle, 0, 0, 1);
=======
                    TaskPoint targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 0, 1);
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
                    singlelayerTaskPointGroup.push_back(targetTask);
                    targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 1, 4);
                    singlelayerTaskPointGroup.push_back(targetTask);
                }
                pointCurrent = mapPointGenerate(singleMapPointGroup[n - 1].x, singleMapPointGroup[n - 1].y, singleMapPointGroup[n - 1].z, singleMapPointGroup[n - 1].angle);
            }
            else {
                for (int i = n - 1; i >= 0; i--) {
                    MapPoint mp = singleMapPointGroup[i];
<<<<<<< HEAD
                    TaskPoint targetTask = taskPointGenerate(mp.x, mp.y, layerGap, mp.angle, 0, 0, 1);
=======
                    TaskPoint targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 0, 1);
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
                    singlelayerTaskPointGroup.push_back(targetTask);
                    targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 1, 4);
                    singlelayerTaskPointGroup.push_back(targetTask);
                }
                pointCurrent = mapPointGenerate(singleMapPointGroup[0].x, singleMapPointGroup[0].y, singleMapPointGroup[0].z, singleMapPointGroup[0].angle);
            }
        }
        break;
    case 3:
        if (singlelayerCommandGroup[0].location == -1) {
            CommandPoint command = singlelayerCommandGroup[0];
            MapPoint mp1, mp2;
            mappingConvert(command.facade - 1, command.layer - 1, 0, mp1.x, mp1.y, mp1.z, mp1.angle);//obtain mp1 based on command
            mappingConvert(command.facade - 1, command.layer - 1, m_countLocation - 1, mp2.x, mp2.y, mp2.z, mp2.angle);//obtain mp2 based on cammand
            // cout << mp1.x << " " << mp1.y << " " << mp2.x << " " << mp2.y << endl;
            // cout << pointCurrent.x << pointCurrent.y << endl;
            MapPoint mpNear = compareTwoEnds(mp1, mp2, pointCurrent);// obtain the nearer map point based on current point
            MapPoint mpFar;
            //obtain the other map point
            if (mpNear.x == mp1.x && mpNear.y == mp1.y)
                mpFar = mapPointGenerate(mp2.x, mp2.y, mp2.z, mp2.angle);
            else
                mpFar = mapPointGenerate(mp1.x, mp1.y, mp1.z, mp1.angle);
            // cout << mpNear.x << mpNear.y << " " << mpFar.x << mpFar.y << endl;
            TaskPoint taskPointNear = taskPointGenerate(mpNear.x, mpNear.y, mpNear.z, mpNear.angle, 0, 0, 1);
            TaskPoint taskPointRevise = taskPointGenerate(mpNear.x, mpNear.y, mpNear.z, mpNear.angle, 0, 0, 2);
            TaskPoint taskPointFar = taskPointGenerate(mpFar.x, mpFar.y, mpFar.z, mpFar.angle, 1, 1, 3);
            singlelayerTaskPointGroup.push_back(taskPointNear);
            singlelayerTaskPointGroup.push_back(taskPointRevise);
            singlelayerTaskPointGroup.push_back(taskPointFar);
            pointCurrent = mapPointGenerate(mpFar.x, mpFar.y, mpFar.z, mpFar.angle);
        }
        else {
            int n = singlelayerCommandGroup.size();
            vector<MapPoint> singleMapPointGroup;
            for (int i = 0; i < n; i++) {
                CommandPoint command = singlelayerCommandGroup[i];
                MapPoint mp;
                mappingConvert(command.facade - 1, command.layer - 1, command.location - 1, mp.x, mp.y, mp.z, mp.angle);//obtain every mp based on command
                singleMapPointGroup.push_back(mp);
            }
            MapPoint mp1 = singleMapPointGroup[0];
            MapPoint mp2 = singleMapPointGroup[n - 1];
            MapPoint mpNear = compareTwoEnds(mp1, mp2, pointCurrent);// obtain the nearer map point based on current point
            bool isAscend = false; // check whether we need to sort the singleTaskPointGroup ascend or descend
            if (mpNear.x == mp1.x && mpNear.y == mp1.y)
                isAscend = true;
            if (isAscend) {
                for (int i = 0; i < n; i++) {
                    MapPoint mp = singleMapPointGroup[i];
                    TaskPoint targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 0, 1);
                    singlelayerTaskPointGroup.push_back(targetTask);
                    if (mp.z < 1)
                        targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 1, 0, 5);
                    else
                        targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 1, 4);
                    singlelayerTaskPointGroup.push_back(targetTask);
                }
                pointCurrent = mapPointGenerate(singleMapPointGroup[n - 1].x, singleMapPointGroup[n - 1].y, singleMapPointGroup[n - 1].z, singleMapPointGroup[n - 1].angle);
            }
            else {
                for (int i = n - 1; i >= 0; i--) {
                    MapPoint mp = singleMapPointGroup[i];
                    TaskPoint targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 0, 1);
                    singlelayerTaskPointGroup.push_back(targetTask);
                    if (mp.z < 1)
                        targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 1, 0, 5);
                    else
                        targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 1, 4);
                    singlelayerTaskPointGroup.push_back(targetTask);
                }
                pointCurrent = mapPointGenerate(singleMapPointGroup[0].x, singleMapPointGroup[0].y, singleMapPointGroup[0].z, singleMapPointGroup[0].angle);
            }
        }
        break;

    default:
        cout << "error" << endl;
        break;
    }
    return singlelayerTaskPointGroup;
}

void TaskPlanner::mappingConvert(int facade, int layer, int location, double& x, double& y, double& z, double& angle) {
    int index = facade * m_countLayer * m_countLocation + layer * m_countLocation + location; // it should be noticed that every base index for facade, layer, location is 0!!!
    x = m_dict[index][0];
    y = m_dict[index][1];
    z = m_dict[index][2];
    angle = m_dict[index][3];
}

MapPoint TaskPlanner::compareTwoEnds(MapPoint mp1, MapPoint mp2, MapPoint pointCurrent) {
    double delta1 = sqrt((mp1.x - pointCurrent.x) * (mp1.x - pointCurrent.x) + (mp1.y - pointCurrent.y) * (mp1.y - pointCurrent.y));
    double delta2 = sqrt((mp2.x - pointCurrent.x) * (mp2.x - pointCurrent.x) + (mp2.y - pointCurrent.y) * (mp2.y - pointCurrent.y));
    if (delta1 < delta2)
        return mp1;
    else
        return mp2;
}

MapPoint TaskPlanner::mapPointGenerate(double x, double y, double z, double angle) {
    MapPoint ret;
    ret.x = x;
    ret.y = y;
    ret.z = z;
    ret.angle = angle;
    return ret;
}

TaskPoint TaskPlanner::taskPointGenerate(double x, double y, double z, double angle, int f1, int f2, int mode) {
    TaskPoint ret;
    ret.x = x;
    ret.y = y;
    ret.z = z;
    ret.angle = angle;
    ret.f1 = f1;
    ret.f2 = f2;
    ret.mode = mode;
    return ret;
}

// int main(int argc, char** argv) {
//     TaskPlanner* task = new TaskPlanner(4, 8, "/home/cyc/guihua/taskA1.txt", "/home/cyc/guihua/map.txt", 0, 0, 0, 90);
//     vector<vector<TaskPoint>> a = task->getTaskPointGroup();

//     for(int i = 0; i < a.size(); i++) {
//         for(int j = 0; j < a[i].size(); j++) {
//             cout << "TaskPoint[" + to_string(i) + "][" + to_string(j) + "]: " << " x: " << a[i][j].x << " y: " << a[i][j].y << " z: " << a[i][j].z << " angle: " << a[i][j].angle << " f1: " << a[i][j].f1          << " f2: " << a[i][j].f2 << " mode: " << a[i][j].mode<<endl;
//         }
//         cout << endl;
//     }
//     return 0;
// }