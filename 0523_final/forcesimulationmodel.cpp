#include "forcesimulationmodel.h"

int ForceSimulationModel::cableNum = 0;
std::vector<std::vector<double>> ForceSimulationModel::endEffector_e = {{0.0}};
std::vector<double> ForceSimulationModel::curCableLen = std::vector<double>({0.0});
double ForceSimulationModel::pulleyRadius = 0.0;
std::vector<std::vector<double>> ForceSimulationModel::curAnchorPos = {{0.0}};
std::vector<double> ForceSimulationModel::endForce_e = {0.0};
std::vector<double> ForceSimulationModel::cableForce_e = {0.0};
std::vector<std::vector<double>> ForceSimulationModel::jacoTrans = {{0.0}};
std::vector<std::vector<double>> ForceSimulationModel::curContactPos = {{0.0}};


ForceSimulationModel::ForceSimulationModel(){
}

ForceSimulationModel::ForceSimulationModel(double _ctrlCycleMs, std::vector<std::vector<double>> endEffectorPos, double _weight,
                                                     std::vector<double> _curCableLen, std::vector<std::vector<double>> _curAnchorPos){
    if(timer){
        return;
    }
    else{
        timer = new QTimer(this);
        //定时器启动后，每过这段时间就自动发出一次 timeout() 信号
        timer->setInterval(ctrlCycleMs);
        connect(timer, &QTimer::timeout, this, &ForceSimulationModel::threadLoop);
    }

    ctrlCycleMs = _ctrlCycleMs;
    endEffector_e = endEffectorPos;
    curCableLen = _curCableLen;
    curAnchorPos = _curAnchorPos;
    cableNum = endEffectorPos.size();
    endForce_e.resize(6);
    endForce_e[2] = -_weight;

    //    qDebug() << "TEST1" << cableNum;
    //    for(int i=0;i<8;++i){
    //        qDebug() << "TEST2" << i << curCableLen->at(i);
    //        qDebug() << "TEST3" << i << curAnchorPos->at(i).at(0) << curAnchorPos->at(i).at(1)<< curAnchorPos->at(i).at(2);
    //    }
}

ForceSimulationModel::ForceSimulationModel(double _ctrlCycleMs, std::vector<std::vector<std::vector<double>>> endEffectorPos,
                                                     bool _useCam,std::string _sta)
    :ctrlCycleMs(_ctrlCycleMs), endEffector_eTemp(endEffectorPos), useCam(_useCam), sta(_sta){
    if(timer){
        return;
    }
    else{
        timer = new QTimer(this);
        timer->setTimerType(Qt::PreciseTimer);
        //定时器启动后，每过这段时间就自动发出一次 timeout() 信号
        timer->setInterval(ctrlCycleMs);
        connect(timer, &QTimer::timeout, this, &ForceSimulationModel::threadLoop);
    }
}

ForceSimulationModel::~ForceSimulationModel(){
}

void ForceSimulationModel::startTimer(){
    timer->start();
}

void ForceSimulationModel::stopTimer(){
    if(timer){
        timer->stop();
        timer->deleteLater();
        timer = nullptr;
    }
    isFirstLoop = true;
}

void ForceSimulationModel::setCableForceMinMax(double min, std::vector<std::vector<double>> max){
    cableMinF = min;
    cableMaxF = max;
}

void ForceSimulationModel::setPulleyRadius(double radius){
    pulleyRadius = radius;
    for(auto& dynModel : cdprDynModelVec){
        dynModel.setPulleyRadius(radius / 1000.0);
    }
}

void ForceSimulationModel::setDynPara(std::vector<double> mass, std::vector<std::vector<std::vector<double>>> inertiaLocal,
                                           std::vector<std::vector<std::vector<double>>> anchorsGlobal,std::vector<std::vector<std::vector<double>>> contactPointsLocal,
                                           std::vector<std::vector<double>> nr, std::vector<double> massAgent, std::vector<std::vector<std::vector<double>>> inertiaLocalAgent,
                                           std::vector<std::vector<double>> Imq, std::vector<std::vector<double>> Fvq,
                                           std::vector<std::vector<double>> Fcq,
                                           std::vector<std::vector<double>> md, std::vector<std::vector<double>> dd, std::vector<std::vector<double>> kd,
                                           std::string _sta){
    useDyn = true;
    cdprDynModelVec.resize(mass.size());
    for(int i=0;i<cdprDynModelVec.size();++i){
        cdprDynModelVec[i].setPhysicalParams(mass[i],MatrixFun::vecToMatrix(inertiaLocal[i]),MatrixFun::vecvecToVecv3d(anchorsGlobal[i]),
                                             MatrixFun::vecvecToVecv3d(contactPointsLocal[i]),MatrixFun::vecToVxd(nr[i]),massAgent[i],
                                             MatrixFun::vecToMatrix(inertiaLocalAgent[i]));
        cdprDynModelVec[i].setPulleyRadius(pulleyRadius / 1000.0);
        cdprDynModelVec[i].setWinchParams(MatrixFun::vecToVxd(Imq[i]).asDiagonal(),MatrixFun::vecToVxd(Fvq[i]).asDiagonal(),
                                          MatrixFun::vecToVxd(Fcq[i]).asDiagonal());
        cdprDynModelVec[i].setImpedanceGains(MatrixFun::vecToVxd(md[i]).asDiagonal(),MatrixFun::vecToVxd(dd[i]).asDiagonal(),
                                             MatrixFun::vecToVxd(kd[i]).asDiagonal());
        cdprDynModelVec[i].setOptParams(_sta,cableMinF,cableMaxF[i]);
        cdprDynModelVec[i].compTor = compTor;
    }
    curAnchorPosTemp = anchorsGlobal;// 用于仿真
}

void ForceSimulationModel::setDynImpCurDesTraj(std::vector<std::vector<double>> pos, std::vector<std::vector<double>> vel,
                                                    std::vector<std::vector<double>> acc){
    if(useDyn){
        for(int i=0;i<cdprDynModelVec.size();++i){
            cdprDynModelVec[i].setDesiredTrajectory(MatrixFun::vecToVxd(pos[i]),MatrixFun::vecToVxd(vel[i]),MatrixFun::vecToVxd(acc[i]));
        }
    }
}

void ForceSimulationModel::threadLoop(){
    QMutexLocker locker(&_dataMutex);

    static int count = 0;
    static double startTimeS,curTimeS;
    static QElapsedTimer loopTimer;
    if(isFirstLoop){
        qDebug()<< "Gravity compensation thread created.";
        emit gcStartSignal();
        loopTimer.start();
        startTimeS = (double)(loopTimer.elapsed())/1000.0;
        isFirstLoop = false;

        curPlatformVel.resize(endEffector_eTemp.size(),std::vector<double>(6,0.0));
        if(useSim){
            if(endEffector_eTemp.size() == 1){// end==1
                curPlatformPose = {{0.0,0.0,200.0,0.0,0.0,0.0}};
                curPlatformVel = {{0.0,0.0,0.0,0.0,0.0,0.0}};
                lastSimPos = curPlatformPose;
//                for(int i=0;i<lastSimPos.size();++i){
//                    for(int ii=0;ii<lastSimPos[i].size();++ii){
//                        lastSimPos[i][ii] *= 0.001;// mm->m
//                    }
//                }
                lastSimVel = {{0.0,0.0,0.0,0.0,0.0,0.0}};
                curPlatformForce = {{0.0,0.0,0.0,0.0,0.0,0.0}};
            }
            else if(endEffector_eTemp.size() == 2){// end==2
                curPlatformPose = {{-1000.0,0.0,500.0,0.0,0.0,0.0},{100.0,0.0,500.0,0.0,0.0,0.0}};
                curPlatformVel = {{0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0}};
                lastSimPos = curPlatformPose;
                lastSimVel = {{0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0}};
                curPlatformForce = {{0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0}};
            }
            // endCableForce = std::vector<std::vector<double>>(1,std::vector<double>(curAnchorPosTemp[0].size(),0.0));// 计算实际受力的，仿真用不上
        }
    }
    curTimeS = (double)(loopTimer.elapsed())/1000.0;

    int endNum = 0;
    QVector<QVector3D> trajPos3D,anchorPos3D,cableEndPos3D;// 仿真用
    // endNum = endEffector_eTemp.size();
    if(useCam || useSim){
        endNum = curPlatformPose.size();
    }
    else{
        endNum = curCableLenTemp.size();
        curPlatformPose.resize(endNum); 
    }

    static std::vector<double> cableForceVec;// 每次循环最后会重置resize(0)

    if(isUpdate){
        if(useDyn){
            if(isFirst){
                impLastPlatformPose.clear();
                impLastPlatformVel.clear();
                impLastTimeS = curTimeS;

                for(int i=0;i<endNum;++i){
                    cdprDynModelVec[i].optCableForce = optCableForce;
                    std::vector<double> curPlatformPoseTmp = curPlatformPose[i];
                    for(int ii=0;ii<curPlatformPoseTmp.size();++ii){// mm->m
                        if(ii<3){
                            curPlatformPoseTmp[ii] *= 0.001;
                        }
                    }

                    std::vector<double> PlatformForce;
                    for(int ii=0;ii<curPlatformForce[i].size();++ii)
                        PlatformForce.push_back(curPlatformForce[i][ii] + expPlatformForce[i][ii]);// expPlatformForce是补偿力，即虚拟的交互力

                    // 实时计算期望轨迹的零点
                    cdprDynModelVec[i].setDesiredTrajectory(MatrixFun::vecToVxd(curPlatformPoseTmp),MatrixFun::vecToVxd(std::vector<double>(6,0.0)),
                                                            MatrixFun::vecToVxd(std::vector<double>(6,0.0)));
                    // 计算绳力
                    // 这里还需要改：curPlatformForce(PlatformForce)实际是六维力传感器读数Fsensor，需要转换成Fext（文档里带r的是智能体的，这里的是整体的，注意区分）
                    VectorXd cableForceVxd = cdprDynModelVec[i].calculateCableForce(MatrixFun::vecToVxd(curPlatformPoseTmp),
                                                                                    MatrixFun::vecToVxd(std::vector<double>(6,0.0)),
                                                                                    MatrixFun::vecToVxd(std::vector<double>(6,0.0)),curTimeS-impLastTimeS);

//                    cdprDynModelVec[i].setDesiredTrajectory(MatrixFun::vecToVxd({-1.460480000000000,0.0,1.0,0.0,0.0,0.0}),
//                                                            MatrixFun::vecToVxd({1.382284764802552,0.0,0.0,0.0,0.0,0.0}),
//                                                            MatrixFun::vecToVxd({0.230611165719344,0.0,0.0,0.0,0.0,0.0}));
//                    VectorXd cableForceVxd = cdprDynModelVec[i].calculateTauM(MatrixFun::vecToVxd({-1.460479053629516,-1.703088517640053e-12,1.000000000060142,2.968431088906092e-10,1.878283595921353e-09,-1.961565184268136e-10}),
//                                                                              MatrixFun::vecToVxd({1.382285106493228,1.184897187127425e-12,-1.842970220877963e-11,-1.209265969085051e-09,-2.635536539531028e-09,4.195661636098623e-10}),
//                                                                              MatrixFun::vecToVxd({0.012652105448478,1.777023220583228e-07,1.264265847567003e-08,-3.630176621165348e-09,2.530420149973241e-04,7.642507770420161e-12}),
//                                                                              0.001);
                    for(int ii=0;ii<cableForceVxd.size();++ii){
                        cableForceVec.push_back(cableForceVxd[ii]);
                    }
                }

                isFirst = false;
            }
            else{
                for(int i=0;i<endNum;++i){
                    std::vector<double> curPlatformAccTmp(6);
                    for(int ii=0;ii<6;++ii){
                        curPlatformVel[i][ii] = (curPlatformPose[i][ii]-impLastPlatformPose[i][ii])/(curTimeS-impLastTimeS);// ***速度差分
                        curPlatformAccTmp[ii] = (curPlatformVel[i][ii]-impLastPlatformVel[i][ii])/(curTimeS-impLastTimeS);
                    }

                    std::vector<double> curPlatformPoseTmp = curPlatformPose[i];
                    std::vector<double> curPlatformVelTmp = curPlatformVel[i];
                    for(int ii=0;ii<6;++ii){
                        if(ii<3){// mm->m
                            curPlatformPoseTmp[ii] *= 0.001;
                            curPlatformVelTmp[ii] *= 0.001;
                        }
                    }

//                    cdprDynModelVec[i].setDesiredTrajectory(MatrixFun::vecToVxd(curPlatformPoseTmp),MatrixFun::vecToVxd(curPlatformVelTmp),
//                                                            MatrixFun::vecToVxd(curPlatformAccTmp));

                    std::vector<double> PlatformForce;
                    for(int ii=0;ii<curPlatformForce[i].size();++ii)
                        PlatformForce.push_back(curPlatformForce[i][ii] + expPlatformForce[i][ii]);// expPlatformForce是补偿力，即虚拟的交互力
                    // 实时计算期望轨迹
                    // 这里还需要改：curPlatformForce(PlatformForce)实际是六维力传感器读数Fsensor，需要转换成F_r_ext（即智能体所受六维力）
                    cdprDynModelVec[i].updateMicrogravityTrajectory(MatrixFun::vecToVxd(PlatformForce),curTimeS-impLastTimeS);
                    // 计算绳力
                    // 这里还需要改：curPlatformForce(PlatformForce)实际是六维力传感器读数Fsensor，需要转换成Fext（即整体所受六维力。文档里带r的是智能体的，这里的是整体的，注意区分）
                    VectorXd cableForceVxd = cdprDynModelVec[i].calculateCableForce(MatrixFun::vecToVxd(curPlatformPoseTmp),// {-0.022468,-0.0107538,0.784688,-0.030594,-0.0669028,-0.0977447}
                                                                                    MatrixFun::vecToVxd(curPlatformVelTmp),// std::vector<double>(6,0.0)
                                                                                    MatrixFun::vecToVxd(PlatformForce),curTimeS-impLastTimeS);
                    if(cdprDynModelVec[i].resultCode < 0 || cdprDynModelVec[i].resultCode < 0){
                        displayInfoSignal((QString("GravityCompensationThread error: nlopt optimization fail:%1").arg(cdprDynModelVec[i].resultCode)).toStdString(),"error");
                    }

//                    cdprDynModelVec[i].setDesiredTrajectory(MatrixFun::vecToVxd({-1.460480000000000,0.0,1.0,0.0,0.0,0.0}),
//                                                            MatrixFun::vecToVxd({1.382284764802552,0.0,0.0,0.0,0.0,0.0}),
//                                                            MatrixFun::vecToVxd({0.230611165719344,0.0,0.0,0.0,0.0,0.0}));
//                    VectorXd cableForceVxd = cdprDynModelVec[i].calculateTauM(MatrixFun::vecToVxd({-1.460479053629516,-1.703088517640053e-12,1.000000000060142,2.968431088906092e-10,1.878283595921353e-09,-1.961565184268136e-10}),
//                                                                              MatrixFun::vecToVxd({1.382285106493228,1.184897187127425e-12,-1.842970220877963e-11,-1.209265969085051e-09,-2.635536539531028e-09,4.195661636098623e-10}),
//                                                                              MatrixFun::vecToVxd({0.012652105448478,1.777023220583228e-07,1.264265847567003e-08,-3.630176621165348e-09,2.530420149973241e-04,7.642507770420161e-12}),
//                                                                              0.001);
                    for(int ii=0;ii<cableForceVxd.size();++ii){
                        cableForceVec.push_back(cableForceVxd[ii]);
                    }// qDebug() << cableForceVec;
                }
            }

            if(!cableForceVec.empty()){
                std::vector<std::vector<double>> PlatformForce = curPlatformForce;
                for(int i=0;i<PlatformForce.size();++i){
                    for(int ii=0;ii<PlatformForce[i].size();++ii){
                        PlatformForce[i][ii] += expPlatformForce[i][ii];// expPlatformForce是补偿力，即虚拟的交互力
                    }
                }

                emit(gcCableForceResultSignal(cableForceVec,curPlatformPose,PlatformForce));
                qDebug() << "GC dyn runtime(s):" << curTimeS-impLastTimeS;

                // 写的不对。应当用动力学（公式2），而非直接Fext/m。现在这种写法，相当于默认绳力抵消了重力，而且忽略了离心力项
                // 仿真
                if(useSim){
                    for(int i=0;i<endNum;++i){
                        curAnchorPos.resize(0);
                        endEffector_e.resize(0);
                        cableNum = curAnchorPosTemp[i].size();
                        for(int anchorIndex = 0;anchorIndex<cableNum;++anchorIndex){
                            curAnchorPos.push_back(curAnchorPosTemp[i][anchorIndex]);
                            endEffector_e.push_back(endEffector_eTemp[i][anchorIndex]);
                        }
                        curContactPos = getContactPose(curPlatformPose[i]);

                        for (int icable = 0; icable < cableNum; icable++){
                            anchorPos3D.push_back({(float)curAnchorPos[icable][0]/1000.0f,(float)curAnchorPos[icable][1]/1000.0f,(float)curAnchorPos[icable][2]/1000.0f});
                            cableEndPos3D.push_back({(float)curContactPos[icable][0]/1000.0f,(float)curContactPos[icable][1]/1000.0f,(float)curContactPos[icable][2]/1000.0f});
                        }
                    }

                    std::vector<std::vector<double>> simAcc(endNum,std::vector<double>(6,0.0)),
                            simVel(endNum,std::vector<double>(6,0.0)),simPos(endNum,std::vector<double>(6,0.0));
                    for(int i=0;i<endNum;++i){
                        for(int ii=0;ii<6;++ii){
                            // 在仿真中，将curPlatformForce认作交互力
                            if(ii==2){
                                simAcc[i][ii] = PlatformForce[i][ii]/(weight[i]/9.8);// z方向
//                                simAcc[i][ii] = (PlatformForce[i][ii]-weight[i])/(weight[i]/9.8);//z方向
                            }
                            else{
                                simAcc[i][ii] = PlatformForce[i][ii]/(weight[i]/9.8);
                            }
                            simVel[i][ii] = lastSimVel[i][ii]+simAcc[i][ii]*1000.0*ctrlCycleMs/1000.0;//统一mm
                            simPos[i][ii] = lastSimPos[i][ii]+simVel[i][ii]*ctrlCycleMs/1000.0;
                        }
                        if(!useEndRot){
                            for(int ii=0;ii<3;++ii)
                                simPos[i][ii+3] = 0.0;
                        }
                        trajPos3D.push_back({(float)simPos[i][0]/1000.0f,(float)simPos[i][1]/1000.0f,(float)simPos[i][2]/1000.0f});
                    }
                    qDebug() << "simVel:" << simVel;
                    qDebug() << "simPos:" << simPos;
                    lastSimVel = simVel;
                    lastSimPos = simPos;

                    curPlatformPose = lastSimPos;// qDebug() << trajPos3D << anchorPos3D << cableEndPos3D;

                    emit update3DViewerSignal({{}},trajPos3D,anchorPos3D,cableEndPos3D);
                }
            }

            cableForceVec.resize(0);// 重置
            impLastTimeS = curTimeS;
            impLastPlatformPose = curPlatformPose;
            impLastPlatformVel = curPlatformVel;
        }



        else{
            std::vector<std::vector<std::vector<double>>> curCableVec,expCableVec;// 各平台的初始绳长向量和当前绳长向量

            endForceByCable_a.resize(0);// 重置
            if(isFirst){
                homePlatformPose.resize(endNum);
                homeCableVecTemp.resize(endNum);
            }

            //        forceDistribution({0.0,0.0,400.0,0.034374,0.00990581,0.000173473});
            //        forceDistribution({-0.141838,-1.51342,399.907,-0.00232259,-0.00226595,-0.00444735});
            // 由于是基于第三方matlab的逻辑修改，因此代码上稍显臃肿
            for(int i=0;i<endNum;++i){// 输入平台位姿，输出其上各绳索分配的力
                //                curAnchorPos = curAnchorPosTemp[i];
                //                cableNum = curAnchorPosTemp[i].size();
                //                endEffector_e = endEffector_eTemp[i];

                qDebug() << "EndIndex:" << i << "\n";
                qDebug() << "expPlatformForce(NM):" << expPlatformForce[i] << "\n";

                // 重置
                cableForce_e.resize(0);
                jacoTrans.resize(0);
                curAnchorPos.resize(0);
                endEffector_e.resize(0);
                curEndCableForce.resize(0);
                curCableMaxF.resize(0);
                cableNum = curAnchorPosTemp[i].size();
                for(int anchorIndex = 0;anchorIndex<cableNum;++anchorIndex){
                    curAnchorPos.push_back(curAnchorPosTemp[i][anchorIndex]);
                    endEffector_e.push_back(endEffector_eTemp[i][anchorIndex]);
                    curEndCableForce.push_back(endCableForce[i][anchorIndex]);
                    curCableMaxF.push_back(cableMaxF[i][anchorIndex]);
                }// qDebug() << "curAnchorPosTemp:" << curAnchorPosTemp;qDebug() << "curAnchorPosTemp2:" << curAnchorPosTemp2;qDebug() << "curAnchorPos:" << curAnchorPos;
                if(useCam || useSim){
                    curContactPos = getContactPose(curPlatformPose[i]);
                }
                else{
                    curCableLen = curCableLenTemp[i];
                    qDebug() << "curCableLen:" << curCableLen << "\n";
                    curPlatformPose[i] = kine();// curContactPos在此该函数内计算
                }
                if(!useEndRot){
                    for(int ii=0;ii<3;++ii)
                        curPlatformPose[i][ii+3] = 0.0;
                }

                endForce_e = expPlatformForce[i];

                std::vector<double> cableForceVecTemp;
                cableForceVecTemp = forceDistribution(curPlatformPose[i]);
                for(int anchorIndex = 0;anchorIndex<cableNum;++anchorIndex){
                    cableForceVec.push_back(cableForceVecTemp[anchorIndex]);
                }

                // 计算随着末端动平台旋转，不产生扭矩时对应的绳索向量
                if(calGCCableVec){
                    // 求当前绳索向量，并与初始绳长向量进行对比，判断绳索是发生了顺时针转动还是逆时针转动
                    // 若顺时针转动，则让对应锚点座也进行顺时针转动，以防止产生扭矩
                    // 锚点座转动幅度通过速度环与PID进行控制，根据绳索转动幅度来决定锚点座转动幅度大小（在PLC中实现）
                    if(isFirst){
                        std::vector<std::vector<double>> temp;
                        MatrixFun::transpose(MatrixFun::getRots(curPlatformPose[i]),temp);
                        homeRotT.push_back(temp);
                    }
                    curCableVecTemp.resize(0);
                    expCableVecTemp.resize(0);// qDebug() << "TEST:" << homeCableVecTemp;
                    for (int icable = 0; icable < cableNum; icable++){// qDebug() << icable;
                        const std::vector<double> contactPoint = {
                            curContactPos[icable][0],
                            curContactPos[icable][1],
                            curContactPos[icable][2]
                        };
                        const MatrixFun::CableLengthResult cableGeometry =
                                MatrixFun::cableLengthCalculate(contactPoint, curAnchorPos[icable], pulleyRadius);
                        const std::vector<double> tangentPoint = {
                            cableGeometry.tangentPoint[0],
                            cableGeometry.tangentPoint[1],
                            cableGeometry.tangentPoint[2]
                        };
                        if(isFirst){
                            homeCableVecTemp[i].push_back(MatrixFun::vecDiff(contactPoint, tangentPoint));// 获取初始绳索向量
                        }
                        // homeRotT意义：若起始末端动平台的存在一定姿态，将以当前姿态作为home，而不是以没有姿态时作为home
                        std::vector<std::vector<double>> temp = MatrixFun::matrixMulti(MatrixFun::matrixMulti(homeRotT[i],MatrixFun::getRots(curPlatformPose[i])),
                                                                                       {{homeCableVecTemp[i][icable][0]},{homeCableVecTemp[i][icable][1]},{homeCableVecTemp[i][icable][2]}});
                        expCableVecTemp.push_back({temp[0][0],temp[1][0],temp[2][0]});
                        curCableVecTemp.push_back(MatrixFun::vecDiff(contactPoint, tangentPoint));// 获取当前绳索向量
                        if(useSim){
                            anchorPos3D.push_back({(float)curAnchorPos[icable][0]/1000.0f,(float)curAnchorPos[icable][1]/1000.0f,(float)curAnchorPos[icable][2]/1000.0f});
                            cableEndPos3D.push_back({(float)curContactPos[icable][0]/1000.0f,(float)curContactPos[icable][1]/1000.0f,(float)curContactPos[icable][2]/1000.0f});
                        }
                    }
                    curCableVec.push_back(curCableVecTemp);
                    expCableVec.push_back(expCableVecTemp);
                }

                // qDebug() << "PlatformPose:" << curPlatformPose[i] << "\n";
                //            qDebug() << homeCableVecTemp;qDebug() << curCableVecTemp;

                if(isFirst){
                    homePlatformPose[i] = curPlatformPose[i];
                }
            }

            if(!cableForceVec.empty()){

                if(calGCCableVec){
                    // 第一次将被用于设置初始绳长向量，而不用于计算
                    if(!isFirst){// 求初始绳长向量和当前绳长向量绕Z轴的转动大小
                        std::vector<std::vector<double>> cableVecXYThetaTemp(endNum,std::vector<double>(expCableVec[0].size(),0.0));
                        for(int i=0;i<endNum;++i){
                            for(int ii=0;ii<expCableVec[i].size();++ii){
                                cableVecXYThetaTemp[i][ii] = MatrixFun::vecTheta(std::vector<double>({expCableVec[i][ii][0],expCableVec[i][ii][1]}),
                                                                                 std::vector<double>({curCableVec[i][ii][0],curCableVec[i][ii][1]}));
                                // 若叉乘小于0，说明当前绳长向量在期望绳长向量的顺时针方向。此时锚点座需要往逆时针方向移动
                                // 又由于逆时针为锚点座正方向，且PLC中PID的期望值是0，实际值为该转角，并进行速度环控制，所以取负，即Δu为正，即往逆时针方向运动
                                if(MatrixFun::vecCross2D(std::vector<double>({expCableVec[i][ii][0],expCableVec[i][ii][1]}),
                                                         std::vector<double>({curCableVec[i][ii][0],curCableVec[i][ii][1]})) < 0)
                                    cableVecXYThetaTemp[i][ii] *= -1.0;
                            }
                        }
                    }
                }

                // qDebug() << "expCableForce:" << cableForceVec << "\n";
                emit(gcCableForceResultSignal(cableForceVec,curPlatformPose,endForceByCable_a));

                if(useSim){
                    std::vector<std::vector<double>> simAcc(endNum,std::vector<double>(6,0.0)),
                            simVel(endNum,std::vector<double>(6,0.0)),simPos(endNum,std::vector<double>(6,0.0));
                    for(int i=0;i<endNum;++i){
                        for(int ii=0;ii<6;++ii){
                            if(ii==2){
                                simAcc[i][ii] = (endForceByCable_a[i][ii]-weight[i])/(weight[i]/9.8);//z方向
                            }
                            else{
                                simAcc[i][ii] = endForceByCable_a[i][ii]/(weight[i]/9.8);
                            }
                            simVel[i][ii] = lastSimVel[i][ii]+simAcc[i][ii]*1000.0*ctrlCycleMs/1000.0;//统一mm
                            simPos[i][ii] = lastSimPos[i][ii]+simVel[i][ii]*ctrlCycleMs/1000.0;
                        }
                        if(!useEndRot){
                            for(int ii=0;ii<3;++ii)
                                simPos[i][ii+3] = 0.0;
                        }
                        trajPos3D.push_back({(float)simPos[i][0]/1000.0f,(float)simPos[i][1]/1000.0f,(float)simPos[i][2]/1000.0f});
                    }
                    qDebug() << "simAcc:" << simAcc;
                    qDebug() << "simVel:" << simVel;
                    qDebug() << "simPos:" << simPos;
                    lastSimVel = simVel;
                    lastSimPos = simPos;

                    curPlatformPose = lastSimPos;

                    emit update3DViewerSignal({{}},trajPos3D,anchorPos3D,cableEndPos3D);
                }

                isFirst = false;
            }

            cableForceVec.resize(0);// 重置
            impLastTimeS = curTimeS;
            impLastPlatformPose = curPlatformPose;
            impLastPlatformVel = curPlatformVel;
            lastEndForceByCable_a = endForceByCable_a;
        }

        isUpdate = false;
        //        forceDistribution(kine());
    }

    count++;
}

std::vector<std::vector<double>> ForceSimulationModel::getContactPose(std::vector<double> PlatformPose){
    std::vector<std::vector<double>> temp = MatrixFun::getTrans(PlatformPose);

    std::vector<std::vector<double>> endEffector_g,temp_g;
    endEffector_g.resize(cableNum);
    for (int i = 0; i < cableNum; i++){// 当前位姿下，各绳索接点的位置
        std::vector<std::vector<double>> te(4,{1.0});
        for(int ii=0;ii<3;++ii)
            te[ii] = {endEffector_e[i][ii]};

        temp_g = MatrixFun::matrixMulti(temp,te);
        endEffector_g[i].resize(4);
        for(int ii=0;ii<4;++ii)
            endEffector_g[i][ii] = temp_g[ii][0];// endEffector_g[i]为更新之后的位置
    }
    return endEffector_g;
}


double myfunc(unsigned n, const double *x, double *grad, void *my_func_data){
    //    ++count1;
    //    return 0.0;
    return pow((x[0]), 2) + pow((x[1]), 2) + pow((x[2]), 2) + pow((x[3]), 2) + pow((x[4]), 2) + pow((x[5]), 2) + pow((x[6]), 2) + pow((x[7]), 2);// 让各个绳索力尽量平均
    //     return pow((x[0] - 10), 2) + pow((x[1] - 10), 2) + pow((x[2] - 10), 2) + pow((x[3] - 10), 2) + pow((x[4] - 10), 2) + pow((x[5] - 10), 2) + pow((x[6] - 10), 2) + pow((x[7] - 10), 2);
    //     return pow((x[0] - 20), 2) + pow((x[1] - 20), 2) + pow((x[2] - 20), 2) + pow((x[3] - 20), 2) + pow((x[4] - 20), 2) + pow((x[5] - 20), 2) + pow((x[6] - 20), 2) + pow((x[7] - 20), 2);
}

// function1_fvec 是优化过程中计算目标函数值和梯度的函数。*ptr指针可以为空
// 需要放在优化算法构造函数kine()之前
void function1_fvec(const real_1d_array &x, real_1d_array &fi, void* ptr){
    ForceSimulationModel gCT;

    std::vector<std::vector<double>> Rotx = {{ 1,0,0,0 },
                                             { 0,cos(x[3]),-sin(x[3]),0 },
                                             { 0,sin(x[3]),cos(x[3]),0 },
                                             { 0,0,0,1 } };
    std::vector<std::vector<double>> Roty = {{cos(x[4]),0,sin(x[4]),0 },
                                             { 0,1,0,0 },
                                             { -sin(x[4]),0,cos(x[4]),0},
                                             { 0,0,0,1 } };
    std::vector<std::vector<double>> Rotz = {{ cos(x[5]),-sin(x[5]),0,0 },
                                             { sin(x[5]),cos(x[5]),0,0 },
                                             { 0,0,1,0 },
                                             { 0,0,0,1 } };
    std::vector<std::vector<double>> Txyz = {{ 1,0,0,x[0] },
                                             { 0,1,0,x[1] },
                                             { 0,0,1,x[2] },
                                             { 0,0,0,1 } };
    //Txyz不断右乘Rz\Ry\Rx，即生成最终的齐次变换矩阵；储存在temp中
    std::vector<std::vector<double>> temp = MatrixFun::matrixMulti(MatrixFun::matrixMulti(MatrixFun::matrixMulti(Txyz, Rotz), Roty), Rotx);

    std::vector<std::vector<double>> endEffector_g,temp_g;
    endEffector_g.resize(gCT.cableNum);
    for (int i = 0; i < gCT.cableNum; i++){// 当前位姿下，各绳索接点的位置
        std::vector<std::vector<double>> te(4,{1.0});
        for(int ii=0;ii<3;++ii)
            te[ii] = {gCT.endEffector_e[i][ii]};

        temp_g = MatrixFun::matrixMulti(temp,te);
        endEffector_g[i].resize(4);
        for(int ii=0;ii<4;++ii)
            endEffector_g[i][ii] = temp_g[ii][0];// endEffector_g[i]为更新之后的位置
    }
    gCT.curContactPos = endEffector_g;

    std::vector<double> cableLength_temp;//定义绳索长度的中间赋值量
    cableLength_temp.resize(gCT.cableNum);
    for (int icable = 0; icable < gCT.cableNum; icable++){// 理论绳长
        cableLength_temp[icable] =
                MatrixFun::cableLengthCalculate(endEffector_g[icable], gCT.curAnchorPos[icable], gCT.pulleyRadius).idealLength;
    }

    for (int i = 0; i < gCT.cableNum; i++){// 绳长变化量
        fi[i] = gCT.curCableLen[i] - cableLength_temp[i];
        //        qDebug() << fi[i];
    }

    //    qDebug() << "TEST1" << gCT.cableNum;
    //    for(int i=0;i<8;++i){
    //        qDebug() << "TEST2" << i << gCT.curCableLen.at(i);
    //        qDebug() << "TEST3" << i << gCT.curAnchorPos->at(i).at(0) << gCT.curAnchorPos->at(i).at(1)<< gCT.curAnchorPos->at(i).at(2);
    //    }
}

std::vector<double> ForceSimulationModel::kine(){
    real_1d_array x = "[0,0,400,0,0,0]";// 优化算法 平台位姿初值
    real_1d_array bndl = "[-3100,-3100,-40,-3.14,-3.14,-3.14]";// 优化算法 下限；Pz 与主界面安全工作空间下界一致
    real_1d_array bndu = "[3100,3100,3100,3.14,3.14,3.14]";// 优化算法 上限
    double epsx = 0.000001;//非线性最小二乘优化的停止条件之一
    ae_int_t maxits = 0;//非线性最小二乘优化的最大迭代次数
    minlmstate state;//用于存储非线性最小二乘优化过程的状态
    minlmreport rep;//用于存储非线性最小二乘优化过程的结果报告
    minlmcreatev(cableNum, x, 0.0001, state);//创建非线性最小二乘优化问题的初始状态 state。cableNum 表示方程数量，x 是变量初值（其长度决定未知数数量），0.0001 是初始步长，state 将被填充为优化问题的初始状态。
    minlmsetbc(state, bndl, bndu);//于设置非线性最小二乘优化问题的变量边界限制。state 是前面创建的状态，bndl 是最小边界限制数组，bndu 是最大边界限制数组。
    minlmsetcond(state, epsx, maxits);//设置非线性最小二乘优化的停止条件。state 是前面创建的状态，epsx 是停止条件之一，表示在变量向量的每个维度上，梯度范数小于此值时停止优化。maxits 是最大迭代次数，用于控制最大的优化迭代次数

    // 约束：计算得到的平台位姿所对应的绳长与已知绳长差别最小
    minlmoptimize(state, function1_fvec);//执行非线性最小二乘优化。state 是前面设置的状态，function1_fvec 是优化过程中计算目标函数值和梯度的函数。这个函数需要在优化前被定义，以便能够进行目标函数值和梯度的计算。
    minlmresults(state, x, rep);// 获取非线性最小二乘优化的结果。state 是前面执行优化后的状态，x 将被填充为最优解，rep 将被填充为优化过程的结果报告，其中包含了优化是否成功、迭代次数等信息
    std::vector<double> result;
    for(int i=0;i<6;++i){
        result.push_back(x[i]);
        //        qDebug() << "TEST KINE" << i << ":" << x[i];
    }
    return result;
}

//绳索力的不等式限制
void cableInequality(unsigned m, double *result,unsigned n, const double *x,
                     double *gradient, void *func_data){
    ForceSimulationModel gCT;
    for(int i=0;i<gCT.cableNum;++i){
        result[i] = 20 - x[i];
    }
}

//八根绳索力的等式限制
void myconstraint(unsigned m, double *result,unsigned n, const double *x,
                  double *gradient,void *func_data){
    ForceSimulationModel gCT;
    for(int j=0;j<m;++j){
        result[j] = 0.0;
        for(int i=0;i<gCT.cableNum;++i){
            result[j] -= gCT.jacoTrans[j][i] * x[i];
        }
        if(j<3)
            result[j] += gCT.endForce_e[j];
        else
            result[j] += gCT.endForce_e[j]*1000.0;// NM->Nmm
    }
}

double newFun(unsigned n, const double *x, double *grad, void *my_func_data){
    ForceSimulationModel gCT;
    double funResult = 0.0;
    std::vector<double> result(6);

    for(int j=0;j<6;++j){
        result[j] = 0.0;
        for(int i=0;i<gCT.cableNum;++i){
            result[j] -= gCT.jacoTrans[j][i] * x[i];
        }
        if(j<3)
            result[j] += gCT.endForce_e[j];
        else
            result[j] += gCT.endForce_e[j]*1000.0;// NM->Nmm

        funResult += pow(result[j],2.0);
    }

    return funResult;
}

void ForceSimulationModel::setMode(std::string modeName){
    if(modeName == "QHSingleEnd")
        mode = 1;
    else if(modeName == "ThirdParty")
        mode = 2;
    else
        mode = 0;
}

void ForceSimulationModel::update(std::vector<std::vector<std::vector<double>>> _curAnchorPos,std::vector<std::vector<double>> _curPlatformPose,
                                       std::vector<std::vector<double>> _expPlatformForce, std::vector<std::vector<double>> _curPlatformForce, double _curTimeStamp){
    // qDebug() << "PlatformPose(before reset rot):" << _curPlatformPose << "\n";
    QMutexLocker locker(&_dataMutex);
    if(!useEndRot){
        for(int i=0;i<_curPlatformPose.size();++i){
            for(int ii=0;ii<3;++ii)
                _curPlatformPose[i][ii+3] = 0.0;
        }
    }

    isUpdate = true;
    curAnchorPosTemp = _curAnchorPos;
    curPlatformPose = _curPlatformPose;
    expPlatformForce = _expPlatformForce;
    curPlatformForce = _curPlatformForce;
    curTimeStamp = _curTimeStamp;
}

void ForceSimulationModel::updateNoCam(std::vector<std::vector<std::vector<double> > > _curAnchorPos, std::vector<std::vector<double> > _curCableLen,
                                            std::vector<std::vector<double> > _expPlatformForce, std::vector<std::vector<double>> _curPlatformForce, double _curTimeStamp){
    QMutexLocker locker(&_dataMutex);
    isUpdate = true;
    curAnchorPosTemp = _curAnchorPos;
    curCableLenTemp = _curCableLen;
    expPlatformForce = _expPlatformForce;
    curPlatformForce = _curPlatformForce;
    curTimeStamp = _curTimeStamp;
}

std::vector<std::vector<double>> ForceSimulationModel::solvePoseFromCableLengths(
        std::vector<std::vector<std::vector<double>>> _curAnchorPos,
        std::vector<std::vector<double>> _curCableLen,
        bool keepRotation)
{
    QMutexLocker locker(&_dataMutex);

    std::vector<std::vector<double>> result;
    const bool previousUseEndRot = useEndRot;
    useEndRot = keepRotation;

    const int endCount = (std::min)(static_cast<int>(_curAnchorPos.size()),
                                    static_cast<int>(_curCableLen.size()));
    result.reserve(endCount);
    for(int endIndex=0; endIndex<endCount; ++endIndex){
        if(endIndex >= static_cast<int>(endEffector_eTemp.size())){
            result.push_back(std::vector<double>(6, 0.0));
            continue;
        }

        cableNum = (std::min)(
                    (std::min)(static_cast<int>(_curAnchorPos[endIndex].size()),
                               static_cast<int>(_curCableLen[endIndex].size())),
                    static_cast<int>(endEffector_eTemp[endIndex].size()));
        if(cableNum <= 0){
            result.push_back(std::vector<double>(6, 0.0));
            continue;
        }

        curAnchorPos.clear();
        curCableLen.clear();
        endEffector_e.clear();
        for(int cableIndex=0; cableIndex<cableNum; ++cableIndex){
            curAnchorPos.push_back(_curAnchorPos[endIndex][cableIndex]);
            curCableLen.push_back(_curCableLen[endIndex][cableIndex]);
            endEffector_e.push_back(endEffector_eTemp[endIndex][cableIndex]);
        }

        std::vector<double> pose = kine();
        if(!keepRotation && pose.size() >= 6){
            for(int i=3; i<6; ++i){
                pose[i] = 0.0;
            }
        }
        result.push_back(pose);
    }

    useEndRot = previousUseEndRot;
    return result;
}

void ForceSimulationModel::updateCurCableForce(std::vector<std::vector<double>> f){
    QMutexLocker locker(&_dataMutex);
    endCableForce = f;
    //    endCableForce = {{8.35643, 8.27412, 8.41261, 8.25316, 8.59914, 8.70542, 8.51598, 8.57757}};
}

double objFunNlopt(unsigned n, const double *x, double *grad, void *my_func_data){
    double result = 0.0;

    ForceSimulationModel gCT;
    std::vector<double> temp(6,0.0);
    for(int j=0;j<6;++j){
        for(int i=0;i<gCT.cableNum;++i){
            temp[j] -= gCT.jacoTrans[j][i] * x[i];
        }
        if(j<3)
            temp[j] += gCT.endForce_e[j];
        else
            temp[j] += gCT.endForce_e[j]*1000.0;// NM->Nmm
        result += pow(temp[j],2.0);
    }

    return result;
}

std::vector<double> ForceSimulationModel::forceDistribution(std::vector<double> pose){
    //    qDebug() << "forceDistribution input pose:" << pose;
    cableForce_e.resize(cableNum);

    std::vector<std::vector<double>> endEffector_g = getEndEffector_g(pose);// 基于平台位姿，求各绳索接点的位置
    //    qDebug() << "contactPointPos:" << endEffector_g << "\n";
    std::vector<double> cableLength_e;// 绳长
    std::vector<std::vector<double>> unitCableVec,centerVec;// 各绳索接点处单位力和平台中心到各接点的向量
    std::vector<std::vector<double>> crossVecTemp;
    cableLength_e.resize(cableNum);
    unitCableVec.resize(cableNum);
    centerVec.resize(cableNum);
    crossVecTemp.resize(cableNum);
    jacoTrans.resize(6);
    for(int i=0;i<6;++i)
        jacoTrans[i].resize(cableNum);

    // 计算雅可比矩阵
    int resultNum = 6;
    for (int iCable = 0; iCable < cableNum; iCable++){
        const MatrixFun::CableLengthResult cableGeometry =
                MatrixFun::cableLengthCalculate(endEffector_g[iCable], curAnchorPos[iCable], pulleyRadius);
        cableLength_e[iCable] = cableGeometry.idealLength;

        unitCableVec[iCable].resize(3);
        centerVec[iCable].resize(3);
        for (int i = 0; i < 3; ++i){
            unitCableVec[iCable][i] = (cableGeometry.tangentPoint[i] - endEffector_g[iCable][i]) / cableLength_e[iCable];
            centerVec[iCable][i] = endEffector_g[iCable][i] - pose[i];
        }
        crossVecTemp[iCable] = MatrixFun::vecCross(centerVec[iCable], unitCableVec[iCable]);

        for (int i = 0; i < 6; ++i){
            if (i < 2){// Fx Fy
                jacoTrans[i][iCable] = unitCableVec[iCable][i];
                // jacoTrans[i][iCable] = 0.0;// 不考虑Fx Fy
            }
            else if(i == 2){// Fz
                jacoTrans[i][iCable] = unitCableVec[iCable][i];
            }
            else{// 力矩
                jacoTrans[i][iCable] = crossVecTemp[iCable][i-3];
                if(!compTor)
                    resultNum = 3;// 不考虑力矩，修改求解量个数（不会循环到雅可比矩阵后三行，即力矩那几行）
            }
        }
    }
    //    qDebug() << jacoTrans;

    double* ub = new double[cableNum];// 上限
    double* lb = new double[cableNum];// 下限
    double* x = new double[cableNum];// 初值
    for (int i = 0; i < cableNum; ++i){
        ub[i] = curCableMaxF[i];
        lb[i] = cableMinF;
        x[i] = lb[i];// 初值取下限，让绳索力尽量小
    }

    nlopt_opt opt;
    opt = nlopt_create(NLOPT_LN_COBYLA, cableNum);
    nlopt_set_upper_bounds(opt, ub);
    nlopt_set_lower_bounds(opt, lb);
    nlopt_set_maxtime(opt, 0.5);// 至多运行0.5s
    nlopt_set_min_objective(opt, myfunc, NULL);
    //nlopt_set_min_objective(opt, newFun, NULL);
    my_constraint_data data[2] = { { 0, 0 }, { 0, 0 } };

    // 绳索力最大值约束（不等式约束）（>20N）
    // 用于代替下限：根据宗凯所言，当使用下限20N，更容易出现无解情况，而用不等式则会缓解。因为不等式并不严格规定下限20，而是尽量大于20（有可能出现小于20）
    // 目前未发现无解情况，因此依旧先用下限
    double* cableInequalityTol = new double[cableNum];// 不等式右端的值的数列
    for(int i=0;i<cableNum;++i)
        cableInequalityTol[i] = 1e-6;// 即<=0。别改成别的值，因为不等式也是以是否在0附近为判据的

    // 绳索力抵消末端平台自身力和力矩，以实现零力状态（等式约束）
    double* cableEqualityTol = new double[6];// 不等式右端的值的数列
    for(int i=0;i<6;++i)
        cableEqualityTol[i] = 1e-6;// 即<=0。别改成别的值，因为不等式也是以是否在0附近为判据的

    // nlopt_add_inequality_mconstraint(opt,cableNum,cableInequality,NULL,cableInequalityTol);

    // 根据实测，使用等式进行力补偿，能在位姿是常量时求出唯一解；而使用最小二乘法，则在位姿是常量时求出的解浮动较大
    if(sta == "equ")
        nlopt_add_equality_mconstraint(opt, resultNum, myconstraint, NULL, cableEqualityTol);// 补偿力和力矩
    else if(sta == "obj")
        nlopt_set_min_objective(opt, objFunNlopt, NULL);// 补偿力和力矩
    else{
        qDebug() << "GravityCompensationThread warning: unknown sta, use equ instead.";
        nlopt_add_equality_mconstraint(opt, resultNum, myconstraint, NULL, cableEqualityTol);// 补偿力和力矩
    }

    nlopt_set_xtol_rel(opt, 1e-6);
    double minf;
    nlopt_result resultCode = nlopt_optimize(opt, x, &minf);

    optErr = true;
    if (resultCode < 0){
        displayInfoSignal("零力线程错误: 力分配优化失败","error");
        qDebug() << "GravityCompensationThread error: nlopt optimization fail.";
    }
    else if(resultCode == NLOPT_MAXTIME_REACHED){
        displayInfoSignal("零力线程错误: 力分配优化失败，因为超过了允许的优化时间","error");
        qDebug() << "GravityCompensationThread warning: nlopt optimization stop because max time is reached.";
    }
    else{
        for (int i = 0; i < cableNum; i++){
            cableForce_e[i] = x[i];
        }
        optErr = false;
    }
    nlopt_destroy(opt);
    delete[] ub;
    delete[] lb;
    delete[] x;
    delete[] cableInequalityTol;
    delete[] cableEqualityTol;

    // 验证：分别计算XYZ方向的合力
    //    std::vector<double> result(6);
    //    for(int j=0;j<6;++j){
    //        result[j] = 0.0;
    //        for(int i=0;i<cableNum;++i){
    //            result[j] -= jacoTrans[j][i] * cableForce_e[i];
    //        }
    //        result[j] -= endForce_e[j];
    //    }
    //    qDebug() << "TEST:" << result;

    std::vector<std::vector<double>> cableDirVec(8);
    std::vector<double> cableForceX(8),cableForceY(8),cableForceZ(8);
    for(int i=0;i<curAnchorPos.size();++i){
        for(int ii=0;ii<3;++ii){
            cableDirVec[i].push_back(unitCableVec[i][ii]);
        }
        cableForceX[i] = cableDirVec[i][0]*cableForce_e[i];
        cableForceY[i] = cableDirVec[i][1]*cableForce_e[i];
        cableForceZ[i] = cableDirVec[i][2]*cableForce_e[i];
        //        qDebug() << "index:" << i << "cableVec:" << cableVec[i] << "dir:" << cableDirVec[i] << "cableForce:" << cableForce_e[i] <<
        //                    "cableForceX:" << cableForceX[i] << "cableForceY:" << cableForceY[i] << "cableForceZ:" << cableForceZ[i];
    }
    double fx(0),fy(0),fz(0),mx(0),my(0),mz(0);
    for(int i=0;i<curAnchorPos.size();++i){
        fx += cableForceX[i];
        fy += cableForceY[i];
        fz += cableForceZ[i];
        mx += crossVecTemp[i][0]*cableForce_e[i];
        my += crossVecTemp[i][1]*cableForce_e[i];
        mz += crossVecTemp[i][2]*cableForce_e[i];
    }
    qDebug() << "Fx(exp):" << fx << "Fy(exp):" << fy << "Fz(exp):" << fz << "Mx(exp):" << mx << "My(exp):" << my << "Mz(exp):" << mz;

    // 计算末端实际受力情况（在没有六维力传感器的情况下）
    double fx_r(0),fy_r(0),fz_r(0),mx_r(0),my_r(0),mz_r(0);
    for(int i=0;i<curAnchorPos.size();++i){// curEndCableForce[i] = cableForce_e[i];
        fx_r += cableDirVec[i][0]*curEndCableForce[i];
        fy_r += cableDirVec[i][1]*curEndCableForce[i];
        fz_r += cableDirVec[i][2]*curEndCableForce[i];
        mx_r += crossVecTemp[i][0]*curEndCableForce[i];
        my_r += crossVecTemp[i][1]*curEndCableForce[i];
        mz_r += crossVecTemp[i][2]*curEndCableForce[i];
    }
    endForceByCable_a.push_back({fx_r,fy_r,fz_r,mx_r,my_r,mz_r});// qDebug() << curEndCableForce;
    qDebug() << "Cable cal:" << "Fx(act):" << fx_r << "Fy(act):" << fy_r << "Fz(act):" << fz_r << "Mx(act):" << mx_r << "My(act):" << my_r << "Mz(act):" << mz_r;
    qDebug() << cableForce_e << curEndCableForce;

    //    qDebug() << "pose:" << pose << "cableLength_e:" << cableLength_e;
    //    for(int i=0;i<cableNum;++i)
    //        qDebug() << "cableForce" << i << ":" << cableForce_e[i];

    return cableForce_e;
}

std::vector<std::vector<double>> ForceSimulationModel::getEndEffector_g(std::vector<double> pose){
    std::vector<std::vector<double>> Rotx = {{ 1,0,0,0 },
                                             { 0,cos(pose[3]),-sin(pose[3]),0 },
                                             { 0,sin(pose[3]),cos(pose[3]),0 },
                                             { 0,0,0,1 } };
    std::vector<std::vector<double>> Roty = {{cos(pose[4]),0,sin(pose[4]),0 },
                                             { 0,1,0,0 },
                                             { -sin(pose[4]),0,cos(pose[4]),0},
                                             { 0,0,0,1 } };
    std::vector<std::vector<double>> Rotz = {{ cos(pose[5]),-sin(pose[5]),0,0 },
                                             { sin(pose[5]),cos(pose[5]),0,0 },
                                             { 0,0,1,0 },
                                             { 0,0,0,1 } };
    std::vector<std::vector<double>> Txyz = {{ 1,0,0,pose[0] },
                                             { 0,1,0,pose[1] },
                                             { 0,0,1,pose[2] },
                                             { 0,0,0,1 } };
    std::vector<std::vector<double>> temp = MatrixFun::matrixMulti(MatrixFun::matrixMulti(MatrixFun::matrixMulti(Txyz, Rotz), Roty), Rotx);// 基于末端平台位姿构建对应的齐次变换矩阵

    std::vector<std::vector<double>> endEffector_g,temp_g;
    endEffector_g.resize(cableNum);
    for (int i = 0; i < cableNum; i++){// 当前位姿下，各绳索接点的位置
        std::vector<std::vector<double>> te(4,{1.0});
        for(int ii=0;ii<3;++ii)
            te[ii] = {endEffector_e[i][ii]};

        temp_g = MatrixFun::matrixMulti(temp,te);
        endEffector_g[i].resize(4);
        for(int ii=0;ii<4;++ii)
            endEffector_g[i][ii] = temp_g[ii][0];// endEffector_g[i]为更新之后的位置
    }
    return endEffector_g;
}
